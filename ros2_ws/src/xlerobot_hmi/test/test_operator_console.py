import asyncio
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavigationPath
import numpy as np
import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image
from xlerobot_hmi.operator_console import (
    _diagnostic_level_for_readiness,
    _image_array,
    _transform_point_xyz,
    _uint8,
    available_workspaces,
    ConsoleApplication,
    goal_info_from_task_id,
    load_named_places,
    map_qos_profile,
    map_visualization_enabled,
    observation_dict,
    OperatorConsoleNode,
    parse_task_request,
    TaskHistory,
    TaskRecord,
    transient_state_qos_profile,
)
from xlerobot_hmi.manual_control import ManualControlCoordinator
from xlerobot_interfaces.msg import CapabilityError, PerceptionObservation, TaskEvent
from xlerobot_interfaces.srv import FinalizeEpisode, ReviewEpisode, ServoCalibrationStep
import yaml
from xlerobot_assets import ArtifactCatalog


def test_site_summary_reads_draft_without_activating_or_modifying_it(tmp_path):
    catalog = ArtifactCatalog(tmp_path)
    draft = tmp_path / 'sites' / '.drafts' / 'home'
    draft.mkdir(parents=True)
    (draft / 'mapping_session.json').write_text(json.dumps({
        'map_name': 'ground-floor', 'map_saved_at': '2026-09-05T09:42:00Z',
    }))
    (draft / 'map.yaml').write_text('image: map.pgm\n')
    (draft / 'places.yaml').write_text(yaml.safe_dump({'named_places': {'places': {
        'table': {'x': .1, 'y': -.07, 'yaw': 2.0, 'dock': True, 'nav_offset_m': .25},
    }}}))
    before = {p.name: p.read_bytes() for p in draft.iterdir()}
    node = SimpleNamespace(catalog=catalog, artifact_root=tmp_path,
                           parameter=lambda name: name == 'enable_engineering_tools')

    async def exercise():
        app = ConsoleApplication(node)
        response = await app.sites(SimpleNamespace(query={'site_id': 'home'}))
        result = json.loads(response.text)
        assert result['active_version'] == ''
        assert result['draft']['map_saved'] is True
        assert result['draft']['map_name'] == 'ground-floor'
        assert result['draft']['ready'] is False
        assert result['draft']['places'] == [{
            'id': 'table', 'x': .1, 'y': -.07, 'yaw': 2.0, 'nav_offset_m': .25,
            'dock': True, 'validated': False,
        }]
        assert {p.name: p.read_bytes() for p in draft.iterdir()} == before
        (draft / 'validation.json').write_text(json.dumps({
            'places': {'table': {'passed': True}},
        }))
        validated = json.loads((await app.sites(SimpleNamespace(query={'site_id': 'home'}))).text)
        assert validated['draft']['ready'] is True
        assert validated['active_version'] == ''
        empty = json.loads((await app.sites(SimpleNamespace(query={'site_id': 'new'}))).text)
        assert empty['draft']['map_saved'] is False
        assert empty['draft']['places'] == []
        assert not (tmp_path / 'sites' / '.drafts' / 'new').exists()
        with pytest.raises(web.HTTPBadRequest):
            await app.sites(SimpleNamespace(query={'site_id': '../escape'}))
        (draft / 'places.yaml').write_text('named_places: [')
        with pytest.raises(web.HTTPConflict, match='Conflict'):
            await app.sites(SimpleNamespace(query={'site_id': 'home'}))

    asyncio.run(exercise())


def test_parse_task_request_uses_stable_execute_task_contract():
    goal = parse_task_request({
        'object_id': ' 羽毛球 ', 'grasp_backend': 'gpd', 'dry_run': True,
    })
    assert goal.object_id == '羽毛球'
    assert goal.source_place == 'table'
    assert goal.recipient_id == 'nearest_person'
    assert goal.grasp_backend == 'gpd'
    assert goal.dry_run is True


@pytest.mark.parametrize('payload', [None, [], {}, {'object_id': ' '}])
def test_parse_task_request_rejects_invalid_payload(payload):
    with pytest.raises(ValueError):
        parse_task_request(payload)


def test_task_history_survives_reopen(tmp_path: Path):
    database = tmp_path / 'console.sqlite3'
    record = TaskRecord(
        task_id='task-1', object_id='ball', source_place='table',
        recipient_id='nearest_person', grasp_backend='act', dry_run=True,
        created_at='2026-07-13T00:00:00+00:00',
        updated_at='2026-07-13T00:00:01+00:00',
    )
    TaskHistory(database).save(record)
    assert TaskHistory(database).recent()[0]['object_id'] == 'ball'
    trace = (tmp_path / 'task-1.jsonl').read_text(encoding='utf-8').splitlines()
    assert len(trace) == 1
    assert 'xlerobot_task_trace/v1' in trace[0]


def test_task_history_index_is_bounded_to_latest_one_hundred(tmp_path: Path):
    history = TaskHistory(tmp_path / 'console.sqlite3')
    for index in range(105):
        stamp = f'2026-07-20T00:00:00.{index:06d}+00:00'
        history.save(TaskRecord(
            task_id=f'task-{index:03d}',
            object_id='ball',
            source_place='table',
            recipient_id='nearest_person',
            grasp_backend='act',
            dry_run=True,
            created_at=stamp,
            updated_at=stamp,
        ))
    records = history.recent(100)
    assert len(records) == 100
    assert records[0]['task_id'] == 'task-104'
    assert records[-1]['task_id'] == 'task-005'


def test_voice_task_events_are_persisted_and_exposed_live():
    saved = []
    published = []
    node = object.__new__(OperatorConsoleNode)
    node._task_lock = threading.Lock()
    node.active_task = None
    node.external_task = None
    node._owned_task_ids = set()
    node._external_stage_started_monotonic = 0.0
    node._teleop_publisher = None
    node._teleop_was_active = False
    node._observation_lock = threading.Lock()
    node._perception_state = None
    node.manual_control = ManualControlCoordinator()
    node.history = SimpleNamespace(
        save=lambda record: saved.append(record),
        recent=lambda: [{'task_id': saved[-1].task_id}],
    )
    node.events = SimpleNamespace(
        publish=lambda event_type, payload: published.append((event_type, payload))
    )
    event = TaskEvent()
    event.stamp.sec = 1_784_057_000
    event.task_id = '00112233445566778899aabbccddeeff'
    event.object_id = '羽毛球'
    event.source_place = 'table'
    event.recipient_id = 'nearest_person'
    event.grasp_backend = 'centroid'
    event.status = 'RUNNING'
    event.current_capability = 'detect_object'
    event.state.phase = 'detecting'
    event.state.progress = 0.25
    OperatorConsoleNode._on_task_event(node, event)
    assert saved[-1].task_id == '00112233445566778899aabbccddeeff'
    assert saved[-1].cancelable is True
    assert published[-2][0] == 'task'
    assert published[-1] == (
        'task_history',
        {'tasks': [{'task_id': '00112233445566778899aabbccddeeff'}]},
    )


def test_perception_observation_is_bounded_for_the_browser():
    message = PerceptionObservation()
    message.observation_id = 'person-1'
    message.kind = 'person'
    message.label = 'person'
    message.camera_id = 'head'
    message.image_width = 640
    message.image_height = 480
    message.bbox_x1, message.bbox_y1 = 10.0, 20.0
    message.bbox_x2, message.bbox_y2 = 100.0, 200.0
    message.target.header.frame_id = 'map'
    message.target.point.x = 1.5
    snapshot = observation_dict(message)
    assert snapshot['bbox'] == [10.0, 20.0, 100.0, 200.0]
    assert snapshot['target'] == {'frame_id': 'map', 'x': 1.5, 'y': 0.0, 'z': 0.0}


def test_navigation_path_is_downsampled_and_published_in_map_frame():
    published = []
    node = object.__new__(OperatorConsoleNode)
    node.manual_control = ManualControlCoordinator()
    node._mapping_lock = threading.Lock()
    node._mapping_state = {'path': None}
    node.events = SimpleNamespace(
        publish=lambda event_type, payload: published.append((event_type, payload))
    )
    path = NavigationPath()
    path.header.frame_id = 'map'
    for index in range(620):
        pose = PoseStamped()
        path.poses.append(pose)
        pose.pose.position.x = index * 0.01
    OperatorConsoleNode._on_navigation_path(node, path)
    assert len(node._mapping_state['path']['points_xy']) <= 310
    assert published[-1][1]['kind'] == 'path'


def test_operator_teleop_rejects_commands_and_zeros_base_while_busy():
    published = []
    node = object.__new__(OperatorConsoleNode)
    node.manual_control = ManualControlCoordinator()
    node._teleop_publisher = SimpleNamespace(
        publish=lambda message: published.append(message)
    )
    node._teleop_was_active = True
    node.parameter = lambda name: {
        'workspace': 'operator',
        'teleop_max_linear_mps': 0.12,
        'teleop_max_angular_radps': 0.50,
    }[name]
    node.task_active = lambda: True
    node.manual_action_active = lambda: False
    node.drive_stop_latched = False
    with pytest.raises(RuntimeError, match='idle task runtime'):
        OperatorConsoleNode.command_teleop(node, 0.10, 0.20, True)
    assert published[-1].linear.x == 0.0
    assert published[-1].angular.z == 0.0
    assert node._teleop_was_active is False


def test_operator_teleop_is_bounded_when_idle():
    published = []
    node = object.__new__(OperatorConsoleNode)
    node.manual_control = ManualControlCoordinator()
    node._teleop_publisher = SimpleNamespace(
        publish=lambda message: published.append(message)
    )
    node._teleop_was_active = False
    node.parameter = lambda name: {
        'workspace': 'operator',
        'teleop_max_linear_mps': 0.12,
        'teleop_max_angular_radps': 0.50,
    }[name]
    node.task_active = lambda: False
    node.manual_action_active = lambda: False
    node.drive_stop_latched = False
    OperatorConsoleNode.command_teleop(node, 1.0, -1.0, True)
    assert published[-1].linear.x == pytest.approx(0.12)
    assert published[-1].angular.z == pytest.approx(-0.50)
    assert node._teleop_was_active is True


def test_operator_is_not_idle_while_manual_goal_is_being_established():
    node = object.__new__(OperatorConsoleNode)
    node.manual_control = ManualControlCoordinator()
    assert node.manual_control.begin_action() is not None
    node.task_snapshot = lambda: None
    assert OperatorConsoleNode.operator_idle(node) is False


def test_manual_action_timeout_cancels_and_drains_before_releasing():
    class ServiceClient:
        def __init__(self):
            self.requests = []

        def service_is_ready(self):
            return True

        def call_async(self, request):
            self.requests.append(bool(request.data))
            future = asyncio.get_running_loop().create_future()
            future.set_result(SimpleNamespace(success=True, message='ok'))
            return future

    class GoalHandle:
        accepted = True

        def __init__(self):
            self.cancel_called = False
            self.result_future = asyncio.get_running_loop().create_future()

        def get_result_async(self):
            return self.result_future

        def cancel_goal_async(self):
            self.cancel_called = True
            result = SimpleNamespace(
                error=SimpleNamespace(
                    code=CapabilityError.CANCELED, message='canceled'
                )
            )
            self.result_future.set_result(SimpleNamespace(
                result=result, status=GoalStatus.STATUS_CANCELED
            ))
            future = asyncio.get_running_loop().create_future()
            future.set_result(SimpleNamespace())
            return future

    class ActionClient:
        def __init__(self, handle):
            self.handle = handle

        def server_is_ready(self):
            return True

        def send_goal_async(self, _goal):
            future = asyncio.get_running_loop().create_future()
            future.set_result(self.handle)
            return future

    async def exercise():
        handle = GoalHandle()
        reservation = ServiceClient()
        node = SimpleNamespace(
            drive_stop_latched=False,
            task_active=lambda: False,
            teleop_active=lambda: False,
            stop_teleop=lambda: None,
            manual_control=ManualControlCoordinator(),
            manual_reservation_client=reservation,
        )
        app = ConsoleApplication(node)
        with pytest.raises(web.HTTPGatewayTimeout) as captured:
            await app._run_operator_action(
                ActionClient(handle), object(), 'ManualAction', 0.01
            )
        assert 'timed out' in captured.value.text
        assert handle.cancel_called
        assert reservation.requests == [True, False]
        assert node.manual_control.action_active() is False

    asyncio.run(exercise())

def test_calibration_action_does_not_require_demo_stop_or_task_reservation():
    class GoalHandle:
        accepted = True

        @staticmethod
        def get_result_async():
            future = asyncio.get_running_loop().create_future()
            future.set_result(SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED,
                result=SimpleNamespace(
                    error=SimpleNamespace(
                        code=CapabilityError.NONE, message='pose reached'
                    )
                ),
            ))
            return future

    class ActionClient:
        @staticmethod
        def server_is_ready():
            return True

        @staticmethod
        def send_goal_async(_goal):
            future = asyncio.get_running_loop().create_future()
            future.set_result(GoalHandle())
            return future

    async def exercise():
        node = SimpleNamespace(
            manual_control=ManualControlCoordinator(),
        )
        result = await ConsoleApplication(node)._run_calibration_action(
            ActionClient(), object(), 'MoveCalibrationPose', 1.0
        )
        assert result.error.code == CapabilityError.NONE
        assert node.manual_control.action_active() is False

    asyncio.run(exercise())


def test_lidar_preview_applies_full_upside_down_mount_transform():
    transform = SimpleNamespace(
        rotation=SimpleNamespace(x=-0.056, y=0.998, z=0.0, w=0.0),
        translation=SimpleNamespace(x=-0.03, y=0.0, z=0.36),
    )
    x, y, z = _transform_point_xyz(transform, (1.0, 0.0, 0.0))
    assert x == pytest.approx(-1.0237, abs=0.002)
    assert y == pytest.approx(-0.1118, abs=0.002)
    assert z == pytest.approx(0.36, abs=0.002)


def test_rgb_ros_image_is_converted_to_bgr_without_copying_row_padding():
    image = Image()
    image.height = 1
    image.width = 2
    image.encoding = 'rgb8'
    image.step = 8
    image.data = bytes([255, 0, 0, 0, 255, 0, 99, 99])
    converted = _image_array(image)
    assert converted.shape == (1, 2, 3)
    assert converted.tolist() == [[[0, 0, 255], [0, 255, 0]]]


def test_ros_uint8_is_normalized_for_both_generator_representations():
    assert _uint8(2) == 2
    assert _uint8(b'\x02') == 2


def test_controller_timing_alarm_degrades_without_blocking_runtime():
    item = {
        'level': 2,
        'message': 'High execution jitter or mean error : [ right_arm_controller ]',
    }
    assert _diagnostic_level_for_readiness(
        'controller_manager: Controllers Activity', item
    ) == 1
    assert _diagnostic_level_for_readiness(
        'controller_manager: Hardware Components Activity', item
    ) == 1


def test_real_controller_error_still_blocks_runtime():
    item = {'level': 2, 'message': 'controller is not active'}
    assert _diagnostic_level_for_readiness(
        'controller_manager: Controllers Activity', item
    ) == 2


def test_launch_profile_exposes_only_its_owned_workspace():
    assert available_workspaces('operator') == ['operator']
    assert available_workspaces('mapping') == ['mapping']
    assert available_workspaces('calibration') == ['calibration']
    assert available_workspaces('collection') == ['collection']
    assert available_workspaces('unknown') == []


def test_map_is_read_only_in_demo_and_absent_from_other_tools():
    assert map_visualization_enabled('operator')
    assert map_visualization_enabled('mapping')
    assert not map_visualization_enabled('calibration')
    assert not map_visualization_enabled('collection')


def test_map_subscription_receives_already_published_active_map():
    profile = map_qos_profile()
    assert profile.reliability == ReliabilityPolicy.RELIABLE
    assert profile.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_task_and_drive_state_subscriptions_receive_latched_state():
    profile = transient_state_qos_profile(20)
    assert profile.reliability == ReliabilityPolicy.RELIABLE
    assert profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert profile.depth == 20


def test_task_event_id_maps_to_standard_action_goal_uuid():
    task_id = '00112233445566778899aabbccddeeff'
    assert bytes(goal_info_from_task_id(task_id).goal_id.uuid).hex() == task_id
    with pytest.raises(ValueError, match='ROS action UUID'):
        goal_info_from_task_id('voice-goal')


def test_external_task_cancel_uses_standard_action_cancel_service():
    class CancelClient:
        def __init__(self):
            self.request = None

        def service_is_ready(self):
            return True

        def call_async(self, request):
            self.request = request
            future = asyncio.get_running_loop().create_future()
            response = CancelGoal.Response()
            response.return_code = CancelGoal.Response.ERROR_NONE
            future.set_result(response)
            return future

    async def exercise():
        client = CancelClient()
        node = object.__new__(OperatorConsoleNode)
        node.task_cancel_client = client
        task_id = '00112233445566778899aabbccddeeff'
        await OperatorConsoleNode.cancel_task_uuid(node, task_id)
        assert bytes(client.request.goal_info.goal_id.uuid).hex() == task_id

    asyncio.run(exercise())


def test_operator_health_requires_fresh_execute_task_live_readiness():
    node = object.__new__(OperatorConsoleNode)
    node.manual_control = ManualControlCoordinator()
    node.task_client = SimpleNamespace(server_is_ready=lambda: True)
    node.manual_reservation_client = SimpleNamespace(service_is_ready=lambda: True)
    node.drive_stop_client = SimpleNamespace(service_is_ready=lambda: True)
    node.task_cancel_client = SimpleNamespace(service_is_ready=lambda: True)
    node.parameter = lambda name: {'workspace': 'operator'}[name]
    node.drive_stop_latched = False
    node.voice_state = 'DISABLED'
    node._camera_lock = threading.Lock()
    node._camera_received_at = {'head': time.monotonic(), 'wrist': time.monotonic()}
    node.diagnostics = {
        'xlerobot/execute_task': {
            'level': 1,
            'message': 'wrist camera is stale',
            'hardware_id': 'reference',
            'values': {'live_ready': 'false'},
            'received_monotonic': time.monotonic(),
        },
    }
    health = OperatorConsoleNode.health(node)
    assert health['requirements']['execute_task_live_ready'] is False
    assert health['readiness'] == 'BLOCKED'
    node.diagnostics['xlerobot/execute_task'].update({
        'level': 0,
        'message': 'task execution ready',
        'values': {'live_ready': 'true'},
    })
    health = OperatorConsoleNode.health(node)
    assert health['requirements']['execute_task_live_ready'] is True
    assert health['readiness'] == 'READY'
    published = []
    node.events = SimpleNamespace(
        publish=lambda kind, payload: published.append((kind, payload))
    )
    node.diagnostics['xlerobot/execute_task']['received_monotonic'] = (
        time.monotonic() - 3.0
    )
    OperatorConsoleNode._publish_health(node)
    assert published[-1][0] == 'health'
    assert (
        published[-1][1]['requirements']['execute_task_live_ready'] is False
    )
    assert published[-1][1]['readiness'] == 'BLOCKED'


def test_acceptance_unknown_recovers_server_uuid_and_remains_cancelable():
    published = []
    canceled = []
    node = object.__new__(OperatorConsoleNode)
    node.manual_control = ManualControlCoordinator()
    node.drive_stop_latched = False
    node.manual_action_active = lambda: False
    node.teleop_active = lambda: False
    node.task_client = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda _goal, feedback_callback: _failed_future(
            RuntimeError('accept response lost')
        ),
    )
    node._task_lock = threading.Lock()
    node._teleop_publisher = None
    node._teleop_was_active = False
    node._observation_lock = threading.Lock()
    node._perception_state = None
    node.active_task = None
    node.external_task = None
    node.active_goal_handle = None
    node.active_server_task_id = None
    node._owned_task_ids = set()
    node._task_cancel_requested = threading.Event()
    node._stage_started_monotonic = 0.0
    node._external_stage_started_monotonic = 0.0
    node.history = SimpleNamespace(
        save=lambda _record: None,
        recent=lambda: [],
    )
    node.events = SimpleNamespace(
        publish=lambda kind, payload: published.append((kind, payload))
    )

    async def cancel_uuid(task_id):
        canceled.append(task_id)

    async def terminal_snapshot(task_id, _timeout):
        return {'task_id': task_id, 'status': 'CANCELED'}

    node.cancel_task_uuid = cancel_uuid
    node.wait_task_terminal = terminal_snapshot
    goal = SimpleNamespace(
        object_id='ball',
        source_place='table',
        recipient_id='nearest_person',
        grasp_backend='act',
        dry_run=False,
    )

    async def exercise():
        with pytest.raises(web.HTTPServiceUnavailable):
            await OperatorConsoleNode.submit_task(node, goal)
        assert published[0][0] == 'task'
        assert published[0][1]['status'] == 'ACCEPTING'
        assert node.active_task.status == 'ACCEPTANCE_UNKNOWN'
        event = TaskEvent()
        event.task_id = '00112233445566778899aabbccddeeff'
        event.object_id = goal.object_id
        event.source_place = goal.source_place
        event.recipient_id = goal.recipient_id
        event.grasp_backend = goal.grasp_backend
        event.dry_run = goal.dry_run
        event.status = 'RUNNING'
        event.current_capability = 'auto_localize'
        OperatorConsoleNode._on_task_event(node, event)
        assert node.active_server_task_id == event.task_id
        local_id = node.active_task.task_id
        result = await OperatorConsoleNode.cancel_task(node, local_id)
        assert result['status'] == 'CANCELED'
        assert canceled == [event.task_id]

    asyncio.run(exercise())


def test_terminal_task_event_before_goal_response_is_not_revived():
    published = []
    server_task_id = '00112233445566778899aabbccddeeff'

    async def exercise():
        acceptance = asyncio.get_running_loop().create_future()
        client = SimpleNamespace(
            server_is_ready=lambda: True,
            send_goal_async=lambda _goal, feedback_callback: acceptance,
        )
        node = _task_test_node(client, published)
        goal = SimpleNamespace(
            object_id='ball',
            source_place='table',
            recipient_id='nearest_person',
            grasp_backend='act',
            dry_run=False,
        )
        submission = asyncio.create_task(
            OperatorConsoleNode.submit_task(node, goal)
        )
        await asyncio.sleep(0)
        event = TaskEvent()
        event.task_id = server_task_id
        event.object_id = goal.object_id
        event.source_place = goal.source_place
        event.recipient_id = goal.recipient_id
        event.grasp_backend = goal.grasp_backend
        event.dry_run = goal.dry_run
        event.status = 'CANCELED'
        event.error.code = CapabilityError.CANCELED
        OperatorConsoleNode._on_task_event(node, event)
        handle = SimpleNamespace(
            accepted=True,
            goal_id=SimpleNamespace(uuid=list(bytes.fromhex(server_task_id))),
            get_result_async=lambda: pytest.fail(
                'terminal TaskEvent already closed ownership'
            ),
        )
        acceptance.set_result(handle)
        result = await submission
        assert result['status'] == 'CANCELED'
        assert node.active_task.status == 'CANCELED'
        assert node.active_goal_handle is None

    asyncio.run(exercise())


def test_succeeded_action_transport_with_capability_error_is_failed_terminal():
    node = _task_test_node(SimpleNamespace(), [])
    now = '2026-07-20T00:00:00+00:00'
    node.active_task = TaskRecord(
        task_id='local-task',
        object_id='ball',
        source_place='table',
        recipient_id='nearest_person',
        grasp_backend='act',
        dry_run=False,
        status='RUNNING',
        created_at=now,
        updated_at=now,
    )
    node.active_goal_handle = object()
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            error=SimpleNamespace(
                code=CapabilityError.INTERNAL_ERROR,
                message='business failure',
            )
        ),
    )
    future = SimpleNamespace(result=lambda: wrapped)
    OperatorConsoleNode._on_task_result(node, future)
    assert node.active_task.status == 'FAILED'
    assert node.active_goal_handle is None


def test_submit_returns_terminal_snapshot_when_result_callback_is_immediate():
    published = []
    server_task_id = '00112233445566778899aabbccddeeff'

    class ImmediateResult:
        def result(self):
            return SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED,
                result=SimpleNamespace(
                    error=SimpleNamespace(
                        code=CapabilityError.NONE, message='complete'
                    )
                ),
            )

        def add_done_callback(self, callback):
            callback(self)

    handle = SimpleNamespace(
        accepted=True,
        goal_id=SimpleNamespace(uuid=list(bytes.fromhex(server_task_id))),
        get_result_async=lambda: ImmediateResult(),
    )

    async def exercise():
        acceptance = asyncio.get_running_loop().create_future()
        acceptance.set_result(handle)
        client = SimpleNamespace(
            server_is_ready=lambda: True,
            send_goal_async=lambda _goal, feedback_callback: acceptance,
        )
        node = _task_test_node(client, published)
        goal = SimpleNamespace(
            object_id='ball',
            source_place='table',
            recipient_id='nearest_person',
            grasp_backend='act',
            dry_run=False,
        )
        result = await OperatorConsoleNode.submit_task(node, goal)
        assert result['status'] == 'SUCCEEDED'
        assert node.active_task.status == 'SUCCEEDED'
        assert [payload['status'] for kind, payload in published if kind == 'task'] == [
            'ACCEPTING', 'RUNNING', 'SUCCEEDED',
        ]

    asyncio.run(exercise())


def _task_test_node(task_client, published):
    node = object.__new__(OperatorConsoleNode)
    node.manual_control = ManualControlCoordinator()
    node.drive_stop_latched = False
    node.manual_action_active = lambda: False
    node.teleop_active = lambda: False
    node.task_client = task_client
    node._task_lock = threading.Lock()
    node._teleop_publisher = None
    node._teleop_was_active = False
    node._observation_lock = threading.Lock()
    node._perception_state = None
    node.active_task = None
    node.external_task = None
    node.active_goal_handle = None
    node.active_server_task_id = None
    node._owned_task_ids = set()
    node._task_cancel_requested = threading.Event()
    node._stage_started_monotonic = 0.0
    node._external_stage_started_monotonic = 0.0
    node.history = SimpleNamespace(
        save=lambda _record: None,
        recent=lambda: [],
    )
    node.events = SimpleNamespace(
        publish=lambda kind, payload: published.append((kind, payload))
    )
    return node


def _failed_future(error: Exception):
    future = asyncio.get_running_loop().create_future()
    future.set_exception(error)
    return future


def test_manual_nonterminal_status_keeps_reservation_fail_closed():
    class ServiceClient:
        requests = []

        @staticmethod
        def service_is_ready():
            return True

        def call_async(self, request):
            self.requests.append(bool(request.data))
            future = asyncio.get_running_loop().create_future()
            future.set_result(SimpleNamespace(success=True, message='ok'))
            return future

    class GoalHandle:
        accepted = True

        @staticmethod
        def get_result_async():
            future = asyncio.get_running_loop().create_future()
            future.set_result(SimpleNamespace(
                result=SimpleNamespace(
                    error=SimpleNamespace(code=0, message='')
                ),
                status=GoalStatus.STATUS_UNKNOWN,
            ))
            return future

    class ActionClient:
        @staticmethod
        def server_is_ready():
            return True

        @staticmethod
        def send_goal_async(_goal):
            future = asyncio.get_running_loop().create_future()
            future.set_result(GoalHandle())
            return future

    async def exercise():
        reservation = ServiceClient()
        node = _manual_test_node(reservation)
        app = ConsoleApplication(node)
        with pytest.raises(web.HTTPServiceUnavailable) as captured:
            await app._run_operator_action(
                ActionClient(), object(), 'ManualAction', 1.0
            )
        assert 'non-terminal' in captured.value.text
        assert reservation.requests == [True]
        assert node.manual_control.action_active() is True

    asyncio.run(exercise())


def test_manual_cancel_ack_failure_still_drains_terminal_result():
    class ServiceClient:
        def __init__(self):
            self.requests = []

        @staticmethod
        def service_is_ready():
            return True

        def call_async(self, request):
            self.requests.append(bool(request.data))
            future = asyncio.get_running_loop().create_future()
            future.set_result(SimpleNamespace(success=True, message='ok'))
            return future

    class GoalHandle:
        accepted = True

        def __init__(self):
            self.result_future = asyncio.get_running_loop().create_future()

        def get_result_async(self):
            return self.result_future

        def cancel_goal_async(self):
            cancel = _failed_future(RuntimeError('cancel ack lost'))
            asyncio.get_running_loop().call_later(
                0.01,
                self.result_future.set_result,
                SimpleNamespace(
                    result=SimpleNamespace(
                        error=SimpleNamespace(
                            code=CapabilityError.CANCELED,
                            message='canceled',
                        )
                    ),
                    status=GoalStatus.STATUS_CANCELED,
                ),
            )
            return cancel

    class ActionClient:
        def __init__(self, handle):
            self.handle = handle

        @staticmethod
        def server_is_ready():
            return True

        def send_goal_async(self, _goal):
            future = asyncio.get_running_loop().create_future()
            future.set_result(self.handle)
            return future

    async def exercise():
        reservation = ServiceClient()
        node = _manual_test_node(reservation)
        app = ConsoleApplication(node)
        with pytest.raises(web.HTTPGatewayTimeout):
            await app._run_operator_action(
                ActionClient(GoalHandle()), object(), 'ManualAction', 0.001
            )
        assert reservation.requests == [True, False]
        assert node.manual_control.action_active() is False
        assert node.logged_errors

    asyncio.run(exercise())


def _manual_test_node(reservation):
    logged_errors = []
    return SimpleNamespace(
        drive_stop_latched=False,
        task_active=lambda: False,
        teleop_active=lambda: False,
        stop_teleop=lambda: None,
        manual_control=ManualControlCoordinator(),
        manual_reservation_client=reservation,
        logged_errors=logged_errors,
        get_logger=lambda: SimpleNamespace(
            error=lambda message: logged_errors.append(message)
        ),
    )


def test_teleop_release_failure_keeps_local_owner_fail_closed():
    released = []
    errors = []
    token = object()
    manual_control = ManualControlCoordinator()
    assert manual_control.claim_teleop(token, task_active=False)
    node = SimpleNamespace(
        manual_control=manual_control,
        release_teleop=lambda owner: (
            released.append(owner), manual_control.release_teleop(owner)
        ),
        get_logger=lambda: SimpleNamespace(
            error=lambda message: errors.append(message)
        ),
    )

    async def exercise():
        app = ConsoleApplication(node)
        app._teleop_token = token

        async def fail_release(_enabled):
            raise web.HTTPServiceUnavailable(text='release unknown')

        app._request_manual_reservation = fail_release
        assert await app._release_teleop_session(token, reserved=True) is False
        assert app._teleop_token is token
        assert released == []
        assert errors

    asyncio.run(exercise())


def test_duplicate_teleop_cleanup_does_not_release_a_new_owner():
    class ServiceClient:
        def __init__(self):
            self.requests = []

        @staticmethod
        def service_is_ready():
            return True

        def call_async(self, request):
            self.requests.append(bool(request.data))
            future = asyncio.get_running_loop().create_future()
            future.set_result(SimpleNamespace(success=True, message='ok'))
            return future

    async def exercise():
        client = ServiceClient()
        released = []
        first = object()
        second = object()
        manual_control = ManualControlCoordinator()
        assert manual_control.claim_teleop(first, task_active=False)
        node = SimpleNamespace(
            manual_reservation_client=client,
            manual_control=manual_control,
            release_teleop=lambda owner: (
                released.append(owner), manual_control.release_teleop(owner)
            ),
            get_logger=lambda: SimpleNamespace(error=lambda _message: None),
        )
        app = ConsoleApplication(node)
        app._teleop_token = first
        assert await app._release_teleop_session(first, reserved=True)
        assert manual_control.claim_teleop(second, task_active=False)
        app._teleop_token = second
        assert await app._release_teleop_session(first, reserved=True)
        assert client.requests == [False]
        assert released == [first]
        assert app._teleop_token is second

    asyncio.run(exercise())


def test_operator_places_come_only_from_the_active_asset(tmp_path):
    places = tmp_path / 'places.yaml'
    places.write_text(
        'named_places:\n  frame_id: map\n  places:\n'
        '    table: {x: 1.0, y: 2.0, yaw: 0.5, nav_offset_m: 0.4}\n'
        '    sofa: {x: 3.0, y: 4.0, yaw: -0.5}\n',
        encoding='utf-8',
    )
    assert load_named_places(str(places)) == [
        {'id': 'table', 'x': 1.0, 'y': 2.0, 'yaw': 0.5, 'nav_offset_m': 0.4},
        {'id': 'sofa', 'x': 3.0, 'y': 4.0, 'yaw': -0.5, 'nav_offset_m': 0.0},
    ]


def test_missing_places_asset_exposes_no_free_form_fallback(tmp_path):
    assert load_named_places(str(tmp_path / 'missing.yaml')) == []


def test_static_assets_support_colcon_symlink_install(tmp_path, monkeypatch):
    web_dist = tmp_path / 'web_dist'
    assets = web_dist / 'assets'
    assets.mkdir(parents=True)
    bundle = tmp_path / 'build' / 'bundle.js'
    bundle.parent.mkdir()
    bundle.write_text('window.xlerobot = true')
    (assets / 'bundle.js').symlink_to(bundle)
    (web_dist / 'index.html').write_text('<div id="root"></div>')
    monkeypatch.setattr(
        ConsoleApplication, '_static_root', staticmethod(lambda: web_dist)
    )

    async def request_asset():
        application = ConsoleApplication(object()).build()
        async with TestClient(TestServer(application)) as client:
            response = await client.get('/assets/bundle.js')
            assert response.status == 200
            assert await response.text() == 'window.xlerobot = true'

    asyncio.run(request_asset())


def _handeye_sample_document():
    identity = np.eye(4).tolist()
    moved = np.eye(4)
    moved[:3, 3] = [0.3, -0.2, 0.1]
    return {
        'schema': 'xlerobot_transform_samples/v1',
        'model': 'fixed_camera_moving_target',
        'calibration_id': 'robot-1',
        'frames': {
            'base': 'base_link',
            'moving': 'right_arm_fixed_jaw_link',
            'camera': 'd455_color_optical_frame',
            'target': 'calibration_target',
        },
        'samples': [
            {
                'index': 0,
                'stamp_sec': 1.0,
                'moving_in_base': identity,
                'target_in_camera': identity,
                'quality': {'reprojection_rmse_px': 0.4},
            },
            {
                'index': 1,
                'stamp_sec': 2.0,
                'moving_in_base': moved.tolist(),
                'target_in_camera': identity,
                'quality': {'reprojection_rmse_px': 0.5},
            },
        ],
    }


def _head_sample_document():
    document = _handeye_sample_document()
    document['model'] = 'moving_camera_fixed_target'
    document['frames'].update({
        'moving': 'head_tilt_link',
        'camera': 'head_camera_link',
    })
    return document


def test_head_capture_restores_existing_sample_count(tmp_path):
    sample_file = tmp_path / 'calibration_work/head_camera/samples.yaml'
    sample_file.parent.mkdir(parents=True)
    sample_file.write_text(
        yaml.safe_dump(_head_sample_document(), sort_keys=False),
        encoding='utf-8',
    )
    values = {
        'enable_engineering_tools': True,
        'workspace': 'calibration',
        'calibration_workflow': 'head_camera',
    }
    node = SimpleNamespace(
        artifact_root=tmp_path,
        parameter=lambda name: values[name],
    )

    async def request_coverage():
        response = await ConsoleApplication(
            node
        ).calibration_sample_coverage(object())
        assert json.loads(response.text)['sample_count'] == 2

    asyncio.run(request_coverage())


def test_handeye_coverage_reads_only_valid_atomic_sample_facts(tmp_path):
    sample_file = (
        tmp_path / 'calibration_work' / 'right_handeye' / 'samples.yaml'
    )
    sample_file.parent.mkdir(parents=True)
    sample_file.write_text(
        yaml.safe_dump(_handeye_sample_document(), sort_keys=False),
        encoding='utf-8',
    )
    values = {
        'enable_engineering_tools': True,
        'workspace': 'calibration',
        'calibration_workflow': 'right_handeye',
    }
    node = SimpleNamespace(
        artifact_root=tmp_path,
        parameter=lambda name: values[name],
    )

    async def request_coverage():
        response = await ConsoleApplication(
            node
        ).calibration_sample_coverage(object())
        payload = json.loads(response.text)
        assert payload['sample_count'] == 2
        assert payload['spans_m'] == pytest.approx({
            'x': 0.3, 'y': 0.2, 'z': 0.1,
        })
        assert len(payload['points']) == 2

    asyncio.run(request_coverage())


def test_handeye_coverage_fails_closed_on_nonfinite_samples(tmp_path):
    sample_file = (
        tmp_path / 'calibration_work' / 'right_handeye' / 'samples.yaml'
    )
    sample_file.parent.mkdir(parents=True)
    document = _handeye_sample_document()
    document['samples'][1]['moving_in_base'][0][3] = float('nan')
    sample_file.write_text(
        yaml.safe_dump(document, sort_keys=False), encoding='utf-8'
    )
    values = {
        'enable_engineering_tools': True,
        'workspace': 'calibration',
        'calibration_workflow': 'right_handeye',
    }
    node = SimpleNamespace(
        artifact_root=tmp_path,
        parameter=lambda name: values[name],
    )

    async def request_coverage():
        with pytest.raises(web.HTTPConflict) as captured:
            await ConsoleApplication(
                node
            ).calibration_sample_coverage(object())
        assert 'invalid' in captured.value.text

    asyncio.run(request_coverage())


def test_capture_only_servo_finalize_returns_result_without_legacy_import():
    values = {
        'enable_engineering_tools': True,
        'workspace': 'calibration',
        'calibration_workflow': 'servo',
        'calibration_capture_only': True,
    }
    audits = []
    node = SimpleNamespace(
        parameter=lambda name: values[name],
        servo_calibration_client=object(),
        calibration_import_client=object(),
        history=SimpleNamespace(audit=lambda *args: audits.append(args)),
    )
    calls = []
    application = ConsoleApplication(node)

    async def call_service(client, message, _timeout):
        calls.append((client, message))
        response = ServoCalibrationStep.Response()
        response.error.code = CapabilityError.NONE
        response.phase = 'COMPLETE'
        response.result_uri = 'file:///capture/result.yaml'
        return response

    application._call_service = call_service

    class Request:
        @staticmethod
        async def json():
            return {
                'command': 'finalize', 'unit_id': 'robot-1',
                'group': 'right_arm', 'joint': 'gripper',
            }

    async def finalize():
        response = await application.servo_calibration_step(Request())
        payload = json.loads(response.text)
        assert payload['result_uri'] == 'file:///capture/result.yaml'
        assert 'draft_uri' not in payload

    asyncio.run(finalize())
    assert len(calls) == 1
    assert calls[0][0] is node.servo_calibration_client
    assert audits


def test_collection_finalize_uses_typed_coordinator_service_and_is_idempotent():
    class FinalizeClient:
        def __init__(self):
            self.requests = []

        @staticmethod
        def service_is_ready():
            return True

        def call_async(self, request):
            self.requests.append(request)
            response = FinalizeEpisode.Response()
            response.error.code = CapabilityError.NONE
            future = asyncio.get_running_loop().create_future()
            future.set_result(response)
            return future

    async def exercise():
        client = FinalizeClient()
        node = object.__new__(OperatorConsoleNode)
        node._collection_lock = threading.Lock()
        node.active_collection = {
            'dataset_id': 'dataset-001',
            'episode_id': 'episode-001',
            'status': 'RUNNING',
            'phase': 'RECORDING',
        }
        node.collection_goal_handle = object()
        node.collection_finalize_client = client

        snapshot = await OperatorConsoleNode.finalize_collection(
            node, 'dataset-001', 'episode-001'
        )
        assert snapshot['status'] == 'RUNNING'
        assert client.requests[0].dataset_id == 'dataset-001'
        assert client.requests[0].episode_id == 'episode-001'

        node.active_collection['status'] = 'SUCCEEDED'
        repeated = await OperatorConsoleNode.finalize_collection(
            node, 'dataset-001', 'episode-001'
        )
        assert repeated['status'] == 'SUCCEEDED'
        assert len(client.requests) == 1

        with pytest.raises(web.HTTPNotFound):
            await OperatorConsoleNode.finalize_collection(
                node, 'previous-dataset', 'episode-001'
            )

    asyncio.run(exercise())


def test_collection_abort_cancels_and_drains_to_incomplete_terminal():
    async def exercise():
        node = object.__new__(OperatorConsoleNode)
        node._collection_lock = threading.Lock()
        node.active_collection = {
            'dataset_id': 'dataset-001',
            'episode_id': 'episode-001',
            'status': 'RUNNING',
            'phase': 'RECORDING',
        }

        class GoalHandle:
            @staticmethod
            def cancel_goal_async():
                response = CancelGoal.Response()
                response.return_code = CancelGoal.Response.ERROR_NONE
                with node._collection_lock:
                    node.active_collection['status'] = 'CANCELED'
                    node.active_collection['message'] = (
                        'raw episode remains incomplete'
                    )
                future = asyncio.get_running_loop().create_future()
                future.set_result(response)
                return future

        node.collection_goal_handle = GoalHandle()
        snapshot = await OperatorConsoleNode.cancel_collection(
            node, 'dataset-001', 'episode-001'
        )
        assert snapshot['status'] == 'CANCELED'
        assert 'incomplete' in snapshot['message']

    asyncio.run(exercise())


def test_collection_abort_ack_loss_still_drains_terminal_result():
    async def exercise():
        node = object.__new__(OperatorConsoleNode)
        node._collection_lock = threading.Lock()
        node.active_collection = {
            'dataset_id': 'dataset-001',
            'episode_id': 'episode-001',
            'status': 'RUNNING',
            'phase': 'RECORDING',
        }

        class GoalHandle:
            @staticmethod
            def cancel_goal_async():
                future = asyncio.get_running_loop().create_future()
                future.set_exception(RuntimeError('cancel response lost'))

                def terminal():
                    with node._collection_lock:
                        node.active_collection['status'] = 'CANCELED'

                asyncio.get_running_loop().call_later(0.01, terminal)
                return future

        node.collection_goal_handle = GoalHandle()
        snapshot = await OperatorConsoleNode.cancel_collection(
            node, 'dataset-001', 'episode-001'
        )
        assert snapshot['status'] == 'CANCELED'

    asyncio.run(exercise())


def test_collection_result_requires_matching_terminal_transport_status():
    published = []
    node = object.__new__(OperatorConsoleNode)
    node._collection_lock = threading.Lock()
    node.active_collection = {
        'dataset_id': 'dataset-001',
        'episode_id': 'episode-001',
        'status': 'RUNNING',
    }
    node.collection_goal_handle = object()
    node.events = SimpleNamespace(
        publish=lambda kind, payload: published.append((kind, payload))
    )
    result = SimpleNamespace(
        error=SimpleNamespace(code=CapabilityError.NONE, message=''),
        episode_uri='file:///episode-001',
        quality_passed=True,
    )
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=result,
    ))
    OperatorConsoleNode._on_collection_result(node, future)
    assert node.active_collection['status'] == 'FAILED'
    assert node.collection_goal_handle is None
    assert published[-1][0] == 'collection'


@pytest.mark.parametrize('duration', ['not-a-number', float('nan'), 0, 120.1])
def test_collection_http_boundary_rejects_invalid_duration(duration):
    class Request:
        @staticmethod
        async def json():
            return {
                'template_id': 'manual',
                'dataset_id': 'act-pick',
                'episode_id': 'episode-001',
                'language_instruction': 'pick the object',
                'max_duration_s': duration,
            }

    async def exercise():
        values = {
            'enable_engineering_tools': True,
            'workspace': 'collection',
        }
        node = SimpleNamespace(parameter=lambda name: values[name])
        with pytest.raises(web.HTTPBadRequest):
            await ConsoleApplication(node).start_collection(Request())

    asyncio.run(exercise())


def test_review_uses_active_dataset_and_completed_real_episode():
    class ReviewClient:
        def __init__(self):
            self.requests = []

        @staticmethod
        def service_is_ready():
            return True

        def call_async(self, request):
            self.requests.append(request)
            response = ReviewEpisode.Response()
            response.error.code = CapabilityError.NONE
            response.review_uri = 'file:///reviews/episode-001.json'
            future = asyncio.get_running_loop().create_future()
            future.set_result(response)
            return future

    class Request:
        match_info = {
            'dataset_id': 'act-pick',
            'episode_id': 'episode-001',
        }

        @staticmethod
        async def json():
            return {
                'status': 'accepted',
                'failure_reason': '',
                'notes': 'usable',
            }

    async def exercise():
        client = ReviewClient()
        audits = []
        values = {
            'enable_engineering_tools': True,
            'workspace': 'collection',
        }
        node = SimpleNamespace(
            parameter=lambda name: values[name],
            collection_snapshot=lambda: {
                'dataset_id': 'act-pick',
                'episode_id': 'episode-001',
                'status': 'SUCCEEDED',
                'dry_run': False,
                'episode_uri': 'file:///raw/episode-001',
            },
            review_client=client,
            history=SimpleNamespace(audit=lambda *items: audits.append(items)),
        )
        response = await ConsoleApplication(node).review_episode(Request())
        assert json.loads(response.text) == {
            'review_uri': 'file:///reviews/episode-001.json',
        }
        assert len(client.requests) == 1
        message = client.requests[0]
        assert message.dataset_id == 'act-pick'
        assert message.episode_id == 'episode-001'
        assert message.status == 'accepted'
        assert audits[-1][3]['dataset_id'] == 'act-pick'

    asyncio.run(exercise())


@pytest.mark.parametrize(
    'active',
    [
        {
            'dataset_id': 'act-pick',
            'episode_id': 'episode-001',
            'status': 'SUCCEEDED',
            'dry_run': True,
            'episode_uri': 'dry-run://episode-001',
        },
        {
            'dataset_id': 'act-pick',
            'episode_id': 'episode-001',
            'status': 'SUCCEEDED',
            'dry_run': False,
            'episode_uri': '',
        },
    ],
)
def test_review_rejects_dry_run_or_unpublished_episode(active):
    class Request:
        match_info = {
            'dataset_id': 'act-pick',
            'episode_id': 'episode-001',
        }

        @staticmethod
        async def json():
            return {'status': 'accepted'}

    async def exercise():
        values = {
            'enable_engineering_tools': True,
            'workspace': 'collection',
        }
        node = SimpleNamespace(
            parameter=lambda name: values[name],
            collection_snapshot=lambda: active,
        )
        with pytest.raises(web.HTTPConflict):
            await ConsoleApplication(node).review_episode(Request())

    asyncio.run(exercise())
