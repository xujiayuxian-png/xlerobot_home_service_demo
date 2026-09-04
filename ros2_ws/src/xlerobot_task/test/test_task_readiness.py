from pathlib import Path
from types import SimpleNamespace
import threading
import time

from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.action import GoalResponse

from xlerobot_task.fetch_deliver_task_node import (
    FetchDeliverTaskNode,
    readiness_joint_state_valid,
)


class ReadyClient:
    def __init__(self, ready=True):
        self.ready = ready

    def server_is_ready(self):
        return self.ready


def bare_node():
    node = object.__new__(FetchDeliverTaskNode)
    node._lock = threading.Lock()
    node._goal_active = False
    node._manual_reserved = False
    node._blocked_reason = ''
    node._stop_latched = False
    node._stop_state_received = True
    node.joint_state_max_age_s = 0.5
    node.camera_max_age_s = 1.0
    node.diagnostic_max_age_s = 3.0
    node.speech_enabled = True
    now = time.monotonic()
    node._latest_joint_state_received_monotonic = now
    node._readiness_joint_state_received_monotonic = now
    node._required_diagnostics = {
        name: (DiagnosticStatus.OK, 'ready', now)
        for name in (
            'xlerobot/startup_ready',
            'xlerobot/drive_safety',
            'xlerobot/person_search',
        )
    }
    node._camera_received_monotonic = {
        'head_color': now,
        'head_depth': now,
        'head_camera_info': now,
        'wrist': now,
    }
    for name in (
        'localize_client',
        'navigate_client',
        'detect_client',
        'head_client',
        'grasp_client',
        'scan_client',
        'approach_client',
        'handover_client',
        'speak_client',
    ):
        setattr(node, name, ReadyClient())
    return node


def test_manual_reservation_and_task_activity_are_atomic():
    node = bare_node()
    response = SimpleNamespace(success=False, message='')

    node._set_manual_control(SimpleNamespace(data=True), response)

    assert response.success
    assert node._manual_reserved

    response = SimpleNamespace(success=True, message='')
    node._set_manual_control(SimpleNamespace(data=True), response)
    assert not response.success
    assert 'already reserved' in response.message

    request = SimpleNamespace(
        object_id='yellow_stick',
        source_place='table',
        recipient_id='nearest_person',
        grasp_backend='act',
        dry_run=True,
    )
    assert node.goal_callback(request) == GoalResponse.REJECT

    response = SimpleNamespace(success=False, message='')
    node._set_manual_control(SimpleNamespace(data=False), response)
    assert response.success
    node._goal_active = True
    response = SimpleNamespace(success=True, message='')
    node._set_manual_control(SimpleNamespace(data=True), response)
    assert not response.success
    assert 'task is active' in response.message


def test_live_readiness_requires_stop_state_startup_sensors_and_actions():
    node = bare_node()
    assert node._live_readiness() == (True, [])

    node._stop_latched = True
    ready, reasons = node._live_readiness()
    assert not ready
    assert 'base software stop is latched' in reasons

    node._stop_latched = False
    node.detect_client.ready = False
    ready, reasons = node._live_readiness()
    assert not ready
    assert any('detect_object' in reason for reason in reasons)

    node.detect_client.ready = True
    node._camera_received_monotonic['head_depth'] = time.monotonic() - 2.0
    ready, reasons = node._live_readiness()
    assert not ready
    assert any('head_depth camera is stale' in reason for reason in reasons)


def test_voice_disabled_does_not_require_speak_action_server():
    node = bare_node()
    node.speech_enabled = False
    node.speak_client.ready = False
    assert node._live_readiness() == (True, [])

    feedback = []
    node._feedback = lambda *args: feedback.append(args)
    node._call = lambda *args: (_ for _ in ()).throw(
        AssertionError('disabled speech must not call its action server')
    )
    node._run_speech_stage(object(), SimpleNamespace(dry_run=False), 1.0)
    assert feedback[0][2] == 'skipped'


def test_stop_latch_change_during_goal_acceptance_rejects_live_goal():
    node = bare_node()
    warnings = []
    node.get_logger = lambda: SimpleNamespace(warning=warnings.append)

    def readiness_then_stop():
        with node._lock:
            node._stop_latched = True
        return True, []

    node._live_readiness = readiness_then_stop
    request = SimpleNamespace(
        object_id='yellow_stick',
        source_place='table',
        recipient_id='nearest_person',
        grasp_backend='act',
        dry_run=False,
    )

    assert node.goal_callback(request) == GoalResponse.REJECT
    assert node._goal_active is False
    assert any('stop state changed' in item for item in warnings)


def test_camera_subscriptions_use_sensor_data_qos():
    source = (
        Path(__file__).parents[1]
        / 'xlerobot_task'
        / 'fetch_deliver_task_node.py'
    ).read_text(encoding='utf-8')
    assert source.count('qos_profile_sensor_data,') >= 4


def test_readiness_joint_state_requires_every_finite_demo_joint():
    names = [
        'right_arm_shoulder_pan',
        'right_arm_shoulder_lift',
        'right_arm_elbow_flex',
        'right_arm_wrist_flex',
        'right_arm_wrist_roll',
        'right_arm_gripper',
        'head_pan_joint',
        'head_tilt_joint',
    ]
    message = SimpleNamespace(name=names, position=[0.0] * len(names))
    assert readiness_joint_state_valid(message)
    message.position[-1] = float('nan')
    assert not readiness_joint_state_valid(message)
    assert not readiness_joint_state_valid(
        SimpleNamespace(name=[], position=[])
    )
