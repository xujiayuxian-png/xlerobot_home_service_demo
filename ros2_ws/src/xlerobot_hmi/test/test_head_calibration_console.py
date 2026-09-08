"""Hardware-free tests of the thin head-calibration HTTP / ROS adapter."""

import asyncio
from concurrent.futures import Future
import json
import threading
import time
from types import SimpleNamespace

from action_msgs.srv import CancelGoal
from aiohttp import web
import pytest
from rclpy.time import Time
from xlerobot_hmi.manual_control import ManualControlCoordinator
from xlerobot_hmi.operator_console import ConsoleApplication, OperatorConsoleNode
from xlerobot_interfaces.msg import CalibrationTargetObservation, HeadCalibrationStatus


class Request:
    def __init__(self, **payload):
        self.payload = payload

    async def json(self):
        return self.payload


def ready_state():
    return dict(available=True, state_fresh=True, action_ready=True, running=False,
                unit_id='robot-1', phase='IDLE', pose_count=25, sample_count=0)


def console():
    values = dict(enable_engineering_tools=True, workspace='calibration',
                  calibration_workflow='head_camera', unit_id='robot-1')
    state = ready_state()
    goals, audits = [], []
    result_future = Future()
    accepted = Future()
    accepted.set_result(SimpleNamespace(accepted=True, get_result_async=lambda: result_future))

    def send(goal):
        goals.append(goal)
        return accepted

    node = SimpleNamespace(
        parameter=lambda name: values[name], head_calibration_snapshot=lambda: dict(state),
        head_calibration_client=SimpleNamespace(server_is_ready=lambda: True, send_goal_async=send),
        head_calibration_cancel_client=object(), calibration_pose_client=object(),
        calibration_capture_client=object(), manual_control=ManualControlCoordinator(),
        history=SimpleNamespace(audit=lambda *args: audits.append(args)),
    )
    return ConsoleApplication(node), values, state, goals, audits, result_future


def test_status_get_is_observational_and_start_returns_on_acceptance_not_completion():
    app, _, _, goals, audits, result = console()

    async def run():
        for _ in range(3):
            response = await app.head_calibration_status(Request())
            assert json.loads(response.text)['request_inflight'] is False
        assert not goals and not audits
        response = await app.start_head_calibration(Request(unit_id='robot-1', hardware_confirmed=True))
        assert response.status == 202 and not result.done()
        assert goals[0].unit_id == 'robot-1'
        assert goals[0].workflow_id == 'head_camera'
        assert goals[0].automatic is True and goals[0].dry_run is False
        assert app._head_auto_inflight
        with pytest.raises(web.HTTPConflict):
            await app.start_head_calibration(Request(unit_id='robot-1', hardware_confirmed=True))
        result.set_result(object())
        assert not app._head_auto_inflight

    asyncio.run(run())


@pytest.mark.parametrize('reason', ['unconfirmed', 'truthy', 'unit', 'stale', 'unavailable',
                                    'status_unit', 'running', 'manual_busy', 'workspace'])
def test_start_rejects_before_sending_motion(reason):
    app, values, state, goals, _, _ = console()
    payload = dict(unit_id='robot-1', hardware_confirmed=True)
    if reason == 'unconfirmed':
        payload.pop('hardware_confirmed')
    elif reason == 'truthy':
        payload['hardware_confirmed'] = 'true'
    elif reason == 'unit':
        payload['unit_id'] = 'another-unit'
    elif reason == 'stale':
        state['state_fresh'] = False
    elif reason == 'unavailable':
        state['action_ready'] = False
    elif reason == 'status_unit':
        state['unit_id'] = 'another-unit'
    elif reason == 'running':
        state['running'] = True
    elif reason == 'manual_busy':
        app.node.manual_control.begin_action()
    else:
        values['calibration_workflow'] = 'servo'

    async def run():
        with pytest.raises(web.HTTPException):
            await app.start_head_calibration(Request(**payload))
        assert not goals

    asyncio.run(run())


def test_pause_cancels_ros_action_without_discarding_or_pretending_motion_stopped():
    app, _, state, _, _, result = console()
    state['running'] = True
    app._head_auto_inflight = True
    calls = []

    async def call(client, request, timeout):
        calls.append(request)
        assert client is app.node.head_calibration_cancel_client
        return CancelGoal.Response(return_code=CancelGoal.Response.ERROR_NONE)

    app._call_service = call

    async def run():
        assert (await app.pause_head_calibration(Request())).status == 202
        assert list(calls[0].goal_info.goal_id.uuid) == [0] * 16
        assert app._head_auto_inflight and not result.done()

    asyncio.run(run())


@pytest.mark.parametrize('operation', ['capture', 'pose'])
@pytest.mark.parametrize('guard', ['running', 'accepting', 'stale'])
def test_manual_capture_and_pose_block_during_auto_or_stale(operation, guard):
    app, _, state, _, _, _ = console()
    if guard == 'running':
        state['running'] = True
    elif guard == 'accepting':
        app._head_auto_inflight = True
    else:
        state['state_fresh'] = False

    async def run():
        method = app.capture_calibration_sample if operation == 'capture' else app.move_calibration_pose
        with pytest.raises(web.HTTPException):
            await method(Request(unit_id='robot-1', pose_index=0))

    asyncio.run(run())


def test_manual_pose_count_comes_from_status_not_old_thirteen_pose_constant():
    app, _, _, _, _, _ = console()
    poses = []

    async def move(client, goal, label, timeout):
        poses.append(goal.pose_index)
        return SimpleNamespace(pose_name='head_25', pose_count=25,
                               error=SimpleNamespace(message='reached'))

    app._run_calibration_action = move

    async def run():
        assert (await app.move_calibration_pose(Request(pose_index=24))).status == 200
        with pytest.raises(web.HTTPBadRequest):
            await app.move_calibration_pose(Request(pose_index=25))
        assert poses == [24]

    asyncio.run(run())


def test_head_samples_are_owned_by_auto_even_while_idle():
    app, _, _, goals, _, _ = console()

    async def run():
        with pytest.raises(web.HTTPConflict) as error:
            await app.capture_calibration_sample(Request(unit_id='robot-1'))
        assert 'managed by automatic' in error.value.text
        assert not goals and not app.node.manual_control.action_active()

    asyncio.run(run())


def test_rejected_goal_does_not_leave_false_busy_guard():
    app, _, _, _, _, _ = console()
    rejected = Future()
    rejected.set_result(SimpleNamespace(accepted=False))
    app.node.head_calibration_client.send_goal_async = lambda _: rejected

    async def run():
        with pytest.raises(web.HTTPConflict):
            await app.start_head_calibration(Request(unit_id='robot-1', hardware_confirmed=True))
        assert not app._head_auto_inflight

    asyncio.run(run())


def test_acceptance_timeout_keeps_guard_and_tracks_eventual_completion(monkeypatch):
    app, _, _, _, _, result = console()
    accepted = Future()
    app.node.head_calibration_client.send_goal_async = lambda _: accepted

    async def timeout(*_args):
        raise web.HTTPGatewayTimeout()

    monkeypatch.setattr('xlerobot_hmi.operator_console._await_rclpy_future', timeout)

    async def run():
        with pytest.raises(web.HTTPGatewayTimeout):
            await app.start_head_calibration(Request(unit_id='robot-1', hardware_confirmed=True))
        assert app._head_auto_inflight
        accepted.set_result(SimpleNamespace(accepted=True, get_result_async=lambda: result))
        assert app._head_auto_inflight
        result.set_result(object())
        assert not app._head_auto_inflight

    asyncio.run(run())


def test_typed_state_snapshot_freshness_and_finite_json(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    node = SimpleNamespace(
        _head_calibration_lock=threading.Lock(), _head_calibration_state=None,
        _head_calibration_target=None,
        get_clock=lambda: SimpleNamespace(now=lambda: Time.from_msg(Time(seconds=100).to_msg())),
        head_calibration_client=SimpleNamespace(server_is_ready=lambda: True),
    )
    empty = OperatorConsoleNode.head_calibration_snapshot(node)
    assert not empty['available'] and not empty['state_fresh']
    message = HeadCalibrationStatus(unit_id='robot-1', phase='WAITING', pose_index=3,
                                    pose_count=25, pose_states=['pending'] * 25,
                                    metric_names=['bad'], metric_values=[float('nan')])
    OperatorConsoleNode._on_head_calibration_status(node, message)
    target = CalibrationTargetObservation(accepted=True, tag_count=16, reprojection_rmse_px=.3)
    target.header.stamp = Time(seconds=99.9).to_msg()
    OperatorConsoleNode._on_calibration_target(node, target)
    current = OperatorConsoleNode.head_calibration_snapshot(node)
    assert current['state_fresh'] and current['target']['accepted']
    assert current['metrics']['bad'] is None
    json.dumps(current, allow_nan=False)
    clock[0] = 100.6
    stale_target = OperatorConsoleNode.head_calibration_snapshot(node)
    assert stale_target['state_fresh'] and not stale_target['target']['accepted']
    clock[0] = 104
    assert not OperatorConsoleNode.head_calibration_snapshot(node)['state_fresh']
