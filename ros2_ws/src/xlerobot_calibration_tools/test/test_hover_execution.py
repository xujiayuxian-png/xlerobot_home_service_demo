"""Local action protocol tests; no ROS node, graph or hardware is created."""
import asyncio
import time
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from xlerobot_calibration_tools.hover import JOINTS
from xlerobot_calibration_tools.hover_node import HoverNode


def future(value):
    result = asyncio.get_running_loop().create_future()
    result.set_result(value)
    return result


def node(enabled=True):
    result = SimpleNamespace(status=4, result=SimpleNamespace(error_code=0))
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: future(result),
                             cancel_goal_async=Mock(side_effect=lambda: future(None)))
    client = SimpleNamespace(server_is_ready=lambda: True,
                             send_goal_async=Mock(side_effect=lambda goal: future(handle)))
    row = {'joints': dict.fromkeys(JOINTS, 0.), 'camera_from_board': np.eye(4).tolist(),
           'scene_base_from_board': np.eye(4).tolist()}
    value = SimpleNamespace(
        check_cancel=lambda: None,
        get_parameter=lambda key: SimpleNamespace(value=enabled),
        plan={'id': 'one', 'start': [0.]*5, 'goal': [.1]*5,
              'base_from_board': np.eye(4).tolist(), 'hardware_executed': False,
              'scene_base_from_board': np.eye(4).tolist()},
        plan_time=time.monotonic(), observe=lambda: row, x=np.eye(4),
        client=client, motion_handle=None, motion_unconfirmed=False,
        joints=lambda: row['joints'])
    value.wait = MethodType(HoverNode.wait, value)
    return value, handle, result


def test_explicit_execution_uses_only_right_arm_and_smooth_12_second_trajectory():
    async def check():
        value, _, _ = node()
        await HoverNode.execute(value, 'one')
        goal = value.client.send_goal_async.call_args.args[0]
        assert goal.trajectory.joint_names == JOINTS
        assert len(goal.trajectory.points) == 31
        assert goal.trajectory.points[-1].time_from_start.sec == 12
        assert all(v == 0 for v in goal.trajectory.points[0].velocities)
        assert all(v == 0 for v in goal.trajectory.points[-1].velocities)
        assert value.plan['hardware_executed'] and not value.motion_unconfirmed
    asyncio.run(check())


@pytest.mark.parametrize('case', ['disabled', 'expired', 'wrong_id', 'moved', 'board_moved', 'scene_moved'])
def test_invalid_preview_never_sends_goal(case):
    async def check():
        value, _, _ = node(enabled=case != 'disabled')
        if case == 'expired':
            value.plan_time -= 61
        if case == 'moved':
            value.plan['start'][0] = .2
        if case == 'board_moved':
            value.plan['base_from_board'][0][3] = .05
        if case == 'scene_moved':
            value.plan['scene_base_from_board'][0][3] = .05
        with pytest.raises(ValueError):
            await HoverNode.execute(value, 'other' if case == 'wrong_id' else 'one')
        value.client.send_goal_async.assert_not_called()
    asyncio.run(check())


def test_controller_failure_is_not_a_successful_measurement_target():
    async def check():
        value, handle, result = node()
        result.status = 6
        with pytest.raises(ValueError, match='取消或失败'):
            await HoverNode.execute(value, 'one')
        assert not value.plan['hardware_executed']
        assert not value.motion_unconfirmed
        handle.cancel_goal_async.assert_called_once()
    asyncio.run(check())


def test_late_accepted_goal_is_canceled_and_owner_remains_unconfirmed():
    async def check():
        value, handle, _ = node()
        pending = asyncio.get_running_loop().create_future()
        value.client.send_goal_async = Mock(return_value=pending)
        async def timeout(*args):
            raise ValueError('timeout')
        value.wait = timeout
        with pytest.raises(ValueError, match='timeout'):
            await HoverNode.execute(value, 'one')
        assert value.motion_unconfirmed
        pending.set_result(handle)
        await asyncio.sleep(0)
        handle.cancel_goal_async.assert_called_once()
        assert not value.plan['hardware_executed']
    asyncio.run(check())
