"""Recovery uses existing controller ownership; no serial ports or motion."""
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock
import pytest

from std_srvs.srv import Trigger
from xlerobot_commissioning.collection_node import CollectionNode


def recovery_node():
    node = object.__new__(CollectionNode)
    node._session_lock = threading.Lock()
    node._goal_active = node._recovery_busy = node._cleanup_blocked = False
    node._torque_released = node._reset_ready = False
    node._begin_requested = threading.Event()
    node._finish_requested = threading.Event()
    node.leader_controllers = object()
    node.follower_controllers = object()
    node.follower_switch = object()
    node._set_teleop = Mock()
    node._set_torque = Mock()
    node._set_right_hardware = Mock()
    node._right_hardware_state = Mock(return_value=2)
    node._controller_states = Mock(return_value=dict.fromkeys([
        'leader_arm_controller', 'right_arm_controller', 'right_gripper_controller',
        'right_policy_controller', 'base_controller'], 'inactive'))
    return node


def test_release_then_reset_remains_passive_and_repeatable():
    node = recovery_node()
    for _ in range(2):
        assert node.release_torque(None, Trigger.Response()).success
        assert node._torque_released and not node._reset_ready
        assert node.reset(None, Trigger.Response()).success
        assert node._torque_released and node._reset_ready
    assert all(call.args == (False,) for call in node._set_right_hardware.call_args_list)
    assert all(call.args == (False,) for call in node._set_torque.call_args_list)
    assert all(call.args == (False,) for call in node._set_teleop.call_args_list)


def test_reset_requires_release_and_confirmed_cleanup():
    node = recovery_node()
    assert not node.reset(None, Trigger.Response()).success
    node.release_torque(None, Trigger.Response())
    node._cleanup_blocked = True
    assert not node.reset(None, Trigger.Response()).success
    assert node._cleanup_blocked and not node._reset_ready


def test_release_refuses_to_race_live_goal():
    node = recovery_node()
    node._goal_active = True
    assert not node.release_torque(None, Trigger.Response()).success
    node._set_right_hardware.assert_not_called()


def test_release_attempts_hardware_off_even_if_teleop_service_fails():
    node = recovery_node()
    node._set_teleop.side_effect = RuntimeError('missing teleop')
    result = node.release_torque(None, Trigger.Response())
    assert not result.success and 'missing teleop' in result.message
    node._set_right_hardware.assert_called_once_with(False)
    assert node._torque_released and not node._reset_ready


def test_release_deactivates_controllers_before_actual_torque_off():
    node = recovery_node()
    events = []
    node._controller_states = lambda client: (
        {'leader_arm_controller': 'inactive'} if client is node.leader_controllers
        else {'right_arm_controller': 'active', 'right_gripper_controller': 'active',
              'base_controller': 'active', 'right_policy_controller': 'inactive'})
    def service(client, request):
        assert client is node.follower_switch
        assert request.deactivate_controllers == [
            'right_arm_controller', 'right_gripper_controller', 'base_controller']
        events.append('controllers off')
        return SimpleNamespace(ok=True)
    node._service = service
    node._set_right_hardware = lambda active: events.append(('hardware', active))
    assert node.release_torque(None, Trigger.Response()).success
    assert events == ['controllers off', ('hardware', False)]


def test_reset_does_not_accept_unconfirmed_hardware_release():
    node = recovery_node()
    node.release_torque(None, Trigger.Response())
    node._right_hardware_state.return_value = 3
    assert not node.reset(None, Trigger.Response()).success
    assert not node._reset_ready


def test_only_explicit_new_start_rearms_follower_not_base():
    node = recovery_node()
    node.release_torque(None, Trigger.Response())
    node.reset(None, Trigger.Response())
    node._validate_leader_positions = Mock()
    node._state_lock = threading.Lock()
    node._joint_state_timeout_s = 0.5
    node._state_received_at = {'follower': time.monotonic()}
    def service(client, request):
        assert client is node.follower_switch
        assert request.activate_controllers == ['right_arm_controller', 'right_gripper_controller']
        return SimpleNamespace(ok=True)
    node._service = service
    node._rearm_after_reset()
    node._set_right_hardware.assert_called_with(True)
    assert not node._torque_released and not node._reset_ready


def test_stale_follower_cannot_be_rearmed():
    node = recovery_node()
    node._torque_released = node._reset_ready = True
    node._validate_leader_positions = Mock()
    node._state_lock = threading.Lock()
    node._joint_state_timeout_s = 0.5
    node._state_received_at = {'follower': 0.0}
    with pytest.raises(RuntimeError, match='stale'):
        node._rearm_after_reset()
    node._set_right_hardware.assert_not_called()
