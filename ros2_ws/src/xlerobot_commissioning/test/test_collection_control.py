"""Lifecycle/pose admission tests, separate from orchestration test doubles."""
import threading
import time
from types import SimpleNamespace

import pytest
from controller_manager_msgs.srv import ListControllers, SwitchController
from std_msgs.msg import String

from xlerobot_commissioning.collection_node import CollectionNode, LEADER_JOINTS


def control_node():
    node = object.__new__(CollectionNode)
    node._state_lock = threading.Lock()
    node._session_lock = threading.Lock()
    node._cleanup_blocked = False
    node._joint_state_timeout_s = 0.5
    node._leader_controller_owned = False
    node._holding_leader = False
    node._states = {'leader': dict.fromkeys(LEADER_JOINTS, 0.0)}
    node._state_received_at = {'leader': time.monotonic()}
    node._leader_description(String(data='<robot>' + ''.join(
        f'<joint name="{name}"><limit lower="-1" upper="1"/></joint>'
        for name in LEADER_JOINTS) + '</robot>'))
    node.leader_controllers, node.follower_controllers, node.leader_switch = object(), object(), object()
    states = {'leader_arm_controller': 'inactive', 'leader_torque_controller': 'active'}
    events = []

    def service(client, request):
        if client is node.leader_switch:
            assert request.strictness == SwitchController.Request.STRICT
            assert request.timeout.sec == 2
            active = bool(request.activate_controllers)
            states['leader_arm_controller'] = 'active' if active else 'inactive'
            events.append('activate' if active else 'deactivate')
            return SimpleNamespace(ok=True)
        assert isinstance(request, ListControllers.Request)
        values = states if client is node.leader_controllers else {
            'right_arm_controller': 'active', 'right_gripper_controller': 'active',
            'head_controller': 'active'}
        return SimpleNamespace(controller=[SimpleNamespace(name=k, state=v) for k, v in values.items()])

    node._service = service
    node._set_torque = lambda value: events.append(f'torque:{value}')
    node._set_teleop = lambda value: events.append(f'teleop:{value}')
    return node, states, events


def test_two_preparations_reacquire_measured_state_without_retaining_ownership():
    node, states, events = control_node()
    for _ in range(2):
        node._check_controllers()
        node._acquire_leader()
        assert node._leader_controller_owned and node._holding_leader
        node._deactivate_leader()
        assert not node._leader_controller_owned
        node._holding_leader = False
        node._set_torque(False)
        assert states['leader_arm_controller'] == 'inactive'
    assert events == ['teleop:False', 'torque:False', 'activate', 'torque:True',
                      'deactivate', 'torque:False'] * 2


@pytest.mark.parametrize('state', ['unconfigured', 'active', 'missing'])
def test_unusable_controller_fails_before_torque(state):
    node, states, events = control_node()
    states['leader_arm_controller'] = state
    with pytest.raises(RuntimeError, match='must be inactive'):
        node._check_controllers()
    assert not events


@pytest.mark.parametrize('value', [1.02, float('nan'), float('inf'), None])
def test_passive_invalid_observation_blocks_acquisition_without_any_command(value):
    node, _, events = control_node()
    node._states['leader'][LEADER_JOINTS[0]] = value
    with pytest.raises(RuntimeError, match='leader_shoulder_pan measured'):
        node._acquire_leader()
    assert not events


def test_target_outside_leader_range_fails_without_command():
    node, _, events = control_node()
    with pytest.raises(RuntimeError, match='target=2.0'):
        node._validate_leader_positions([2.0] + [0.0] * 5)
    assert not events


def test_missing_description_fails_closed():
    node, _, events = control_node()
    node._leader_description(String(data='<robot/>'))
    with pytest.raises(RuntimeError, match='physical limits are unavailable'):
        node._acquire_leader()
    assert not events


def test_lost_activation_response_is_owned_for_cleanup():
    node, _, events = control_node()
    service = node._service

    def lost(client, request):
        response = service(client, request)
        if client is node.leader_switch and request.activate_controllers:
            raise TimeoutError('switch reply lost')
        return response

    node._service = lost
    with pytest.raises(TimeoutError):
        node._acquire_leader()
    assert node._leader_controller_owned
    assert not node._release_controls(teleop_enabled=False, leader_torque_enabled=True)
    assert events[-2:] == ['torque:False', 'deactivate']


def test_cleanup_reports_failed_deactivation_and_blocks_next_attempt():
    node, _, _ = control_node()
    node._acquire_leader()
    service = node._service
    node._service = lambda client, request: (
        SimpleNamespace(ok=False, message='controller unavailable')
        if client is node.leader_switch else service(client, request))
    errors = node._release_controls(teleop_enabled=False, leader_torque_enabled=True)
    assert 'controller unavailable' in errors[0]
    assert node._cleanup_blocked
