"""Inspect launch composition and expand URDF; never execute a hardware node."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest


ROOT = Path(__file__).resolve().parents[1]


def load_launch():
    spec = importlib.util.spec_from_file_location(
        'head_only_platform_runtime_launch', ROOT / 'launch/platform_runtime.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(**overrides):
    result = LaunchContext()
    result.launch_configurations.update({
        'hardware_enabled': 'true',
        'right_bus': '/dev/test-right',
        'left_bus': '/dev/test-left',
        'geometry_file': str(ROOT.parent / 'xlerobot_description/config/two_wheel_reference_geometry.yaml'),
        'servo_calibration_file': str(ROOT.parent / 'xlerobot_description/config/two_wheel_reference_servos.yaml'),
        'controllers_file': str(ROOT / 'config/platform_controllers.yaml'),
        **overrides,
    })
    return result


def record_actions(module, monkeypatch):
    nodes = []
    events = []

    def node(**kwargs):
        result = SimpleNamespace(**kwargs)
        nodes.append(result)
        return result

    def event(**kwargs):
        result = SimpleNamespace(**kwargs)
        events.append(result)
        return result

    monkeypatch.setattr(module, 'Node', node)
    monkeypatch.setattr(module, 'OnProcessExit', event)
    monkeypatch.setattr(module, 'RegisterEventHandler', lambda handler: handler)
    return nodes, events


def test_head_only_is_opt_in_and_still_needs_explicit_hardware():
    module = load_launch()
    declared = {action.name: action for action in module.generate_launch_description().entities
                if isinstance(action, DeclareLaunchArgument)}
    assert ''.join(value.perform(LaunchContext()) for value in
                   declared['head_only_control'].default_value) == 'false'
    with pytest.raises(RuntimeError, match='no device was opened'):
        module._runtime_nodes(context(head_only_control='true', hardware_enabled='false'))


@pytest.mark.parametrize('startup', ['startup_ready', 'startup_head_only'])
def test_head_only_refuses_automatic_startup_pose(startup):
    with pytest.raises(RuntimeError, match='startup motions must be false'):
        load_launch()._runtime_nodes(context(head_only_control='true', **{startup: 'true'}))


def test_head_only_loads_only_head_and_state_controllers(monkeypatch):
    module = load_launch()
    nodes, events = record_actions(module, monkeypatch)
    launch_context = context(head_only_control='true')
    module._runtime_nodes(launch_context)
    assert sorted(node.executable for node in nodes) == [
        'position_ready_spawner', 'robot_state_publisher', 'ros2_control_node', 'spawner',
    ]
    assert [node.arguments for node in nodes if hasattr(node, 'arguments')] == [
        ['joint_state_broadcaster', '-c', '/controller_manager'],
        ['head_controller', '-c', '/controller_manager'],
    ]
    assert len(events) == 1
    assert events[0].on_exit(SimpleNamespace(returncode=1), launch_context) == []
    assert events[0].on_exit(SimpleNamespace(returncode=0), launch_context)[0].arguments[0] == 'head_controller'
    manager = next(node for node in nodes if node.executable == 'ros2_control_node')
    # Evaluating xacro here is hardware-free and proves the launch-selected wiring.
    xml = manager.parameters[0]['robot_description'].evaluate(launch_context)
    owner = ET.fromstring(xml).findall('ros2_control')
    assert len(owner) == 1
    assert owner[0].attrib['name'] == 'left_bus_system'
    assert owner[0].find('hardware/param[@name="port"]').text == '/dev/test-left'
    assert [joint.attrib['name'] for joint in owner[0].findall('joint')] == [
        'head_pan_joint', 'head_tilt_joint',
    ]


def test_default_full_runtime_still_loads_verified_controller_set(monkeypatch):
    module = load_launch()
    nodes, events = record_actions(module, monkeypatch)
    module._runtime_nodes(context())
    assert 'drive_safety_node' in [node.executable for node in nodes]
    assert [node.arguments[0] for node in nodes if hasattr(node, 'arguments')] == [
        'joint_state_broadcaster', 'base_controller', 'right_arm_controller', 'left_arm_controller',
    ]
    assert len(events) == 1
    manager = next(node for node in nodes if node.executable == 'ros2_control_node')
    xml = manager.parameters[0]['robot_description'].evaluate(context())
    assert [owner.attrib['name'] for owner in ET.fromstring(xml).findall('ros2_control')] == [
        'right_bus_system', 'left_bus_system',
    ]


def test_handeye_owns_exact_arm_and_head_without_wheels_or_left_arm(monkeypatch):
    module = load_launch()
    nodes, events = record_actions(module, monkeypatch)
    launch_context = context(right_handeye_control='true')
    module._runtime_nodes(launch_context)
    assert sorted(node.executable for node in nodes) == [
        'position_ready_spawner', 'robot_state_publisher', 'ros2_control_node', 'spawner']
    spawner = next(node for node in nodes if node.executable == 'position_ready_spawner')
    assert spawner.arguments == ['head_controller', 'right_arm_controller', 'right_gripper_controller',
                                 '-c', '/controller_manager']
    manager = next(node for node in nodes if node.executable == 'ros2_control_node')
    xml = manager.parameters[0]['robot_description'].evaluate(launch_context)
    owners = ET.fromstring(xml).findall('ros2_control')
    assert [[int(j.find('param[@name="servo_id"]').text) for j in owner.findall('joint')]
            for owner in owners] == [[1, 2, 3, 4, 5, 6], [7, 8]]
    assert owners[0].find('hardware/param[@name="arm_only_control"]').text == 'true'
    assert owners[1].find('hardware/param[@name="head_only_control"]').text == 'true'
    assert events[0].on_exit(SimpleNamespace(returncode=1), launch_context) == []


def test_handeye_rejects_startup_motion_and_conflicting_modes():
    with pytest.raises(RuntimeError, match='startup motions must be false'):
        load_launch()._runtime_nodes(context(right_handeye_control='true', startup_ready='true'))
    with pytest.raises(RuntimeError, match='only one'):
        load_launch()._runtime_nodes(context(right_handeye_control='true', head_only_control='true'))
