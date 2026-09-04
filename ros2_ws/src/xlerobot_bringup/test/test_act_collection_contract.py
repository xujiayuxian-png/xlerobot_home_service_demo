import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
import pytest


ROOT = Path(__file__).parents[1]


def _load_launch(path):
    spec = importlib.util.spec_from_file_location('act_collection_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _include_arguments(actions, filename):
    matches = [
        action
        for action in actions
        if isinstance(action, IncludeLaunchDescription)
        and filename in str(action.launch_description_source.location)
    ]
    assert len(matches) == 1
    return dict(matches[0].launch_arguments)


def test_collection_profile_isolated_leader_and_no_act_executor_or_imu():
    launch_path = ROOT / 'launch/act_collection.launch.py'
    collection = launch_path.read_text()
    leader = (ROOT / 'launch/leader_runtime.launch.py').read_text()
    assert "'workspace': 'collection'" in collection
    assert "'leader_runtime.launch.py'" in collection
    assert "'/leader/controller_manager'" in leader
    assert 'robot_state_publisher' not in leader
    assert 'streaming_executor' not in collection
    assert 'act_policy_adapter' not in collection
    assert 'xlerobot_mock' not in collection
    assert 'technician_pin' not in collection
    assert "DeclareLaunchArgument('mode'" not in collection
    assert 'imu' not in collection.lower()
    for argument in ('geometry_file', 'servo_calibration_file', 'controllers_file'):
        assert f"DeclareLaunchArgument('{argument}'" in collection

    context = LaunchContext()
    context.launch_configurations.update({
        'hardware_enabled': 'true',
        'right_bus': '/dev/test-right',
        'left_bus': '/dev/test-left',
        'lidar_port': '/dev/test-lidar',
        'd455_serial': 'head-serial',
        'control_enable_lease_s': '1.4',
        'grasp_alignment_file': '/tmp/grasp-alignment.yaml',
    })
    actions = _load_launch(launch_path).runtime(context)
    platform = _include_arguments(actions, 'platform_runtime.launch.py')
    leader_runtime = _include_arguments(actions, 'leader_runtime.launch.py')
    sensors = _include_arguments(actions, 'sensors.launch.py')
    grasp = _include_arguments(actions, 'grasp_object.launch.py')
    assert platform['hardware_enabled'] == 'true'
    assert leader_runtime['hardware_enabled'] == 'true'
    assert platform['right_bus'].perform(context) == '/dev/test-right'
    assert platform['left_bus'].perform(context) == '/dev/test-left'
    assert sensors['lidar_port'].perform(context) == '/dev/test-lidar'
    assert sensors['d455_serial'].perform(context) == 'head-serial'
    assert (
        grasp['grasp_alignment_file'].perform(context)
        == '/tmp/grasp-alignment.yaml'
    )
    assert (
        leader_runtime['control_enable_lease_s'].perform(context) == '1.4'
    )
    source = launch_path.read_text(encoding='utf-8')
    assert "'unit_id': LaunchConfiguration('unit_id')" in source
    assert "'default_dataset_id': LaunchConfiguration('dataset_id')" in source
    assert "DeclareLaunchArgument(\n            'dataset_id'" in source
    assert (
        "'calibration_version': LaunchConfiguration("
        in source
    )


def test_collection_fails_closed_without_explicit_hardware_consent():
    module = _load_launch(ROOT / 'launch/act_collection.launch.py')
    context = LaunchContext()
    context.launch_configurations['hardware_enabled'] = 'false'

    with pytest.raises(RuntimeError, match='no device was opened'):
        module.runtime(context)


def test_collection_control_lease_is_shared_by_both_backends_and_coordinator():
    collection = (ROOT / 'launch/act_collection.launch.py').read_text()
    leader_runtime = (ROOT / 'launch/leader_runtime.launch.py').read_text()
    leader_config = (ROOT / 'config/leader_controllers.yaml').read_text()

    assert collection.count(
        "'control_enable_lease_s': LaunchConfiguration("
    ) == 2
    assert (
        "'enable_lease_s': LaunchConfiguration("
        in collection
    )
    assert (
        "'control_heartbeat_period_s': LaunchConfiguration("
        in collection
    )
    assert (
        "'recorder_first_sample_timeout_s': LaunchConfiguration("
        in collection
    )
    assert "'--controller', 'leader_torque_controller'" in leader_runtime
    assert "'--controller-ros-args'" in leader_runtime
    assert "LaunchConfiguration('control_enable_lease_s')" in leader_runtime
    assert '/leader/leader_torque_controller:' in leader_config
    assert 'enable_lease_s: 1.0' in leader_config


def test_collection_profile_records_verified_follower_next_state_action():
    profile = (Path(__file__).parents[2] /
               'xlerobot_dataset_tools/config/two_wheel_pick.yaml').read_text()
    assert 'topic: /joint_states' in profile
    assert 'kind: follower_next_state' in profile
    assert 'leader_joint_state' not in profile


def test_leader_uses_its_frozen_attachment_calibration_not_follower_values():
    description = (Path(__file__).parents[2] /
                   'xlerobot_description/urdf/leader_attachment.urdf.xacro').read_text()
    calibration = (Path(__file__).parents[2] /
                   'xlerobot_description/config/two_wheel_reference_leader_servos.yaml').read_text()
    assert 'two_wheel_reference_leader_servos.yaml' in description
    assert "calibration['joints']" in description
    assert (
        'name="leader_gripper" calibration="${right[\'gripper\']}" '
        'initial_value="0.05"' in description
    )
    assert 'attachment: right_leader' in calibration
    assert 'offset: 1305' in calibration
