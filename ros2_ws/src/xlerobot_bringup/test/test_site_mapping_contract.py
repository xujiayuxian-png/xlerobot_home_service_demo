import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
import pytest


def _load_launch(path):
    spec = importlib.util.spec_from_file_location('site_mapping_launch', path)
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


def test_site_mapping_profile_has_isolated_tools_and_no_imu_dependency():
    launch_path = (
        Path(__file__).resolve().parents[1] / 'launch' / 'site_mapping.launch.py'
    )
    source = launch_path.read_text(encoding='utf-8')
    assert "'platform_runtime.launch.py'" in source
    assert "'sensors.launch.py'" in source
    assert "'slam_mapping.launch.py'" in source
    assert "'amcl_localization.launch.py'" in source
    assert "'nav2_navigation.launch.py'" in source
    assert "'auto_localizer.launch.py'" in source
    assert "'activate_navigation_after_localization': 'true'" in source
    assert "'named_navigation.launch.py'" in source
    assert "phase not in {'build', 'validate'}" in source
    assert "'mapping_phase': phase" in source
    assert "package='xlerobot_commissioning'" in source
    assert "'workspace': 'mapping'" in source
    assert "'enable_engineering_tools': True" in source
    assert "'enable_d455': 'false'" in source
    assert 'technician_pin' not in source
    assert "DeclareLaunchArgument('mode'" not in source
    assert 'imu' not in source.lower()
    for argument in ('geometry_file', 'servo_calibration_file', 'controllers_file'):
        assert f"DeclareLaunchArgument('{argument}'" in source

    context = LaunchContext()
    context.launch_configurations.update({
        'hardware_enabled': 'true',
        'phase': 'build',
        'artifact_root': '.xlerobot/artifacts',
        'site_id': 'test-site',
        'right_bus': '/dev/test-right',
        'left_bus': '/dev/test-left',
        'lidar_port': '/dev/test-lidar',
    })
    actions = _load_launch(launch_path).runtime(context)
    platform = _include_arguments(actions, 'platform_runtime.launch.py')
    sensors = _include_arguments(actions, 'sensors.launch.py')
    assert platform['hardware_enabled'] == 'true'
    assert platform['right_bus'].perform(context) == '/dev/test-right'
    assert platform['left_bus'].perform(context) == '/dev/test-left'
    assert sensors['lidar_port'].perform(context) == '/dev/test-lidar'


def test_site_mapping_fails_closed_without_explicit_hardware_consent():
    launch_path = (
        Path(__file__).resolve().parents[1] / 'launch' / 'site_mapping.launch.py'
    )
    context = LaunchContext()
    context.launch_configurations['hardware_enabled'] = 'false'

    with pytest.raises(RuntimeError, match='no device was opened'):
        _load_launch(launch_path).runtime(context)
