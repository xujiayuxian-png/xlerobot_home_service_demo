import importlib.util
from pathlib import Path
import sys
import time
from types import SimpleNamespace

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.events.process import ProcessExited
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import pytest
import yaml


def _load_launch(path):
    spec = importlib.util.spec_from_file_location('fetch_deliver_demo_launch', path)
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


def test_complete_demo_has_one_canonical_platform_owner_and_required_inputs():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'fetch_deliver_demo.launch.py'
    )
    module = _load_launch(launch_path)
    source = launch_path.read_text(encoding='utf-8')

    description = module.generate_launch_description()
    assert isinstance(description, LaunchDescription)
    declared = {
        action.name
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert {
        'hardware_enabled', 'right_bus', 'left_bus', 'lidar_port', 'd455_serial'
    } <= declared
    assert {'kws_model_dir', 'whisper_model', 'tts_cache_dir'} <= declared
    assert "'platform_runtime.launch.py'" in source
    assert "'startup_ready': enabled" in source
    assert 'xlerobot_mock' not in source
    assert "DeclareLaunchArgument('mode'" not in source
    assert 'ros2_control_node' not in source
    assert "DeclareLaunchArgument('map'" in source
    assert "'places_file'" in source
    for package in (
        'xlerobot_navigation',
        'xlerobot_moveit_config',
        'xlerobot_manipulation',
        'xlerobot_policy',
        'xlerobot_perception',
        'xlerobot_task',
        'xlerobot_voice',
        'xlerobot_hmi',
    ):
        assert package in source
    assert "'enable_web'" in source
    assert "'places_file': LaunchConfiguration('places_file')" in source
    assert "'kws_model_dir': LaunchConfiguration('kws_model_dir')" in source
    assert "'whisper_model': LaunchConfiguration('whisper_model')" in source
    assert "'edge_cache_dir': LaunchConfiguration('tts_cache_dir')" in source
    assert "'speech_enabled': LaunchConfiguration('enable_tts')" in source
    assert "'pixel_format': 'MJPG'" in source
    assert "'maintenance_presets.launch.py'" in source
    assert "'scan_map_consistency.launch.py'" in source
    assert "'activate_navigation_after_localization': enabled" in source
    assert "'verify_grasp.launch.py'" in source
    assert "'execution_enabled': enabled" in source
    assert "'grasp_alignment_file': LaunchConfiguration(" in source
    assert "'manual_retreat_extra_timeout_s': 2.0" in source
    assert "'backup_action': '/backup'" not in source
    for argument in ('geometry_file', 'servo_calibration_file', 'controllers_file'):
        assert f"DeclareLaunchArgument('{argument}'" in source

    context = LaunchContext()
    context.launch_configurations.update({
        'hardware_enabled': 'true',
        'map': '/tmp/map.yaml',
        'places_file': '/tmp/places.yaml',
        'enable_voice': 'false',
        'enable_web': 'false',
        'right_bus': '/dev/test-right',
        'left_bus': '/dev/test-left',
        'lidar_port': '/dev/test-lidar',
        'd455_serial': 'head-serial',
    })
    actions = module._runtime(context)
    platform = _include_arguments(actions, 'platform_runtime.launch.py')
    sensors = _include_arguments(actions, 'sensors.launch.py')
    assert platform['hardware_enabled'] == 'true'
    assert platform['right_bus'].perform(context) == '/dev/test-right'
    assert platform['left_bus'].perform(context) == '/dev/test-left'
    assert sensors['lidar_port'].perform(context) == '/dev/test-lidar'
    assert sensors['d455_serial'].perform(context) == 'head-serial'


def test_complete_demo_fails_closed_without_explicit_hardware_consent():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'fetch_deliver_demo.launch.py'
    )
    context = LaunchContext()
    context.launch_configurations['hardware_enabled'] = 'false'

    with pytest.raises(RuntimeError, match='no device was opened'):
        _load_launch(launch_path)._runtime(context)


def test_operator_console_is_registered_as_a_fail_closed_critical_process():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'fetch_deliver_demo.launch.py'
    )
    module = _load_launch(launch_path)
    context = LaunchContext()
    context.launch_configurations.update({
        'enable_voice': 'false',
        'enable_web': 'true',
    })
    actions = []

    module._append_entrypoints(actions, context, True)

    consoles = [action for action in actions if isinstance(action, Node)]
    handlers = [
        action
        for action in actions
        if isinstance(action, RegisterEventHandler)
    ]
    assert len(consoles) == 1
    assert len(handlers) == 1
    assert actions.index(handlers[0]) < actions.index(consoles[0])

    exit_event = ProcessExited(
        action=consoles[0],
        name='xlerobot_operator_console',
        cmd=['operator_console'],
        cwd=None,
        env=None,
        pid=123,
        returncode=0,
    )
    assert handlers[0].event_handler.matches(exit_event)
    with pytest.raises(RuntimeError, match='requires an explicit operator restart'):
        handlers[0].event_handler.handle(exit_event, LaunchContext())

    assert (
        module._fail_closed_on_operator_console_exit(
            exit_event, SimpleNamespace(is_shutdown=True)
        )
        is None
    )


def test_critical_process_exit_fails_launch_and_stops_sibling_processes():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'fetch_deliver_demo.launch.py'
    )
    module = _load_launch(launch_path)
    launch_service = LaunchService(argv=[], noninteractive=True)
    launch_service.include_launch_description(
        LaunchDescription(
            [
                ExecuteProcess(
                    cmd=[
                        sys.executable,
                        '-c',
                        'import time; time.sleep(30)',
                    ],
                ),
                ExecuteProcess(
                    cmd=[sys.executable, '-c', 'pass'],
                    on_exit=module._fail_closed_on_operator_console_exit,
                ),
            ]
        )
    )

    started_at = time.monotonic()
    return_code = launch_service.run()

    assert return_code == 1
    assert time.monotonic() - started_at < 5.0


def test_leaf_hardware_launches_consume_profile_device_arguments():
    launch_root = Path(__file__).resolve().parents[1] / 'launch'
    platform_path = launch_root / 'platform_runtime.launch.py'
    platform_source = platform_path.read_text(encoding='utf-8')
    platform = _load_launch(platform_path).generate_launch_description()
    platform_arguments = {
        action.name
        for action in platform.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert {'hardware_enabled', 'right_bus', 'left_bus'} <= platform_arguments
    assert (
        "' right_bus_port:=', LaunchConfiguration('right_bus')"
        in platform_source
    )
    assert (
        "' left_bus_port:=', LaunchConfiguration('left_bus')"
        in platform_source
    )

    context = LaunchContext()
    context.launch_configurations['hardware_enabled'] = 'false'
    with pytest.raises(RuntimeError, match='no device was opened'):
        _load_launch(platform_path)._runtime_nodes(context)

    leader_path = launch_root / 'leader_runtime.launch.py'
    leader_module = _load_launch(leader_path)
    leader_arguments = {
        action.name
        for action in leader_module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert 'hardware_enabled' in leader_arguments
    with pytest.raises(RuntimeError, match='no device was opened'):
        leader_module.runtime(context)

    sensors_path = launch_root / 'sensors.launch.py'
    sensors = _load_launch(sensors_path).generate_launch_description()
    sensor_arguments = {
        action.name: action
        for action in sensors.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert {'lidar_port', 'd455_serial', 'd455_config_file'} <= sensor_arguments.keys()
    context = LaunchContext()
    context.launch_configurations.update({
        'lidar_port': '/dev/test-lidar',
        'd455_serial': 'head-serial',
        'd455_config_file': '/tmp/test-d455.yaml',
    })
    includes = [
        action
        for action in sensors.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    lidar = _include_arguments(includes, 'lidar.launch.py')
    realsense = _include_arguments(includes, 'rs_launch.py')
    assert isinstance(lidar['port_name'], LaunchConfiguration)
    assert lidar['port_name'].perform(context) == '/dev/test-lidar'
    assert isinstance(realsense['serial_no'], LaunchConfiguration)
    assert realsense['serial_no'].perform(context) == 'head-serial'
    assert isinstance(realsense['config_file'], LaunchConfiguration)
    assert realsense['config_file'].perform(context) == '/tmp/test-d455.yaml'
    empty_context = LaunchContext()
    sensor_arguments['d455_serial'].execute(empty_context)
    assert empty_context.launch_configurations['d455_serial'] == ''


def test_low_precision_position_controllers_do_not_abort_on_path_error():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'platform_controllers.yaml'
    )
    controllers = yaml.safe_load(config_path.read_text(encoding='utf-8'))

    for controller_name in ('right_arm_controller', 'head_controller'):
        parameters = controllers[controller_name]['ros__parameters']
        assert 'constraints' not in parameters


def test_startup_ready_pose_centers_the_head():
    config_path = (
        Path(__file__).resolve().parents[2]
        / 'xlerobot_manipulation'
        / 'config'
        / 'startup_ready.yaml'
    )
    parameters = yaml.safe_load(config_path.read_text(encoding='utf-8'))[
        'startup_ready_pose'
    ]['ros__parameters']

    assert parameters['head_joint_names'] == ['head_pan_joint', 'head_tilt_joint']
    assert parameters['head_ready_positions'] == [0.0, 0.0]
    assert parameters['head_action'] == (
        '/head_controller/follow_joint_trajectory'
    )
