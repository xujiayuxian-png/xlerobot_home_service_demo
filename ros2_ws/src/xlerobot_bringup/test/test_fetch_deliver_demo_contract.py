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
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters
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


@pytest.mark.parametrize('voice,web', [('false', 'false'), ('true', 'false'),
                                      ('false', 'true'), ('true', 'true')])
def test_staged_startup_covers_all_components_without_executing_hardware(monkeypatch, voice, web):
    from xlerobot_bringup import startup_sequence
    module = _load_launch(Path(__file__).resolve().parents[1] / 'launch/fetch_deliver_demo.launch.py')
    context = LaunchContext()
    context.launch_configurations.update({
        'hardware_enabled': 'true', 'x1_low_load': 'true', 'x1_staged_startup': 'true',
        'map': '/tmp/not-opened-map.yaml', 'places_file': '/tmp/not-opened-places.yaml',
        'enable_voice': voice, 'enable_web': web})
    captured = []
    def capture(phases, factory):
        captured.extend(phases)
        # Validate parameter serialization as launch will do, without executing a node.
        probe = factory('sensors', startup_sequence.REQUIREMENTS['sensors'])
        params = evaluate_parameters(context, probe._Node__parameters)
        assert isinstance(params[0]['stage_spec'], str)
        return ['constructed-only']
    monkeypatch.setattr(startup_sequence, 'chain_phases', capture)
    assert module._runtime(context) == ['constructed-only']
    assert [name for name, _ in captured] == [name for name, _ in startup_sequence.PHASE_KEYS]
    assert all(actions for _, actions in captured)
    assert not any('planner' in name or 'navigation: Nav2' in name
                   for name in startup_sequence.REQUIREMENTS['navigation_moveit']['diagnostics'])
    with pytest.raises(ValueError, match='phase missing'):
        startup_sequence.staged_demo_actions([ExecuteProcess(cmd=['true'])], context)
    context.launch_configurations['x1_low_load'] = 'false'
    with pytest.raises(ValueError, match='requires x1_low_load'):
        module._runtime(context)
    context.launch_configurations['hardware_enabled'] = 'false'
    with pytest.raises(RuntimeError, match='no device was opened'):
        module._runtime(context)


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


@pytest.mark.parametrize('local_npu', [False, True])
def test_x1_profile_propagates_typed_parameters_and_selects_fixed_audio(monkeypatch, tmp_path, local_npu):
    module = _load_launch(Path(__file__).resolve().parents[1] / 'launch/fetch_deliver_demo.launch.py')
    context = LaunchContext()
    context.launch_configurations.update({
        'hardware_enabled': 'true', 'x1_low_load': 'true', 'x1_act_wrist_only': 'true',
        'x1_asr_threads': '2', 'map': '/tmp/map.yaml', 'places_file': '/tmp/places.yaml',
        'enable_voice': 'true', 'enable_web': 'false',
        'vlm_backend': 'npu' if local_npu else 'lmstudio',
        'vlm_model': 'qwen2.5-vl-3b-instruct-672x672-qnn2.36-w4a16-qcs8550' if local_npu else 'qwen/qwen3-vl-4b',
        'asr_backend': 'npu' if local_npu else 'cpu',
        'person_backend': 'npu' if local_npu else 'cpu',
    })
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = []
    def capture_node(**kwargs):
        nodes.append(kwargs)
        return Node(**kwargs)
    monkeypatch.setattr(module, 'Node', capture_node)
    actions = module._runtime(context)  # Construct only: never execute motor actions.
    assert not any(isinstance(action, IncludeLaunchDescription) and
        'maintenance_presets.launch.py' in str(action.launch_description_source.location)
        for action in actions)
    assert {'robot_state_gateway', 'wrist_camera_native'} <= {
        node['executable'] for node in nodes}
    assert not context.launch_configurations.get('global_params')
    voice = next(node for node in nodes if node['executable'] == 'voice_assistant')
    parameters = evaluate_parameters(context, normalize_parameters(voice['parameters']))
    args = ['--ros-args', '-r', '__node:=voice_assistant']
    for index, item in enumerate(parameters):
        if isinstance(item, dict):
            path = tmp_path / f'params-{index}.yaml'
            path.write_text(yaml.safe_dump({'/**': {'ros__parameters': item}}))
        else:
            path = item
        args.extend(['--params-file', str(path)])
    # Exercise the real ROS argument parser, but only create a generic node:
    # no audio, model, controller or motor device is opened.
    import rclpy
    from rclpy.node import Node as RosNode
    ros_context = rclpy.context.Context()
    rclpy.init(args=args, context=ros_context)
    node = RosNode('voice_assistant', context=ros_context,
                   automatically_declare_parameters_from_overrides=True)
    try:
        for name, value in {'audio_enabled': True, 'intent_backend_enabled': True,
                            'task_dry_run': False, 'speech_enabled': True,
                            'x1_low_load': True, 'x1_asr_threads': 2}.items():
            assert node.get_parameter(name).value == value
        assert node.get_parameter('vlm_backend').value == context.launch_configurations['vlm_backend']
        assert node.get_parameter('lmstudio_model').value == context.launch_configurations['vlm_model']
        assert node.get_parameter('asr_backend').value == context.launch_configurations['asr_backend']
    finally:
        node.destroy_node()
        ros_context.shutdown()
    speech = _include_arguments(actions, 'speak_text.launch.py')
    assert speech['config_file'].perform(context).endswith('/config/speak_fixed.yaml')
    assert speech['audio_predecode_pcm'] == 'false'
    assert speech['audio_player_device'] == ''
    for filename in ('detect_object.launch.py', 'verify_grasp.launch.py'):
        arguments = _include_arguments(actions, filename)
        assert arguments['vlm_backend'].perform(context) == context.launch_configurations['vlm_backend']
        assert arguments['vlm_model'].perform(context) == context.launch_configurations['vlm_model']


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


def test_humble_base_spawner_uses_manager_odom_remap(monkeypatch):
    module = _load_launch(Path(__file__).resolve().parents[1] /
                         'launch' / 'platform_runtime.launch.py')
    monkeypatch.setenv('ROS_DISTRO', 'humble')
    created = []
    original = module.Node

    def record_node(**kwargs):
        created.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, 'Node', record_node)
    context = LaunchContext()
    context.launch_configurations['hardware_enabled'] = 'true'
    module._runtime_nodes(context)
    base = next(item for item in created
                if item.get('arguments', [None])[0] == 'base_controller')
    assert '--controller-ros-args' not in base['arguments']
    manager = next(item for item in created
                   if item['executable'] == 'ros2_control_node')
    assert ('/base_controller/odom', '/odom') in manager['remappings']


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
    for serial in ('head-serial', '201523062481', ''):
        context.launch_configurations['d455_serial'] = serial
        value = perform_substitutions(
            context, normalize_to_list_of_substitutions(realsense['serial_no'])
        )
        assert yaml.safe_load(value) == serial
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
