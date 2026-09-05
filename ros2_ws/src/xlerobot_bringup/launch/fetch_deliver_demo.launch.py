"""One-command composition for the complete fetch-and-deliver demo."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', filename])
        ),
        launch_arguments=(arguments or {}).items(),
    )


def _fail_closed_on_operator_console_exit(event, context):
    if context.is_shutdown:
        return None
    raise RuntimeError(
        'critical operator console exited with code '
        f'{event.returncode}; the demo profile requires an explicit operator restart'
    )


def _append_entrypoints(actions, context, live):
    if LaunchConfiguration('enable_voice').perform(context) == 'true':
        actions.append(
            Node(
                package='xlerobot_voice',
                executable='voice_assistant',
                name='voice_assistant',
                output='screen',
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare('xlerobot_voice'),
                            'config',
                            'voice_assistant.yaml',
                        ]
                    ),
                    {
                        'audio_enabled': live,
                        'intent_backend_enabled': live,
                        'task_dry_run': not live,
                        'default_grasp_backend': LaunchConfiguration(
                            'default_grasp_backend'
                        ),
                        'kws_model_dir': LaunchConfiguration('kws_model_dir'),
                        'whisper_model': LaunchConfiguration('whisper_model'),
                        'speech_enabled': live,
                        'speech_dry_run': not live,
                        'lmstudio_url': LaunchConfiguration('vlm_base_url'),
                        'audio_input_device': LaunchConfiguration('audio_input_device'),
                    },
                ],
            )
        )
    if LaunchConfiguration('enable_web').perform(context) == 'true':
        operator_console = Node(
            package='xlerobot_hmi',
            executable='operator_console',
            name='xlerobot_operator_console',
            output='screen',
            parameters=[
                {
                    'bind_host': LaunchConfiguration('web_bind_host'),
                    'port': LaunchConfiguration('web_port'),
                    'artifact_root': LaunchConfiguration('artifact_root'),
                    'release_id': LaunchConfiguration('release_id'),
                    'unit_id': LaunchConfiguration('unit_id'),
                    'site_id': LaunchConfiguration('site_id'),
                    'places_file': LaunchConfiguration('places_file'),
                    'workspace': 'operator',
                    'enable_engineering_tools': False,
                }
            ],
        )
        actions.extend(
            [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=operator_console,
                        on_exit=_fail_closed_on_operator_console_exit,
                    )
                ),
                operator_console,
            ]
        )


def _runtime(context):
    if LaunchConfiguration('hardware_enabled').perform(context) != 'true':
        raise RuntimeError(
            'live demo requires hardware_enabled:=true; no device was opened'
        )
    live = True
    enabled = 'true'
    perception_mode = 'observe'
    map_file = LaunchConfiguration('map').perform(context)
    places_file = LaunchConfiguration('places_file').perform(context)
    if not map_file or not places_file:
        raise ValueError('map and places_file are required')

    actions = [
        _include(
            'xlerobot_bringup', 'platform_runtime.launch.py',
            {
                'startup_ready': enabled,
                'hardware_enabled': enabled,
                'geometry_file': LaunchConfiguration('geometry_file'),
                'servo_calibration_file': LaunchConfiguration('servo_calibration_file'),
                'controllers_file': LaunchConfiguration('controllers_file'),
                'right_bus': LaunchConfiguration('right_bus'),
                'left_bus': LaunchConfiguration('left_bus'),
            },
        ),
        _include(
            'xlerobot_bringup',
            'sensors.launch.py',
            {
                'enable_lidar': enabled,
                'enable_d455': enabled,
                'lidar_port': LaunchConfiguration('lidar_port'),
                'd455_serial': LaunchConfiguration('d455_serial'),
            },
        ),
    ]

    actions.extend(
        [
            _include(
                'xlerobot_navigation',
                'amcl_localization.launch.py',
                {'map': map_file},
            ),
            _include('xlerobot_navigation', 'nav2_navigation.launch.py'),
            _include('xlerobot_navigation', 'scan_map_consistency.launch.py'),
            _include(
                'xlerobot_navigation',
                'auto_localizer.launch.py',
                {
                    'execution_enabled': enabled,
                    'activate_navigation_after_localization': enabled,
                },
            ),
            _include(
                'xlerobot_navigation',
                'named_navigation.launch.py',
                {'execution_enabled': enabled, 'places_file': places_file},
            ),
            _include(
                'xlerobot_navigation',
                'approach_target.launch.py',
                {'execution_enabled': enabled},
            ),
            _include(
                'xlerobot_moveit_config',
                'move_group.launch.py',
                {
                    'launch_robot_state_publisher': 'false',
                    'geometry_file': LaunchConfiguration('geometry_file'),
                    'servo_calibration_file': LaunchConfiguration('servo_calibration_file'),
                },
            ),
            _include(
                'xlerobot_manipulation',
                'streaming_executor.launch.py',
                {'execution_enabled': enabled},
            ),
            _include(
                'xlerobot_manipulation',
                'maintenance_presets.launch.py',
                {'execution_enabled': enabled},
            ),
            Node(
                package='xlerobot_policy',
                executable='act_policy_adapter',
                output='screen',
                parameters=[
                    {
                        'backend_enabled': live,
                        'capture_only_mode': False,
                        'predict_url': LaunchConfiguration('act_predict_url'),
                    }
                ],
            ),
            _include(
                'xlerobot_perception',
                'detect_object.launch.py',
                {
                    'backend_enabled': enabled,
                    'dry_run_mode': perception_mode,
                    'vlm_base_url': LaunchConfiguration('vlm_base_url'),
                    'classical_base_url': LaunchConfiguration('classical_base_url'),
                    'grasp_alignment_file': LaunchConfiguration('grasp_alignment_file'),
                },
            ),
            _include(
                'xlerobot_perception',
                'verify_grasp.launch.py',
                {
                    'backend_enabled': enabled,
                    'dry_run_mode': perception_mode,
                    'vlm_base_url': LaunchConfiguration('vlm_base_url'),
                },
            ),
            Node(
                package='xlerobot_task',
                executable='person_search_node',
                name='person_search',
                output='screen',
                parameters=[
                    {
                        'retreat_distance_m': 0.40,
                        'retreat_speed_mps': 0.08,
                        'manual_retreat_extra_timeout_s': 2.0,
                    }
                ],
            ),
            _include(
                'xlerobot_perception',
                'scan_for_person.launch.py',
                {
                    'backend_enabled': enabled,
                    'dry_run_mode': perception_mode,
                    'model_path': LaunchConfiguration('person_model_path'),
                    'action_name': '/scan_for_person_view',
                },
            ),
            _include(
                'xlerobot_manipulation',
                'grasp_object.launch.py',
                {
                    'execution_enabled': enabled,
                    'grasp_alignment_file': LaunchConfiguration(
                        'grasp_alignment_file'
                    ),
                },
            ),
            _include(
                'xlerobot_manipulation',
                'handover_object.launch.py',
                {'execution_enabled': enabled},
            ),
            _include(
                'xlerobot_voice',
                'speak_text.launch.py',
                {
                    'backend_enabled': LaunchConfiguration('enable_tts'),
                    'audio_player_device': LaunchConfiguration('audio_output_device'),
                    'edge_cache_dir': LaunchConfiguration('tts_cache_dir'),
                },
            ),
            _include(
                'xlerobot_task', 'fetch_deliver_task.launch.py',
                {'speech_enabled': LaunchConfiguration('enable_tts')},
            ),
        ]
    )

    actions.append(
        Node(
            package='xlerobot_bringup',
            executable='wrist_camera_node',
            name='right_wrist_camera',
            output='screen',
            parameters=[
                {
                    'video_device': LaunchConfiguration('wrist_camera_device'),
                    'image_width': 640,
                    'image_height': 480,
                    'fps': 30.0,
                    'pixel_format': 'MJPG',
                }
            ],
        )
    )
    _append_entrypoints(actions, context, live)
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
            DeclareLaunchArgument(
                'hardware_enabled', default_value='false', choices=['true', 'false'],
                description='Explicit consent for the live robot demo.',
            ),
            DeclareLaunchArgument('map', description='Absolute path to the map YAML file.'),
            DeclareLaunchArgument(
                'places_file', description='Absolute path to map-specific named places YAML.'
            ),
            DeclareLaunchArgument(
                'act_predict_url', default_value='http://127.0.0.1:8766/predict'
            ),
            DeclareLaunchArgument(
                'vlm_base_url', default_value='http://127.0.0.1:1234'
            ),
            DeclareLaunchArgument(
                'classical_base_url', default_value='http://127.0.0.1:8765'
            ),
            DeclareLaunchArgument(
                'default_grasp_backend',
                default_value='act',
                choices=['act', 'centroid', 'gpd'],
                description='Backend used by voice requests that do not name one.',
            ),
            DeclareLaunchArgument(
                'grasp_alignment_file', default_value='',
                description='Activated grasp alignment; required by every backend.',
            ),
            DeclareLaunchArgument('person_model_path', default_value=''),
            DeclareLaunchArgument('right_bus', default_value='/dev/right_arm'),
            DeclareLaunchArgument('left_bus', default_value='/dev/left_arm'),
            DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),
            DeclareLaunchArgument(
                'd455_serial',
                default_value='',
                description='Optional head D455 serial; empty selects the only device.',
            ),
            DeclareLaunchArgument(
                'wrist_camera_device', default_value=''
            ),
            DeclareLaunchArgument(
                'enable_voice', default_value='false', choices=['true', 'false']
            ),
            DeclareLaunchArgument('audio_input_device', default_value='auto'),
            DeclareLaunchArgument('audio_output_device', default_value=''),
            DeclareLaunchArgument(
                'tts_cache_dir', default_value='.xlerobot/cache/tts'
            ),
            DeclareLaunchArgument(
                'kws_model_dir',
                default_value='',
                description='Absolute path to the verified sherpa-onnx KWS model.',
            ),
            DeclareLaunchArgument(
                'whisper_model',
                default_value='',
                description='Absolute path to the verified faster-whisper snapshot.',
            ),
            DeclareLaunchArgument(
                'enable_web', default_value='false', choices=['true', 'false']
            ),
            DeclareLaunchArgument('web_bind_host', default_value='0.0.0.0'),
            DeclareLaunchArgument('web_port', default_value='8080'),
            DeclareLaunchArgument(
                'artifact_root', default_value='.xlerobot/artifacts'
            ),
            DeclareLaunchArgument('release_id', default_value='development'),
            DeclareLaunchArgument('unit_id', default_value='reference-two-wheel'),
            DeclareLaunchArgument('site_id', default_value=''),
            DeclareLaunchArgument('geometry_file', default_value=PathJoinSubstitution([
                FindPackageShare('xlerobot_description'), 'config',
                'two_wheel_reference_geometry.yaml',
            ])),
            DeclareLaunchArgument('servo_calibration_file', default_value=PathJoinSubstitution([
                FindPackageShare('xlerobot_description'), 'config',
                'two_wheel_reference_servos.yaml',
            ])),
            DeclareLaunchArgument('controllers_file', default_value=PathJoinSubstitution([
                FindPackageShare('xlerobot_bringup'), 'config',
                'platform_controllers.yaml',
            ])),
            DeclareLaunchArgument(
                'enable_tts', default_value='true', choices=['true', 'false']
            ),
            OpaqueFunction(function=_runtime),
        ]
    )
