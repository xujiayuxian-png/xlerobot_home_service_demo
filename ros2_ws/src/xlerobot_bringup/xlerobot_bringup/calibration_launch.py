"""Shared composition for the four isolated live calibration entrypoints."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


WORKFLOWS = {'servo', 'base_geometry', 'head_camera', 'right_handeye'}


def _require_explicit_hardware(context):
    """Fail before any node can open a device when consent is absent."""
    if LaunchConfiguration('hardware_enabled').perform(context) != 'true':
        raise RuntimeError(
            'live calibration requires hardware_enabled:=true; no device was opened'
        )
    return []


def calibration_launch(workflow_id: str) -> LaunchDescription:
    if workflow_id not in WORKFLOWS:
        raise ValueError(f'unknown calibration workflow: {workflow_id}')
    artifact_root = LaunchConfiguration('artifact_root')
    sample_file = PathJoinSubstitution([
        artifact_root, 'calibration_work', workflow_id, 'samples.yaml'
    ])
    result_file = PathJoinSubstitution([
        artifact_root, 'calibration_work', workflow_id, 'result.yaml'
    ])
    actions = [
        DeclareLaunchArgument(
            'hardware_enabled', default_value='false', choices=['true', 'false'],
            description='Explicit consent for this live calibration session.',
        ),
        OpaqueFunction(function=_require_explicit_hardware),
        DeclareLaunchArgument('artifact_root', default_value='.xlerobot'),
        DeclareLaunchArgument('state_root', default_value='.xlerobot'),
        DeclareLaunchArgument('repo_root', default_value=''),
        DeclareLaunchArgument(
            'task_history_root', default_value='.xlerobot/logs/tasks'
        ),
        DeclareLaunchArgument('release_id', default_value='development'),
        DeclareLaunchArgument('unit_id', default_value='reference-two-wheel'),
        DeclareLaunchArgument('right_bus', default_value='/dev/right_arm'),
        DeclareLaunchArgument('left_bus', default_value='/dev/left_arm'),
        DeclareLaunchArgument(
            'd455_serial',
            default_value='',
            description='Optional head D455 serial; empty selects the only device.',
        ),
        DeclareLaunchArgument('geometry_file', default_value=PathJoinSubstitution([
            FindPackageShare('xlerobot_description'), 'config',
            'two_wheel_reference_geometry.yaml',
        ])),
        DeclareLaunchArgument(
            'servo_calibration_file', default_value=PathJoinSubstitution([
                FindPackageShare('xlerobot_description'), 'config',
                'two_wheel_reference_servos.yaml',
            ])
        ),
        DeclareLaunchArgument('controllers_file', default_value=PathJoinSubstitution([
            FindPackageShare('xlerobot_bringup'), 'config',
            'platform_controllers.yaml',
        ])),
        DeclareLaunchArgument('web_bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('existing_servo_file', default_value=''),
        DeclareLaunchArgument('existing_servo_version', default_value=''),
        Node(
            package='xlerobot_commissioning', executable='calibration_workbench',
            name=f'{workflow_id}_calibration', output='screen', parameters=[{
                'artifact_root': LaunchConfiguration('artifact_root'),
                'software_revision': LaunchConfiguration('release_id'),
                'workflow_id': workflow_id,
                'sample_file': sample_file,
                'capture_only': True,
                'capture_result_file': result_file,
            }],
        ),
        Node(
            package='xlerobot_hmi', executable='operator_console',
            name='xlerobot_operator_console', output='screen', parameters=[{
                'bind_host': LaunchConfiguration('web_bind_host'),
                'port': LaunchConfiguration('web_port'),
                'artifact_root': LaunchConfiguration('artifact_root'),
                'release_id': LaunchConfiguration('release_id'),
                'unit_id': LaunchConfiguration('unit_id'),
                'workspace': 'calibration',
                'calibration_workflow': workflow_id,
                'calibration_capture_only': True,
                'enable_engineering_tools': True,
                'task_history_root': LaunchConfiguration('task_history_root'),
            }],
        ),
    ]
    if workflow_id == 'servo':
        actions.append(Node(
            package='xlerobot_feetech', executable='servo_calibration_server',
            name='servo_calibration_server', output='screen', parameters=[{
                'right_port': LaunchConfiguration('right_bus'),
                'left_port': LaunchConfiguration('left_bus'),
                'unit_id': LaunchConfiguration('unit_id'),
                'existing_servo_file': LaunchConfiguration('existing_servo_file'),
                'existing_servo_version': LaunchConfiguration('existing_servo_version'),
                'result_file': PathJoinSubstitution([
                    artifact_root, 'calibration_work', 'servo', 'result.yaml'
                ]),
            }],
        ))
    if workflow_id in {'head_camera', 'right_handeye'}:
        actions.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('xlerobot_bringup'), 'launch',
                    'platform_runtime.launch.py',
                ])),
                launch_arguments={
                    'hardware_enabled': 'true',
                    'startup_ready': 'false',
                    'startup_head_only': 'false',
                    'head_only_control': 'true' if workflow_id == 'head_camera' else 'false',
                    'geometry_file': LaunchConfiguration('geometry_file'),
                    'servo_calibration_file': LaunchConfiguration(
                        'servo_calibration_file'
                    ),
                    'controllers_file': LaunchConfiguration('controllers_file'),
                    'right_bus': LaunchConfiguration('right_bus'),
                    'left_bus': LaunchConfiguration('left_bus'),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('xlerobot_bringup'), 'launch',
                    'sensors.launch.py',
                ])),
                launch_arguments={
                    'enable_lidar': 'false', 'enable_d455': 'true',
                    'd455_serial': LaunchConfiguration('d455_serial'),
                    'd455_config_file': PathJoinSubstitution([
                        FindPackageShare('xlerobot_calibration_tools'), 'config',
                        'd455_head_calibration.yaml',
                    ]),
                }.items(),
            ),
            Node(
                package='xlerobot_calibration_tools',
                executable='detect_calibration_target',
                name='calibration_target_detector', output='screen',
                parameters=[{'workflow_id': workflow_id}],
            ),
            Node(
                package='xlerobot_calibration_tools',
                executable='collect_transform_samples',
                name='transform_sample_collector', output='screen',
                parameters=[{
                    'calibration_id': LaunchConfiguration('unit_id'),
                    'model': (
                        'moving_camera_fixed_target'
                        if workflow_id == 'head_camera'
                        else 'fixed_camera_moving_target'
                    ),
                    'output': sample_file,
                    'base_frame': 'base_link',
                    'moving_frame': (
                        'head_tilt_link' if workflow_id == 'head_camera'
                        else 'right_arm_fixed_jaw_link'
                    ),
                    # Head calibration solves the URDF mount frame, while
                    # hand-eye intentionally solves base-to-optical internally.
                    'camera_frame': (
                        'head_camera_link' if workflow_id == 'head_camera'
                        else 'd455_color_optical_frame'
                    ),
                    'target_frame': 'calibration_target',
                }],
            ),
            Node(
                package='xlerobot_manipulation',
                executable='calibration_pose_server',
                name='calibration_pose_server', output='screen',
                parameters=[{
                    'execution_enabled': True,
                    'workflow_id': workflow_id,
                    'head_pose_file': PathJoinSubstitution([
                        FindPackageShare('xlerobot_calibration_tools'), 'config',
                        'head_camera_poses.yaml',
                    ]),
                }],
            ),
        ])
    if workflow_id == 'head_camera':
        actions.append(Node(
            package='xlerobot_calibration_tools', executable='auto_head_calibration',
            name='auto_head_calibration', output='screen', parameters=[{
                'execution_enabled': True,
                'unit_id': LaunchConfiguration('unit_id'),
                'artifact_root': LaunchConfiguration('artifact_root'),
                'state_root': LaunchConfiguration('state_root'),
                'repo_root': LaunchConfiguration('repo_root'),
                'pose_file': PathJoinSubstitution([
                    FindPackageShare('xlerobot_calibration_tools'), 'config',
                    'head_camera_poses.yaml',
                ]),
            }],
        ))
    return LaunchDescription(actions)
