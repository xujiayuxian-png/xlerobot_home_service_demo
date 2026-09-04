"""Shared composition for the four isolated calibration tool entrypoints."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


WORKFLOWS = {'servo', 'base_geometry', 'head_camera', 'right_handeye'}


def calibration_launch(workflow_id: str) -> LaunchDescription:
    if workflow_id not in WORKFLOWS:
        raise ValueError(f'unknown calibration workflow: {workflow_id}')
    artifact_root = LaunchConfiguration('artifact_root')
    sample_file = PathJoinSubstitution([
        artifact_root, 'calibration_work', workflow_id, 'samples.yaml'
    ])
    actions = [
        DeclareLaunchArgument('artifact_root', default_value='/var/lib/xlerobot'),
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
        Node(
            package='xlerobot_commissioning', executable='calibration_workbench',
            name=f'{workflow_id}_calibration', output='screen', parameters=[{
                'artifact_root': LaunchConfiguration('artifact_root'),
                'software_revision': LaunchConfiguration('release_id'),
                'workflow_id': workflow_id,
                'sample_file': sample_file,
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
                'enable_engineering_tools': True,
            }],
        ),
    ]
    if workflow_id == 'servo':
        actions.append(Node(
            package='xlerobot_feetech', executable='servo_calibration_server',
            name='servo_calibration_server', output='screen', parameters=[{
                'right_port': LaunchConfiguration('right_bus'),
                'left_port': LaunchConfiguration('left_bus'),
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
                    'startup_ready': 'false',
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
                parameters=[{'execution_enabled': True}],
            ),
        ])
    return LaunchDescription(actions)
