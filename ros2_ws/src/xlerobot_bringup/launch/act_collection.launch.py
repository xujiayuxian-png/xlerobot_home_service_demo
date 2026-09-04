"""Canonical Follower plus isolated Leader ACT demonstration collection profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, name, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', name
        ])), launch_arguments=(arguments or {}).items(),
    )


def runtime(context):
    enabled = 'true'
    actions = [
        include('xlerobot_bringup', 'platform_runtime.launch.py', {
            'geometry_file': LaunchConfiguration('geometry_file'),
            'servo_calibration_file': LaunchConfiguration('servo_calibration_file'),
            'controllers_file': LaunchConfiguration('controllers_file'),
            'right_bus': LaunchConfiguration('right_bus'),
            'left_bus': LaunchConfiguration('left_bus'),
        }),
        include('xlerobot_bringup', 'leader_runtime.launch.py', {
            'leader_port': LaunchConfiguration('leader_port'),
            'control_enable_lease_s': LaunchConfiguration(
                'control_enable_lease_s'
            ),
        }),
        include('xlerobot_bringup', 'sensors.launch.py', {
            'enable_lidar': 'false', 'enable_d455': 'true',
            'lidar_port': LaunchConfiguration('lidar_port'),
            'd455_serial': LaunchConfiguration('d455_serial'),
        }),
        Node(
            package='xlerobot_leader_teleop', executable='leader_follower_teleop',
            output='screen', parameters=[{
                'enable_lease_s': LaunchConfiguration(
                    'control_enable_lease_s'
                ),
            }],
        ),
        Node(
            package='xlerobot_dataset_tools', executable='record_episode_node',
            output='screen', parameters=[{
                'artifact_root': LaunchConfiguration('artifact_root'),
                'software_revision': LaunchConfiguration('release_id'),
                'unit_id': LaunchConfiguration('unit_id'),
                'calibration_version': LaunchConfiguration(
                    'calibration_version'
                ),
            }],
        ),
        Node(
            package='xlerobot_dataset_tools', executable='episode_review_node',
            output='screen', parameters=[{
                'artifact_root': LaunchConfiguration('artifact_root'),
            }],
        ),
        Node(
            package='xlerobot_commissioning', executable='collect_episode',
            output='screen', parameters=[{
                'control_enable_lease_s': LaunchConfiguration(
                    'control_enable_lease_s'
                ),
                'control_heartbeat_period_s': LaunchConfiguration(
                    'control_heartbeat_period_s'
                ),
                'recorder_first_sample_timeout_s': LaunchConfiguration(
                    'recorder_first_sample_timeout_s'
                ),
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
                'workspace': 'collection', 'enable_engineering_tools': True,
            }],
        ),
        include('xlerobot_moveit_config', 'move_group.launch.py', {
            'launch_robot_state_publisher': 'false',
            'geometry_file': LaunchConfiguration('geometry_file'),
            'servo_calibration_file': LaunchConfiguration('servo_calibration_file'),
        }),
        include('xlerobot_perception', 'detect_object.launch.py', {
            'backend_enabled': enabled, 'dry_run_mode': 'observe',
            'vlm_base_url': LaunchConfiguration('vlm_base_url'),
        }),
        include('xlerobot_manipulation', 'grasp_object.launch.py', {
            'execution_enabled': enabled,
        }),
        Node(
            package='xlerobot_bringup', executable='wrist_camera_node',
            name='right_wrist_camera', output='screen', parameters=[{
                'video_device': LaunchConfiguration('wrist_camera_device'),
                'image_width': 640, 'image_height': 480, 'fps': 30.0,
                'pixel_format': 'MJPG',
            }],
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('artifact_root', default_value='/var/lib/xlerobot'),
        DeclareLaunchArgument('release_id', default_value='development'),
        DeclareLaunchArgument('unit_id', default_value='reference-two-wheel'),
        DeclareLaunchArgument('calibration_version', default_value=''),
        DeclareLaunchArgument('right_bus', default_value='/dev/right_arm'),
        DeclareLaunchArgument('left_bus', default_value='/dev/left_arm'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),
        DeclareLaunchArgument(
            'd455_serial',
            default_value='',
            description='Optional head D455 serial; empty selects the only device.',
        ),
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
        DeclareLaunchArgument('leader_port', default_value='/dev/right_master_arm'),
        DeclareLaunchArgument('control_enable_lease_s', default_value='1.0'),
        DeclareLaunchArgument(
            'control_heartbeat_period_s', default_value='0.2'
        ),
        DeclareLaunchArgument(
            'recorder_first_sample_timeout_s', default_value='2.0'
        ),
        DeclareLaunchArgument('wrist_camera_device', default_value=''),
        DeclareLaunchArgument('vlm_base_url', default_value='http://127.0.0.1:1234'),
        DeclareLaunchArgument('web_bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        OpaqueFunction(function=runtime),
    ])
