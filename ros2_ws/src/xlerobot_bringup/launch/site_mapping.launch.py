"""Build or validate one site draft using an isolated mapping profile."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
        ),
        launch_arguments=(arguments or {}).items(),
    )


def runtime(context):
    phase = LaunchConfiguration('phase').perform(context)
    artifact_root = Path(LaunchConfiguration('artifact_root').perform(context))
    site_id = LaunchConfiguration('site_id').perform(context).strip()
    if phase not in {'build', 'validate'}:
        raise ValueError('mapping phase must be build or validate')
    if not site_id:
        raise ValueError('site_id is required for mapping and validation')

    actions = [
        include('xlerobot_bringup', 'platform_runtime.launch.py', {
            'geometry_file': LaunchConfiguration('geometry_file'),
            'servo_calibration_file': LaunchConfiguration('servo_calibration_file'),
            'controllers_file': LaunchConfiguration('controllers_file'),
            'right_bus': LaunchConfiguration('right_bus'),
            'left_bus': LaunchConfiguration('left_bus'),
        }),
        include('xlerobot_bringup', 'sensors.launch.py', {
            'enable_lidar': 'true', 'enable_d455': 'false',
            'lidar_port': LaunchConfiguration('lidar_port'),
        }),
    ]
    if phase == 'build':
        actions.append(include('xlerobot_navigation', 'slam_mapping.launch.py'))
    else:
        draft = artifact_root / 'sites' / '.drafts' / site_id
        map_file = draft / 'map.yaml'
        places_file = draft / 'places.yaml'
        if not map_file.is_file() or not places_file.is_file():
            raise ValueError(
                f'site draft must contain map.yaml and places.yaml: {draft}'
            )
        actions.extend([
            include('xlerobot_navigation', 'amcl_localization.launch.py', {
                'map': str(map_file),
            }),
            include('xlerobot_navigation', 'nav2_navigation.launch.py'),
            include('xlerobot_navigation', 'scan_map_consistency.launch.py'),
            include('xlerobot_navigation', 'auto_localizer.launch.py', {
                'execution_enabled': 'true',
                'activate_navigation_after_localization': 'true',
            }),
            include('xlerobot_navigation', 'named_navigation.launch.py', {
                'execution_enabled': 'true', 'places_file': str(places_file),
            }),
        ])

    actions.extend([
        Node(
            package='xlerobot_commissioning', executable='site_manager',
            name='site_manager', output='screen',
            parameters=[{
                'artifact_root': LaunchConfiguration('artifact_root'),
                'software_revision': LaunchConfiguration('release_id'),
                'operator': 'operator',
            }],
        ),
        Node(
            package='xlerobot_hmi', executable='operator_console',
            name='xlerobot_operator_console', output='screen',
            parameters=[{
                'bind_host': LaunchConfiguration('web_bind_host'),
                'port': LaunchConfiguration('web_port'),
                'artifact_root': LaunchConfiguration('artifact_root'),
                'release_id': LaunchConfiguration('release_id'),
                'unit_id': LaunchConfiguration('unit_id'),
                'site_id': LaunchConfiguration('site_id'),
                'workspace': 'mapping',
                'mapping_phase': phase,
                'enable_engineering_tools': True,
            }],
        ),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('phase', default_value='build', choices=['build', 'validate']),
        DeclareLaunchArgument('artifact_root', default_value='/var/lib/xlerobot'),
        DeclareLaunchArgument('release_id', default_value='development'),
        DeclareLaunchArgument('unit_id', default_value='reference-two-wheel'),
        DeclareLaunchArgument('site_id'),
        DeclareLaunchArgument('right_bus', default_value='/dev/right_arm'),
        DeclareLaunchArgument('left_bus', default_value='/dev/left_arm'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),
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
        DeclareLaunchArgument('web_bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        OpaqueFunction(function=runtime),
    ])
