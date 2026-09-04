from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('xlerobot_navigation'))
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    common_parameters = [params_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'map', description='Required absolute path to a map YAML file.'
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=str(share / 'config' / 'nav2_two_wheel_reference.yaml'),
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[*common_parameters, {'yaml_filename': map_file}],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=common_parameters,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'autostart': True, 'node_names': ['map_server', 'amcl']},
                    {'use_sim_time': use_sim_time},
                ],
            ),
        ]
    )
