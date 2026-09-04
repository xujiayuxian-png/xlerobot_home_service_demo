from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navigation_share = Path(get_package_share_directory('xlerobot_navigation'))
    slam_share = Path(get_package_share_directory('slam_toolbox'))
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(slam_share / 'launch' / 'online_async_launch.py')
        ),
        launch_arguments={
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
            'use_sim_time': use_sim_time,
            'slam_params_file': params_file,
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=str(
                    navigation_share / 'config' / 'slam_toolbox_mapping.yaml'
                ),
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            slam,
        ]
    )
