from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory('xlerobot_manipulation'))
        / 'config'
        / 'handover_object.yaml'
    )
    execution_enabled = LaunchConfiguration('execution_enabled')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'execution_enabled',
                default_value='false',
                choices=['true', 'false'],
                description='Allow the gated right-arm handover sequence.',
            ),
            Node(
                package='xlerobot_manipulation',
                executable='handover_object_server',
                name='handover_object_server',
                output='screen',
                parameters=[config, {'execution_enabled': execution_enabled}],
            ),
        ]
    )
