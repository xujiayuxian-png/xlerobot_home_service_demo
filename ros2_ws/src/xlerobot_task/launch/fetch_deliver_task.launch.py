"""Launch the backend-independent fetch-and-deliver orchestrator."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(
        Path(get_package_share_directory('xlerobot_task'))
        / 'config'
        / 'fetch_deliver_task.yaml'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'speech_enabled', default_value='true', choices=['true', 'false']
            ),
            Node(
                package='xlerobot_task',
                executable='fetch_deliver_task_node',
                name='fetch_deliver_task',
                output='screen',
                parameters=[config, {
                    'speech_enabled': ParameterValue(
                        LaunchConfiguration('speech_enabled'), value_type=bool
                    ),
                }],
            )
        ]
    )
