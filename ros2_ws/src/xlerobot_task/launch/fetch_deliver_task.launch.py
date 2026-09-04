"""Launch the backend-independent fetch-and-deliver orchestrator."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory('xlerobot_task'))
        / 'config'
        / 'fetch_deliver_task.yaml'
    )
    return LaunchDescription(
        [
            Node(
                package='xlerobot_task',
                executable='fetch_deliver_task_node',
                name='fetch_deliver_task',
                output='screen',
                parameters=[config],
            )
        ]
    )
