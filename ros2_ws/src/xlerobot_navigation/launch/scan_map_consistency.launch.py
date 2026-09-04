from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('xlerobot_navigation'))
    return LaunchDescription([
        Node(
            package='xlerobot_navigation',
            executable='scan_map_consistency',
            output='screen',
            parameters=[str(share / 'config' / 'scan_map_consistency.yaml')],
        ),
    ])
