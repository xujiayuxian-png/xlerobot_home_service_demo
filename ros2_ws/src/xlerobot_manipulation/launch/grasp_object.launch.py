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
        / 'grasp_object.yaml'
    )
    execution_enabled = LaunchConfiguration('execution_enabled')
    grasp_alignment_file = LaunchConfiguration('grasp_alignment_file')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'execution_enabled',
                default_value='false',
                choices=['true', 'false'],
                description='Independent gate for controller-facing grasp stages.',
            ),
            DeclareLaunchArgument('grasp_alignment_file', default_value=''),
            Node(
                package='xlerobot_manipulation',
                executable='grasp_object_server',
                name='grasp_object_server',
                output='screen',
                parameters=[
                    config,
                    {
                        'execution_enabled': execution_enabled,
                        'grasp_alignment_file': grasp_alignment_file,
                    },
                ],
            ),
        ]
    )
