from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory('xlerobot_navigation'))
    execution_enabled = LaunchConfiguration('execution_enabled')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'execution_enabled',
                default_value='false',
                choices=['true', 'false'],
                description='Allow gated NavigateToPose approach goals.',
            ),
            Node(
                package='xlerobot_navigation',
                executable='approach_target_server',
                output='screen',
                parameters=[
                    str(package_share / 'config' / 'approach_target.yaml'),
                    {
                        'execution_enabled': ParameterValue(
                            execution_enabled, value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
