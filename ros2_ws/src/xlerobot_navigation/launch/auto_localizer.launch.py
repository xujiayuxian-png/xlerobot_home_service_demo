from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('xlerobot_navigation'))
    execution_enabled = LaunchConfiguration('execution_enabled')
    activate_navigation = LaunchConfiguration(
        'activate_navigation_after_localization'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'execution_enabled',
                default_value='false',
                description='Allow AMCL scatter and Nav2 Spin execution.',
            ),
            DeclareLaunchArgument(
                'activate_navigation_after_localization',
                default_value='false',
                description='Start the map-dependent Nav2 lifecycle group after convergence.',
            ),
            Node(
                package='xlerobot_navigation',
                executable='auto_localizer_server',
                output='screen',
                parameters=[
                    str(share / 'config' / 'auto_localizer.yaml'),
                    {
                        'execution_enabled': ParameterValue(
                            execution_enabled, value_type=bool
                        ),
                        'activate_navigation_after_localization': ParameterValue(
                            activate_navigation, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
