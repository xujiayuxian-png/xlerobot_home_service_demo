"""Launch bounded maintenance presets through normal ros2_control actions."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'execution_enabled', default_value='false', choices=['true', 'false']
        ),
        Node(
            package='xlerobot_manipulation',
            executable='maintenance_preset_server',
            name='maintenance_preset_server',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('xlerobot_manipulation'),
                    'config', 'maintenance_presets.yaml',
                ]),
                {'execution_enabled': LaunchConfiguration('execution_enabled')},
            ],
        ),
    ])
