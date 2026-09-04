"""Launch the learned-policy executor with execution disabled by default."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    execution_enabled = LaunchConfiguration('execution_enabled')
    params = PathJoinSubstitution(
        [
            FindPackageShare('xlerobot_manipulation'),
            'config',
            'streaming_executor.yaml',
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('execution_enabled', default_value='false'),
            Node(
                package='xlerobot_manipulation',
                executable='streaming_joint_executor',
                output='screen',
                parameters=[params, {'execution_enabled': execution_enabled}],
            ),
        ]
    )
