"""Launch read-only post-grasp wrist-camera verification."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(
        Path(get_package_share_directory('xlerobot_perception'))
        / 'config' / 'verify_grasp.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend_enabled', default_value='false', choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'dry_run_mode', default_value='contract_only',
            choices=['contract_only', 'observe'],
        ),
        DeclareLaunchArgument('vlm_base_url', default_value='http://127.0.0.1:1234'),
        Node(
            package='xlerobot_perception',
            executable='verify_grasp_vlm',
            name='verify_grasp_vlm',
            output='screen',
            parameters=[
                config,
                {
                    'backend_enabled': ParameterValue(
                        LaunchConfiguration('backend_enabled'), value_type=bool
                    ),
                    'dry_run_mode': LaunchConfiguration('dry_run_mode'),
                    'vlm_base_url': LaunchConfiguration('vlm_base_url'),
                },
            ],
        ),
    ])
