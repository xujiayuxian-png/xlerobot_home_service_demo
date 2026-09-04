"""Launch the read-only current-view ScanForPerson adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Keep local observation explicitly gated and motion-free."""
    config = str(
        Path(get_package_share_directory('xlerobot_perception'))
        / 'config'
        / 'scan_for_person.yaml'
    )
    backend_enabled = LaunchConfiguration('backend_enabled')
    dry_run_mode = LaunchConfiguration('dry_run_mode')
    model_path = LaunchConfiguration('model_path')
    action_name = LaunchConfiguration('action_name')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'backend_enabled',
                default_value='false',
                choices=['true', 'false'],
                description='Allow the read-only RGBD/YOLO backend.',
            ),
            DeclareLaunchArgument(
                'dry_run_mode',
                default_value='contract_only',
                choices=['contract_only', 'observe'],
                description='Choose contract-only or real read-only observation.',
            ),
            DeclareLaunchArgument('model_path', default_value=''),
            DeclareLaunchArgument('action_name', default_value='scan_for_person'),
            Node(
                package='xlerobot_perception',
                executable='scan_for_person_detector',
                name='scan_for_person_detector',
                output='screen',
                parameters=[
                    config,
                    {
                        'backend_enabled': ParameterValue(
                            backend_enabled, value_type=bool
                        ),
                        'dry_run_mode': dry_run_mode,
                        'model_path': model_path,
                        'action_name': action_name,
                    },
                ],
            ),
        ]
    )
