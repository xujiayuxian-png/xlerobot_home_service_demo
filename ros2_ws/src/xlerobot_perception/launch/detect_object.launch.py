"""Launch the read-only DetectObject adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Keep remote observation explicitly gated and motion-free."""
    config = str(
        Path(get_package_share_directory('xlerobot_perception'))
        / 'config'
        / 'detect_object.yaml'
    )
    backend_enabled = LaunchConfiguration('backend_enabled')
    dry_run_mode = LaunchConfiguration('dry_run_mode')
    vlm_base_url = LaunchConfiguration('vlm_base_url')
    classical_base_url = LaunchConfiguration('classical_base_url')
    grasp_alignment_file = LaunchConfiguration('grasp_alignment_file')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'backend_enabled',
                default_value='false',
                choices=['true', 'false'],
                description='Allow the read-only RGBD/VLM backend.',
            ),
            DeclareLaunchArgument(
                'dry_run_mode',
                default_value='contract_only',
                choices=['contract_only', 'observe'],
                description='Choose contract-only or real read-only observation.',
            ),
            DeclareLaunchArgument(
                'vlm_base_url', default_value='http://127.0.0.1:1234'
            ),
            DeclareLaunchArgument(
                'classical_base_url', default_value='http://127.0.0.1:8765'
            ),
            DeclareLaunchArgument('grasp_alignment_file', default_value=''),
            Node(
                package='xlerobot_perception',
                executable='detect_object_vlm',
                name='detect_object_vlm',
                output='screen',
                parameters=[
                    config,
                    {
                        'backend_enabled': ParameterValue(
                            backend_enabled, value_type=bool
                        ),
                        'dry_run_mode': dry_run_mode,
                        'vlm_base_url': vlm_base_url,
                        'classical_base_url': classical_base_url,
                        'grasp_alignment_file': grasp_alignment_file,
                    },
                ],
            ),
        ]
    )
