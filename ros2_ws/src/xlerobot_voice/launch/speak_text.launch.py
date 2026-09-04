"""Launch the gated local SpeakText adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Keep process creation disabled unless explicitly enabled."""
    config = str(
        Path(get_package_share_directory('xlerobot_voice'))
        / 'config'
        / 'speak_text.yaml'
    )
    backend_enabled = LaunchConfiguration('backend_enabled')
    audio_player_device = LaunchConfiguration('audio_player_device')
    edge_cache_dir = LaunchConfiguration('edge_cache_dir')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'backend_enabled',
                default_value='false',
                choices=['true', 'false'],
                description='Allow the configured local player process.',
            ),
            DeclareLaunchArgument(
                'audio_player_device',
                default_value='',
                description='Optional mpg123 ALSA output, for example plughw:0,0.',
            ),
            DeclareLaunchArgument(
                'edge_cache_dir',
                default_value='.xlerobot/cache/tts',
                description='Ignored local cache for generated speech.',
            ),
            Node(
                package='xlerobot_voice',
                executable='speak_text',
                name='speak_text_server',
                output='screen',
                parameters=[
                    config,
                    {
                        'backend_enabled': ParameterValue(
                            backend_enabled, value_type=bool
                        ),
                        'audio_player_device': audio_player_device,
                        'edge_cache_dir': edge_cache_dir,
                    },
                ],
            ),
        ]
    )
