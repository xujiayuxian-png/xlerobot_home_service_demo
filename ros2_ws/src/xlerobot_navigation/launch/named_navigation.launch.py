"""Launch named navigation, optionally loading map-specific places from YAML."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _place_parameters(path_text):
    if not path_text:
        return {}
    path = Path(path_text).expanduser()
    with path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    root = document.get('named_places', document)
    places = root.get('places')
    if not isinstance(places, dict) or not places:
        raise ValueError('places_file must contain named_places.places')
    parameters = {'place_ids': list(places)}
    default_frame = str(root.get('frame_id', 'map'))
    for place_id, place in places.items():
        if not isinstance(place, dict):
            raise ValueError(f'named place {place_id!r} must be a mapping')
        prefix = f'places.{place_id}.'
        parameters[prefix + 'frame_id'] = str(place.get('frame_id', default_frame))
        for field in ('x', 'y', 'yaw'):
            if field not in place:
                raise ValueError(f'named place {place_id!r} is missing {field}')
            parameters[prefix + field] = float(place[field])
        offset = float(place.get('nav_offset_m', 0.0))
        parameters[prefix + 'nav_offset_m'] = offset
        parameters[prefix + 'dock'] = bool(place.get('dock', offset > 0.0))
    return parameters


def _server(context):
    share = Path(get_package_share_directory('xlerobot_navigation'))
    places = _place_parameters(LaunchConfiguration('places_file').perform(context))
    return [
        Node(
            package='xlerobot_navigation',
            executable='named_navigation_server',
            output='screen',
            parameters=[
                str(share / 'config' / 'named_navigation.yaml'),
                places,
                {
                    'execution_enabled': ParameterValue(
                        LaunchConfiguration('execution_enabled'), value_type=bool
                    )
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'execution_enabled',
                default_value='false',
                description='Allow Nav2, Spin, and docking execution.',
            ),
            DeclareLaunchArgument(
                'places_file',
                default_value='',
                description='Optional map-specific named_places YAML file.',
            ),
            OpaqueFunction(function=_server),
        ]
    )
