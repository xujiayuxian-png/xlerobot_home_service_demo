import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'xlerobot_lidar_driver'
    scan_topic = LaunchConfiguration('scan_topic')
    mock_hardware = LaunchConfiguration('mock_hardware')
    hardware_enabled = LaunchConfiguration('hardware_enabled')
    port_name = LaunchConfiguration('port_name')
    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'lidar_params.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_topic',
            default_value='scan',
            description='LaserScan output topic.',
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='true',
            description='Publish deterministic scans without opening a serial device.',
        ),
        DeclareLaunchArgument(
            'hardware_enabled',
            default_value='false',
            description='Second key required before /dev/lidar may be opened.',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/lidar',
            description='Serial device used only when both hardware gates are open.',
        ),
        Node(
            package=pkg_name,
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[
                config_path,
                {
                    'mock_hardware': ParameterValue(mock_hardware, value_type=bool),
                    'hardware_enabled': ParameterValue(hardware_enabled, value_type=bool),
                    'port_name': port_name,
                },
            ],
            remappings=[('scan', scan_topic)],
        ),
    ])
