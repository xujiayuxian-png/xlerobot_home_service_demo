"""Bring up the lidar and D455 for a product hardware profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_launch(package, filename, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_d455 = LaunchConfiguration("enable_d455")
    lidar_port = LaunchConfiguration("lidar_port")
    d455_serial = LaunchConfiguration("d455_serial")
    default_realsense_config = PathJoinSubstitution(
        [FindPackageShare("xlerobot_bringup"), "config", "realsense_d455.yaml"]
    )
    realsense_config = LaunchConfiguration("d455_config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_lidar", default_value="false"),
            DeclareLaunchArgument("enable_d455", default_value="false"),
            DeclareLaunchArgument(
                "lidar_port",
                default_value="/dev/lidar",
                description="Stable lidar device alias.",
            ),
            DeclareLaunchArgument(
                "d455_serial",
                default_value="",
                description=(
                    "Optional head D455 serial number; empty selects the only "
                    "connected RealSense device."
                ),
            ),
            DeclareLaunchArgument(
                "d455_config_file",
                default_value=default_realsense_config,
                description="RealSense YAML profile for this capture/runtime.",
            ),
            package_launch(
                "xlerobot_lidar_driver",
                "lidar.launch.py",
                {
                    "mock_hardware": "false",
                    "hardware_enabled": "true",
                    "port_name": lidar_port,
                },
                condition=IfCondition(enable_lidar),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                    )
                ),
                launch_arguments={
                    "camera_name": "d455",
                    "camera_namespace": "xlerobot",
                    "config_file": realsense_config,
                    # rs_launch evaluates untyped launch values as YAML.
                    # Preserve numeric serials (and the empty default) as strings.
                    "serial_no": ["'", d455_serial, "'"],
                }.items(),
                condition=IfCondition(enable_d455),
            ),
        ]
    )
