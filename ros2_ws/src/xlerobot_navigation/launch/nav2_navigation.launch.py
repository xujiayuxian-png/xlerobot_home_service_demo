from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('xlerobot_navigation'))
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    common_parameters = [params_file, {'use_sim_time': use_sim_time}]
    localization_motion_nodes = [
        'controller_server',
        'smoother_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
    ]
    navigation_nodes = [
        'planner_server',
        'bt_navigator',
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=str(share / 'config' / 'nav2_two_wheel_reference.yaml'),
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=common_parameters,
                remappings=[('cmd_vel', 'cmd_vel_nav_raw')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=common_parameters,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=common_parameters,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=common_parameters,
                remappings=[('cmd_vel', 'cmd_vel_nav_raw')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[
                    *common_parameters,
                    {
                        'default_nav_to_pose_bt_xml': str(
                            share
                            / 'behavior_trees'
                            / 'navigate_to_pose_position_only.xml'
                        )
                    },
                ],
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=common_parameters,
                remappings=[('cmd_vel', 'cmd_vel_nav_raw')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                parameters=common_parameters,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization_motion',
                output='screen',
                parameters=[
                    {'autostart': True, 'node_names': localization_motion_nodes},
                    {'use_sim_time': use_sim_time},
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'autostart': False, 'node_names': navigation_nodes},
                    {'use_sim_time': use_sim_time},
                ],
            ),
        ]
    )
