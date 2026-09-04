"""Launch MoveIt planning with execution and hardware disabled by default."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def load_text(path):
    with open(path, 'r', encoding='utf-8') as stream:
        return stream.read()


def launch_setup(_context):
    description_share = get_package_share_directory('xlerobot_description')
    config_share = get_package_share_directory('xlerobot_moveit_config')
    xacro_file = os.path.join(
        description_share, 'urdf', 'two_wheel_reference.urdf.xacro'
    )

    def config(name):
        return os.path.join(config_share, 'config', name)
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                xacro_file,
                ' mock_hardware:=true',
                ' hardware_enabled:=false',
                ' torque_enabled:=false',
                ' geometry_file:=', LaunchConfiguration('geometry_file'),
                ' servo_calibration_file:=', LaunchConfiguration('servo_calibration_file'),
            ]),
            value_type=str,
        )
    }
    semantic = {
        'robot_description_semantic': load_text(
            config('two_wheel_reference.srdf')
        )
    }
    kinematics = {'robot_description_kinematics': load_yaml(config('kinematics.yaml'))}
    move_group_options = {
        'allow_trajectory_execution': False,
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'planning_scene_monitor_options': {
            'name': 'planning_scene_monitor',
            'robot_description': 'robot_description',
            'joint_state_topic': '/joint_states',
            'attached_collision_object_topic': '/planning_scene',
            'publish_planning_scene_topic': '/planning_scene',
            'monitored_planning_scene_topic': '/monitored_planning_scene',
            'wait_for_initial_state_timeout': 0.0,
        },
        'planning_scene_monitor.publish_planning_scene': True,
        'planning_scene_monitor.publish_geometry_updates': True,
        'planning_scene_monitor.publish_state_updates': True,
        'planning_scene_monitor.publish_transforms_updates': True,
        'publish_robot_description': False,
        'publish_robot_description_semantic': True,
    }
    common = [
        robot_description,
        semantic,
        kinematics,
        load_yaml(config('joint_limits.yaml')),
        load_yaml(config('ompl_planning.yaml')),
        load_yaml(config('moveit_controllers.yaml')),
    ]
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            condition=IfCondition(LaunchConfiguration('launch_robot_state_publisher')),
            output='screen',
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            parameters=[*common, move_group_options],
            sigterm_timeout='15',
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_robot_state_publisher', default_value='true'),
        DeclareLaunchArgument(
            'geometry_file', default_value=os.path.join(
                get_package_share_directory('xlerobot_description'), 'config',
                'two_wheel_reference_geometry.yaml'),
        ),
        DeclareLaunchArgument(
            'servo_calibration_file', default_value=os.path.join(
                get_package_share_directory('xlerobot_description'), 'config',
                'two_wheel_reference_servos.yaml'),
        ),
        OpaqueFunction(function=launch_setup),
    ])
