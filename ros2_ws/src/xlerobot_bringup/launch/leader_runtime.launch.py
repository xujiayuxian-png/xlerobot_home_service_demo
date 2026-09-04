"""Independent Leader attachment ros2_control runtime; no robot RSP is started."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def runtime(context):
    if LaunchConfiguration('hardware_enabled').perform(context) != 'true':
        raise RuntimeError(
            'Leader runtime requires hardware_enabled:=true; no device was opened'
        )
    model = PathJoinSubstitution([
        FindPackageShare('xlerobot_description'), 'urdf', 'leader_attachment.urdf.xacro'
    ])
    description = {'robot_description': ParameterValue(Command([
        'xacro ', model,
        ' mock_hardware:=false',
        ' hardware_enabled:=true',
        ' port:=', LaunchConfiguration('leader_port'),
    ]), value_type=str)}
    config = PathJoinSubstitution([
        FindPackageShare('xlerobot_bringup'), 'config', 'leader_controllers.yaml'
    ])
    joint_states = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['leader_joint_state_broadcaster', '-c', '/leader/controller_manager'],
    )
    controllers = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            '-c', '/leader/controller_manager', '--activate-as-group',
            '--controller', 'leader_arm_controller',
            '--controller', 'leader_torque_controller',
            '--controller-ros-args', [
                '--ros-args -p enable_lease_s:=',
                LaunchConfiguration('control_enable_lease_s'),
            ],
        ],
    )
    return [
        Node(
            package='xlerobot_bringup', executable='robot_description_publisher',
            namespace='leader', parameters=[description], output='screen',
        ),
        Node(
            package='controller_manager', executable='ros2_control_node',
            namespace='leader', parameters=[description, config], output='screen',
        ),
        joint_states,
        RegisterEventHandler(OnProcessExit(
            target_action=joint_states, on_exit=[controllers]
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_enabled', default_value='false', choices=['true', 'false'],
            description='Explicit consent to open the Leader motor bus.',
        ),
        DeclareLaunchArgument('leader_port', default_value='/dev/right_master_arm'),
        DeclareLaunchArgument('control_enable_lease_s', default_value='1.0'),
        OpaqueFunction(function=runtime),
    ])
