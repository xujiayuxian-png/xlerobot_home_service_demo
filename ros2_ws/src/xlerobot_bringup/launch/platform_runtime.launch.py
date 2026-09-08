"""Start the canonical runtime, optionally owning only the two head servos."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _runtime_nodes(context):
    if LaunchConfiguration('hardware_enabled').perform(context) != 'true':
        raise RuntimeError(
            'platform runtime requires hardware_enabled:=true; no device was opened'
        )
    head_only_control = LaunchConfiguration(
        'head_only_control', default='false').perform(context) == 'true'
    right_handeye_control = LaunchConfiguration(
        'right_handeye_control', default='false').perform(context) == 'true'
    if head_only_control and right_handeye_control:
        raise RuntimeError('choose only one isolated calibration control mode')
    if (head_only_control or right_handeye_control) and any(
        LaunchConfiguration(name, default='false').perform(context) == 'true'
        for name in ('startup_ready', 'startup_head_only')
    ):
        raise RuntimeError(
            'head_only_control holds the measured head pose; startup motions must be false'
        )
    model = PathJoinSubstitution(
        [FindPackageShare('xlerobot_description'), 'urdf', 'two_wheel_reference.urdf.xacro']
    )
    description = {
        'robot_description': ParameterValue(
            Command(
                [
                    'xacro ', model,
                    ' mock_hardware:=false',
                    ' hardware_enabled:=true',
                    ' torque_enabled:=true',
                    ' read_only:=false',
                    ' geometry_file:=', LaunchConfiguration('geometry_file'),
                    ' servo_calibration_file:=', LaunchConfiguration('servo_calibration_file'),
                    ' include_right_bus_control:=', 'false' if head_only_control else 'true',
                    ' include_left_bus_control:=true',
                    ' head_only_control:=', 'true' if head_only_control else 'false',
                    ' right_handeye_control:=', 'true' if right_handeye_control else 'false',
                    ' right_bus_port:=', LaunchConfiguration('right_bus'),
                    ' left_bus_port:=', LaunchConfiguration('left_bus'),
                ]
            ),
            value_type=str,
        )
    }
    controllers = LaunchConfiguration('controllers_file')

    joint_state_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
    )
    if head_only_control or right_handeye_control:
        # Same bus owner, codecs and measured-pose admission as the full runtime.
        # Head IDs 7/8, plus right-arm IDs 1–6 for hand-eye. Other actuators
        # are absent, not merely left without an active controller.
        head_spawner = Node(
            package='xlerobot_bringup', executable='position_ready_spawner', output='screen',
            parameters=[description],
            arguments=(['head_controller', 'right_arm_controller', 'right_gripper_controller']
                       if right_handeye_control else ['head_controller']) + ['-c', '/controller_manager'],
        )
        return [
            Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                parameters=[description], output='screen',
            ),
            Node(
                package='controller_manager', executable='ros2_control_node',
                parameters=[description, controllers],
                output='screen',
            ),
            joint_state_spawner,
            RegisterEventHandler(OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=lambda event, context: [head_spawner] if event.returncode == 0 else [],
            )),
        ]
    base_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'base_controller', '-c', '/controller_manager',
            '--controller-ros-args', '--ros-args --remap ~/odom:=/odom',
        ],
    )
    normal_spawner = Node(
        package='xlerobot_bringup', executable='position_ready_spawner', output='screen',
        parameters=[description],
        arguments=[
            'right_arm_controller', 'right_gripper_controller', 'head_controller',
            '-c', '/controller_manager', '--activate-as-group',
        ],
    )
    inactive_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'left_arm_controller', 'left_gripper_controller', 'right_policy_controller',
            '-c', '/controller_manager', '--inactive',
        ],
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[base_spawner, normal_spawner, inactive_spawner],
        )
    )
    actions = [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[description], output='screen',
        ),
        Node(
            package='controller_manager', executable='ros2_control_node',
            parameters=[description, controllers], output='screen',
        ),
        Node(
            package='xlerobot_base', executable='drive_safety_node',
            name='drive_safety', output='screen',
            parameters=[PathJoinSubstitution(
                [FindPackageShare('xlerobot_bringup'), 'config', 'drive_safety.yaml']
            )],
        ),
        joint_state_spawner,
        start_controllers,
    ]
    head_only = LaunchConfiguration('startup_head_only', default='false').perform(context) == 'true'
    if LaunchConfiguration('startup_ready', default='false').perform(context) == 'true' or head_only:
        startup_ready = Node(
            package='xlerobot_manipulation', executable='startup_ready_pose',
            name='startup_ready_pose', output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('xlerobot_manipulation'), 'config',
                    'startup_ready.yaml',
                ]),
                {'execution_enabled': True, 'head_only': head_only,
                 'head_ready_positions': [0.0, float(LaunchConfiguration(
                     'startup_head_tilt', default='0.0').perform(context))],
                 'head_duration_s': 3.0 if head_only else 2.0,
                 'arrival_tolerance_rad': 0.04 if head_only else 0.12},
            ],
        )
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=normal_spawner,
                          on_exit=lambda event, context: [startup_ready] if event.returncode == 0 else [])
        ))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'hardware_enabled', default_value='false', choices=['true', 'false'],
                description='Explicit consent to open motor buses.',
            ),
            DeclareLaunchArgument(
                'head_only_control', default_value='false', choices=['true', 'false'],
                description='Own only left-bus head servos 7/8; do not open the right bus.',
            ),
            DeclareLaunchArgument(
                'right_handeye_control', default_value='false', choices=['true', 'false'],
                description='Own only right arm/gripper IDs 1–6 and left-bus head IDs 7/8.',
            ),
            DeclareLaunchArgument(
                'startup_ready', default_value='false', choices=['true', 'false'],
                description='Move the right arm and gripper to the verified demo ready pose.',
            ),
            DeclareLaunchArgument('startup_head_only', default_value='false', choices=['true', 'false']),
            DeclareLaunchArgument('startup_head_tilt', default_value='0.0'),
            DeclareLaunchArgument(
                'right_bus',
                default_value='/dev/right_arm',
                description='Stable device path for the right Follower serial bus.',
            ),
            DeclareLaunchArgument(
                'left_bus',
                default_value='/dev/left_arm',
                description='Stable device path for the left Follower serial bus.',
            ),
            DeclareLaunchArgument(
                'geometry_file', default_value=PathJoinSubstitution([
                    FindPackageShare('xlerobot_description'), 'config',
                    'two_wheel_reference_geometry.yaml',
                ]),
            ),
            DeclareLaunchArgument(
                'servo_calibration_file', default_value=PathJoinSubstitution([
                    FindPackageShare('xlerobot_description'), 'config',
                    'two_wheel_reference_servos.yaml',
                ]),
            ),
            DeclareLaunchArgument(
                'controllers_file', default_value=PathJoinSubstitution([
                    FindPackageShare('xlerobot_bringup'), 'config',
                    'platform_controllers.yaml',
                ]),
            ),
            OpaqueFunction(function=_runtime_nodes),
        ]
    )
