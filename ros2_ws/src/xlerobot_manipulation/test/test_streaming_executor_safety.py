import os
import time
import unittest

from action_msgs.msg import GoalStatus
import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory
from xlerobot_interfaces.action import ExecutePolicyStream
from xlerobot_interfaces.msg import CapabilityError


JOINTS = [
    'right_arm_shoulder_pan',
    'right_arm_shoulder_lift',
    'right_arm_elbow_flex',
    'right_arm_wrist_flex',
    'right_arm_wrist_roll',
    'right_arm_gripper',
]


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '73'
    executor = launch_ros.actions.Node(
        package='xlerobot_manipulation',
        executable='streaming_joint_executor',
        parameters=[{'execution_enabled': False}],
        output='screen',
    )
    return launch.LaunchDescription(
        [executor, launch_testing.actions.ReadyToTest()]
    )


class StreamingExecutorSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])
        cls.node = rclpy.create_node('streaming_executor_safety_test')
        cls.client = ActionClient(
            cls.node, ExecutePolicyStream, '/execute_policy_stream'
        )
        cls.commands = []
        cls.subscription = cls.node.create_subscription(
            JointTrajectory,
            '/right_policy_controller/joint_trajectory',
            cls.commands.append,
            10,
        )
        assert cls.client.wait_for_server(timeout_sec=10.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_01_dry_run_is_backend_free_and_motionless(self):
        wrapped = self._execute(dry_run=True)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(self.commands, [])

    def test_02_execution_is_rejected_by_default(self):
        wrapped = self._execute(dry_run=False)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            wrapped.result.error.code, CapabilityError.SAFETY_REJECTED
        )
        self.assertIn('execution_enabled', wrapped.result.error.message)
        self.assertEqual(self.commands, [])

    def test_03_capture_only_is_never_accepted_by_motor_executor(self):
        wrapped = self._execute(dry_run=False, capture_only=True)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            wrapped.result.error.code, CapabilityError.SAFETY_REJECTED
        )
        self.assertIn('capture-only', wrapped.result.error.message)
        self.assertEqual(self.commands, [])

    def _execute(self, dry_run, capture_only=False):
        goal = ExecutePolicyStream.Goal()
        goal.session_id = 'streaming-safety-test'
        goal.source_id = 'test-policy'
        goal.joint_names = JOINTS
        goal.max_duration.sec = 2
        goal.dry_run = dry_run
        goal.capture_only = capture_only
        goal_handle = self._wait_future(
            self.client.send_goal_async(goal), 5.0
        )
        self.assertTrue(goal_handle.accepted)
        return self._wait_future(goal_handle.get_result_async(), 5.0)

    def _wait_future(self, future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if not future.done():
            raise AssertionError('future timed out')
        return future.result()
