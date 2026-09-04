import os
import time
import unittest

from action_msgs.msg import GoalStatus
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from xlerobot_interfaces.action import ApproachTarget
from xlerobot_interfaces.msg import CapabilityError


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '71'
    server = Node(
        package='xlerobot_navigation',
        executable='approach_target_server',
        name='approach_target_server',
        parameters=[{'execution_enabled': False}],
        output='screen',
    )
    return LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class ApproachTargetSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('approach_target_safety_client')
        cls.client = ActionClient(cls.node, ApproachTarget, '/approach_target')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        assert cls.client.wait_for_server(timeout_sec=5.0)

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        cls.executor.remove_node(cls.node)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_live_goal_is_rejected_before_tf_or_nav2(self):
        goal = self._goal(dry_run=False)
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertTrue(handle.accepted)
        wrapped = self._wait(handle.get_result_async(), 3.0)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.SAFETY_REJECTED)
        self.assertIn('execution_enabled', wrapped.result.error.message)

    def test_invalid_geometry_is_rejected_at_action_boundary(self):
        goal = self._goal(dry_run=True)
        goal.target.header.frame_id = ''
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertFalse(handle.accepted)

        goal = self._goal(dry_run=True)
        goal.standoff_m = 3.0
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertFalse(handle.accepted)

    def _goal(self, *, dry_run):
        goal = ApproachTarget.Goal()
        goal.target.header.frame_id = 'map'
        goal.target.header.stamp = self.node.get_clock().now().to_msg()
        goal.target.point.x = 2.0
        goal.target.point.z = 1.0
        goal.standoff_m = 0.8
        goal.dry_run = dry_run
        return goal

    def _wait(self, future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        if not future.done():
            self.fail('future timed out')
        return future.result()
