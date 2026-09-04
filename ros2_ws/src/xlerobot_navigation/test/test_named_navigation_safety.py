import os
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient
from xlerobot_interfaces.action import NavigateToNamedPlace
from xlerobot_interfaces.msg import CapabilityError


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '71'
    server = launch_ros.actions.Node(
        package='xlerobot_navigation',
        executable='named_navigation_server',
        parameters=[{'execution_enabled': False}],
        output='screen',
    )
    return launch.LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class TestNamedNavigationSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_named_navigation_safety')
        cls.client = ActionClient(
            cls.node, NavigateToNamedPlace, 'navigate_to_named_place'
        )
        cls.commands = []
        cls.subscription = cls.node.create_subscription(
            Twist, 'cmd_vel_dock', cls.commands.append, 10
        )
        assert cls.client.wait_for_server(timeout_sec=10.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def send_goal(self, place_id, dry_run):
        goal = NavigateToNamedPlace.Goal(place_id=place_id, dry_run=dry_run)
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)
        self.assertTrue(result_future.done())
        return result_future.result()

    def test_01_dry_run_is_self_contained_and_motionless(self):
        wrapped = self.send_goal('table', True)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertFalse(
            any(abs(msg.linear.x) > 0.0 or abs(msg.angular.z) > 0.0 for msg in self.commands)
        )

    def test_02_execution_requires_independent_enable(self):
        wrapped = self.send_goal('table', False)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.SAFETY_REJECTED)
        self.assertIn('execution_enabled', wrapped.result.error.message)

    def test_03_unknown_place_is_structured_error(self):
        wrapped = self.send_goal('does_not_exist', True)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.INVALID_GOAL)
