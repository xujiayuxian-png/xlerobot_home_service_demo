import os
import unittest

from action_msgs.msg import GoalStatus
import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient
from xlerobot_interfaces.action import AutoLocalize
from xlerobot_interfaces.msg import CapabilityError


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '71'
    server = launch_ros.actions.Node(
        package='xlerobot_navigation',
        executable='auto_localizer_server',
        parameters=[{'execution_enabled': False}],
        output='screen',
    )
    return launch.LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class TestAutoLocalizerSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_auto_localizer_safety')
        cls.client = ActionClient(cls.node, AutoLocalize, 'auto_localize')
        assert cls.client.wait_for_server(timeout_sec=10.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def send_goal(self, dry_run):
        send_future = self.client.send_goal_async(AutoLocalize.Goal(dry_run=dry_run))
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)
        self.assertTrue(result_future.done())
        return result_future.result()

    def test_01_dry_run_needs_no_amcl_or_spin_backend(self):
        wrapped = self.send_goal(True)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)

    def test_02_execution_requires_independent_enable(self):
        wrapped = self.send_goal(False)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.SAFETY_REJECTED)
        self.assertIn('execution_enabled', wrapped.result.error.message)
