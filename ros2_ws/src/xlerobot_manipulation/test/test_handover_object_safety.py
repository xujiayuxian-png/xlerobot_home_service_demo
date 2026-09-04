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
from xlerobot_interfaces.action import HandoverObject
from xlerobot_interfaces.msg import CapabilityError


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '73'
    server = Node(
        package='xlerobot_manipulation',
        executable='handover_object_server',
        name='handover_object_server',
        parameters=[{'execution_enabled': False}],
        output='screen',
    )
    return LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class HandoverObjectSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('handover_object_safety_client')
        cls.client = ActionClient(cls.node, HandoverObject, '/handover_object')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        assert cls.client.wait_for_server(timeout_sec=5.0)

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        cls.executor.remove_node(cls.node)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_dry_run_is_backend_free_and_live_is_gated(self):
        dry_handle = self._wait(
            self.client.send_goal_async(self._goal(dry_run=True)), 3.0
        )
        self.assertTrue(dry_handle.accepted)
        dry = self._wait(dry_handle.get_result_async(), 3.0)
        self.assertEqual(dry.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(dry.result.error.code, CapabilityError.NONE)

        live_handle = self._wait(
            self.client.send_goal_async(self._goal(dry_run=False)), 3.0
        )
        self.assertTrue(live_handle.accepted)
        live = self._wait(live_handle.get_result_async(), 3.0)
        self.assertEqual(live.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(live.result.error.code, CapabilityError.SAFETY_REJECTED)
        self.assertIn('execution_enabled', live.result.error.message)

    def test_empty_identifiers_are_rejected(self):
        goal = self._goal(dry_run=True)
        goal.object_id = ''
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertFalse(handle.accepted)

    @staticmethod
    def _goal(*, dry_run):
        goal = HandoverObject.Goal()
        goal.object_id = 'camping_lamp'
        goal.recipient_id = 'nearest_person'
        goal.dry_run = dry_run
        return goal

    def _wait(self, future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        if not future.done():
            self.fail('future timed out')
        return future.result()
