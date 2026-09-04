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
from xlerobot_interfaces.action import SetMaintenancePreset
from xlerobot_interfaces.msg import CapabilityError


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '73'
    server = Node(
        package='xlerobot_manipulation',
        executable='maintenance_preset_server',
        parameters=[{'execution_enabled': False}],
        output='screen',
    )
    return LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class MaintenancePresetSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('maintenance_preset_safety_client')
        cls.client = ActionClient(
            cls.node, SetMaintenancePreset, '/maintenance_preset'
        )
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        assert cls.client.wait_for_server(timeout_sec=5.0)

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        cls.executor.remove_node(cls.node)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_dry_run_succeeds_without_controller_and_live_is_gated(self):
        dry = self._execute('arm_ready', dry_run=True)
        self.assertEqual(dry.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(dry.result.error.code, CapabilityError.NONE)

        live = self._execute('head_ready', dry_run=False)
        self.assertEqual(live.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(live.result.error.code, CapabilityError.UNAVAILABLE)
        self.assertIn('disabled', live.result.error.message)

    def test_unknown_preset_is_rejected(self):
        goal = SetMaintenancePreset.Goal()
        goal.preset = 'arbitrary_joint_target'
        goal.dry_run = True
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertFalse(handle.accepted)

    def _execute(self, preset, *, dry_run):
        goal = SetMaintenancePreset.Goal()
        goal.preset = preset
        goal.dry_run = dry_run
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertTrue(handle.accepted)
        return self._wait(handle.get_result_async(), 3.0)

    def _wait(self, future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        if not future.done():
            self.fail('future timed out')
        return future.result()
