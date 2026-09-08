import os
import time
import unittest
from pathlib import Path

from action_msgs.msg import GoalStatus
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from xlerobot_interfaces.action import MoveCalibrationPose
from xlerobot_interfaces.msg import CapabilityError


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '74'
    server = Node(
        package='xlerobot_manipulation',
        executable='calibration_pose_server',
        parameters=[{'execution_enabled': False, 'head_pose_file': str(
            Path(__file__).resolve().parents[2] / 'xlerobot_calibration_tools'
            / 'config/head_camera_poses.yaml'
        ), 'handeye_pose_file': str(Path(__file__).resolve().parents[2]
                                  / 'xlerobot_calibration_tools/config/right_handeye_poses.yaml')}],
        output='screen',
    )
    return LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class CalibrationPoseSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('calibration_pose_safety_client')
        cls.client = ActionClient(
            cls.node, MoveCalibrationPose, '/calibration/move_pose'
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

    def test_dry_run_succeeds_and_live_motion_is_gated(self):
        dry = self._execute('head_camera', 24, dry_run=True)
        self.assertEqual(dry.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(dry.result.error.code, CapabilityError.NONE)
        self.assertEqual(dry.result.pose_count, 25)
        heldout = self._execute('right_handeye', 25, dry_run=True)
        self.assertEqual(heldout.result.pose_count, 26)
        self.assertEqual(heldout.status, GoalStatus.STATUS_SUCCEEDED)

        live = self._execute('right_handeye', 19, dry_run=False)
        self.assertEqual(live.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(live.result.error.code, CapabilityError.UNAVAILABLE)
        self.assertIn('disabled', live.result.error.message)

    def test_unknown_workflow_and_out_of_range_pose_are_rejected(self):
        self.assertFalse(self._send('arbitrary', 0, dry_run=True).accepted)
        self.assertFalse(self._send('head_camera', 25, dry_run=True).accepted)
        self.assertFalse(self._send('right_handeye', 26, dry_run=True).accepted)

    def _send(self, workflow, pose_index, *, dry_run):
        goal = MoveCalibrationPose.Goal()
        goal.workflow_id = workflow
        goal.pose_index = pose_index
        goal.dry_run = dry_run
        return self._wait(self.client.send_goal_async(goal), 3.0)

    def _execute(self, workflow, pose_index, *, dry_run):
        handle = self._send(workflow, pose_index, dry_run=dry_run)
        self.assertTrue(handle.accepted)
        return self._wait(handle.get_result_async(), 3.0)

    def _wait(self, future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        if not future.done():
            self.fail('future timed out')
        return future.result()
