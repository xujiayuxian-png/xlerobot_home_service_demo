import os
import threading
import time
import unittest
from pathlib import Path

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
import rclpy
from rclpy.action import (
    ActionClient, ActionServer, CancelResponse, GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as RclpyNode
from xlerobot_interfaces.action import MoveCalibrationPose
from xlerobot_interfaces.msg import CapabilityError


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '75'
    server = Node(
        package='xlerobot_manipulation',
        executable='calibration_pose_server',
        parameters=[{'execution_enabled': True, 'head_pose_file': str(
            Path(__file__).resolve().parents[2] / 'xlerobot_calibration_tools'
            / 'config/head_camera_poses.yaml'
        ), 'handeye_pose_file': str(Path(__file__).resolve().parents[2]
                                  / 'xlerobot_calibration_tools/config/right_handeye_poses.yaml')}],
        output='screen',
    )
    return LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class FakeCalibrationController(RclpyNode):
    def __init__(self):
        super().__init__('fake_calibration_controller')
        self.started = threading.Event()
        self.terminal = threading.Event()
        self.delay_next_goal = False
        self.delayed_execution = False
        self.immediate = False
        self.requests = []
        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory',
            execute_callback=self.execute,
            goal_callback=self.goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.arm_server = ActionServer(
            self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute, goal_callback=self.goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def goal(self, _request):
        if self.delay_next_goal:
            self.delay_next_goal = False
            self.delayed_execution = True
            time.sleep(6.0)
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        self.requests.append(goal_handle.request.trajectory)
        self.started.set()
        if self.delayed_execution or self.immediate:
            self.delayed_execution = False
            time.sleep(0.2)
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            self.terminal.set()
            return result
        while not goal_handle.is_cancel_requested:
            time.sleep(0.01)
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.canceled()
        self.terminal.set()
        return result


class CalibrationPoseRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.fake = FakeCalibrationController()
        cls.client_node = RclpyNode('calibration_pose_runtime_client')
        cls.client = ActionClient(
            cls.client_node, MoveCalibrationPose, '/calibration/move_pose',
        )
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.fake)
        cls.executor.add_node(cls.client_node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        assert cls.client.wait_for_server(timeout_sec=5.0)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown(timeout_sec=3.0)
        cls.thread.join(timeout=3.0)
        cls.client.destroy()
        cls.client_node.destroy_node()
        cls.fake.destroy_node()
        rclpy.shutdown()

    def test_parent_cancel_waits_for_controller_terminal_result(self):
        self.fake.started.clear()
        self.fake.terminal.clear()
        goal = MoveCalibrationPose.Goal()
        goal.workflow_id = MoveCalibrationPose.Goal.HEAD_CAMERA
        goal.pose_index = 0
        goal.dry_run = False
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertTrue(handle.accepted)
        self.assertTrue(self.fake.started.wait(timeout=3.0))

        cancellation = self._wait(handle.cancel_goal_async(), 2.0)
        self.assertEqual(len(cancellation.goals_canceling), 1)
        wrapped = self._wait(handle.get_result_async(), 8.0)

        self.assertTrue(self.fake.terminal.is_set())
        self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.CANCELED)

    def test_handeye_motion_uses_yaml_and_only_fixed_head_then_right_arm(self):
        import yaml
        document = yaml.safe_load((Path(__file__).resolve().parents[2]
                                  / 'xlerobot_calibration_tools/config/right_handeye_poses.yaml').read_text())
        self.fake.immediate = True
        self.fake.requests.clear()
        try:
            handle = self._wait(self.client.send_goal_async(MoveCalibrationPose.Goal(
                workflow_id='right_handeye', pose_index=25, dry_run=False)), 3)
            self.assertTrue(handle.accepted)
            result = self._wait(handle.get_result_async(), 5)
            self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
            self.assertEqual(len(self.fake.requests), 2)
            head, arm = self.fake.requests
            self.assertEqual(head.joint_names, ['head_pan_joint', 'head_tilt_joint'])
            self.assertEqual(list(head.points[0].positions), document['head_pose'])
            self.assertEqual(arm.joint_names, document['joint_order'])
            self.assertEqual(list(arm.points[0].positions), document['poses'][25])
        finally:
            self.fake.immediate = False

    def test_z_unknown_controller_goal_response_latches_motion_owner(self):
        self.fake.started.clear()
        self.fake.terminal.clear()
        self.fake.delay_next_goal = True
        goal = MoveCalibrationPose.Goal()
        goal.workflow_id = MoveCalibrationPose.Goal.HEAD_CAMERA
        goal.pose_index = 0
        goal.dry_run = False
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertTrue(handle.accepted)

        wrapped = self._wait(handle.get_result_async(), 8.0)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            wrapped.result.error.code, CapabilityError.BACKEND_FAILURE,
        )
        self.assertIn('ownership is unconfirmed', wrapped.result.error.message)

        second = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertFalse(second.accepted)
        self.assertTrue(self.fake.started.wait(timeout=3.0))
        self.assertTrue(self.fake.terminal.wait(timeout=3.0))

    @staticmethod
    def _wait(future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise AssertionError('future timed out')
        return future.result()
