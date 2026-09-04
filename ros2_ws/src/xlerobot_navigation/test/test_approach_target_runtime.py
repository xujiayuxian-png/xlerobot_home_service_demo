import os
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as RclpyNode
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from xlerobot_interfaces.action import ApproachTarget
from xlerobot_interfaces.msg import CapabilityError


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '71'
    server = Node(
        package='xlerobot_navigation',
        executable='approach_target_server',
        name='approach_target_server',
        parameters=[
            {
                'execution_enabled': True,
                'server_timeout_s': 2.0,
                'path_timeout_s': 3.0,
                'nav_attempt_timeout_s': 1.0,
                'nav_timeout_s': 5.0,
            }
        ],
        output='screen',
    )
    return LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class FakeApproachBackends(RclpyNode):
    def __init__(self):
        super().__init__('fake_approach_backends')
        self.group = ReentrantCallbackGroup()
        self.events = []
        self.path_goals = []
        self.stall_nav = False
        self.stall_nav_attempts = 0
        self.nav_canceled = threading.Event()
        self.backup_goals = []
        self.path_server = ActionServer(
            self,
            ComputePathToPose,
            '/compute_path_to_pose',
            execute_callback=self.compute_path,
            callback_group=self.group,
        )
        self.nav_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            execute_callback=self.navigate,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.group,
        )
        self.backup_server = ActionServer(
            self,
            BackUp,
            '/backup',
            execute_callback=self.backup,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.group,
        )
        self.tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)

    def compute_path(self, goal_handle):
        self.events.append('path')
        self.path_goals.append(goal_handle.request.goal)
        result = ComputePathToPose.Result()
        result.error_code = ComputePathToPose.Result.NONE
        result.path.header = goal_handle.request.goal.header
        result.path.poses = [goal_handle.request.goal]
        goal_handle.succeed()
        return result

    def navigate(self, goal_handle):
        self.events.append('nav')
        result = NavigateToPose.Result()
        stall = self.stall_nav or self.stall_nav_attempts > 0
        if self.stall_nav_attempts > 0:
            self.stall_nav_attempts -= 1
        if stall:
            deadline = time.monotonic() + 5.0
            while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(0.01)
            if goal_handle.is_cancel_requested:
                result.error_msg = 'fake navigation canceled'
                goal_handle.canceled()
                self.nav_canceled.set()
                return result
        result.error_code = NavigateToPose.Result.NONE
        goal_handle.succeed()
        return result

    def backup(self, goal_handle):
        self.events.append('backup')
        self.backup_goals.append(goal_handle.request)
        result = BackUp.Result()
        result.error_code = BackUp.Result.NONE
        goal_handle.succeed()
        return result


class ApproachTargetRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.fake = FakeApproachBackends()
        cls.client_node = RclpyNode('approach_target_runtime_client')
        cls.client = ActionClient(cls.client_node, ApproachTarget, '/approach_target')
        cls.executor = MultiThreadedExecutor(num_threads=8)
        cls.executor.add_node(cls.fake)
        cls.executor.add_node(cls.client_node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        assert cls.client.wait_for_server(timeout_sec=5.0)
        time.sleep(0.2)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown(timeout_sec=3.0)
        cls.thread.join(timeout=3.0)
        cls.client.destroy()
        cls.client_node.destroy_node()
        cls.fake.destroy_node()
        rclpy.shutdown()

    def test_01_dry_run_plans_standoff_without_navigation(self):
        self.fake.events.clear()
        self.fake.path_goals.clear()
        wrapped = self._run(self._goal(dry_run=True))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(self.fake.events, ['path'])
        self.assertAlmostEqual(self.fake.path_goals[-1].pose.position.x, 1.2)
        self.assertAlmostEqual(self.fake.path_goals[-1].pose.position.y, 0.0)

    def test_02_live_path_then_navigation(self):
        self.fake.events.clear()
        wrapped = self._run(self._goal(dry_run=False))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(self.fake.events, ['path', 'nav'])

    def test_03_outer_cancel_propagates_to_nav2(self):
        self.fake.events.clear()
        self.fake.stall_nav = True
        self.fake.nav_canceled.clear()
        handle = self._wait(self.client.send_goal_async(self._goal(dry_run=False)), 3.0)
        self._wait_for_nav()
        canceled = self._wait(handle.cancel_goal_async(), 2.0)
        self.assertEqual(len(canceled.goals_canceling), 1)
        wrapped = self._wait(handle.get_result_async(), 3.0)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.CANCELED)
        self.assertTrue(self.fake.nav_canceled.wait(timeout=2.0))
        self.fake.stall_nav = False

    def test_04_runtime_timeout_backs_up_once_then_retries_requested_standoff(self):
        self.fake.events.clear()
        self.fake.path_goals.clear()
        self.fake.backup_goals.clear()
        self.fake.stall_nav_attempts = 1
        goal = self._goal(dry_run=False)
        goal.standoff_m = 0.5
        wrapped = self._run(goal)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(self.fake.events, ['path', 'nav', 'backup', 'path', 'nav'])
        self.assertAlmostEqual(self.fake.path_goals[0].pose.position.x, 1.5)
        self.assertAlmostEqual(self.fake.path_goals[1].pose.position.x, 1.5)
        self.assertAlmostEqual(self.fake.backup_goals[0].target.x, -0.2)

    def _goal(self, *, dry_run):
        goal = ApproachTarget.Goal()
        goal.target.header.frame_id = 'map'
        goal.target.header.stamp = self.client_node.get_clock().now().to_msg()
        goal.target.point.x = 2.0
        goal.target.point.z = 1.0
        goal.standoff_m = 0.8
        goal.dry_run = dry_run
        return goal

    def _run(self, goal):
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertTrue(handle.accepted)
        return self._wait(handle.get_result_async(), 5.0)

    def _wait_for_nav(self):
        deadline = time.monotonic() + 3.0
        while 'nav' not in self.fake.events and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertIn('nav', self.fake.events)

    @staticmethod
    def _wait(future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise AssertionError('future timed out')
        return future.result()
