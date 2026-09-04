import os
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState
from xlerobot_interfaces.action import HandoverObject
from xlerobot_interfaces.msg import CapabilityError


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '73'
    server = Node(
        package='xlerobot_manipulation',
        executable='handover_object_server',
        name='handover_object_server',
        parameters=[
            {
                'execution_enabled': True,
                'handover_duration_s': 0.1,
                'release_wait_s': 0.1,
                'gripper_open_duration_s': 0.1,
                'ready_duration_s': 0.1,
                'gripper_close_duration_s': 0.1,
                'service_timeout_s': 2.0,
                'controller_timeout_margin_s': 2.0,
                'joint_state_timeout_s': 1.0,
            }
        ],
        output='screen',
    )
    return LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class FakeHandoverBackends(RclpyNode):
    def __init__(self):
        super().__init__('fake_handover_backends')
        self.group = ReentrantCallbackGroup()
        self.events = []
        self.gripper_position = 0.0
        self.stall = False
        self.child_canceled = threading.Event()
        self.arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory',
            execute_callback=lambda handle: self.follow(handle, 'arm'),
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.group,
        )
        self.gripper_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/right_gripper_controller/follow_joint_trajectory',
            execute_callback=lambda handle: self.follow(handle, 'gripper'),
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.group,
        )
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_state)

    def follow(self, goal_handle, label):
        positions = list(goal_handle.request.trajectory.points[-1].positions)
        self.events.append((label, positions))
        result = FollowJointTrajectory.Result()
        if self.stall:
            deadline = time.monotonic() + 5.0
            while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(0.01)
            if goal_handle.is_cancel_requested:
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.canceled()
                self.child_canceled.set()
                return result
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result

    def publish_state(self):
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = ['right_arm_gripper']
        state.position = [self.gripper_position]
        self.state_pub.publish(state)


class HandoverObjectRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.fake = FakeHandoverBackends()
        cls.client_node = RclpyNode('handover_object_runtime_client')
        cls.client = ActionClient(cls.client_node, HandoverObject, '/handover_object')
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

    def test_01_dry_run_has_no_controller_goals(self):
        self.fake.events.clear()
        wrapped = self._run(self._goal(dry_run=True))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(self.fake.events, [])

    def test_02_live_sequence_uses_forward_pose_and_gripper_values(self):
        self.fake.events.clear()
        wrapped = self._run(self._goal(dry_run=False))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(
            self.fake.events,
            [
                ('arm', [-0.3344, -0.0952, -0.4458, 0.5291, 0.0]),
                ('gripper', [1.35]),
                ('arm', [-0.11505, 1.62142, 1.62602, 0.50621, 0.00153]),
                ('gripper', [0.0]),
            ],
        )

    def test_03_open_gripper_state_is_rejected_before_controller(self):
        self.fake.events.clear()
        self.fake.gripper_position = 0.5
        time.sleep(0.15)
        wrapped = self._run(self._goal(dry_run=False))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.SAFETY_REJECTED)
        self.assertEqual(self.fake.events, [])
        self.fake.gripper_position = 0.0
        time.sleep(0.15)

    def test_04_outer_cancel_propagates_to_controller(self):
        self.fake.events.clear()
        self.fake.stall = True
        self.fake.child_canceled.clear()
        handle = self._wait(self.client.send_goal_async(self._goal(dry_run=False)), 3.0)
        self._wait_for_child()
        canceled = self._wait(handle.cancel_goal_async(), 2.0)
        self.assertEqual(len(canceled.goals_canceling), 1)
        wrapped = self._wait(handle.get_result_async(), 3.0)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.CANCELED)
        self.assertTrue(self.fake.child_canceled.wait(timeout=2.0))
        self.fake.stall = False

    @staticmethod
    def _goal(*, dry_run):
        goal = HandoverObject.Goal()
        goal.object_id = 'camping_lamp'
        goal.recipient_id = 'nearest_person'
        goal.dry_run = dry_run
        return goal

    def _run(self, goal):
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertTrue(handle.accepted)
        return self._wait(handle.get_result_async(), 5.0)

    def _wait_for_child(self):
        deadline = time.monotonic() + 3.0
        while not self.fake.events and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(self.fake.events)

    @staticmethod
    def _wait(future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise AssertionError('future timed out')
        return future.result()
