import os
import time
import unittest

from action_msgs.msg import GoalStatus
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient
from xlerobot_interfaces.action import GraspObject
from xlerobot_interfaces.msg import CapabilityError


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '73'
    server = Node(
        package='xlerobot_manipulation',
        executable='grasp_object_server',
        name='grasp_object_server',
        parameters=[{'execution_enabled': False}],
        output='screen',
    )
    return LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class GraspObjectSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('grasp_object_safety_test')
        cls.client = ActionClient(cls.node, GraspObject, '/grasp_object')
        assert cls.client.wait_for_server(timeout_sec=5.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_01_live_goal_is_rejected_before_any_backend_or_controller(self):
        handle = self._wait(self.client.send_goal_async(self._goal()), 3.0)
        self.assertTrue(handle.accepted)
        wrapped = self._wait(handle.get_result_async(), 3.0)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            wrapped.result.error.code, CapabilityError.SAFETY_REJECTED
        )
        self.assertIn('execution_enabled', wrapped.result.error.message)
        self.assertEqual(
            self.node.get_publishers_info_by_topic(
                '/right_arm_controller/joint_trajectory'
            ),
            [],
        )
        self.assertEqual(
            self.node.get_publishers_info_by_topic('/head_controller/joint_trajectory'),
            [],
        )

    def test_02_unknown_backend_is_rejected_at_goal_boundary(self):
        goal = self._goal()
        goal.backend = 'direct_controller'
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertFalse(handle.accepted)

    def _goal(self):
        goal = GraspObject.Goal()
        goal.object_id = 'camping_lamp'
        goal.target.header.frame_id = 'base_link'
        goal.target.header.stamp = self.node.get_clock().now().to_msg()
        goal.target.point.x = 0.2
        goal.target.point.y = -0.2
        goal.target.point.z = 0.4
        return goal

    def _wait(self, future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(future.done())
        return future.result()
