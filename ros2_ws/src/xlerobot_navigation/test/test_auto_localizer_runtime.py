import os
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import launch
import launch_ros.actions
import launch_testing.actions
from nav2_msgs.action import Spin
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty, Trigger
from xlerobot_interfaces.action import AutoLocalize
from xlerobot_interfaces.msg import CapabilityError


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '71'
    server = launch_ros.actions.Node(
        package='xlerobot_navigation',
        executable='auto_localizer_server',
        parameters=[
            {
                'execution_enabled': True,
                'activate_navigation_after_localization': True,
                'max_revolutions': 0.2,
                'max_duration_s': 3.0,
                'service_timeout_s': 1.0,
                'server_timeout_s': 1.0,
                'control_rate_hz': 20.0,
                'settle_timeout_s': 0.5,
                'settle_hold_s': 0.05,
                'convergence.min_rotation_rad': 0.30,
                'convergence.position_stddev_m': 0.15,
                'convergence.yaw_stddev_rad': 0.10,
                'convergence.hold_s': 0.10,
            }
        ],
        output='screen',
    )
    return launch.LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class FakeLocalizationBackend(rclpy.node.Node):
    def __init__(self):
        super().__init__('fake_localization_backend')
        self._lock = threading.Lock()
        self.events = []
        self.converge = True
        self.single_good_pose = False
        self.publish_odom = True
        self.reject_spin_goals = 0
        self.navigation_active = False
        self.navigation_startup_delay_s = 1.1
        self._pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'amcl_pose', 10
        )
        self._odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.create_service(
            Empty, '/reinitialize_global_localization', self._scatter
        )
        self.create_service(
            Trigger,
            '/lifecycle_manager_navigation/is_active',
            self._navigation_is_active,
        )
        self.create_service(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes',
            self._manage_navigation,
        )
        self._spin_server = ActionServer(
            self,
            Spin,
            'spin',
            execute_callback=self._execute_spin,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            goal_callback=self._accept_spin,
        )
        self.create_timer(0.02, self._publish_stopped_odom)

    def _scatter(self, _request, response):
        with self._lock:
            self.events.append('scatter')
        return response

    def _navigation_is_active(self, _request, response):
        with self._lock:
            response.success = self.navigation_active
        return response

    def _manage_navigation(self, request, response):
        time.sleep(self.navigation_startup_delay_s)
        with self._lock:
            if request.command == ManageLifecycleNodes.Request.STARTUP:
                self.navigation_active = True
                self.events.append('navigation_startup')
                response.success = True
            else:
                response.success = False
        return response

    def _accept_spin(self, _goal):
        with self._lock:
            if self.reject_spin_goals > 0:
                self.reject_spin_goals -= 1
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_spin(self, goal_handle):
        with self._lock:
            self.events.append('spin')
        rotated = 0.0
        target = abs(goal_handle.request.target_yaw)
        while rotated < target:
            if goal_handle.is_cancel_requested:
                with self._lock:
                    self.events.append('spin_canceled')
                goal_handle.canceled()
                return Spin.Result(error_code=Spin.Result.NONE)
            rotated += 0.08
            feedback = Spin.Feedback()
            feedback.angular_distance_traveled = rotated
            goal_handle.publish_feedback(feedback)
            with self._lock:
                single_good_pose = self.single_good_pose
            if not single_good_pose or rotated <= 0.08:
                self._publish_pose(rotated, force_converged=single_good_pose)
            time.sleep(0.02)
        goal_handle.succeed()
        return Spin.Result(error_code=Spin.Result.NONE)

    def _publish_pose(self, rotated, force_converged=False):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = 0.4
        pose.pose.pose.position.y = -0.2
        covariance = [0.0] * 36
        with self._lock:
            converged = force_converged or (self.converge and rotated >= 0.35)
        if converged:
            covariance[0] = 0.0025
            covariance[7] = 0.0025
            covariance[35] = 0.0025
        else:
            covariance[0] = 0.25
            covariance[7] = 0.25
            covariance[35] = 0.25
        pose.pose.covariance = covariance
        self._pose_publisher.publish(pose)

    def _publish_stopped_odom(self):
        with self._lock:
            publish_odom = self.publish_odom
        if not publish_odom:
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.twist.twist.angular.z = 0.0
        self._odom_publisher.publish(odom)

    def reset(
        self,
        converge,
        publish_odom=True,
        single_good_pose=False,
        reject_spin_goals=0,
    ):
        with self._lock:
            self.events.clear()
            self.converge = converge
            self.publish_odom = publish_odom
            self.single_good_pose = single_good_pose
            self.reject_spin_goals = reject_spin_goals
            self.navigation_active = False

    def event_snapshot(self):
        with self._lock:
            return list(self.events)


class TestAutoLocalizerRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.backend = FakeLocalizationBackend()
        cls.client_node = rclpy.create_node('test_auto_localizer_runtime')
        cls.client = ActionClient(cls.client_node, AutoLocalize, 'auto_localize')
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.backend)
        cls.executor.add_node(cls.client_node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()
        assert cls.client.wait_for_server(timeout_sec=10.0)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown(timeout_sec=2.0)
        cls.spin_thread.join(timeout=2.0)
        cls.client_node.destroy_node()
        cls.backend.destroy_node()
        rclpy.shutdown()

    def accepted_goal(self):
        send_future = self.client.send_goal_async(AutoLocalize.Goal(dry_run=False))
        self.assertTrue(wait_done(send_future, 5.0))
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)
        return goal_handle

    def test_01_converges_and_stops_nav2_spin(self):
        self.backend.reset(converge=True)
        goal_handle = self.accepted_goal()
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 8.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertLess(wrapped.result.position_stddev_m, 0.15)
        self.assertLess(wrapped.result.yaw_stddev_rad, 0.10)
        self.assertEqual(
            self.backend.event_snapshot(),
            ['scatter', 'spin', 'spin_canceled', 'navigation_startup'],
        )

    def test_02_missing_fresh_odom_prevents_success(self):
        self.backend.reset(converge=True, publish_odom=False)
        goal_handle = self.accepted_goal()
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 4.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.TIMEOUT)
        self.assertIn('stable stop', wrapped.result.error.message)

    def test_03_outer_cancel_propagates_to_nav2_spin(self):
        self.backend.reset(converge=False)
        goal_handle = self.accepted_goal()
        time.sleep(0.15)
        cancel_future = goal_handle.cancel_goal_async()
        self.assertTrue(wait_done(cancel_future, 2.0))
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 3.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.CANCELED)
        self.assertEqual(
            self.backend.event_snapshot(),
            ['scatter', 'spin', 'spin_canceled'],
        )

    def test_04_single_stale_good_pose_cannot_converge(self):
        self.backend.reset(converge=False, single_good_pose=True)
        goal_handle = self.accepted_goal()
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 4.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.BACKEND_FAILURE)
        self.assertIn('without convergence', wrapped.result.error.message)

    def test_05_retries_spin_rejected_during_lifecycle_bringup(self):
        self.backend.reset(converge=True, reject_spin_goals=1)
        goal_handle = self.accepted_goal()
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 8.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(
            self.backend.event_snapshot(),
            ['scatter', 'spin', 'spin_canceled', 'navigation_startup'],
        )


def wait_done(future, timeout_s):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.done()
