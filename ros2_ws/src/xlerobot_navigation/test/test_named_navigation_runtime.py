import math
import os
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Twist
import launch
import launch_ros.actions
import launch_testing.actions
from nav2_msgs.action import BackUp, NavigateToPose, Spin
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster
from xlerobot_interfaces.action import NavigateToNamedPlace
from xlerobot_interfaces.msg import CapabilityError


TARGET_X = 0.20
TARGET_Y = 0.10
TARGET_YAW = 0.50


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '71'
    server = launch_ros.actions.Node(
        package='xlerobot_navigation',
        executable='named_navigation_server',
        parameters=[
            {
                'execution_enabled': True,
                'server_timeout_s': 1.0,
                'nav_timeout_s': 1.5,
                'nav_attempt_timeout_s': 0.25,
                'nav_cancel_timeout_s': 0.4,
                'backup_distance_m': 0.20,
                'backup_speed_mps': 0.08,
                'backup_timeout_s': 0.4,
                'already_reached_position_tolerance_m': 0.08,
                'already_reached_yaw_tolerance_rad': 0.08,
                'spin_timeout_s': 2.0,
                'tf_timeout_s': 0.5,
                'dock_timeout_s': 5.0,
                'dock_control_rate_hz': 20.0,
                'stop_publish_count': 2,
                'place_ids': ['table'],
                'places.table.frame_id': 'map',
                'places.table.x': TARGET_X,
                'places.table.y': TARGET_Y,
                'places.table.yaw': TARGET_YAW,
                'places.table.nav_offset_m': 0.05,
                'places.table.dock': True,
                'dock.yaw_align_stable_s': 0.05,
                'dock.max_linear_speed_mps': 0.10,
                'dock.linear_kp': 2.0,
            }
        ],
        output='screen',
    )
    return launch.LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class FakeNavigationBackend(rclpy.node.Node):
    def __init__(self):
        super().__init__('fake_navigation_backend')
        self._lock = threading.Lock()
        self.pose = [0.0, 0.0, 0.0]
        self.events = []
        self.nav_delay_s = 0.0
        self.nav_delays = []
        self.spin_lateral_offsets = []
        self.backup_collision = False
        self._dock_seen = False
        self._tf = TransformBroadcaster(self)
        self._nav_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self._execute_nav,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self._spin_server = ActionServer(
            self,
            Spin,
            'spin',
            execute_callback=self._execute_spin,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self._backup_server = ActionServer(
            self,
            BackUp,
            'backup',
            execute_callback=self._execute_backup,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.create_subscription(Twist, 'cmd_vel_dock', self._on_dock_command, 10)
        self.create_timer(0.02, self._broadcast_tf)

    def _execute_nav(self, goal_handle):
        with self._lock:
            self.events.append('navigate')
            delay_s = (
                self.nav_delays.pop(0) if self.nav_delays else self.nav_delay_s
            )
        deadline = time.monotonic() + delay_s
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.01)
        request = goal_handle.request.pose.pose
        with self._lock:
            self.pose[0] = request.position.x
            self.pose[1] = request.position.y
        goal_handle.succeed()
        return NavigateToPose.Result(error_code=NavigateToPose.Result.NONE)

    def _execute_backup(self, goal_handle):
        with self._lock:
            self.events.append('backup')
            if self.backup_collision:
                result = BackUp.Result(error_code=BackUp.Result.COLLISION_AHEAD)
                result.error_msg = 'fake obstacle behind robot'
                goal_handle.abort()
                return result
            distance = float(goal_handle.request.target.x)
            self.pose[0] -= abs(distance) * math.cos(self.pose[2])
            self.pose[1] -= abs(distance) * math.sin(self.pose[2])
        goal_handle.succeed()
        return BackUp.Result(error_code=BackUp.Result.NONE)

    def _execute_spin(self, goal_handle):
        with self._lock:
            self.events.append('spin')
            self.pose[2] = normalize_angle(
                self.pose[2] + goal_handle.request.target_yaw
            )
            lateral_offset = (
                self.spin_lateral_offsets.pop(0)
                if self.spin_lateral_offsets else 0.0
            )
            self.pose[0] += lateral_offset * math.sin(self.pose[2])
            self.pose[1] -= lateral_offset * math.cos(self.pose[2])
        goal_handle.succeed()
        return Spin.Result(error_code=Spin.Result.NONE)

    def _on_dock_command(self, command):
        if abs(command.linear.x) <= 1.0e-12 and abs(command.angular.z) <= 1.0e-12:
            return
        with self._lock:
            if not self._dock_seen:
                self.events.append('dock')
                self._dock_seen = True
            dt_s = 0.05
            self.pose[2] = normalize_angle(self.pose[2] + command.angular.z * dt_s)
            self.pose[0] += command.linear.x * math.cos(self.pose[2]) * dt_s
            self.pose[1] += command.linear.x * math.sin(self.pose[2]) * dt_s

    def _broadcast_tf(self):
        with self._lock:
            x, y, yaw = self.pose
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf.sendTransform(transform)

    def reset_events(self, pose=None):
        with self._lock:
            self.pose = list(pose or [0.0, 0.0, 0.0])
            self.events.clear()
            self._dock_seen = False
            self.nav_delays = []
            self.spin_lateral_offsets = []
            self.backup_collision = False
        # Let the 50 Hz broadcaster replace the previous test's TF before the
        # server evaluates whether Spin and dock can be skipped.
        time.sleep(0.1)

    def event_snapshot(self):
        with self._lock:
            return list(self.events)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TestNamedNavigationRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.backend = FakeNavigationBackend()
        cls.client_node = rclpy.create_node('test_named_navigation_runtime')
        cls.client = ActionClient(
            cls.client_node, NavigateToNamedPlace, 'navigate_to_named_place'
        )
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.backend)
        cls.executor.add_node(cls.client_node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()
        assert cls.client.wait_for_server(timeout_sec=10.0)
        time.sleep(0.3)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown(timeout_sec=2.0)
        cls.spin_thread.join(timeout=2.0)
        cls.client_node.destroy_node()
        cls.backend.destroy_node()
        rclpy.shutdown()

    def accepted_goal(self, *, skip_if_already_reached=False):
        goal = NavigateToNamedPlace.Goal(
            place_id='table',
            dry_run=False,
            skip_if_already_reached=skip_if_already_reached,
        )
        send_future = self.client.send_goal_async(goal)
        self.assertTrue(wait_done(send_future, 5.0))
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)
        return goal_handle

    def test_01_nav_timeout_backs_up_once_then_reports_timeout(self):
        self.backend.reset_events()
        self.backend.nav_delay_s = 2.0
        goal_handle = self.accepted_goal()
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 4.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.TIMEOUT)
        self.assertEqual(
            self.backend.event_snapshot(),
            ['navigate', 'backup', 'navigate'],
        )

    def test_02_outer_cancel_propagates(self):
        self.backend.reset_events()
        self.backend.nav_delay_s = 2.0
        goal_handle = self.accepted_goal()
        time.sleep(0.1)
        cancel_future = goal_handle.cancel_goal_async()
        self.assertTrue(wait_done(cancel_future, 2.0))
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 2.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.CANCELED)

    def test_03_stalled_predock_backs_up_replans_and_continues(self):
        self.backend.reset_events()
        self.backend.nav_delay_s = 0.0
        self.backend.nav_delays = [2.0, 0.0]
        goal_handle = self.accepted_goal()
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 10.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(
            self.backend.event_snapshot(),
            ['navigate', 'backup', 'navigate', 'spin', 'dock'],
        )

    def test_04_full_three_stage_sequence(self):
        self.backend.reset_events()
        self.backend.nav_delay_s = 0.0
        goal_handle = self.accepted_goal()
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 10.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(self.backend.event_snapshot(), ['navigate', 'spin', 'dock'])
        with self.backend._lock:
            x, y, yaw = self.backend.pose
        self.assertLessEqual(abs(x - TARGET_X), 0.012)
        self.assertLessEqual(abs(y - TARGET_Y), 0.082)
        self.assertLessEqual(abs(normalize_angle(yaw - TARGET_YAW)), 0.025)

    def test_04a_post_spin_lateral_error_gets_one_predock_correction(self):
        self.backend.reset_events()
        self.backend.spin_lateral_offsets = [0.10]
        goal_handle = self.accepted_goal()
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 10.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(
            self.backend.event_snapshot(),
            ['navigate', 'spin', 'navigate', 'dock'],
        )

    def test_05_trusted_goal_already_at_table_skips_all_motion(self):
        self.backend.reset_events([TARGET_X, TARGET_Y, TARGET_YAW])
        goal_handle = self.accepted_goal(skip_if_already_reached=True)
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 3.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(self.backend.event_snapshot(), [])

    def test_06_collision_rejected_backup_aborts_without_blind_forward_motion(self):
        self.backend.reset_events()
        self.backend.backup_collision = True
        self.backend.nav_delays = [2.0]
        goal_handle = self.accepted_goal()
        result_future = goal_handle.get_result_async()
        self.assertTrue(wait_done(result_future, 10.0))
        wrapped = result_future.result()
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.SAFETY_REJECTED)
        self.assertEqual(
            self.backend.event_snapshot(),
            ['navigate', 'backup'],
        )


def wait_done(future, timeout_s):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.done()
