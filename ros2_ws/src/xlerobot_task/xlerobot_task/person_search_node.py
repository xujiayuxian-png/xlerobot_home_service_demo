"""Coordinate the verified body/head search around read-only person views."""

from __future__ import annotations

import math
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav2_msgs.action import Spin
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from xlerobot_interfaces.action import ScanForPerson
from xlerobot_interfaces.msg import CapabilityError


class SearchFailure(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = int(code)


class PersonSearchNode(Node):
    """Expose ScanForPerson while standard controllers own all motion."""

    def __init__(self):
        super().__init__('person_search')
        self.group = ReentrantCallbackGroup()
        self.view_action = str(self.declare_parameter(
            'view_action', '/scan_for_person_view'
        ).value)
        self.scan_pan_rad = [float(value) for value in self.declare_parameter(
            'scan_pan_rad', [1.2, 0.0, -1.2]
        ).value]
        self.head_tilt_rad = float(self.declare_parameter('head_tilt_rad', 0.0).value)
        self.body_turns_rad = [float(value) for value in self.declare_parameter(
            'body_turns_rad', [math.pi / 2.0, -math.pi]
        ).value]
        self.retreat_distance_m = float(self.declare_parameter(
            'retreat_distance_m', 0.40
        ).value)
        self.retreat_speed_mps = float(self.declare_parameter(
            'retreat_speed_mps', 0.08
        ).value)
        self.manual_retreat_extra_timeout_s = float(self.declare_parameter(
            'manual_retreat_extra_timeout_s', 2.0
        ).value)
        self.child_cancel_timeout_s = float(self.declare_parameter(
            'child_cancel_timeout_s', 3.0
        ).value)
        self.head_move_s = float(self.declare_parameter('head_move_s', 1.0).value)
        self.settle_s = float(self.declare_parameter('settle_s', 0.4).value)
        if (
            not self.scan_pan_rad or not self.body_turns_rad
            or not all(
                math.isfinite(value)
                for value in (
                    *self.scan_pan_rad,
                    *self.body_turns_rad,
                    self.head_tilt_rad,
                    self.retreat_distance_m,
                    self.retreat_speed_mps,
                    self.manual_retreat_extra_timeout_s,
                    self.child_cancel_timeout_s,
                    self.head_move_s,
                    self.settle_s,
                )
            )
            or self.retreat_distance_m < 0.0 or self.retreat_speed_mps <= 0.0
            or self.manual_retreat_extra_timeout_s < 0.0
            or self.child_cancel_timeout_s <= 0.0
            or self.head_move_s <= 0.0 or self.settle_s < 0.0
        ):
            raise ValueError('person search motion parameters are invalid')

        self.view_client = ActionClient(
            self, ScanForPerson, self.view_action, callback_group=self.group
        )
        self.head_client = ActionClient(
            self, FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory', callback_group=self.group,
        )
        self.spin_client = ActionClient(
            self, Spin, '/spin', callback_group=self.group
        )
        self._odom_lock = threading.Lock()
        self._base_xy = None
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10, callback_group=self.group
        )
        self._lock = threading.Lock()
        self._goal_active = False
        self.readiness_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self.readiness_timer = self.create_timer(
            1.0, self._publish_readiness, callback_group=self.group
        )
        self.server = ActionServer(
            self, ScanForPerson, '/scan_for_person',
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group,
        )
        self.get_logger().info('Person search coordinator ready')

    def goal_callback(self, _request):
        with self._lock:
            if self._goal_active:
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        # The execute loop observes the accepted parent cancellation within
        # 20 ms, cancels the child, and waits for its terminal result before
        # releasing ownership.
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        result = ScanForPerson.Result()
        try:
            if goal_handle.is_cancel_requested:
                raise SearchFailure(
                    CapabilityError.CANCELED, 'person search canceled'
                )
            if goal_handle.request.dry_run:
                result.error.code = CapabilityError.NONE
                result.error.message = 'person search plan valid'
                result.person.header.frame_id = 'map'
                result.distance_m = float('nan')
                if goal_handle.is_cancel_requested:
                    raise SearchFailure(
                        CapabilityError.CANCELED, 'person search canceled'
                    )
                goal_handle.succeed()
                return result

            if self.retreat_distance_m > 0.0:
                self._feedback(goal_handle, 'retreat', 0.05, 'retreating before body scan')
                retreat_start = self._current_base_xy()
                self._manual_retreat(
                    goal_handle, retreat_start, abs(self.retreat_distance_m)
                )

            candidates = []
            total_views = len(self.body_turns_rad) * len(self.scan_pan_rad)
            view_index = 0
            for body_index, turn in enumerate(self.body_turns_rad):
                spin = Spin.Goal()
                spin.target_yaw = turn
                spin.time_allowance.sec = 18
                self._require_success(
                    self.spin_client, spin, 21.0, goal_handle, 'body spin'
                )
                # Serpentine order avoids the old 2.4 rad cross-body head jump
                # between scan sectors while preserving all six views.
                pans = (
                    self.scan_pan_rad
                    if body_index % 2 == 0
                    else list(reversed(self.scan_pan_rad))
                )
                for pan in pans:
                    view_index += 1
                    progress = 0.10 + 0.80 * view_index / total_views
                    self._feedback(
                        goal_handle, 'scan', progress,
                        f'view {view_index}/{total_views}',
                    )
                    self._move_head(pan, goal_handle)
                    self._wait_settle(goal_handle)
                    view_goal = ScanForPerson.Goal()
                    view_goal.recipient_id = goal_handle.request.recipient_id
                    view_goal.dry_run = False
                    wrapped = self._call(
                        self.view_client, view_goal, 40.0, goal_handle, 'person view'
                    )
                    error_code = int(wrapped.result.error.code)
                    if (
                        wrapped.status == GoalStatus.STATUS_SUCCEEDED
                        and error_code == CapabilityError.NONE
                    ):
                        candidates.append(wrapped.result)
                    elif (
                        wrapped.status == GoalStatus.STATUS_ABORTED
                        and error_code == CapabilityError.NOT_FOUND
                    ):
                        continue
                    else:
                        if goal_handle.is_cancel_requested:
                            normalized_code = CapabilityError.CANCELED
                        elif error_code in {
                                CapabilityError.NONE,
                                CapabilityError.CANCELED,
                        }:
                            normalized_code = CapabilityError.BACKEND_FAILURE
                        else:
                            normalized_code = error_code
                        raise SearchFailure(
                            normalized_code,
                            wrapped.result.error.message or (
                                'person view returned inconsistent action status '
                                'and capability result'
                            ),
                        )

            self._move_head(0.0, goal_handle)
            if not candidates:
                raise SearchFailure(CapabilityError.NOT_FOUND, 'no person found during scan')
            nearest = min(candidates, key=lambda item: float(item.distance_m))
            result.person = nearest.person
            result.distance_m = nearest.distance_m
            result.observation = nearest.observation
            self.get_logger().info(
                f'selected nearest person from {len(candidates)} views: '
                f'map=({result.person.point.x:.3f}, '
                f'{result.person.point.y:.3f}, {result.person.point.z:.3f}) '
                f'distance={result.distance_m:.3f}m'
            )
            result.error.code = CapabilityError.NONE
            result.error.message = 'nearest person found across body/head scan'
            self._feedback(goal_handle, 'complete', 1.0, result.error.message)
            if goal_handle.is_cancel_requested:
                raise SearchFailure(
                    CapabilityError.CANCELED, 'person search canceled'
                )
            goal_handle.succeed()
            return result
        except SearchFailure as exc:
            parent_canceled = bool(goal_handle.is_cancel_requested)
            error_code = int(exc.code)
            error_message = str(exc)
            if parent_canceled:
                error_code = CapabilityError.CANCELED
                error_message = 'person search canceled'
            elif error_code == CapabilityError.CANCELED:
                error_code = CapabilityError.BACKEND_FAILURE
                error_message = (
                    'a child capability reported cancellation without a '
                    f'parent cancellation request: {exc}'
                )
            result.error.code = error_code
            result.error.message = error_message
            if parent_canceled:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:  # Defensive conversion at the action boundary.
            if goal_handle.is_cancel_requested:
                result.error.code = CapabilityError.CANCELED
                result.error.message = 'person search canceled'
                goal_handle.canceled()
            else:
                result.error.code = CapabilityError.INTERNAL_ERROR
                result.error.message = f'person search internal error: {exc}'
                goal_handle.abort()
            return result
        finally:
            with self._lock:
                self._goal_active = False

    def _move_head(self, pan, parent):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(pan), self.head_tilt_rad]
        point.time_from_start.sec = int(self.head_move_s)
        point.time_from_start.nanosec = int(
            (self.head_move_s - int(self.head_move_s)) * 1.0e9
        )
        goal.trajectory.points = [point]
        wrapped = self._require_success(
            self.head_client, goal, self.head_move_s + 5.0, parent, 'head scan'
        )
        if wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise SearchFailure(CapabilityError.BACKEND_FAILURE, 'head scan failed')

    def _require_success(self, client, goal, timeout_s, parent, label):
        wrapped = self._call(client, goal, timeout_s, parent, label)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise SearchFailure(CapabilityError.BACKEND_FAILURE, f'{label} failed')
        return wrapped

    def _call(self, client, goal, timeout_s, parent, label):
        deadline = time.monotonic() + timeout_s
        if not self._wait_for_action_server(
            client, min(5.0, max(0.0, deadline - time.monotonic())), parent
        ):
            raise SearchFailure(CapabilityError.UNAVAILABLE, f'{label} unavailable')
        if time.monotonic() >= deadline:
            raise SearchFailure(CapabilityError.TIMEOUT, f'{label} timed out')
        send_future = client.send_goal_async(goal)
        handle = self._wait_goal_response(send_future, deadline, parent, label)
        if not handle.accepted:
            raise SearchFailure(CapabilityError.BACKEND_FAILURE, f'{label} rejected')
        try:
            result_future = handle.get_result_async()
        except Exception as exc:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001 - ownership remains held below
                pass
            self._hold_until_shutdown(
                f'cannot observe {label} terminal result: {exc}'
            )
            raise SearchFailure(
                CapabilityError.BACKEND_FAILURE,
                f'{label} result tracking failed; restart the demo profile',
            ) from exc
        wrapped = self._wait_result(
            handle, result_future, deadline, parent, label
        )
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            if parent.is_cancel_requested:
                raise SearchFailure(
                    CapabilityError.CANCELED, 'person search canceled'
                )
            raise SearchFailure(
                CapabilityError.BACKEND_FAILURE,
                f'{label} was canceled without a parent cancellation request',
            )
        return wrapped

    def _wait_goal_response(self, future, deadline, parent, label):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        while not event.wait(0.02):
            if parent.is_cancel_requested:
                self._cancel_pending_goal_and_wait_terminal(future, label)
                raise SearchFailure(CapabilityError.CANCELED, 'person search canceled')
            if time.monotonic() >= deadline:
                self._cancel_pending_goal_and_wait_terminal(future, label)
                raise SearchFailure(CapabilityError.TIMEOUT, f'{label} timed out')
            if not rclpy.ok():
                raise SearchFailure(
                    CapabilityError.UNAVAILABLE, f'{label} interrupted by shutdown'
                )
        if parent.is_cancel_requested:
            self._cancel_pending_goal_and_wait_terminal(future, label)
            raise SearchFailure(CapabilityError.CANCELED, 'person search canceled')
        if time.monotonic() >= deadline:
            self._cancel_pending_goal_and_wait_terminal(future, label)
            raise SearchFailure(CapabilityError.TIMEOUT, f'{label} timed out')
        try:
            return future.result()
        except Exception as exc:
            self._hold_until_shutdown(
                f'cannot prove whether {label} was accepted: {exc}'
            )
            raise SearchFailure(
                CapabilityError.BACKEND_FAILURE,
                f'{label} goal response failed; restart the demo profile',
            ) from exc

    def _wait_result(self, child, result_future, deadline, parent, label):
        event = threading.Event()
        result_future.add_done_callback(lambda _future: event.set())
        while not event.wait(0.02):
            if parent.is_cancel_requested:
                self._cancel_child_and_wait_terminal(child, result_future)
                raise SearchFailure(CapabilityError.CANCELED, 'person search canceled')
            if time.monotonic() >= deadline:
                self._cancel_child_and_wait_terminal(child, result_future)
                raise SearchFailure(CapabilityError.TIMEOUT, f'{label} timed out')
            if not rclpy.ok():
                self._cancel_child_and_wait_terminal(child, result_future)
                raise SearchFailure(
                    CapabilityError.UNAVAILABLE, f'{label} interrupted by shutdown'
                )
        if parent.is_cancel_requested:
            self._cancel_child_and_wait_terminal(child, result_future)
            raise SearchFailure(CapabilityError.CANCELED, 'person search canceled')
        if time.monotonic() >= deadline:
            self._cancel_child_and_wait_terminal(child, result_future)
            raise SearchFailure(CapabilityError.TIMEOUT, f'{label} timed out')
        try:
            wrapped = result_future.result()
        except Exception as exc:
            self._hold_until_shutdown(
                f'cannot prove {label} terminal result: {exc}'
            )
            raise SearchFailure(
                CapabilityError.BACKEND_FAILURE,
                f'{label} result failed; restart the demo profile',
            ) from exc
        if not self._terminal_result_observed(wrapped):
            self._hold_until_shutdown(
                f'{label} returned non-terminal status '
                f'{getattr(wrapped, "status", "missing")}'
            )
            raise SearchFailure(
                CapabilityError.BACKEND_FAILURE,
                f'{label} returned non-terminal status; restart the demo profile',
            )
        return wrapped

    def _cancel_child_and_wait_terminal(self, child, result_future):
        cancel_future = None
        try:
            cancel_future = child.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(f'Unable to request child cancellation: {exc}')
        deadline = time.monotonic() + self.child_cancel_timeout_s
        while (
            rclpy.ok() and cancel_future is not None and not cancel_future.done()
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        while (
            rclpy.ok() and not result_future.done()
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        if not result_future.done():
            self.get_logger().error(
                'Child action did not reach terminal state within the '
                'cancellation budget; holding the parent action until it does'
            )
        while rclpy.ok() and not result_future.done():
            time.sleep(0.02)
        if result_future.done():
            try:
                wrapped = result_future.result()
            except Exception as exc:
                self._hold_until_shutdown(
                    f'cannot prove child terminal result: {exc}'
                )
                return
            if not self._terminal_result_observed(wrapped):
                self._hold_until_shutdown(
                    f'child returned non-terminal status '
                    f'{getattr(wrapped, "status", "missing")}'
                )

    def _cancel_pending_goal_and_wait_terminal(self, send_future, label):
        deadline = time.monotonic() + self.child_cancel_timeout_s
        while (
            rclpy.ok() and not send_future.done()
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        if not send_future.done():
            self.get_logger().error(
                f'{label} goal response exceeded the cancellation budget; '
                'holding the parent action until acceptance is known'
            )
        while rclpy.ok() and not send_future.done():
            time.sleep(0.02)
        if not send_future.done():
            return
        try:
            child = send_future.result()
        except Exception as exc:
            self._hold_until_shutdown(
                f'cannot prove whether {label} was accepted: {exc}'
            )
            return
        if child is None or not child.accepted:
            return
        try:
            result_future = child.get_result_async()
        except Exception as exc:
            try:
                child.cancel_goal_async()
            except Exception:  # noqa: BLE001 - ownership remains held below
                pass
            self._hold_until_shutdown(
                f'cannot observe {label} terminal result: {exc}'
            )
            return
        self._cancel_child_and_wait_terminal(child, result_future)

    def _hold_until_shutdown(self, message):
        self.get_logger().error(
            f'{message}; holding ownership until profile restart'
        )
        while rclpy.ok():
            time.sleep(0.1)

    @staticmethod
    def _terminal_result_observed(wrapped):
        return getattr(wrapped, 'status', GoalStatus.STATUS_UNKNOWN) in {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }

    @staticmethod
    def _wait_for_action_server(client, timeout_s, parent):
        deadline = time.monotonic() + max(0.0, timeout_s)
        while rclpy.ok():
            if parent.is_cancel_requested:
                raise SearchFailure(
                    CapabilityError.CANCELED, 'person search canceled'
                )
            if client.server_is_ready():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)
        raise SearchFailure(
            CapabilityError.UNAVAILABLE,
            'person search interrupted while waiting for action server',
        )

    def _publish_readiness(self):
        clients = {
            'person_view': self.view_client,
            'head_controller': self.head_client,
            'spin': self.spin_client,
        }
        unavailable = [
            name for name, client in clients.items() if not client.server_is_ready()
        ]
        status = DiagnosticStatus()
        status.name = 'xlerobot/person_search'
        status.hardware_id = 'two_wheel_reference'
        status.level = (
            DiagnosticStatus.WARN if unavailable else DiagnosticStatus.OK
        )
        status.message = (
            'child actions unavailable: ' + ', '.join(unavailable)
            if unavailable
            else 'person search child actions ready'
        )
        for name, client in clients.items():
            status.values.append(KeyValue(
                key=name, value=str(client.server_is_ready()).lower()
            ))
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status.append(status)
        self.readiness_publisher.publish(message)

    def _on_odom(self, message):
        position = message.pose.pose.position
        with self._odom_lock:
            self._base_xy = (float(position.x), float(position.y))

    def _current_base_xy(self):
        with self._odom_lock:
            return self._base_xy

    def _remaining_retreat(self, start_xy):
        current = self._current_base_xy()
        if start_xy is None or current is None:
            return abs(self.retreat_distance_m)
        moved = math.hypot(current[0] - start_xy[0], current[1] - start_xy[1])
        return max(0.0, abs(self.retreat_distance_m) - moved)

    def _manual_retreat(self, parent, original_start_xy, initial_remaining):
        speed = max(0.01, abs(self.retreat_speed_mps))
        deadline = (
            time.monotonic()
            + initial_remaining / speed
            + max(0.0, self.manual_retreat_extra_timeout_s)
        )
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if parent.is_cancel_requested:
                    raise SearchFailure(
                        CapabilityError.CANCELED, 'person search canceled'
                    )
                if (
                    original_start_xy is not None
                    and self._remaining_retreat(original_start_xy) <= 0.02
                ):
                    return
                command = Twist()
                command.linear.x = -speed
                self.cmd_vel_pub.publish(command)
                time.sleep(0.05)
        finally:
            self.cmd_vel_pub.publish(Twist())

    def _wait_settle(self, parent):
        deadline = time.monotonic() + self.settle_s
        while time.monotonic() < deadline:
            if parent.is_cancel_requested:
                raise SearchFailure(CapabilityError.CANCELED, 'person search canceled')
            time.sleep(0.02)

    @staticmethod
    def _feedback(goal_handle, phase, progress, message):
        feedback = ScanForPerson.Feedback()
        feedback.state.phase = phase
        feedback.state.progress = float(progress)
        feedback.state.message = message
        goal_handle.publish_feedback(feedback)


def main():
    rclpy.init()
    node = PersonSearchNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
