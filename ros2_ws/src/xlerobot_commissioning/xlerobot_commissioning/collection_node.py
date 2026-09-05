"""CollectEpisode state machine; the browser never assembles motor steps."""

import math
import re
import threading
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

from xlerobot_interfaces.action import (
    CollectEpisode, DetectObject, PrepareGrasp, RecordEpisode,
)
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_interfaces.srv import BeginEpisode, FinalizeEpisode, MarkEpisodeEvent


PHASES = (
    'PREFLIGHT', 'DETECT_OBJECT', 'RECORDER_ADMISSION',
    'PREPARE_PREGRASP', 'ALIGN_LEADER', 'WAITING_HOME', 'STARTING_RECORDING', 'RECORDING',
    'STOPPING_TELEOP', 'FINALIZING', 'REVIEW',
)
TARGET_JOINTS = (
    'right_arm_shoulder_pan', 'right_arm_shoulder_lift',
    'right_arm_elbow_flex', 'right_arm_wrist_flex',
    'right_arm_wrist_roll', 'right_arm_gripper',
)
LEADER_JOINTS = (
    'leader_shoulder_pan', 'leader_shoulder_lift', 'leader_elbow_flex',
    'leader_wrist_flex', 'leader_wrist_roll', 'leader_gripper',
)
IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$')
COLLECTION_PROFILE_ID = 'two_wheel_pick'
MAX_COLLECTION_DURATION_S = 120.0
SERVICE_DISCOVERY_TIMEOUT_S = 2.0
SERVICE_RESPONSE_TIMEOUT_S = 5.0
MAX_RECORDER_FIRST_SAMPLE_TIMEOUT_S = 5.0
RECORDER_SCHEDULING_MARGIN_S = 5.0
# Once Mark(teleop_enabled) starts the recorder clock, the coordinator may
# still consume the remainder of that response, the first-pair barrier, and
# four bounded service calls: teleop on, teleop off, Mark(disabled), Finalize.
RECORDER_FAILSAFE_GRACE_S = (
    SERVICE_RESPONSE_TIMEOUT_S
    + MAX_RECORDER_FIRST_SAMPLE_TIMEOUT_S
    + 4.0 * (SERVICE_DISCOVERY_TIMEOUT_S + SERVICE_RESPONSE_TIMEOUT_S)
    + RECORDER_SCHEDULING_MARGIN_S
)
ALIGNMENT_TOLERANCE_RAD = 0.15
ALIGNMENT_SETTLE_TIMEOUT_S = 2.0
TERMINAL_ACTION_STATUSES = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
}


class CollectionCanceled(RuntimeError):
    """An accepted parent cancellation observed by the coordinator."""


def _wait(future, timeout: float):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    if not future.done():
        raise TimeoutError('ROS request timed out')
    return future.result()


class CollectionNode(Node):
    def __init__(self):
        super().__init__('collect_episode')
        group = ReentrantCallbackGroup()
        self.detect = ActionClient(self, DetectObject, '/detect_object', callback_group=group)
        self.prepare = ActionClient(self, PrepareGrasp, '/prepare_grasp', callback_group=group)
        self.recorder = ActionClient(self, RecordEpisode, '/record_episode', callback_group=group)
        self.align = ActionClient(
            self, FollowJointTrajectory,
            '/leader/leader_arm_controller/follow_joint_trajectory',
            callback_group=group,
        )
        self.teleop = self.create_client(
            SetBool, '/leader_follower_teleop/set_enabled', callback_group=group
        )
        self.torque = self.create_client(
            SetBool, '/leader_bus/set_torque_enabled', callback_group=group
        )
        self.finalize = self.create_client(
            FinalizeEpisode, '/record_episode/finalize', callback_group=group
        )
        self.episode_event = self.create_client(
            MarkEpisodeEvent, '/record_episode/mark_event', callback_group=group
        )
        self._session_lock = threading.Lock()
        self._goal_active = False
        self._active_dataset_id = ''
        self._active_episode_id = ''
        self._phase = ''
        self._finish_requested = threading.Event()
        self._begin_requested = threading.Event()
        self._cleanup_blocked = False
        self._joint_state_timeout_s = float(
            self.declare_parameter('joint_state_timeout_s', 0.5).value
        )
        self._control_enable_lease_s = float(
            self.declare_parameter('control_enable_lease_s', 1.0).value
        )
        self._control_heartbeat_period_s = float(
            self.declare_parameter('control_heartbeat_period_s', 0.2).value
        )
        self._recorder_first_sample_timeout_s = float(
            self.declare_parameter(
                'recorder_first_sample_timeout_s', 2.0
            ).value
        )
        if (
            not math.isfinite(self._joint_state_timeout_s)
            or self._joint_state_timeout_s <= 0.0
        ):
            raise ValueError('joint_state_timeout_s must be finite and positive')
        if (
            not math.isfinite(self._control_enable_lease_s)
            or self._control_enable_lease_s <= 0.0
            or self._control_enable_lease_s > 2.0
        ):
            raise ValueError(
                'control_enable_lease_s must be finite, positive, and no '
                'greater than 2 seconds'
            )
        if (
            not math.isfinite(self._control_heartbeat_period_s)
            or self._control_heartbeat_period_s <= 0.0
            or 3.0 * self._control_heartbeat_period_s
            > self._control_enable_lease_s
        ):
            raise ValueError(
                'control_heartbeat_period_s must be finite, positive, and at '
                'most one third of the control lease'
            )
        if (
            not math.isfinite(self._recorder_first_sample_timeout_s)
            or self._recorder_first_sample_timeout_s <= 0.0
            or self._recorder_first_sample_timeout_s
            > MAX_RECORDER_FIRST_SAMPLE_TIMEOUT_S
        ):
            raise ValueError(
                'recorder_first_sample_timeout_s must be finite, positive, '
                'and no greater than 5 seconds'
            )
        self.create_service(
            FinalizeEpisode,
            '/collect_episode/finalize',
            self.request_finalize,
            callback_group=group,
        )
        self.create_service(BeginEpisode, '/collect_episode/begin', self.request_begin,
                            callback_group=group)
        self._states = {'leader': {}, 'follower': {}}
        self._state_received_at = {'leader': 0.0, 'follower': 0.0}
        self._state_lock = threading.Lock()
        self._leader_limits = {}
        self._leader_controller_owned = False
        self._holding_leader = False
        self.leader_controllers = self.create_client(
            ListControllers, '/leader/controller_manager/list_controllers', callback_group=group)
        self.follower_controllers = self.create_client(
            ListControllers, '/controller_manager/list_controllers', callback_group=group)
        self.leader_switch = self.create_client(
            SwitchController, '/leader/controller_manager/switch_controller', callback_group=group)
        self.create_subscription(
            String, '/leader/robot_description', self._leader_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=group)
        self.create_subscription(
            JointState, '/leader/joint_states',
            lambda msg: self._state('leader', msg), 10, callback_group=group,
        )
        self.create_subscription(
            JointState, '/joint_states',
            lambda msg: self._state('follower', msg), 10, callback_group=group,
        )
        self.server = ActionServer(
            self, CollectEpisode, '/collect_episode', execute_callback=self.execute,
            goal_callback=self.goal, cancel_callback=self.cancel,
            callback_group=group,
        )

    def goal(self, goal):
        if goal.template_id not in {'pick', 'manual'}:
            return GoalResponse.REJECT
        if (
            not IDENTIFIER.fullmatch(goal.dataset_id)
            or not IDENTIFIER.fullmatch(goal.episode_id)
            or goal.collection_profile_id != COLLECTION_PROFILE_ID
            or not goal.language_instruction.strip()
        ):
            return GoalResponse.REJECT
        if goal.template_id == 'pick' and not goal.object_id.strip():
            return GoalResponse.REJECT
        if (
            not math.isfinite(goal.max_duration_s)
            or goal.max_duration_s <= 0.0
            or goal.max_duration_s > MAX_COLLECTION_DURATION_S
        ):
            return GoalResponse.REJECT
        with self._session_lock:
            if self._goal_active or self._cleanup_blocked:
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def cancel(self, _goal_handle):
        with self._session_lock:
            if (
                not self._goal_active
                or self._phase in {'STOPPING_TELEOP', 'FINALIZING', 'REVIEW'}
            ):
                return CancelResponse.REJECT
        return CancelResponse.ACCEPT

    def request_begin(self, request, response):
        with self._session_lock:
            if (not self._goal_active or request.dataset_id != self._active_dataset_id
                    or request.episode_id != self._active_episode_id):
                response.error.code = CapabilityError.NOT_FOUND
                response.error.message = 'prepared collection episode was not found'
            elif self._phase == 'RECORDING' and self._begin_requested.is_set():
                response.error.code = CapabilityError.NONE
                response.error.message = 'recording already started'
            elif self._phase != 'WAITING_HOME':
                response.error.code = CapabilityError.INVALID_GOAL
                response.error.message = f'Home is unavailable during {self._phase}'
            else:
                self._begin_requested.set()
                response.error.code = CapabilityError.NONE
                response.error.message = 'Home accepted; beginning recording'
        return response

    def request_finalize(self, request, response):
        """Request a graceful end without bypassing coordinator stop ordering."""
        with self._session_lock:
            if (
                not self._goal_active
                or not self._active_dataset_id
                or not self._active_episode_id
                or request.dataset_id != self._active_dataset_id
                or request.episode_id != self._active_episode_id
            ):
                response.error.code = CapabilityError.NOT_FOUND
                response.error.message = 'active collection episode was not found'
                return response
            if self._finish_requested.is_set():
                response.error.code = CapabilityError.NONE
                response.error.message = 'graceful finalization already requested'
                return response
            if self._phase in {'STOPPING_TELEOP', 'FINALIZING', 'REVIEW'}:
                response.error.code = CapabilityError.NONE
                response.error.message = 'graceful finalization is already in progress'
                return response
            if self._phase != 'RECORDING':
                response.error.code = CapabilityError.INVALID_GOAL
                response.error.message = (
                    f'episode cannot end normally during {self._phase or "startup"}'
                )
                return response
            self._finish_requested.set()
        response.error.code = CapabilityError.NONE
        response.error.message = 'graceful finalization requested'
        return response

    def _state(self, which: str, message: JointState):
        if len(message.name) != len(message.position):
            return
        values = {
            name: float(value) for name, value in zip(message.name, message.position)
            if math.isfinite(value)
        }
        with self._state_lock:
            self._states[which] = values
            self._state_received_at[which] = time.monotonic()

    def _set_phase(self, phase: str) -> None:
        with self._session_lock:
            if self._goal_active:
                self._phase = phase

    def _feedback(self, handle, phase: str, progress: float, message: str):
        self._set_phase(phase)
        feedback = CollectEpisode.Feedback()
        feedback.state.phase = phase
        feedback.state.progress = progress
        feedback.state.message = message
        feedback.frame_count = getattr(self, '_recorded_frames', 0)
        feedback.elapsed_s = getattr(self, '_recorded_elapsed', 0.0)
        handle.publish_feedback(feedback)

    def _service(
        self, client, request, timeout=SERVICE_RESPONSE_TIMEOUT_S
    ):
        if not client.wait_for_service(
            timeout_sec=SERVICE_DISCOVERY_TIMEOUT_S
        ):
            raise RuntimeError('required collection service is unavailable')
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        next_refresh = time.monotonic() + self._control_heartbeat_period_s
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('ROS request timed out')
            if (self._holding_leader and client is not self.torque
                    and time.monotonic() >= next_refresh):
                self._set_torque(True)
                next_refresh = time.monotonic() + self._control_heartbeat_period_s
            time.sleep(0.02)
        return future.result()

    def _leader_description(self, message):
        try:
            root = ET.fromstring(message.data)
            limits = {}
            for name in LEADER_JOINTS:
                joint = root.find(f"./joint[@name='{name}']/limit")
                low, high = float(joint.attrib['lower']), float(joint.attrib['upper'])
                if not math.isfinite(low) or not math.isfinite(high) or low >= high:
                    raise ValueError('invalid bounds')
                limits[name] = (low, high)
        except (ET.ParseError, AttributeError, KeyError, ValueError):
            limits = {}
        with self._state_lock:
            self._leader_limits = limits

    def _validate_leader_positions(self, target=None):
        with self._state_lock:
            limits = dict(self._leader_limits)
            state = dict(self._states['leader'])
            received = self._state_received_at['leader']
        if len(limits) != len(LEADER_JOINTS):
            raise RuntimeError('Leader description/physical limits are unavailable')
        if received <= 0 or time.monotonic() - received > self._joint_state_timeout_s:
            raise RuntimeError('Leader measured state is stale; preparation not started')
        if target is not None and len(target) != len(LEADER_JOINTS):
            raise RuntimeError('Leader target must contain six joints')
        for index, name in enumerate(LEADER_JOINTS):
            low, high = limits[name]
            values = [('measured', state.get(name))]
            if target is not None:
                values.append(('target', target[index]))
            for label, value in values:
                if value is None or not math.isfinite(value) or not low <= value <= high:
                    raise RuntimeError(
                        f'Leader {name} {label}={value} outside [{low:.3f}, {high:.3f}] rad; '
                        'position control remains unavailable until pose/target is in range')

    def _controller_states(self, client):
        return {c.name: c.state for c in self._service(client, ListControllers.Request()).controller}

    def _check_controllers(self):
        follower = self._controller_states(self.follower_controllers)
        for name in ('right_arm_controller', 'right_gripper_controller', 'head_controller'):
            if follower.get(name) != 'active':
                raise RuntimeError(f'{name} is {follower.get(name, "missing")}; restart collection after checking hardware')
        leader = self._controller_states(self.leader_controllers)
        if leader.get('leader_torque_controller') != 'active':
            raise RuntimeError('Leader torque controller is not active; restart collection')
        if leader.get('leader_arm_controller') != 'inactive':
            raise RuntimeError(
                'Leader position controller must be inactive before preparation; '
                f'actual={leader.get("leader_arm_controller", "missing")}; restart collection')
        self._validate_leader_positions()

    def _switch_leader(self, active):
        request = SwitchController.Request(
            activate_controllers=['leader_arm_controller'] if active else [],
            deactivate_controllers=[] if active else ['leader_arm_controller'],
            strictness=SwitchController.Request.STRICT,
            timeout=Duration(sec=2))
        response = self._service(self.leader_switch, request)
        if not response.ok:
            raise RuntimeError(f'Leader position controller switch failed: {response.message}')
        state = self._controller_states(self.leader_controllers).get('leader_arm_controller')
        if state != ('active' if active else 'inactive'):
            raise RuntimeError(f'Leader position controller switch unconfirmed: {state}')

    def _acquire_leader(self):
        # Detection/recorder admission may take seconds after preflight.
        self._check_controllers()
        try:
            # Clear a previous teleop fault only at an explicit new attempt.
            self._set_teleop(False)
        except Exception:
            self._block_after_cleanup_failure()
            raise
        self._set_torque(False)
        # Activation is from measured state. Torque stays off during ownership
        # transfer, so neither an old goal nor an old command can move the arm.
        self._leader_controller_owned = True  # Also own an ambiguous switch reply.
        self._switch_leader(True)
        self._set_torque(True)
        self._holding_leader = True

    def _deactivate_leader(self):
        if self._leader_controller_owned:
            state = self._controller_states(self.leader_controllers).get('leader_arm_controller')
            if state == 'inactive':
                self._leader_controller_owned = False
                return
            if state != 'active':
                raise RuntimeError(f'Leader controller ownership is unconfirmed: {state}')
            self._switch_leader(False)
            self._leader_controller_owned = False

    @staticmethod
    def _require_action(client, label: str) -> None:
        if not client.wait_for_server(
            timeout_sec=SERVICE_DISCOVERY_TIMEOUT_S
        ):
            raise RuntimeError(f'{label} action is unavailable')

    @staticmethod
    def _require_service(client, label: str) -> None:
        if not client.wait_for_service(
            timeout_sec=SERVICE_DISCOVERY_TIMEOUT_S
        ):
            raise RuntimeError(f'{label} service is unavailable')

    def _preflight(self, goal) -> None:
        """Resolve every live dependency before the first motor command."""
        self._check_controllers()
        actions = [
            (self.recorder, 'RecordEpisode'),
            (self.align, 'Leader alignment'),
        ]
        if goal.template_id == 'pick':
            actions.extend([
                (self.detect, 'DetectObject'),
                (self.prepare, 'PrepareGrasp'),
            ])
        for client, label in actions:
            self._require_action(client, label)
        for client, label in (
            (self.teleop, 'Leader/Follower teleop'),
            (self.torque, 'Leader torque'),
            (self.finalize, 'RecordEpisode finalize'),
            (self.episode_event, 'RecordEpisode event'),
        ):
            self._require_service(client, label)

    @staticmethod
    def _drain_child_cancel(
        child_handle, result_future, label: str, timeout: float = 25.0
    ):
        """Cancel an owned child and retain ownership until a terminal result."""
        try:
            _wait(child_handle.cancel_goal_async(), 5.0)
        except Exception:
            # The terminal action result is authoritative when the transport
            # acknowledgment is lost.
            pass
        try:
            wrapped = _wait(result_future, timeout)
        except Exception as error:
            raise RuntimeError(
                f'{label} cancellation did not yield a terminal result: {error}'
            ) from error
        if wrapped is None or wrapped.status not in TERMINAL_ACTION_STATUSES:
            status = getattr(wrapped, 'status', 'missing')
            raise RuntimeError(
                f'{label} cancellation returned non-terminal status {status}'
            )
        return wrapped

    def _action(
        self,
        client,
        goal,
        parent_handle,
        *,
        label: str,
        timeout: float = 30.0,
        feedback_callback=None,
        heartbeat_callback=None,
    ):
        """Run one child action while propagating parent cancel and timeout."""
        next_heartbeat = (
            time.monotonic() + self._control_heartbeat_period_s
            if heartbeat_callback is not None else None
        )

        def heartbeat_if_due():
            nonlocal next_heartbeat
            if (
                heartbeat_callback is not None
                and time.monotonic() >= next_heartbeat
            ):
                heartbeat_callback()
                next_heartbeat = (
                    time.monotonic() + self._control_heartbeat_period_s
                )

        self._raise_if_canceled(parent_handle)
        if not client.wait_for_server(
            timeout_sec=SERVICE_DISCOVERY_TIMEOUT_S
        ):
            raise RuntimeError(f'{label} action is unavailable')
        self._raise_if_canceled(parent_handle)
        goal_future = client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        response_deadline = time.monotonic() + 5.0
        cancel_requested = False
        heartbeat_error = None
        while not goal_future.done():
            cancel_requested = (
                cancel_requested or parent_handle.is_cancel_requested
            )
            if heartbeat_error is None:
                try:
                    heartbeat_if_due()
                except Exception as error:
                    # The control lease itself now fails safe, but the action
                    # response still has to be acquired so an accepted motion
                    # goal can be canceled and drained.
                    heartbeat_error = error
            if goal_future.done():
                break
            if time.monotonic() >= response_deadline:
                self._block_after_cleanup_failure()
                raise RuntimeError(
                    f'{label} goal response timed out; ownership is unknown'
                )
            time.sleep(0.02)
        try:
            child_handle = goal_future.result()
        except Exception as error:
            self._block_after_cleanup_failure()
            raise RuntimeError(
                f'{label} goal response failed; ownership is unknown: {error}'
            ) from error
        if child_handle is None or not child_handle.accepted:
            if cancel_requested or parent_handle.is_cancel_requested:
                raise CollectionCanceled('collection cancellation requested')
            raise RuntimeError(f'{label} action rejected')

        try:
            result_future = child_handle.get_result_async()
        except Exception as error:
            try:
                _wait(child_handle.cancel_goal_async(), 5.0)
            except Exception:
                pass
            self._block_after_cleanup_failure()
            raise RuntimeError(
                f'{label} result ownership could not be acquired: {error}'
            ) from error
        if heartbeat_error is not None:
            try:
                self._drain_child_cancel(
                    child_handle, result_future, label
                )
            except Exception:
                self._block_after_cleanup_failure()
                raise
            raise RuntimeError(
                f'{label} control lease refresh failed: {heartbeat_error}'
            ) from heartbeat_error
        deadline = time.monotonic() + timeout
        while not result_future.done():
            if cancel_requested or parent_handle.is_cancel_requested:
                try:
                    self._drain_child_cancel(
                        child_handle, result_future, label
                    )
                except Exception:
                    self._block_after_cleanup_failure()
                    raise
                raise CollectionCanceled('collection cancellation requested')
            try:
                heartbeat_if_due()
            except Exception as error:
                try:
                    self._drain_child_cancel(
                        child_handle, result_future, label
                    )
                except Exception:
                    self._block_after_cleanup_failure()
                    raise
                raise RuntimeError(
                    f'{label} control lease refresh failed: {error}'
                ) from error
            if time.monotonic() >= deadline:
                try:
                    self._drain_child_cancel(
                        child_handle, result_future, label
                    )
                except Exception:
                    self._block_after_cleanup_failure()
                    raise
                raise TimeoutError(f'{label} action timed out and was canceled')
            time.sleep(0.02)
        try:
            wrapped = result_future.result()
        except Exception as error:
            self._block_after_cleanup_failure()
            raise RuntimeError(
                f'{label} result transport failed; ownership is unknown: {error}'
            ) from error
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = getattr(wrapped, 'status', 'missing')
            payload = getattr(wrapped, 'result', None)
            error = getattr(payload, 'error', None)
            detail = getattr(error, 'message', '') or getattr(payload, 'error_string', '')
            raise RuntimeError(
                f'{label} action terminated with status {status}' + (f': {detail}' if detail else '')
            )
        return wrapped.result

    def _start_recorder(
        self,
        goal,
        parent_handle,
        feedback_callback,
        *,
        response_timeout: float = SERVICE_RESPONSE_TIMEOUT_S,
    ):
        """Acquire recorder ownership or fail closed when it is unknowable."""
        self._raise_if_canceled(parent_handle)
        goal_future = self.recorder.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        deadline = time.monotonic() + response_timeout
        cancel_requested = False
        while not goal_future.done():
            cancel_requested = (
                cancel_requested or parent_handle.is_cancel_requested
            )
            if time.monotonic() >= deadline:
                self._block_after_cleanup_failure()
                raise RuntimeError(
                    'RecordEpisode goal response timed out; ownership is unknown. '
                    'Its bounded fail-safe will retain incomplete data; restart '
                    'the collection profile before retrying'
                )
            time.sleep(0.02)
        try:
            recorder_handle = goal_future.result()
        except Exception as error:
            self._block_after_cleanup_failure()
            raise RuntimeError(
                'RecordEpisode goal response failed; ownership is unknown. Its '
                'bounded fail-safe will retain incomplete data; restart the '
                f'collection profile before retrying: {error}'
            ) from error
        if recorder_handle is None or not recorder_handle.accepted:
            if cancel_requested or parent_handle.is_cancel_requested:
                raise CollectionCanceled('collection cancellation requested')
            raise RuntimeError('RecordEpisode rejected the episode')
        try:
            result_future = recorder_handle.get_result_async()
        except Exception as error:
            try:
                _wait(recorder_handle.cancel_goal_async(), 5.0)
            except Exception:
                pass
            self._block_after_cleanup_failure()
            raise RuntimeError(
                f'RecordEpisode result ownership could not be acquired: {error}'
            ) from error
        if cancel_requested or parent_handle.is_cancel_requested:
            try:
                self._cancel_recorder(recorder_handle, result_future)
            except Exception:
                self._block_after_cleanup_failure()
                raise
            raise CollectionCanceled('collection cancellation requested')
        return recorder_handle, result_future

    def _set_torque(self, enabled: bool):
        response = self._service(self.torque, SetBool.Request(data=enabled))
        if not response.success:
            raise RuntimeError(response.message)

    def _set_teleop(self, enabled: bool):
        response = self._service(self.teleop, SetBool.Request(data=enabled))
        if not response.success:
            raise RuntimeError(response.message)

    def _align_leader(self, positions: list[float], parent_handle):
        self._validate_leader_positions(positions)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(LEADER_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=2)
        goal.trajectory.points = [point]
        result = self._action(
            self.align,
            goal,
            parent_handle,
            label='Leader alignment',
            timeout=8.0,
            heartbeat_callback=lambda: self._set_torque(True),
        )
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'Leader alignment failed: {result.error_string}')
        # Trajectory completion and joint-state delivery are asynchronous. Keep
        # holding the target while feedback catches up; success is still based
        # on measured positions, never just the controller's success result.
        completed_at = time.monotonic()
        deadline = completed_at + ALIGNMENT_SETTLE_TIMEOUT_S
        while True:
            self._raise_if_canceled(parent_handle)
            self._set_torque(True)
            with self._state_lock:
                state = dict(self._states['leader'])
                received_at = self._state_received_at['leader']
            now = time.monotonic()
            problems = []
            if received_at <= 0.0 or now - received_at > self._joint_state_timeout_s:
                problems.append('joint state is stale or missing')
            elif received_at < completed_at:
                problems.append('waiting for joint state after trajectory completion')
            for name, position in zip(LEADER_JOINTS, positions):
                measured = state.get(name)
                if measured is None or not math.isfinite(measured):
                    problems.append(f'{name}: missing or non-finite position')
                elif abs(measured - position) > ALIGNMENT_TOLERANCE_RAD:
                    problems.append(
                        f'{name}: target={position:.3f}, actual={measured:.3f}, '
                        f'error={abs(measured - position):.3f} rad '
                        f'(limit={ALIGNMENT_TOLERANCE_RAD:.2f})'
                    )
            if not problems:
                return
            if now >= deadline:
                raise RuntimeError(
                    'Leader alignment did not settle; preparation canceled '
                    '(not waiting for Home): ' + '; '.join(problems)
                )
            time.sleep(min(self._control_heartbeat_period_s, deadline - now))

    def _prepare_pair(self, goal, parent_handle):
        """Use the canonical validated target to overlap both preparation motions."""
        stop = threading.Event()
        target_ready = threading.Event()
        done = threading.Event()
        outcome = {}

        class PairHandle:
            @property
            def is_cancel_requested(self):
                return parent_handle.is_cancel_requested or stop.is_set()

        paired = PairHandle()

        def feedback(message):
            joints = message.feedback.planned_joints
            if not joints.name or target_ready.is_set():
                return
            mapping = dict(zip(joints.name, joints.position))
            if (len(joints.name) != len(joints.position)
                    or any(name not in mapping or not math.isfinite(mapping[name])
                           for name in TARGET_JOINTS)):
                outcome['error'] = RuntimeError('PrepareGrasp returned invalid planned joints')
                stop.set()
                return
            outcome['target'] = [mapping[name] for name in TARGET_JOINTS]
            target_ready.set()

        def prepare():
            try:
                result = self._action(self.prepare, goal, paired, label='PrepareGrasp',
                                      timeout=60.0, feedback_callback=feedback,
                                      heartbeat_callback=lambda: self._set_torque(True))
                if result.error.code != CapabilityError.NONE:
                    raise RuntimeError(result.error.message)
                outcome['result'] = result
            except Exception as error:
                outcome.setdefault('error', error)
                stop.set()
            finally:
                done.set()

        worker = threading.Thread(target=prepare, daemon=True)
        worker.start()
        try:
            while not target_ready.wait(0.02):
                if done.is_set():
                    raise outcome.get('error', RuntimeError('PrepareGrasp omitted planned target'))
                self._raise_if_canceled(paired)
            self._align_leader(outcome['target'], paired)
            while not done.wait(self._control_heartbeat_period_s):
                self._raise_if_canceled(paired)
                self._set_torque(True)
            if 'error' in outcome:
                raise outcome['error']
            return outcome['result']
        except Exception:
            stop.set()
            # Both paths use the existing child cancel/drain semantics.
            worker.join(timeout=35.0)
            if worker.is_alive():
                self._block_after_cleanup_failure()
                raise RuntimeError('paired preparation cleanup did not complete')
            if ('error' in outcome and not isinstance(outcome['error'], CollectionCanceled)
                    and not parent_handle.is_cancel_requested):
                raise outcome['error']
            raise

    def _validate_live_teleop_alignment(self) -> None:
        """Require a fresh, complete paired snapshot immediately before teleop."""
        now = time.monotonic()
        with self._state_lock:
            leader = dict(self._states['leader'])
            follower = dict(self._states['follower'])
            leader_received_at = self._state_received_at['leader']
            follower_received_at = self._state_received_at['follower']
        if (
            leader_received_at <= 0.0
            or follower_received_at <= 0.0
            or now - leader_received_at > self._joint_state_timeout_s
            or now - follower_received_at > self._joint_state_timeout_s
            or any(name not in leader for name in LEADER_JOINTS)
            or any(name not in follower for name in TARGET_JOINTS)
        ):
            raise RuntimeError(
                'fresh complete Leader and Follower joint states are required '
                'before teleoperation'
            )
        leader_positions = [leader[name] for name in LEADER_JOINTS]
        follower_positions = [follower[name] for name in TARGET_JOINTS]
        if not all(
            math.isfinite(position)
            for position in (*leader_positions, *follower_positions)
        ):
            raise RuntimeError(
                'Leader or Follower joint state is non-finite before '
                'teleoperation'
            )
        if any(
            abs(leader_position - follower_position)
            > ALIGNMENT_TOLERANCE_RAD
            for leader_position, follower_position
            in zip(leader_positions, follower_positions)
        ):
            raise RuntimeError(
                'Leader/Follower live alignment exceeds '
                f'{ALIGNMENT_TOLERANCE_RAD:.2f} rad before teleoperation'
            )

    @staticmethod
    def _raise_if_canceled(handle) -> None:
        if handle.is_cancel_requested:
            raise CollectionCanceled('collection cancellation requested')

    @staticmethod
    def _validate_recorder_terminal(wrapped):
        if wrapped is None:
            raise RuntimeError('RecordEpisode returned no terminal result')
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            detail = getattr(getattr(getattr(wrapped, 'result', None), 'error', None), 'message', '')
            raise RuntimeError(
                f'RecordEpisode terminated with action status {wrapped.status}'
                + (f': {detail}' if detail else '')
            )
        recorded = wrapped.result
        if recorded.error.code != CapabilityError.NONE:
            raise RuntimeError(recorded.error.message)
        return recorded

    @staticmethod
    def _cancel_recorder(recorder_handle, recorder_result_future) -> None:
        try:
            _wait(recorder_handle.cancel_goal_async(), 5.0)
        except Exception:
            # The terminal child result, not transport acknowledgement, is
            # authoritative. Keep ownership and drain it even if the cancel
            # response was lost.
            pass
        wrapped = _wait(recorder_result_future, 25.0)
        if (
            wrapped.status != GoalStatus.STATUS_CANCELED
            or wrapped.result.error.code != CapabilityError.CANCELED
        ):
            raise RuntimeError(
                'RecordEpisode cancellation did not retain an incomplete episode'
            )

    def _manual_follower_positions(self) -> list[float]:
        now = time.monotonic()
        with self._state_lock:
            follower = dict(self._states['follower'])
            received_at = self._state_received_at['follower']
        age = now - received_at
        if (
            received_at <= 0.0
            or age > self._joint_state_timeout_s
            or any(name not in follower for name in TARGET_JOINTS)
        ):
            raise RuntimeError(
                'fresh complete Follower joint state is required for manual collection'
            )
        positions = [follower[name] for name in TARGET_JOINTS]
        if not all(math.isfinite(position) for position in positions):
            raise RuntimeError('Follower joint state contains non-finite positions')
        return positions

    def _wait_follower_stopped(self, handle):
        # Teleop's last short trajectory can still be running after publication
        # stops. Do not mark the recording's end boundary until measured rest.
        deadline = time.monotonic() + 3.0
        previous = self._manual_follower_positions()
        stable_since = None
        while time.monotonic() < deadline:
            self._raise_if_canceled(handle)
            time.sleep(0.1)
            current = self._manual_follower_positions()
            if max(abs(a - b) for a, b in zip(current, previous)) <= 0.01:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= 0.2:
                    return
            else:
                stable_since = None
            previous = current
        raise RuntimeError('Follower did not settle after teleop stopped; incomplete data retained')

    def _block_after_cleanup_failure(self) -> None:
        with self._session_lock:
            self._cleanup_blocked = True

    def _release_controls(
        self, *, teleop_enabled: bool, leader_torque_enabled: bool
    ) -> list[str]:
        """Release locally owned controls and report every unconfirmed release."""
        errors = []
        if teleop_enabled:
            try:
                self._set_teleop(False)
            except Exception as error:
                errors.append(f'teleop disable failed: {error}')
        if leader_torque_enabled:
            self._holding_leader = False
            try:
                self._set_torque(False)
            except Exception as error:
                errors.append(f'leader torque disable failed: {error}')
        try:
            self._deactivate_leader()
        except Exception as error:
            errors.append(f'Leader position controller release failed: {error}')
        if errors:
            self._block_after_cleanup_failure()
        return errors

    def execute(self, handle):
        result = CollectEpisode.Result()
        recorder_handle = None
        recorder_result_future = None
        teleop_enabled = False
        leader_torque_enabled = False
        try:
            goal = handle.request
            self._recorded_frames = 0
            self._recorded_elapsed = 0.0
            with self._session_lock:
                self._active_dataset_id = goal.dataset_id
                self._active_episode_id = goal.episode_id
                self._phase = 'PREFLIGHT'
                self._finish_requested.clear()
                self._begin_requested.clear()
            self._feedback(handle, 'PREFLIGHT', 0.02, 'checking collection profile and runtimes')
            self._raise_if_canceled(handle)
            if goal.dry_run:
                for index, phase in enumerate(PHASES[1:], start=1):
                    self._raise_if_canceled(handle)
                    self._feedback(handle, phase, index / len(PHASES), 'dry-run state transition')
                result.error.code = CapabilityError.NONE
                result.error.message = 'collection dry-run complete; no command was published'
                result.quality_passed = True
                handle.succeed()
                return result

            self._preflight(goal)
            self._raise_if_canceled(handle)
            detected = None
            positions = None
            if goal.template_id == 'pick':
                self._feedback(handle, 'DETECT_OBJECT', 0.08, 'detecting object once')
                detect_goal = DetectObject.Goal(
                    object_id=goal.object_id,
                    target_frame='base_link',
                    grasp_backend='act',
                    dry_run=False,
                )
                detected = self._action(
                    self.detect,
                    detect_goal,
                    handle,
                    label='DetectObject',
                    timeout=30.0,
                )
                if detected.error.code != CapabilityError.NONE:
                    raise RuntimeError(detected.error.message)
                self._raise_if_canceled(handle)
            else:
                positions = self._manual_follower_positions()

            ready = threading.Event()
            first_sample = threading.Event()
            recorder_goal = RecordEpisode.Goal()
            recorder_goal.dataset_id = goal.dataset_id
            recorder_goal.episode_id = goal.episode_id
            recorder_goal.collection_profile_id = goal.collection_profile_id
            recorder_goal.language_instruction = goal.language_instruction
            recorder_goal.object_label = goal.object_id
            recorder_goal.max_duration = Duration(
                sec=math.ceil(
                    goal.max_duration_s + RECORDER_FAILSAFE_GRACE_S
                )
            )

            def recorder_feedback(message):
                with self._session_lock:
                    if (self._active_dataset_id, self._active_episode_id) != (goal.dataset_id, goal.episode_id):
                        return
                    self._recorded_frames = max(self._recorded_frames, int(message.feedback.frame_count))
                    self._recorded_elapsed = max(self._recorded_elapsed, float(message.feedback.elapsed_s))
                if message.feedback.state.phase == 'READY':
                    ready.set()
                elif (
                    message.feedback.state.phase == 'RECORDING'
                    and message.feedback.frame_count > 0
                ):
                    first_sample.set()

            if not self.recorder.wait_for_server(
                timeout_sec=SERVICE_DISCOVERY_TIMEOUT_S
            ):
                raise RuntimeError('RecordEpisode is unavailable')
            recorder_handle, recorder_result_future = self._start_recorder(
                recorder_goal,
                handle,
                recorder_feedback,
            )

            def require_recorder_running(stage: str) -> None:
                nonlocal recorder_handle, recorder_result_future
                if (
                    recorder_result_future is None
                    or not recorder_result_future.done()
                ):
                    return
                try:
                    wrapped = recorder_result_future.result()
                except Exception as error:
                    raise RuntimeError(
                        'RecordEpisode result transport failed while '
                        f'{stage}: {error}'
                    ) from error
                recorder_handle = None
                recorder_result_future = None
                status = getattr(wrapped, 'status', 'missing')
                recorded = getattr(wrapped, 'result', None)
                detail = getattr(
                    getattr(recorded, 'error', None), 'message', ''
                )
                suffix = f': {detail}' if detail else ''
                raise RuntimeError(
                    f'RecordEpisode terminated early while {stage} '
                    f'with action status {status}{suffix}'
                )

            def keep_torque_leased(duration_s: float, stage: str) -> None:
                intervals = max(
                    1,
                    math.ceil(
                        duration_s / self._control_heartbeat_period_s
                    ),
                )
                interval_s = duration_s / intervals
                for _ in range(intervals):
                    self._raise_if_canceled(handle)
                    require_recorder_running(stage)
                    self._set_torque(True)
                    time.sleep(interval_s)

            self._feedback(
                handle, 'RECORDER_ADMISSION', 0.14,
                'checking episode ID, disk, Follower joints, and both cameras',
            )
            ready_deadline = time.monotonic() + 8.0
            while not ready.wait(0.05):
                self._raise_if_canceled(handle)
                require_recorder_running('admission')
                if time.monotonic() >= ready_deadline:
                    raise RuntimeError('recorder admission did not become ready')
            self._raise_if_canceled(handle)
            require_recorder_running('admission')

            if goal.template_id == 'pick':
                self._feedback(
                    handle, 'PREPARE_PREGRASP', 0.22,
                    'moving both arms to the canonical validated pregrasp target',
                )
                prepare_goal = PrepareGrasp.Goal(
                    object_id=goal.object_id, target=detected.target, dry_run=False
                )
                leader_torque_enabled = True
                self._acquire_leader()
                prepared = self._prepare_pair(prepare_goal, handle)
                if prepared.error.code != CapabilityError.NONE:
                    raise RuntimeError(prepared.error.message)
                self._raise_if_canceled(handle)
                mapping = dict(zip(
                    prepared.start_context.joint_names,
                    prepared.start_context.positions,
                ))
                if any(name not in mapping for name in TARGET_JOINTS):
                    raise RuntimeError(
                        'PrepareGrasp omitted the required Follower start context'
                    )
                positions = [mapping[name] for name in TARGET_JOINTS]
                if not all(math.isfinite(position) for position in positions):
                    raise RuntimeError(
                        'PrepareGrasp returned non-finite Follower positions'
                    )
                require_recorder_running('Follower pregrasp')

            self._raise_if_canceled(handle)
            if goal.template_id == 'manual':
                self._feedback(handle, 'ALIGN_LEADER', 0.30, 'moving Leader to current Follower state')
                # Own torque until an explicit disable, including lost replies.
                leader_torque_enabled = True
                self._acquire_leader()
                self._align_leader(positions, handle)
            self._raise_if_canceled(handle)
            require_recorder_running('Leader alignment')
            self._feedback(handle, 'WAITING_HOME', 0.38,
                           'pregrasp ready; Leader torque held; press Home to record')
            home_deadline = time.monotonic() + 60.0
            while not self._begin_requested.is_set():
                if time.monotonic() >= home_deadline:
                    raise TimeoutError('Home was not pressed within 60 seconds; preparation canceled')
                keep_torque_leased(self._control_heartbeat_period_s, 'waiting for Home')
            self._raise_if_canceled(handle)
            require_recorder_running('Home')
            self._feedback(handle, 'STARTING_RECORDING', 0.39,
                           'preparing first recorded sample while Leader still holds torque')
            first_sample.clear()
            event = self._service(
                self.episode_event,
                MarkEpisodeEvent.Request(
                    dataset_id=goal.dataset_id,
                    episode_id=goal.episode_id,
                    event='teleop_enabled',
                ),
            )
            if event.error.code != CapabilityError.NONE:
                raise RuntimeError(event.error.message)
            self._raise_if_canceled(handle)
            require_recorder_running('after recording start event')
            first_sample_deadline = (
                time.monotonic() + self._recorder_first_sample_timeout_s
            )
            while not first_sample.wait(0.02):
                self._raise_if_canceled(handle)
                require_recorder_running('first baseline sample')
                if time.monotonic() >= first_sample_deadline:
                    raise RuntimeError(
                        'recorder did not persist its first baseline sample '
                        'before the start deadline'
                    )
                self._set_torque(True)
            self._raise_if_canceled(handle)
            require_recorder_running('before teleoperation enable')
            self._validate_live_teleop_alignment()
            self._raise_if_canceled(handle)
            require_recorder_running('after live alignment validation')
            self._deactivate_leader()
            self._raise_if_canceled(handle)
            # As with torque, cleanup owns a possibly-applied enable even when
            # the transport acknowledgement is lost.
            teleop_enabled = True
            self._set_teleop(True)
            self._holding_leader = False
            self._set_torque(False)
            leader_torque_enabled = False
            started = time.monotonic()
            duration = goal.max_duration_s
            next_teleop_heartbeat = (
                started + self._control_heartbeat_period_s
            )
            self._set_phase('RECORDING')
            while (
                time.monotonic() - started < duration
                and not self._finish_requested.is_set()
            ):
                self._raise_if_canceled(handle)
                require_recorder_running('recording')
                if time.monotonic() >= next_teleop_heartbeat:
                    self._set_teleop(True)
                    next_teleop_heartbeat = (
                        time.monotonic()
                        + self._control_heartbeat_period_s
                    )
                elapsed = time.monotonic() - started
                feedback = CollectEpisode.Feedback()
                feedback.state.phase = 'RECORDING'
                feedback.state.progress = min(0.85, 0.4 + 0.4 * elapsed / duration)
                feedback.state.message = (
                    'Leader controls Follower; Follower next-state targets are recorded'
                )
                feedback.elapsed_s = self._recorded_elapsed
                feedback.frame_count = self._recorded_frames
                handle.publish_feedback(feedback)
                time.sleep(0.1)

            self._feedback(
                handle, 'STOPPING_TELEOP', 0.86,
                'holding Follower before recorder stop',
            )
            self._set_teleop(False)
            teleop_enabled = False
            self._wait_follower_stopped(handle)
            self._raise_if_canceled(handle)
            stop_event = self._service(
                self.episode_event,
                MarkEpisodeEvent.Request(
                    dataset_id=goal.dataset_id,
                    episode_id=goal.episode_id,
                    event='teleop_disabled',
                ),
            )
            if stop_event.error.code != CapabilityError.NONE:
                raise RuntimeError(stop_event.error.message)
            self._raise_if_canceled(handle)
            require_recorder_running('recording stop event')
            finalize_error = None
            try:
                response = self._service(
                    self.finalize,
                    FinalizeEpisode.Request(
                        dataset_id=goal.dataset_id,
                        episode_id=goal.episode_id,
                    ),
                )
                if response is None or response.error.code != CapabilityError.NONE:
                    message = (
                        'RecordEpisode finalize returned no response'
                        if response is None
                        else response.error.message
                    )
                    raise RuntimeError(message)
            except Exception as error:
                # The service may have applied the stop before its response was
                # lost. The child action terminal result is authoritative.
                finalize_error = error
            self._feedback(
                handle, 'FINALIZING', 0.92,
                'validating NPZ arrays, dual MP4 videos, and checksums',
            )
            try:
                wrapped = _wait(recorder_result_future, 25.0)
            except Exception as terminal_error:
                try:
                    _wait(recorder_handle.cancel_goal_async(), 5.0)
                except Exception:
                    pass
                try:
                    wrapped = _wait(recorder_result_future, 25.0)
                except Exception as drain_error:
                    self._block_after_cleanup_failure()
                    detail = (
                        f'; finalize response: {finalize_error}'
                        if finalize_error is not None
                        else ''
                    )
                    raise RuntimeError(
                        'RecordEpisode finalization could not be confirmed and '
                        f'its cancellation did not reach terminal state: {drain_error}'
                        f'{detail}'
                    ) from terminal_error
            # A terminal result releases recorder ownership even when it reports
            # failure, so exception cleanup must not issue a second cancel.
            recorder_handle = None
            recorder_result_future = None
            recorded = self._validate_recorder_terminal(wrapped)
            if recorded.message_counts:
                self._recorded_frames = int(recorded.message_counts[0])
            self._recorded_elapsed = recorded.duration_s
            result.episode_uri = recorded.episode_uri
            result.quality_passed = True
            result.error.code = CapabilityError.NONE
            result.error.message = 'episode ready for review'
            self._feedback(handle, 'REVIEW', 1.0, 'immutable episode finalized')
            handle.succeed()
        except CollectionCanceled:
            cleanup_errors = self._release_controls(
                teleop_enabled=teleop_enabled,
                leader_torque_enabled=leader_torque_enabled,
            )
            try:
                if recorder_handle is not None and recorder_result_future is not None:
                    self._cancel_recorder(recorder_handle, recorder_result_future)
            except Exception as error:
                cleanup_errors.append(
                    f'recorder cancellation could not be confirmed: {error}'
                )
                self._block_after_cleanup_failure()
            if cleanup_errors:
                result.error.code = CapabilityError.BACKEND_FAILURE
                result.error.message = (
                    'collection cancellation left ownership unconfirmed; '
                    'restart the collection profile: '
                    + '; '.join(cleanup_errors)
                )
                handle.abort()
            else:
                result.error.code = CapabilityError.CANCELED
                result.error.message = (
                    'collection canceled; raw episode remains incomplete'
                    if recorder_handle is not None
                    else 'collection canceled before recording'
                )
                handle.canceled()
        except Exception as error:
            cleanup_errors = self._release_controls(
                teleop_enabled=teleop_enabled,
                leader_torque_enabled=leader_torque_enabled,
            )
            if recorder_handle is not None and recorder_result_future is not None:
                try:
                    self._cancel_recorder(
                        recorder_handle, recorder_result_future
                    )
                except Exception as cleanup_error:
                    cleanup_errors.append(
                        'recorder cancellation could not be confirmed: '
                        f'{cleanup_error}'
                    )
                    self._block_after_cleanup_failure()
            result.error.code = CapabilityError.BACKEND_FAILURE
            result.error.message = str(error)
            if cleanup_errors:
                result.error.message += (
                    '; ownership cleanup failed; restart the collection profile: '
                    + '; '.join(cleanup_errors)
                )
            handle.abort()
        finally:
            with self._session_lock:
                self._active_dataset_id = ''
                self._active_episode_id = ''
                self._phase = ''
                self._finish_requested.clear()
                self._goal_active = False
        return result


def main():
    rclpy.init()
    node = CollectionNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
