"""Session-scoped ACT producer whose only output is PolicyJointChunk."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import math
import os
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from xlerobot_interfaces.action import ExecuteLearnedPolicy, ExecutePolicyStream
from xlerobot_interfaces.msg import CapabilityError, PolicyJointChunk
from xlerobot_policy.act_http import ActHttpClient
from xlerobot_policy.postprocess import (
    freeze_joint,
    GripperGovernorConfig,
    PhaseAwareGripperGovernor,
)
from xlerobot_policy.realtime_queue import RealtimeActionQueue, RealtimeQueueConfig


JOINTS = [
    'right_arm_shoulder_pan',
    'right_arm_shoulder_lift',
    'right_arm_elbow_flex',
    'right_arm_wrist_flex',
    'right_arm_wrist_roll',
    'right_arm_gripper',
]


def duration_seconds(message):
    """Convert a ROS duration message to finite seconds."""
    return float(message.sec) + float(message.nanosec) * 1.0e-9


def duration_message(seconds):
    """Convert positive seconds without nanosecond overflow."""
    whole = math.floor(seconds)
    message = Duration()
    message.sec = int(whole)
    message.nanosec = int(round((seconds - whole) * 1.0e9))
    if message.nanosec == 1_000_000_000:
        message.sec += 1
        message.nanosec = 0
    return message


def stamp_ns(message):
    """Convert a ROS time message to integer nanoseconds."""
    return int(message.sec) * 1_000_000_000 + int(message.nanosec)


def validate_start_context(
    context,
    *,
    observed_positions,
    now_ns,
    tolerance_rad,
    max_age_s,
    future_tolerance_s,
):
    """Validate the orchestrator's pregrasp evidence against a live snapshot."""
    if not context.context_id.strip():
        raise PolicyFailure(
            CapabilityError.SAFETY_REJECTED,
            'live ACT requires a pregrasp start context',
        )
    if list(context.joint_names) != JOINTS:
        raise PolicyFailure(
            CapabilityError.SAFETY_REJECTED,
            'pregrasp start context has the wrong joint layout',
        )
    expected = [float(value) for value in context.positions]
    if len(expected) != len(JOINTS) or not all(math.isfinite(value) for value in expected):
        raise PolicyFailure(
            CapabilityError.SAFETY_REJECTED,
            'pregrasp start context positions are incomplete or nonfinite',
        )
    established_ns = stamp_ns(context.established_at)
    age_s = (int(now_ns) - established_ns) * 1.0e-9
    if (
        established_ns <= 0
        or not math.isfinite(age_s)
        or age_s < -future_tolerance_s
        or age_s > max_age_s
    ):
        raise PolicyFailure(
            CapabilityError.SAFETY_REJECTED,
            'pregrasp start context is stale or future-dated',
        )
    if observed_positions is not None:
        observed = [float(value) for value in observed_positions]
        if len(observed) != len(expected) or not all(
            math.isfinite(value) for value in observed
        ):
            raise PolicyFailure(
                CapabilityError.SAFETY_REJECTED,
                'live joint state cannot verify the pregrasp start context',
            )
        for name, actual, target in zip(JOINTS, observed, expected):
            if abs(actual - target) > tolerance_rad:
                raise PolicyFailure(
                    CapabilityError.SAFETY_REJECTED,
                    f'live joint state left pregrasp context at {name}',
                )


@dataclass
class PolicyFailure(RuntimeError):
    """Internal failure converted to a typed action result."""

    code: int
    message: str

    def __str__(self):
        return self.message


class ExecutorFeedback:
    """Thread-safe view of executor session and queue state."""

    def __init__(self, session_id):
        self.condition = threading.Condition()
        self.session_id = session_id
        self.protocol_error = ''
        self.terminal_error = None
        self.ready = False
        self.accepted_chunks = 0
        self.executed_samples = 0
        self.queued_horizon_s = 0.0

    def update(self, feedback):
        with self.condition:
            if feedback.session_id and feedback.session_id != self.session_id:
                self.protocol_error = 'executor feedback changed the session ID'
                self.condition.notify_all()
                return
            self.ready = self.ready or feedback.state.phase in {
                'armed',
                'armed_capture_only',
                'running',
                'capturing_no_execution',
                'complete',
            }
            self.accepted_chunks = int(feedback.accepted_chunks)
            self.executed_samples = int(feedback.executed_samples)
            self.queued_horizon_s = float(feedback.queued_horizon_s)
            self.condition.notify_all()

    def finish(self, future):
        """Wake queue waits on termination, preserving the executor's cause."""
        with self.condition:
            try:
                wrapped = future.result()
                result = wrapped.result
                if result.session_id != self.session_id:
                    self.protocol_error = 'executor result changed the session ID'
                elif result.error.code != CapabilityError.NONE:
                    self.terminal_error = PolicyFailure(
                        result.error.code, result.error.message
                    )
                elif wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                    self.terminal_error = PolicyFailure(
                        CapabilityError.BACKEND_FAILURE,
                        'executor terminated without a successful result',
                    )
                else:
                    self.accepted_chunks = int(result.accepted_chunks)
                    self.executed_samples = int(result.executed_samples)
            except Exception as exc:
                self.terminal_error = PolicyFailure(
                    CapabilityError.BACKEND_FAILURE, f'executor result failed: {exc}'
                )
            self.condition.notify_all()

    def raise_if_failed(self):
        if self.protocol_error:
            raise PolicyFailure(CapabilityError.BACKEND_FAILURE, self.protocol_error)
        if self.terminal_error is not None:
            raise self.terminal_error


class ActPolicyNode(Node):
    """Translate one bounded ACT proposal into executor-owned stream chunks."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__('act_policy_adapter', parameter_overrides=parameter_overrides)
        self.group = ReentrantCallbackGroup()
        self.backend_enabled = bool(self.declare_parameter('backend_enabled', False).value)
        self.capture_only_mode = bool(self.declare_parameter(
            'capture_only_mode', False
        ).value)
        self.policy_id = str(self.declare_parameter(
            'policy_id', 'act_xlerobot_box_wrist_only'
        ).value)
        self.predict_url = str(self.declare_parameter(
            'predict_url', 'http://127.0.0.1:8766/predict'
        ).value)
        self.control_hz = float(self.declare_parameter('control_hz', 30.0).value)
        self.request_hz = float(self.declare_parameter('request_hz', 3.0).value)
        self.chunk_samples = int(self.declare_parameter('executor_chunk_samples', 10).value)
        self.queued_action_steps = int(self.declare_parameter(
            'queued_action_steps', 60
        ).value)
        self.refill_threshold_steps = int(self.declare_parameter(
            'realtime_refill_threshold_steps', 15
        ).value)
        self.blend_steps = int(self.declare_parameter(
            'realtime_blend_steps', 10
        ).value)
        self.queue_threshold_s = float(self.declare_parameter(
            'publish_queue_threshold_s', 1.0
        ).value)
        self.max_duration_s = float(self.declare_parameter('max_duration_s', 20.0).value)
        self.sensor_timeout_s = float(self.declare_parameter('sensor_timeout_s', 5.0).value)
        self.max_sensor_age_s = float(self.declare_parameter('max_sensor_age_s', 0.5).value)
        self.image_sync_tolerance_s = float(self.declare_parameter(
            'image_sync_tolerance_s', 0.25
        ).value)
        self.executor_timeout_s = float(self.declare_parameter(
            'executor_response_timeout_s', 5.0
        ).value)
        self.chunk_transport_settle_s = float(self.declare_parameter(
            'chunk_transport_settle_s', 0.10
        ).value)
        self.gripper_close_hold_s = float(self.declare_parameter(
            'gripper_close_hold_s', 1.0
        ).value)
        self.freeze_shoulder_pan = bool(self.declare_parameter(
            'freeze_shoulder_pan', True
        ).value)
        self.start_context_tolerance_rad = float(self.declare_parameter(
            'start_context_tolerance_rad', 0.05
        ).value)
        self.max_start_context_age_s = float(self.declare_parameter(
            'max_start_context_age_s', 5.0
        ).value)
        self.start_context_future_tolerance_s = float(self.declare_parameter(
            'start_context_future_tolerance_s', 0.10
        ).value)
        self.start_context_wait_s = float(self.declare_parameter(
            'start_context_wait_s', 0.25
        ).value)
        self._validate_parameters()
        governor_config = GripperGovernorConfig(
            open_floor=float(self.declare_parameter('gripper_open_floor', 1.0).value),
            closed_value=float(self.declare_parameter('gripper_closed_value', 0.0).value),
            raw_close_threshold=float(self.declare_parameter(
                'gripper_raw_close_threshold', 0.30
            ).value),
            close_confirm_steps=int(self.declare_parameter(
                'gripper_close_confirm_steps', 1
            ).value),
            close_lookahead_steps=int(self.declare_parameter(
                'gripper_close_lookahead_steps', 100
            ).value),
            near_bottom_margin_rad=float(self.declare_parameter(
                'gripper_near_bottom_margin_rad', 0.08
            ).value),
            bottom_lookahead_steps=int(self.declare_parameter(
                'gripper_bottom_lookahead_steps', 30
            ).value),
            min_close_delay_s=float(self.declare_parameter(
                'gripper_min_close_delay_s', 0.8
            ).value),
            close_ramp_s=float(self.declare_parameter(
                'gripper_close_ramp_s', 0.25
            ).value),
            lock_closed=bool(self.declare_parameter(
                'gripper_lock_closed', True
            ).value),
            control_hz=self.control_hz,
        )
        self.governor = PhaseAwareGripperGovernor(governor_config)
        self.queue = RealtimeActionQueue(
            self.governor,
            RealtimeQueueConfig(
                queued_action_steps=self.queued_action_steps,
                refill_threshold_steps=self.refill_threshold_steps,
                blend_steps=self.blend_steps,
                close_hold_steps=round(self.gripper_close_hold_s * self.control_hz),
                control_hz=self.control_hz,
            ),
        )
        auth_token_env = str(self.declare_parameter(
            'auth_token_env', 'XLEROBOT_ACT_TOKEN'
        ).value)
        self.http = ActHttpClient(
            predict_url=self.predict_url,
            timeout_s=float(self.declare_parameter('http_timeout_s', 3.0).value),
            request_width=int(self.declare_parameter('request_width', 640).value),
            jpeg_quality=int(self.declare_parameter('jpeg_quality', 95).value),
            auth_token=os.environ.get(auth_token_env, '') if auth_token_env else '',
        )
        self.bridge = CvBridge()
        self.inference_pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix='act-inference'
        )
        self.data_condition = threading.Condition()
        self.latest = {'head': None, 'wrist': None, 'joints': None}
        self.goal_lock = threading.Lock()
        self.goal_active = False
        self.executor_client = ActionClient(
            self, ExecutePolicyStream, 'execute_policy_stream', callback_group=self.group
        )
        self.chunk_publisher = self.create_publisher(
            PolicyJointChunk, 'policy_joint_chunks', 10
        )
        self.create_subscription(
            Image,
            str(self.declare_parameter(
                'head_image_topic', '/xlerobot/d455/color/image_raw'
            ).value),
            lambda message: self._store('head', message),
            qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.create_subscription(
            Image,
            str(self.declare_parameter(
                'wrist_image_topic', '/right_wrist_camera/image_raw'
            ).value),
            lambda message: self._store('wrist', message),
            qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.create_subscription(
            JointState,
            str(self.declare_parameter('joint_state_topic', '/joint_states').value),
            lambda message: self._store('joints', message),
            10,
            callback_group=self.group,
        )
        self.server = ActionServer(
            self,
            ExecuteLearnedPolicy,
            'execute_learned_policy',
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group,
        )
        mode = 'enabled' if self.backend_enabled else 'disabled (dry-run only)'
        self.get_logger().info(f'ACT policy adapter ready; backend {mode}')

    def _validate_parameters(self):
        values = (
            self.control_hz,
            self.request_hz,
            self.queue_threshold_s,
            self.max_duration_s,
            self.sensor_timeout_s,
            self.max_sensor_age_s,
            self.image_sync_tolerance_s,
            self.executor_timeout_s,
            self.chunk_transport_settle_s,
            self.start_context_tolerance_rad,
            self.max_start_context_age_s,
            self.start_context_wait_s,
        )
        if not self.policy_id or self.chunk_samples <= 0:
            raise ValueError('policy id and executor chunk size must be nonempty')
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('ACT adapter timing parameters must be finite and positive')
        if self.gripper_close_hold_s < 0.0:
            raise ValueError('gripper_close_hold_s must not be negative')
        if (
            not math.isfinite(self.start_context_future_tolerance_s)
            or self.start_context_future_tolerance_s < 0.0
        ):
            raise ValueError('start context future tolerance must be finite and nonnegative')

    def goal_callback(self, request):
        duration = duration_seconds(request.max_duration)
        if (
            request.policy_id != self.policy_id
            or not request.object_id.strip()
            or not math.isfinite(duration)
            or duration <= 0.0
            or duration > self.max_duration_s
        ):
            return GoalResponse.REJECT
        with self.goal_lock:
            if self.goal_active:
                return GoalResponse.REJECT
            self.goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _store(self, key, message):
        with self.data_condition:
            self.latest[key] = message
            self.data_condition.notify_all()

    def execute(self, goal_handle):
        result = ExecuteLearnedPolicy.Result()
        executor_handle = None
        requested_duration = duration_seconds(goal_handle.request.max_duration)
        deadline = time.monotonic() + requested_duration
        try:
            self._feedback(goal_handle, None, 'validating', 0.05, 'policy goal accepted')
            if not goal_handle.request.dry_run and not self.backend_enabled:
                raise PolicyFailure(
                    CapabilityError.SAFETY_REJECTED,
                    'ACT backend is disabled; set backend_enabled=true explicitly',
                )
            frozen_pan = None
            if not goal_handle.request.dry_run:
                context = goal_handle.request.start_context
                has_context = bool(
                    context.context_id.strip()
                    or context.joint_names
                    or context.positions
                    or stamp_ns(context.established_at)
                )
                if not has_context and not self.capture_only_mode:
                    raise PolicyFailure(
                        CapabilityError.SAFETY_REJECTED,
                        'live ACT requires a pregrasp start context',
                    )
                if has_context:
                    validate_start_context(
                        context,
                        observed_positions=None,
                        now_ns=self.get_clock().now().nanoseconds,
                        tolerance_rad=self.start_context_tolerance_rad,
                        max_age_s=self.max_start_context_age_s,
                        future_tolerance_s=self.start_context_future_tolerance_s,
                    )
                head, wrist, joint_state = self._snapshot_at_start_context(
                    goal_handle, context if has_context else None
                )
                if has_context:
                    validate_start_context(
                        context,
                        observed_positions=joint_state,
                        now_ns=self.get_clock().now().nanoseconds,
                        tolerance_rad=self.start_context_tolerance_rad,
                        max_age_s=self.max_start_context_age_s,
                        future_tolerance_s=self.start_context_future_tolerance_s,
                    )
                self._feedback(goal_handle, None, 'inference', 0.15, 'requesting raw ACT chunk')
                frozen_pan = joint_state[0] if self.freeze_shoulder_pan else None
                raw = self._predict(head, wrist, joint_state, frozen_pan)
                self.queue.reset()
                merged = self.queue.merge_prediction(
                    raw,
                    prediction_id=0,
                    observation_step=0,
                    future_step=0,
                )
                if not merged.accepted:
                    raise PolicyFailure(
                        CapabilityError.BACKEND_FAILURE,
                        'initial ACT proposal was stale or empty after alignment',
                    )

            executor_goal = ExecutePolicyStream.Goal()
            executor_goal.session_id = f'policy-stream-{uuid.uuid4().hex}'
            executor_goal.source_id = f'act:{self.policy_id}'
            executor_goal.joint_names = list(JOINTS)
            executor_goal.max_duration = goal_handle.request.max_duration
            executor_goal.dry_run = goal_handle.request.dry_run
            executor_goal.capture_only = self.capture_only_mode
            executor_goal.start_context = goal_handle.request.start_context
            state = ExecutorFeedback(executor_goal.session_id)
            executor_handle = self._start_executor(goal_handle, executor_goal, state)
            if not goal_handle.request.dry_run:
                self._wait_executor_ready(goal_handle, executor_handle, state)
                self._wait_chunk_transport(goal_handle, executor_handle)
                self._stream_realtime(
                    goal_handle,
                    executor_handle,
                    state,
                    frozen_pan,
                    deadline,
                    requested_duration,
                )
            remaining = max(0.0, deadline - time.monotonic())
            wrapped = self._wait_executor_result(
                goal_handle,
                executor_handle,
                remaining + self.executor_timeout_s,
            )
            if wrapped.result.session_id != state.session_id:
                raise PolicyFailure(
                    CapabilityError.BACKEND_FAILURE,
                    'executor result did not echo the requested session ID',
                )
            result.executor_session_id = wrapped.result.session_id
            result.accepted_chunks = wrapped.result.accepted_chunks
            result.executed_samples = wrapped.result.executed_samples
            result.error = wrapped.result.error
            if (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and wrapped.result.error.code == CapabilityError.NONE
            ):
                goal_handle.succeed()
            elif wrapped.status == GoalStatus.STATUS_CANCELED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except PolicyFailure as exc:
            if executor_handle is not None and exc.code != CapabilityError.CANCELED:
                self._request_executor_cancel(executor_handle)
            result.error.code = int(exc.code)
            result.error.message = exc.message
            if exc.code == CapabilityError.CANCELED or goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:
            if executor_handle is not None:
                self._request_executor_cancel(executor_handle)
            result.error.code = CapabilityError.INTERNAL_ERROR
            result.error.message = str(exc)
            goal_handle.abort()
            return result
        finally:
            with self.goal_lock:
                self.goal_active = False

    def _snapshot(self, goal_handle, executor_handle=None):
        deadline = time.monotonic() + self.sensor_timeout_s
        with self.data_condition:
            while rclpy.ok() and time.monotonic() < deadline:
                self._check_cancel(goal_handle, executor_handle)
                head, wrist, joints = self.latest.values()
                if head is not None and wrist is not None and joints is not None:
                    image_times = [
                        Time.from_msg(item.header.stamp).nanoseconds
                        for item in (head, wrist)
                    ]
                    joint_time = Time.from_msg(joints.header.stamp).nanoseconds
                    now = self.get_clock().now().nanoseconds
                    if (
                        min(image_times) > 0
                        and joint_time > 0
                        and (max(image_times) - min(image_times)) * 1.0e-9
                        <= self.image_sync_tolerance_s
                        and -self.image_sync_tolerance_s
                        <= (now - max(image_times)) * 1.0e-9
                        <= self.max_sensor_age_s
                        and -self.image_sync_tolerance_s
                        <= (now - joint_time) * 1.0e-9
                        <= self.max_sensor_age_s
                    ):
                        positions = dict(zip(joints.name, joints.position))
                        try:
                            state = [float(positions[name]) for name in JOINTS]
                        except KeyError:
                            state = None
                        if state is not None and all(math.isfinite(value) for value in state):
                            return head, wrist, state
                self.data_condition.wait(timeout=0.05)
        raise PolicyFailure(
            CapabilityError.TIMEOUT,
            'timed out waiting for synchronized images and complete joint state',
        )

    def _snapshot_at_start_context(self, goal_handle, context):
        if context is None:
            return self._snapshot(goal_handle)
        deadline = time.monotonic() + self.start_context_wait_s
        last_mismatch = None
        while rclpy.ok() and time.monotonic() < deadline:
            head, wrist, state = self._snapshot(goal_handle)
            try:
                validate_start_context(
                    context,
                    observed_positions=state,
                    now_ns=self.get_clock().now().nanoseconds,
                    tolerance_rad=self.start_context_tolerance_rad,
                    max_age_s=self.max_start_context_age_s,
                    future_tolerance_s=self.start_context_future_tolerance_s,
                )
                return head, wrist, state
            except PolicyFailure as exc:
                if not exc.message.startswith(
                    'live joint state left pregrasp context'
                ):
                    raise
                last_mismatch = exc
            with self.data_condition:
                observed = self.latest['joints']
                remaining = deadline - time.monotonic()
                if remaining > 0.0:
                    self.data_condition.wait_for(
                        lambda: self.latest['joints'] is not observed,
                        timeout=min(0.05, remaining),
                    )
        if last_mismatch is not None:
            raise last_mismatch
        raise PolicyFailure(
            CapabilityError.TIMEOUT,
            'timed out waiting for the pregrasp start state',
        )

    def _predict(self, head, wrist, state, frozen_pan):
        try:
            raw = self.http.predict(
                head_bgr=self.bridge.imgmsg_to_cv2(head, desired_encoding='bgr8'),
                wrist_bgr=self.bridge.imgmsg_to_cv2(wrist, desired_encoding='bgr8'),
                state=state,
            )
        except Exception as exc:
            raise PolicyFailure(CapabilityError.BACKEND_FAILURE, str(exc)) from exc
        if self.freeze_shoulder_pan:
            raw = freeze_joint(raw, 0, frozen_pan)
        return raw

    def _start_executor(self, outer_goal, executor_goal, state):
        if not self.executor_client.wait_for_server(timeout_sec=self.executor_timeout_s):
            raise PolicyFailure(CapabilityError.UNAVAILABLE, 'streaming executor unavailable')

        def feedback(message):
            state.update(message.feedback)
            self._feedback(
                outer_goal,
                state,
                message.feedback.state.phase,
                message.feedback.state.progress,
                message.feedback.state.message,
            )

        future = self.executor_client.send_goal_async(
            executor_goal, feedback_callback=feedback
        )
        handle = self._wait_future(outer_goal, future, self.executor_timeout_s)
        if handle is None or not handle.accepted:
            raise PolicyFailure(CapabilityError.BACKEND_FAILURE, 'executor rejected policy goal')
        handle.get_result_async().add_done_callback(state.finish)
        return handle

    def _wait_executor_ready(self, outer_goal, executor_handle, state):
        deadline = time.monotonic() + self.executor_timeout_s
        with state.condition:
            while not state.ready:
                state.raise_if_failed()
                self._check_cancel(outer_goal, executor_handle)
                if time.monotonic() >= deadline:
                    raise PolicyFailure(
                        CapabilityError.TIMEOUT,
                        'executor did not arm the requested policy session',
                    )
                state.condition.wait(timeout=0.02)

    def _wait_chunk_transport(self, outer_goal, executor_handle):
        deadline = time.monotonic() + self.executor_timeout_s
        matched_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            self._check_cancel(outer_goal, executor_handle)
            if self.chunk_publisher.get_subscription_count() > 0:
                now = time.monotonic()
                if matched_since is None:
                    matched_since = now
                elif now - matched_since >= self.chunk_transport_settle_s:
                    return
            else:
                matched_since = None
            time.sleep(0.01)
        raise PolicyFailure(
            CapabilityError.UNAVAILABLE,
            'executor chunk transport did not match a subscriber',
        )

    def _stream_realtime(
        self,
        outer_goal,
        executor_handle,
        state,
        frozen_pan,
        deadline,
        requested_duration,
    ):
        sequence = 0
        prediction_id = 1
        pending = None
        ready = None
        last_request_at = time.monotonic()
        request_period_s = 1.0 / self.request_hz
        max_samples = math.floor(requested_duration * self.control_hz)
        while rclpy.ok():
            with state.condition:
                state.raise_if_failed()
            self._check_cancel(outer_goal, executor_handle)
            if time.monotonic() >= deadline:
                raise PolicyFailure(
                    CapabilityError.TIMEOUT,
                    'ACT did not complete its verified gripper-close phase in time',
                )

            if pending is not None and pending[0].done():
                future, current_id, observation_step = pending
                try:
                    raw = future.result()
                except PolicyFailure:
                    raise
                except Exception as exc:
                    raise PolicyFailure(
                        CapabilityError.BACKEND_FAILURE, str(exc)
                    ) from exc
                ready = (raw, current_id, observation_step)
                pending = None

            if self.queue.needs_refill and ready is not None:
                raw, current_id, observation_step = ready
                future_step = self._future_step(state)
                merged = self.queue.merge_prediction(
                    raw,
                    prediction_id=current_id,
                    observation_step=observation_step,
                    future_step=future_step,
                )
                ready = None
                detail = (
                    f'ACT refill {current_id} aligned by {merged.skipped_steps} step(s); '
                    f'raw reserve={self.queue.raw_steps}'
                )
                self._feedback(outer_goal, state, 'streaming', 0.4, detail)

            now = time.monotonic()
            if (
                not self.queue.locked_after_close
                and pending is None
                and now - last_request_at >= request_period_s
            ):
                head, wrist, joint_state = self._snapshot(
                    outer_goal, executor_handle
                )
                with state.condition:
                    observation_step = state.executed_samples
                future = self.inference_pool.submit(
                    self._predict, head, wrist, joint_state, frozen_pan
                )
                pending = (future, prediction_id, observation_step)
                prediction_id += 1
                last_request_at = now
                self._feedback(
                    outer_goal,
                    state,
                    'inference',
                    0.3,
                    'refreshing observation for ACT queue refill',
                )

            self._wait_queue_room(
                outer_goal, executor_handle, state, deadline
            )
            piece = self.queue.emit(self.chunk_samples)
            if piece:
                if self.queue.emitted_samples > max_samples:
                    raise PolicyFailure(
                        CapabilityError.TIMEOUT,
                        'governed ACT stream exceeds the goal duration',
                    )
                final_chunk = self.queue.complete
                self._publish_piece(
                    state,
                    sequence,
                    piece,
                    final_chunk=final_chunk,
                )
                self._wait_chunk_acceptance(
                    outer_goal, executor_handle, state, sequence + 1
                )
                sequence += 1
                if final_chunk:
                    if pending is not None:
                        pending[0].cancel()
                    return
                continue

            if self.queue.complete:
                raise PolicyFailure(
                    CapabilityError.INTERNAL_ERROR,
                    'ACT queue completed without a final executor chunk',
                )
            time.sleep(0.01)

        raise PolicyFailure(CapabilityError.CANCELED, 'policy runtime stopped')

    def _future_step(self, state):
        with state.condition:
            executed = state.executed_samples
            queued = max(0, math.ceil(state.queued_horizon_s * self.control_hz))
        return executed + queued + self.queue.raw_steps + self.queue.hold_steps

    def _publish_piece(self, state, sequence, piece, *, final_chunk):
        message = PolicyJointChunk()
        message.session_id = state.session_id
        message.source_id = f'act:{self.policy_id}'
        message.sequence = sequence
        message.generated_at = self.get_clock().now().to_msg()
        message.sample_period = duration_message(1.0 / self.control_hz)
        message.final_chunk = final_chunk
        message.joint_names = list(JOINTS)
        message.sample_count = len(piece)
        message.positions = [value for row in piece for value in row]
        self.chunk_publisher.publish(message)

    def _wait_queue_room(self, outer_goal, executor_handle, state, deadline):
        with state.condition:
            while state.queued_horizon_s > self.queue_threshold_s:
                state.raise_if_failed()
                self._check_cancel(outer_goal, executor_handle)
                if time.monotonic() >= deadline:
                    raise PolicyFailure(CapabilityError.TIMEOUT, 'executor queue did not drain')
                state.condition.wait(timeout=0.02)

    def _wait_chunk_acceptance(self, outer_goal, executor_handle, state, count):
        deadline = time.monotonic() + self.executor_timeout_s
        with state.condition:
            while state.accepted_chunks < count:
                state.raise_if_failed()
                self._check_cancel(outer_goal, executor_handle)
                if time.monotonic() >= deadline:
                    raise PolicyFailure(CapabilityError.TIMEOUT, 'policy chunk was not accepted')
                state.condition.wait(timeout=0.02)

    def _wait_executor_result(self, outer_goal, executor_handle, timeout_s):
        return self._wait_future(
            outer_goal, executor_handle.get_result_async(), timeout_s, executor_handle
        )

    def _wait_future(self, outer_goal, future, timeout_s, executor_handle=None):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not event.wait(0.02):
            self._check_cancel(outer_goal, executor_handle)
            if time.monotonic() >= deadline:
                raise PolicyFailure(CapabilityError.TIMEOUT, 'policy action timed out')
        if not event.is_set():
            raise PolicyFailure(CapabilityError.CANCELED, 'policy action interrupted')
        return future.result()

    @staticmethod
    def _check_cancel(outer_goal, executor_handle=None):
        if outer_goal.is_cancel_requested:
            if executor_handle is not None:
                ActPolicyNode._request_executor_cancel(executor_handle)
            raise PolicyFailure(CapabilityError.CANCELED, 'learned policy canceled')

    @staticmethod
    def _request_executor_cancel(executor_handle):
        """Cancel a child goal and always consume the asynchronous result."""
        future = executor_handle.cancel_goal_async()

        def consume_result(completed):
            try:
                completed.result()
            except Exception:
                # Cancellation is best effort while the outer action is already
                # resolving. Consuming the exception prevents a late ROS entity
                # teardown from becoming an unhandled Future warning.
                pass

        future.add_done_callback(consume_result)

    @staticmethod
    def _feedback(goal_handle, state, phase, progress, message):
        feedback = ExecuteLearnedPolicy.Feedback()
        feedback.state.phase = phase
        feedback.state.progress = float(progress)
        feedback.state.message = message
        if state is not None:
            feedback.executor_session_id = state.session_id
            feedback.accepted_chunks = state.accepted_chunks
            feedback.executed_samples = state.executed_samples
            feedback.queued_horizon_s = state.queued_horizon_s
        goal_handle.publish_feedback(feedback)

    def destroy_node(self):
        """Bound shutdown by the HTTP timeout and then destroy ROS entities."""
        self.inference_pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    """Run the policy adapter without granting controller ownership."""
    rclpy.init()
    node = ActPolicyNode()
    executor = MultiThreadedExecutor(num_threads=4)
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


if __name__ == '__main__':
    main()
