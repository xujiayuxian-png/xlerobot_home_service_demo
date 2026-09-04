"""Capture governed policy chunks without owning or commanding a controller."""

from __future__ import annotations

import math
import os
from pathlib import Path
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from xlerobot_interfaces.action import ExecutePolicyStream
from xlerobot_interfaces.msg import CapabilityError, PolicyJointChunk
import yaml


REPLAY_FORMAT = 'xlerobot_policy_replay/v1'


def _duration_seconds(message) -> float:
    return float(message.sec) + float(message.nanosec) * 1.0e-9


def _stamp_ns(message) -> int:
    return int(message.sec) * 1_000_000_000 + int(message.nanosec)


class CaptureSession:
    """Validate ordering and build an explicit no-execution replay document."""

    def __init__(
        self,
        *,
        session_id: str,
        source_id: str,
        joint_names: list[str],
        start_positions: list[float],
        start_velocities: list[float],
        start_context: dict | None = None,
    ):
        self.session_id = session_id
        self.source_id = source_id
        self.joint_names = list(joint_names)
        self.start_positions = list(start_positions)
        self.start_velocities = list(start_velocities)
        self.start_context = start_context
        self.chunks = []

    def accept(self, message: PolicyJointChunk, *, received_at_ns: int) -> None:
        """Accept one exact, finite, session-scoped chunk in sequence."""
        if message.session_id != self.session_id:
            raise ValueError('chunk session ID does not match capture session')
        if message.source_id != self.source_id:
            raise ValueError('chunk source ID does not match capture goal')
        if list(message.joint_names) != self.joint_names:
            raise ValueError('chunk joint layout does not match capture goal')
        if int(message.sequence) != len(self.chunks):
            raise ValueError('chunk sequence is not contiguous from zero')
        sample_count = int(message.sample_count)
        joint_count = len(self.joint_names)
        positions = [float(value) for value in message.positions]
        if sample_count <= 0 or len(positions) != sample_count * joint_count:
            raise ValueError('chunk dimensions do not match sample and joint counts')
        if not all(math.isfinite(value) for value in positions):
            raise ValueError('chunk contains nonfinite positions')
        sample_period_s = _duration_seconds(message.sample_period)
        if not math.isfinite(sample_period_s) or sample_period_s <= 0.0:
            raise ValueError('chunk sample period must be finite and positive')
        generated_at_ns = _stamp_ns(message.generated_at)
        age_s = (received_at_ns - generated_at_ns) * 1.0e-9
        if generated_at_ns <= 0 or not math.isfinite(age_s):
            raise ValueError('chunk generation timestamp is invalid')
        rows = [
            positions[index:index + joint_count]
            for index in range(0, len(positions), joint_count)
        ]
        self.chunks.append(
            {
                'sequence': int(message.sequence),
                'age_s': age_s,
                'sample_period_s': sample_period_s,
                'final_chunk': bool(message.final_chunk),
                'positions': rows,
            }
        )

    @property
    def final_received(self) -> bool:
        return bool(self.chunks and self.chunks[-1]['final_chunk'])

    @property
    def sample_count(self) -> int:
        return sum(len(chunk['positions']) for chunk in self.chunks)

    def document(self) -> dict:
        if not self.final_received:
            raise ValueError('capture session has no final chunk')
        kind = (
            'post_pregrasp_policy_capture'
            if self.start_context is not None
            else 'arbitrary_pose_transport_capture'
        )
        document = {
            'format': REPLAY_FORMAT,
            'provenance': {
                'kind': kind,
                'note': (
                    'Post-governor policy proposals captured by a controller-free '
                    'sink; executed_samples are virtual queue consumption only. '
                    'No motor command was published or executed.'
                ),
            },
            'session_id': self.session_id,
            'source_id': self.source_id,
            'joint_names': self.joint_names,
            'start_positions': self.start_positions,
            'start_velocities': self.start_velocities,
            'chunks': self.chunks,
        }
        if self.start_context is not None:
            document['start_context'] = self.start_context
        return document


def write_replay(path: Path, session: CaptureSession) -> None:
    """Atomically expose one complete replay without overwriting evidence."""
    path = path.expanduser().resolve()
    if path.exists():
        raise FileExistsError(f'replay output already exists: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.parent / f'.{path.name}.incomplete-{uuid.uuid4().hex[:8]}'
    try:
        temporary.write_text(
            yaml.safe_dump(session.document(), sort_keys=False),
            encoding='utf-8',
        )
        os.replace(temporary, path)
    except Exception:
        if temporary.exists():
            temporary.unlink()
        raise


class PolicyCaptureSink(Node):
    """Virtual executor that records chunks and has no command publisher."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            'policy_capture_sink', parameter_overrides=parameter_overrides
        )
        self.group = ReentrantCallbackGroup()
        output = str(self.declare_parameter('output_path', '').value).strip()
        if not output:
            raise ValueError('output_path is required')
        self.output_path = Path(output)
        self.joint_topic = str(
            self.declare_parameter('joint_state_topic', '/joint_states').value
        )
        self.max_joint_age_s = float(
            self.declare_parameter('max_joint_state_age_s', 0.5).value
        )
        self.max_start_context_age_s = float(
            self.declare_parameter('max_start_context_age_s', 5.0).value
        )
        self.start_context_tolerance_rad = float(
            self.declare_parameter('start_context_tolerance_rad', 0.05).value
        )
        self.start_context_wait_s = float(
            self.declare_parameter('start_context_wait_s', 0.25).value
        )
        if (
            not math.isfinite(self.max_joint_age_s)
            or self.max_joint_age_s <= 0.0
            or not math.isfinite(self.max_start_context_age_s)
            or self.max_start_context_age_s <= 0.0
            or not math.isfinite(self.start_context_tolerance_rad)
            or self.start_context_tolerance_rad <= 0.0
            or not math.isfinite(self.start_context_wait_s)
            or self.start_context_wait_s <= 0.0
        ):
            raise ValueError('capture state tolerances must be finite and positive')
        self.condition = threading.Condition()
        self.latest_joint_state = None
        self.goal_reserved = False
        self.active_goal = None
        self.session = None
        self.failure = ''
        self.create_subscription(
            JointState,
            self.joint_topic,
            self._on_joint_state,
            QoSProfile(depth=1),
            callback_group=self.group,
        )
        self.create_subscription(
            PolicyJointChunk,
            'capture_policy_joint_chunks',
            self._on_chunk,
            10,
            callback_group=self.group,
        )
        self.server = ActionServer(
            self,
            ExecutePolicyStream,
            'capture_policy_stream',
            goal_callback=self._goal_callback,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            execute_callback=self._execute,
            callback_group=self.group,
        )
        self.get_logger().warning(
            'capture sink ready; it has no controller or motor command publisher'
        )

    def _goal_callback(self, request):
        duration = _duration_seconds(request.max_duration)
        with self.condition:
            if (
                request.dry_run
                or not request.capture_only
                or not request.session_id.strip()
                or not request.source_id.strip()
                or not request.joint_names
                or len(set(request.joint_names)) != len(request.joint_names)
                or not math.isfinite(duration)
                or duration <= 0.0
                or self.goal_reserved
                or self.output_path.exists()
            ):
                return GoalResponse.REJECT
            self.goal_reserved = True
        return GoalResponse.ACCEPT

    def _on_joint_state(self, message):
        with self.condition:
            self.latest_joint_state = message
            self.condition.notify_all()

    def _fresh_joint_vector(self, joint_names, deadline):
        with self.condition:
            while rclpy.ok() and time.monotonic() < deadline:
                message = self.latest_joint_state
                if message is not None:
                    now_ns = self.get_clock().now().nanoseconds
                    age_s = (now_ns - _stamp_ns(message.header.stamp)) * 1.0e-9
                    positions = dict(zip(message.name, message.position))
                    velocities = dict(zip(message.name, message.velocity))
                    try:
                        start = [float(positions[name]) for name in joint_names]
                        speed = [float(velocities[name]) for name in joint_names]
                    except KeyError:
                        start = []
                        speed = []
                    if (
                        -0.05 <= age_s <= self.max_joint_age_s
                        and len(start) == len(joint_names)
                        and len(speed) == len(joint_names)
                        and all(math.isfinite(value) for value in start)
                        and all(math.isfinite(value) for value in speed)
                    ):
                        return start, speed
                self.condition.wait(timeout=0.05)
        raise TimeoutError('timed out waiting for a fresh complete joint state')

    def _validated_start_context(self, context, joint_names, positions):
        has_any = bool(
            context.context_id.strip()
            or context.joint_names
            or context.positions
            or _stamp_ns(context.established_at)
        )
        if not has_any:
            return None
        if not context.context_id.strip():
            raise ValueError('partial pregrasp start context is invalid')
        if list(context.joint_names) != joint_names:
            raise ValueError('pregrasp start context joint layout does not match capture')
        expected = [float(value) for value in context.positions]
        if len(expected) != len(joint_names) or not all(
            math.isfinite(value) for value in expected
        ):
            raise ValueError('pregrasp start context positions are invalid')
        now_ns = self.get_clock().now().nanoseconds
        established_ns = _stamp_ns(context.established_at)
        age_s = (now_ns - established_ns) * 1.0e-9
        if established_ns <= 0 or age_s < -0.1 or age_s > self.max_start_context_age_s:
            raise ValueError('pregrasp start context is stale or future-dated')
        for name, actual, target in zip(joint_names, positions, expected):
            if abs(actual - target) > self.start_context_tolerance_rad:
                raise ValueError(f'capture state left pregrasp context at {name}')
        return {
            'context_id': context.context_id,
            'established_at_ns': established_ns,
            'joint_names': list(context.joint_names),
            'positions': expected,
        }

    def _fresh_start_vector(self, context, joint_names, deadline):
        context_deadline = min(
            deadline, time.monotonic() + self.start_context_wait_s
        )
        last_mismatch = None
        while rclpy.ok() and time.monotonic() < context_deadline:
            positions, velocities = self._fresh_joint_vector(
                joint_names, context_deadline
            )
            try:
                validated = self._validated_start_context(
                    context, joint_names, positions
                )
                return positions, velocities, validated
            except ValueError as exc:
                if not str(exc).startswith('capture state left pregrasp context'):
                    raise
                last_mismatch = exc
            with self.condition:
                observed = self.latest_joint_state
                remaining = context_deadline - time.monotonic()
                if remaining > 0.0:
                    self.condition.wait_for(
                        lambda: self.latest_joint_state is not observed,
                        timeout=min(0.05, remaining),
                    )
        if last_mismatch is not None:
            raise last_mismatch
        raise TimeoutError('timed out waiting for the pregrasp start state')

    def _on_chunk(self, message, message_info):
        with self.condition:
            if self.session is None or self.failure:
                return
            try:
                received_at_ns = int(
                    message_info.get('received_timestamp', 0)
                )
                if received_at_ns <= 0:
                    received_at_ns = self.get_clock().now().nanoseconds
                self.session.accept(
                    message,
                    received_at_ns=received_at_ns,
                )
            except ValueError as exc:
                self.failure = str(exc)
                self.condition.notify_all()
                return
            feedback = ExecutePolicyStream.Feedback()
            feedback.state.phase = 'capturing_no_execution'
            feedback.state.progress = 0.5
            feedback.state.message = (
                'chunk recorded; executed_samples represents virtual consumption'
            )
            feedback.session_id = self.session.session_id
            feedback.accepted_chunks = len(self.session.chunks)
            feedback.executed_samples = self.session.sample_count
            feedback.queued_horizon_s = 0.0
            if self.active_goal is not None:
                self.active_goal.publish_feedback(feedback)
            self.condition.notify_all()

    def _execute(self, goal_handle):
        result = ExecutePolicyStream.Result()
        deadline = time.monotonic() + _duration_seconds(
            goal_handle.request.max_duration
        )
        try:
            positions, velocities, start_context = self._fresh_start_vector(
                goal_handle.request.start_context,
                list(goal_handle.request.joint_names),
                deadline,
            )
            with self.condition:
                self.active_goal = goal_handle
                self.failure = ''
                self.session = CaptureSession(
                    session_id=goal_handle.request.session_id,
                    source_id=goal_handle.request.source_id,
                    joint_names=list(goal_handle.request.joint_names),
                    start_positions=positions,
                    start_velocities=velocities,
                    start_context=start_context,
                )
                feedback = ExecutePolicyStream.Feedback()
                feedback.state.phase = 'armed_capture_only'
                feedback.state.progress = 0.1
                feedback.state.message = 'controller-free capture session armed'
                feedback.session_id = self.session.session_id
                goal_handle.publish_feedback(feedback)
                while (
                    rclpy.ok()
                    and not self.session.final_received
                    and not self.failure
                    and not goal_handle.is_cancel_requested
                    and time.monotonic() < deadline
                ):
                    self.condition.wait(timeout=0.05)
                    if (
                        not self.session.chunks
                        and not self.failure
                        and not goal_handle.is_cancel_requested
                    ):
                        goal_handle.publish_feedback(feedback)
                if goal_handle.is_cancel_requested:
                    result.error.code = CapabilityError.CANCELED
                    result.error.message = 'capture canceled'
                    goal_handle.canceled()
                    return result
                if self.failure:
                    raise ValueError(self.failure)
                if not self.session.final_received:
                    raise TimeoutError('capture timed out before the final chunk')
                session = self.session
            write_replay(self.output_path, session)
            result.error.code = CapabilityError.NONE
            result.error.message = (
                'captured only; no commands were published or executed'
            )
            result.session_id = session.session_id
            result.accepted_chunks = len(session.chunks)
            result.executed_samples = session.sample_count
            goal_handle.succeed()
            return result
        except Exception as exc:
            result.error.code = CapabilityError.INTERNAL_ERROR
            result.error.message = str(exc)
            goal_handle.abort()
            return result
        finally:
            with self.condition:
                self.goal_reserved = False
                self.active_goal = None
                self.session = None
                self.failure = ''


def main():
    """Run the no-controller capture sink."""
    rclpy.init()
    node = PolicyCaptureSink()
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


if __name__ == '__main__':
    main()
