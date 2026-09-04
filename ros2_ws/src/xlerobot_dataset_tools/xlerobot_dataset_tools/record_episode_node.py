"""RecordEpisode action using the verified Follower-next-state ACT semantics."""

from __future__ import annotations

import math
from pathlib import Path
import re
import shutil
import threading
import time

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from xlerobot_interfaces.action import RecordEpisode
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_interfaces.srv import FinalizeEpisode, MarkEpisodeEvent
import yaml

from .episode_io import EpisodeWriter, JOINT_NAMES


IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$')
MAX_RECORDING_DURATION_S = 180.0
START_EVENT_TIMEOUT_S = 120.0


def duration_seconds(duration) -> float:
    return duration.sec + duration.nanosec / 1e9


def load_profile(profile_id: str, profile_root: Path) -> dict:
    path = (profile_root / f'{profile_id}.yaml').resolve()
    if not path.is_relative_to(profile_root.resolve()) or not path.is_file():
        raise FileNotFoundError(f'unknown collection profile: {profile_id}')
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if document.get('schema') != 'xlerobot_collection_profile/v1':
        raise ValueError('invalid collection profile schema')
    if document.get('action', {}).get('kind') != 'follower_next_state':
        raise ValueError('collection profile must use follower_next_state action')
    if list(document['observation']['joints']) != JOINT_NAMES:
        raise ValueError('collection profile joint order differs from ACT baseline')
    return document


class RecordEpisodeNode(Node):
    def __init__(self):
        super().__init__('record_episode')
        root = Path(str(self.declare_parameter(
            'artifact_root', '.xlerobot/artifacts'
        ).value)).expanduser().resolve()
        self.dataset_root = root / 'datasets'
        default_profiles = Path(get_package_share_directory(
            'xlerobot_dataset_tools'
        )) / 'config'
        self.profile_root = Path(str(self.declare_parameter(
            'collection_profile_root', str(default_profiles)
        ).value)).resolve()
        self.profile = load_profile('two_wheel_pick', self.profile_root)
        self.revision = str(self.declare_parameter('software_revision', 'development').value)
        self.unit_id = str(self.declare_parameter('unit_id', 'reference-two-wheel').value)
        self.calibration_version = str(self.declare_parameter('calibration_version', '').value)
        self.minimum_free_bytes = int(float(self.declare_parameter(
            'minimum_free_gb', 2.0
        ).value) * 1024 ** 3)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.io_lock = threading.Lock()
        self.active = False
        self.goal_reserved = False
        self.cancel_requested = False
        self.committing = False
        self.armed = False
        self.dataset_id = ''
        self.episode_id = ''
        self.writer = None
        self.goal_handle = None
        self.stop = threading.Event()
        self.latest_state: dict[str, float] = {}
        self.latest_images: dict[str, np.ndarray] = {}
        self.started_at = 0.0
        self.frame_count = 0
        self.sample_error = ''
        self.input_freshness_s = float(self.declare_parameter(
            'input_freshness_s', 0.5
        ).value)
        if (
            not math.isfinite(self.input_freshness_s)
            or self.input_freshness_s <= 0.0
        ):
            raise ValueError('input_freshness_s must be finite and positive')
        self.camera_topics = {
            name: spec['topic'] for name, spec in self.profile['cameras'].items()
        }
        self.input_received_at = {
            'joint_state': 0.0,
            **{name: 0.0 for name in self.camera_topics},
        }
        self.create_subscription(JointState, self.profile['observation']['topic'],
                                 self._joint_state, 10)
        for name, topic in self.camera_topics.items():
            self.create_subscription(
                Image, topic, lambda message, key=name: self._image(key, message), 10
            )
        self.create_timer(1.0 / float(self.profile['fps']), self._sample)
        self.create_service(FinalizeEpisode, '/record_episode/finalize', self.finalize)
        self.create_service(MarkEpisodeEvent, '/record_episode/mark_event', self.mark_event)
        self.server = ActionServer(
            self, RecordEpisode, '/record_episode', execute_callback=self.execute,
            goal_callback=self.goal, cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    def goal(self, goal):
        if not IDENTIFIER.fullmatch(goal.dataset_id) or not IDENTIFIER.fullmatch(goal.episode_id):
            return GoalResponse.REJECT
        if goal.collection_profile_id != self.profile['profile_id']:
            return GoalResponse.REJECT
        maximum = duration_seconds(goal.max_duration)
        if maximum <= 0.0 or maximum > MAX_RECORDING_DURATION_S:
            return GoalResponse.REJECT
        with self.lock:
            if self.active or self.goal_reserved:
                return GoalResponse.REJECT
            self.goal_reserved = True
            self.cancel_requested = False
            self.committing = False
        return GoalResponse.ACCEPT

    def cancel(self, _goal_handle):
        with self.lock:
            if self.committing or not (self.active or self.goal_reserved):
                return CancelResponse.REJECT
            self.cancel_requested = True
        return CancelResponse.ACCEPT

    def _canceling(self, handle) -> bool:
        with self.lock:
            requested = self.cancel_requested
        return requested or handle.is_cancel_requested

    def _joint_state(self, message):
        if len(message.name) != len(message.position):
            return
        values = {
            name: float(value)
            for name, value in zip(message.name, message.position)
            if math.isfinite(value)
        }
        with self.lock:
            self.latest_state = values
            self.input_received_at['joint_state'] = time.monotonic()

    def _image(self, name, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
        except Exception as error:
            self.get_logger().warning(f'could not decode {name} image: {error}')
            return
        with self.lock:
            self.latest_images[name] = image
            self.input_received_at[name] = time.monotonic()

    def _ready(self, now: float | None = None) -> bool:
        """Return true only while every required live input is fresh."""
        current = time.monotonic() if now is None else now
        required = ('joint_state', *self.camera_topics)
        return (
            all(name in self.latest_state for name in JOINT_NAMES)
            and all(name in self.latest_images for name in self.camera_topics)
            and all(
                self.input_received_at.get(name, 0.0) > 0.0
                and current - self.input_received_at[name]
                <= self.input_freshness_s
                for name in required
            )
        )

    def _sample(self):
        with self.lock:
            if (
                not self.active or not self.armed or self.stop.is_set()
                or self.writer is None
            ):
                return
            if not self._ready():
                self.sample_error = (
                    'Follower joint state or dual-camera input became stale'
                )
                self.stop.set()
                return
            state = [self.latest_state[name] for name in JOINT_NAMES]
            images = {name: image.copy() for name, image in self.latest_images.items()}
            elapsed = time.monotonic() - self.started_at
            writer = self.writer
        append_error = None
        with self.io_lock:
            try:
                if (
                    self.stop.is_set()
                    or not self.active
                    or not self.armed
                    or writer is not self.writer
                ):
                    return
                writer.append(state, images, elapsed)
            except Exception as error:
                # Latch failure while still owning the same I/O lock that
                # guards finish(), so a failed append can never race a commit.
                if writer is self.writer:
                    self.sample_error = str(error)
                    self.armed = False
                    self.stop.set()
                append_error = error
        if append_error is not None:
            self.get_logger().error(
                f'episode sample failed: {append_error}'
            )
            return
        with self.lock:
            if (
                writer is not self.writer
                or not self.active
                or self.stop.is_set()
            ):
                return
            self.frame_count = len(writer.states)
            handle = self.goal_handle
        if handle is not None:
            feedback = RecordEpisode.Feedback()
            feedback.state.phase = 'RECORDING'
            feedback.state.progress = 0.5
            feedback.state.message = 'recording NPZ and head/wrist MP4'
            feedback.frame_count = self.frame_count
            feedback.elapsed_s = elapsed
            feedback.required_topics_ready = True
            handle.publish_feedback(feedback)

    def finalize(self, request, response):
        with self.lock:
            if (
                not self.active
                or request.dataset_id != self.dataset_id
                or request.episode_id != self.episode_id
                or self.writer is None
            ):
                response.error.code = CapabilityError.NOT_FOUND
                response.error.message = 'episode is not recording'
                return response
            if not self.writer.manifest.get('teleop_disabled_at'):
                response.error.code = CapabilityError.INVALID_GOAL
                response.error.message = (
                    'teleoperation must be confirmed disabled before finalization'
                )
                return response
            self.armed = False
            self.stop.set()
        response.error.code = CapabilityError.NONE
        response.error.message = 'graceful finalization requested'
        return response

    def mark_event(self, request, response):
        with self.lock:
            if (
                not self.active
                or request.dataset_id != self.dataset_id
                or request.episode_id != self.episode_id
                or self.writer is None
            ):
                response.error.code = CapabilityError.NOT_FOUND
                response.error.message = 'episode is not recording'
                return response
            if request.event not in {'teleop_enabled', 'teleop_disabled'}:
                response.error.code = CapabilityError.INVALID_GOAL
                response.error.message = 'unsupported episode event'
                return response
            if self.stop.is_set() or self.committing:
                response.error.code = CapabilityError.INVALID_GOAL
                response.error.message = (
                    'episode is already stopping or committing'
                )
                return response
            if (
                request.event == 'teleop_enabled'
                and self.armed
                and self.writer.manifest.get('teleop_enabled_at')
            ):
                response.recorded_at = self.get_clock().now().to_msg()
                response.error.code = CapabilityError.NONE
                response.error.message = 'episode recording is already enabled'
                return response
            if request.event == 'teleop_disabled':
                if self.writer.manifest.get('teleop_disabled_at'):
                    response.recorded_at = self.get_clock().now().to_msg()
                    response.error.code = CapabilityError.NONE
                    response.error.message = (
                        'teleoperation disable boundary is already recorded'
                    )
                    return response
                if (
                    not self.armed
                    or not self.writer.manifest.get('teleop_enabled_at')
                ):
                    response.error.code = CapabilityError.INVALID_GOAL
                    response.error.message = (
                        'teleoperation was not recorded as enabled'
                    )
                    return response
                try:
                    with self.io_lock:
                        if self.stop.is_set() or self.committing:
                            response.error.code = CapabilityError.INVALID_GOAL
                            response.error.message = (
                                'episode became terminal before teleoperation '
                                'disable boundary'
                            )
                            return response
                        self.writer.mark_teleop_disabled()
                except Exception as error:
                    response.error.code = CapabilityError.BACKEND_FAILURE
                    response.error.message = (
                        'could not persist teleoperation disable boundary: '
                        f'{error}'
                    )
                    return response
                # Stop accepting timer samples, but wait for the separately
                # scoped FinalizeEpisode request before allowing commit.
                self.armed = False
                response.recorded_at = self.get_clock().now().to_msg()
                response.error.code = CapabilityError.NONE
                response.error.message = (
                    'teleoperation disable boundary recorded'
                )
                return response
            if self.writer.manifest.get('teleop_disabled_at'):
                response.error.code = CapabilityError.INVALID_GOAL
                response.error.message = (
                    'recording cannot restart after teleoperation was disabled'
                )
                return response
            if self.writer.manifest.get('teleop_enabled_at'):
                response.error.code = CapabilityError.INVALID_GOAL
                response.error.message = (
                    'recording enable boundary is already terminal'
                )
                return response
            if not self._ready():
                response.error.code = CapabilityError.UNAVAILABLE
                response.error.message = (
                    'fresh Follower joint state and both cameras are required '
                    'to start recording'
                )
                return response
            try:
                with self.io_lock:
                    if self.stop.is_set() or self.committing:
                        response.error.code = CapabilityError.INVALID_GOAL
                        response.error.message = (
                            'episode became terminal before recording start'
                        )
                        return response
                    self.writer.mark_teleop_enabled()
            except Exception as error:
                response.error.code = CapabilityError.BACKEND_FAILURE
                response.error.message = (
                    f'could not persist recording start event: {error}'
                )
                return response
            self.started_at = time.monotonic()
            self.sample_error = ''
            self.armed = True
            handle = self.goal_handle
        if handle is not None:
            feedback = RecordEpisode.Feedback()
            feedback.state.phase = 'ARMED'
            feedback.state.progress = 0.05
            feedback.state.message = 'Follower next-state recorder is active'
            feedback.required_topics_ready = True
            handle.publish_feedback(feedback)
        response.recorded_at = self.get_clock().now().to_msg()
        response.error.code = CapabilityError.NONE
        response.error.message = 'episode recording enabled'
        return response

    def execute(self, handle):
        request = handle.request
        result = RecordEpisode.Result()
        writer = None
        committed = False
        try:
            dataset = self.dataset_root / request.dataset_id / 'raw'
            dataset.mkdir(parents=True, exist_ok=True)
            if shutil.disk_usage(dataset).free < self.minimum_free_bytes:
                result.error.code = CapabilityError.BACKEND_FAILURE
                result.error.message = 'insufficient disk space'
                handle.abort()
                return result
            writer = EpisodeWriter(
                dataset,
                request.episode_id,
                instruction=request.language_instruction,
                object_label=request.object_label,
                profile_id=request.collection_profile_id,
                revision=self.revision,
                unit_id=self.unit_id,
                calibration_version=self.calibration_version,
                fps=int(self.profile['fps']),
            )
            writer.begin()
            self.stop.clear()
            with self.lock:
                self.active = True
                self.armed = False
                self.dataset_id = request.dataset_id
                self.episode_id = request.episode_id
                self.writer = writer
                self.goal_handle = handle
                self.started_at = time.monotonic()
                self.frame_count = 0
                self.sample_error = ''
            ready_deadline = time.monotonic() + 6.0
            while rclpy.ok():
                if self._canceling(handle):
                    break
                with self.lock:
                    ready = self._ready()
                if ready:
                    break
                if time.monotonic() >= ready_deadline:
                    raise RuntimeError('Follower joint state or camera topic is not ready')
                feedback = RecordEpisode.Feedback()
                feedback.state.phase = 'WAITING_FOR_TOPICS'
                feedback.state.progress = 0.0
                feedback.state.message = 'waiting for Follower joints and both cameras'
                feedback.required_topics_ready = False
                handle.publish_feedback(feedback)
                time.sleep(0.1)
            if self._canceling(handle):
                with self.lock:
                    self.armed = False
                    self.stop.set()
                with self.io_lock:
                    writer.abort('canceled before armed')
                result.error.code = CapabilityError.CANCELED
                result.error.message = 'episode canceled before recorder armed'
                handle.canceled()
                return result
            if not rclpy.ok():
                raise RuntimeError(
                    'ROS shutdown before recorder armed; incomplete data retained'
                )
            feedback = RecordEpisode.Feedback()
            feedback.state.phase = 'READY'
            feedback.state.progress = 0.02
            feedback.state.message = (
                'dataset, disk, Follower joints, and both cameras are ready'
            )
            feedback.required_topics_ready = True
            handle.publish_feedback(feedback)
            maximum = duration_seconds(request.max_duration)
            start_deadline = time.monotonic() + START_EVENT_TIMEOUT_S
            start_wait_expired = False
            while rclpy.ok() and not self.stop.is_set():
                if self._canceling(handle):
                    break
                with self.lock:
                    recording_started = self.armed
                if recording_started:
                    break
                if time.monotonic() >= start_deadline:
                    start_wait_expired = True
                    break
                time.sleep(0.05)
            if self._canceling(handle):
                with self.lock:
                    self.armed = False
                    self.stop.set()
                with self.io_lock:
                    writer.abort('canceled before recording started')
                result.error.code = CapabilityError.CANCELED
                result.error.message = (
                    'episode canceled before recording started; incomplete '
                    'data retained'
                )
                handle.canceled()
                return result
            if not rclpy.ok():
                raise RuntimeError(
                    'ROS shutdown before recording started; incomplete data retained'
                )
            if start_wait_expired:
                raise RuntimeError(
                    'recording start event timed out; incomplete data retained'
                )
            with self.lock:
                recording_started = self.armed
            if not recording_started:
                raise RuntimeError(
                    'finalize arrived before recording started; incomplete '
                    'data retained'
                )
            failsafe_expired = False
            while rclpy.ok() and not self.stop.wait(0.05):
                if self._canceling(handle):
                    break
                if time.monotonic() - self.started_at >= maximum:
                    failsafe_expired = True
                    break
            with self.lock:
                self.armed = False
                sample_error = self.sample_error
                cancel_before_commit = (
                    self.cancel_requested or handle.is_cancel_requested
                )
                if not cancel_before_commit and not sample_error:
                    self.committing = True
            if cancel_before_commit:
                self.stop.set()
                with self.io_lock:
                    writer.abort('canceled')
                result.error.code = CapabilityError.CANCELED
                result.error.message = 'episode canceled; incomplete data retained'
                handle.canceled()
                return result
            if not rclpy.ok():
                raise RuntimeError(
                    'ROS shutdown before coordinated finalize; incomplete data retained'
                )
            if sample_error:
                raise RuntimeError(f'episode sampling failed: {sample_error}')
            if failsafe_expired:
                raise RuntimeError(
                    'recorder fail-safe deadline expired before coordinated '
                    'finalize; incomplete data retained'
                )
            with self.io_lock:
                if self.sample_error:
                    raise RuntimeError(
                        f'episode sampling failed: {self.sample_error}'
                    )
                if not writer.manifest.get('teleop_enabled_at'):
                    raise RuntimeError(
                        'teleop_enabled event is missing; incomplete data retained'
                    )
                if not writer.manifest.get('teleop_disabled_at'):
                    raise RuntimeError(
                        'teleop_disabled event is missing; incomplete data retained'
                    )
                output = writer.finish('user')
            committed = True
            result.episode_uri = output.as_uri()
            result.topic_names = [
                self.profile['observation']['topic'],
                *self.camera_topics.values(),
            ]
            result.message_counts = [len(writer.states)] * len(result.topic_names)
            result.duration_s = writer.timestamps[-1]
            result.error.code = CapabilityError.NONE
            result.error.message = 'episode finalized atomically'
            handle.succeed()
            return result
        except Exception as error:
            abort_error = None
            if writer is not None and not committed:
                try:
                    with self.lock:
                        self.armed = False
                        self.stop.set()
                    with self.io_lock:
                        writer.abort(str(error))
                except Exception as failure:
                    abort_error = failure
            result.error.code = CapabilityError.BACKEND_FAILURE
            result.error.message = str(error)
            if abort_error is not None:
                result.error.message += (
                    f'; incomplete episode cleanup failed: {abort_error}'
                )
            handle.abort()
            return result
        finally:
            # Drain any timer callback that captured the old writer before
            # allowing the next goal reservation to be accepted.
            with self.io_lock:
                pass
            with self.lock:
                self.active = False
                self.goal_reserved = False
                self.cancel_requested = False
                self.committing = False
                self.armed = False
                self.dataset_id = ''
                self.episode_id = ''
                self.writer = None
                self.goal_handle = None


def main():
    rclpy.init()
    node = RecordEpisodeNode()
    executor = MultiThreadedExecutor(num_threads=3)
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
