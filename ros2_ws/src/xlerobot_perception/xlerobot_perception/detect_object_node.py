"""Read-only RGBD object localization action backed by an explicit VLM gate."""

from __future__ import annotations

from dataclasses import dataclass
import re
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from xlerobot_interfaces.action import DetectObject
from xlerobot_interfaces.msg import CapabilityError, PerceptionObservation
from xlerobot_perception.execution_modes import validate_dry_run_mode
from xlerobot_perception.geometry.pinhole import project_pixel_to_point
from xlerobot_perception.geometry.transforms import transform_point
from xlerobot_perception.observations import make_observation
from xlerobot_perception.rgbd.depth import depth_median
from xlerobot_perception.rgbd.frame_buffer import RgbdFrameBuffer
from xlerobot_perception.rgbd.frame_buffer import stamp_ns
from xlerobot_perception.rgbd.images import color_to_bgr
from xlerobot_perception.vlm.lmstudio import LmStudioVlmClient


_FRAME_PATTERN = re.compile(r'^[A-Za-z][A-Za-z0-9_/]*$')


@dataclass
class CapabilityFailure(RuntimeError):
    """Internal exception converted to a typed action result."""

    code: int
    message: str

    def __str__(self):
        return self.message


def validate_goal_fields(object_id: str, target_frame: str) -> None:
    """Validate identifiers before reserving the single backend slot."""
    if not object_id.strip() or len(object_id) > 128:
        raise ValueError('object_id must contain 1 to 128 characters')
    if any(ord(character) < 32 for character in object_id):
        raise ValueError('object_id must not contain control characters')
    if not _FRAME_PATTERN.fullmatch(target_frame):
        raise ValueError('target_frame must be a nonempty relative ROS frame id')


class DetectObjectNode(Node):
    """Localize an object without publishing any command or motion topic."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__('detect_object_vlm', parameter_overrides=parameter_overrides)
        self.group = ReentrantCallbackGroup()
        self.backend_enabled = bool(self.declare_parameter('backend_enabled', False).value)
        self.dry_run_mode = str(self.declare_parameter(
            'dry_run_mode', 'contract_only'
        ).value)
        self.color_topic = str(self.declare_parameter(
            'color_topic', '/xlerobot/d455/color/image_raw'
        ).value)
        self.depth_topic = str(self.declare_parameter(
            'depth_topic', '/xlerobot/d455/aligned_depth_to_color/image_raw'
        ).value)
        self.camera_info_topic = str(self.declare_parameter(
            'camera_info_topic', '/xlerobot/d455/color/camera_info'
        ).value)
        self.sync_tolerance_s = float(self.declare_parameter(
            'rgbd_sync_tolerance_s', 0.20
        ).value)
        self.sensor_timeout_s = float(self.declare_parameter('sensor_timeout_s', 8.0).value)
        self.max_sensor_age_s = float(self.declare_parameter('max_sensor_age_s', 1.0).value)
        self.tf_timeout_s = float(self.declare_parameter('tf_timeout_s', 3.0).value)
        self.min_depth_m = float(self.declare_parameter('min_depth_m', 0.10).value)
        self.max_depth_m = float(self.declare_parameter('max_depth_m', 4.0).value)
        self._validate_parameters()
        self.vlm = LmStudioVlmClient(
            base_url=str(self.declare_parameter(
                'vlm_base_url', 'http://127.0.0.1:1234'
            ).value),
            model=str(self.declare_parameter('vlm_model', 'qwen/qwen3-vl-4b').value),
            timeout_s=float(self.declare_parameter('vlm_timeout_s', 30.0).value),
            bbox_format=str(self.declare_parameter('vlm_bbox_format', 'norm1000').value),
            jpeg_quality=int(self.declare_parameter('vlm_jpeg_quality', 90).value),
            request_long_edge_px=int(self.declare_parameter(
                'vlm_request_long_edge_px', 1280
            ).value),
        )

        self.frames = RgbdFrameBuffer()
        self._goal_lock = threading.Lock()
        self._goal_active = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.observation_publisher = self.create_publisher(
            PerceptionObservation, '/perception/observations', 10
        )
        self.create_subscription(
            Image, self.color_topic, self._on_color, qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.create_subscription(
            Image, self.depth_topic, self._on_depth, qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_info, qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.server = ActionServer(
            self,
            DetectObject,
            'detect_object',
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group,
        )
        mode = 'enabled' if self.backend_enabled else 'disabled (dry-run only)'
        self.get_logger().info(f'DetectObject VLM action ready; backend {mode}')

    def _validate_parameters(self):
        positive = {
            'rgbd_sync_tolerance_s': self.sync_tolerance_s,
            'sensor_timeout_s': self.sensor_timeout_s,
            'max_sensor_age_s': self.max_sensor_age_s,
            'tf_timeout_s': self.tf_timeout_s,
            'min_depth_m': self.min_depth_m,
            'max_depth_m': self.max_depth_m,
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            raise ValueError(f'parameters must be positive: {invalid}')
        if self.max_depth_m <= self.min_depth_m:
            raise ValueError('max_depth_m must be greater than min_depth_m')
        validate_dry_run_mode(self.dry_run_mode)

    def goal_callback(self, request):
        try:
            validate_goal_fields(request.object_id, request.target_frame)
        except ValueError as exc:
            self.get_logger().warning(f'Rejecting DetectObject goal: {exc}')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_active:
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _on_color(self, message):
        self._store('color', message)

    def _on_depth(self, message):
        self._store('depth', message)

    def _on_info(self, message):
        self._store('info', message)

    def _store(self, key, message):
        self.frames.store(key, message)

    def execute(self, goal_handle):
        result = DetectObject.Result()
        try:
            self._feedback(goal_handle, 'validating', 0.05, 'goal accepted')
            if goal_handle.request.dry_run and self.dry_run_mode == 'contract_only':
                result.target.header.frame_id = goal_handle.request.target_frame
                result.error.code = CapabilityError.NONE
                result.error.message = 'dry-run validated; no sensor or VLM request made'
                result.confidence = 0.0
                goal_handle.succeed()
                self._feedback(goal_handle, 'complete', 1.0, result.error.message)
                return result
            if not self.backend_enabled:
                raise CapabilityFailure(
                    CapabilityError.SAFETY_REJECTED,
                    'VLM backend is disabled; set backend_enabled=true explicitly',
                )

            self._feedback(
                goal_handle, 'waiting_fresh_frame', 0.20,
                'waiting for post-head-arrival RGBD frame',
            )
            color, depth, info = self._snapshot(goal_handle)
            self.get_logger().info(
                'using newest post-head-arrival RGBD frame: '
                f'color_stamp_ns={stamp_ns(color)}, depth_stamp_ns={stamp_ns(depth)}, '
                f'info_stamp_ns={stamp_ns(info)}'
            )
            self._feedback(goal_handle, 'detecting', 0.35, 'requesting VLM grounding')
            try:
                detections, latency_s = self.vlm.detect(
                    color_to_bgr(color), goal_handle.request.object_id.strip()
                )
            except Exception as exc:
                raise CapabilityFailure(CapabilityError.BACKEND_FAILURE, str(exc)) from exc
            self._check_canceled(goal_handle)
            if not detections:
                raise CapabilityFailure(
                    CapabilityError.NOT_FOUND,
                    f'object not found: {goal_handle.request.object_id.strip()}',
                )
            detection = max(detections, key=lambda item: item.confidence)
            x1, y1, x2, y2 = detection.bbox_px
            u = 0.5 * (x1 + x2)
            v = 0.5 * (y1 + y2)
            # Match the proven prototype: sample a small expanding window at
            # the VLM box center. The bbox edges never participate in depth,
            # so a box that also covers the robot/table border cannot raise the
            # target from those edge pixels.
            depth_m = depth_median(depth, u, v)
            if depth_m is None or not self.min_depth_m <= depth_m <= self.max_depth_m:
                raise CapabilityFailure(
                    CapabilityError.NOT_FOUND,
                    'detection has no depth inside the configured range',
                )
            camera_frame = info.header.frame_id or depth.header.frame_id
            if not camera_frame:
                raise CapabilityFailure(CapabilityError.UNAVAILABLE, 'camera frame id is empty')
            camera_point = project_pixel_to_point(
                u, v, depth_m, info, frame_id=camera_frame, stamp=depth.header.stamp
            )
            self._feedback(goal_handle, 'transforming', 0.80, 'looking up timestamped TF')
            result.target = self._target_point(
                camera_point, goal_handle.request.target_frame
            )
            result.observation = make_observation(
                kind='object',
                label=detection.label,
                camera_id='head',
                image=color,
                bbox=detection.bbox_px,
                confidence=detection.confidence,
                target=result.target,
            )
            self.observation_publisher.publish(result.observation)
            self.get_logger().info(
                f'grounded {detection.label} bbox={detection.bbox_px} '
                f'center=({u:.1f},{v:.1f}) depth={depth_m:.4f}m '
                f'target_{goal_handle.request.target_frame}=('
                f'{result.target.point.x:.5f},{result.target.point.y:.5f},'
                f'{result.target.point.z:.5f})'
            )
            result.confidence = float(detection.confidence)
            result.error.code = CapabilityError.NONE
            result.error.message = f'localized {detection.label}; VLM latency {latency_s:.2f}s'
            goal_handle.succeed()
            self._feedback(goal_handle, 'complete', 1.0, result.error.message)
            return result
        except CapabilityFailure as exc:
            result.error.code = int(exc.code)
            result.error.message = exc.message
            if exc.code == CapabilityError.CANCELED or goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self._feedback(goal_handle, 'failed', 1.0, exc.message)
            return result
        except Exception as exc:
            result.error.code = CapabilityError.INTERNAL_ERROR
            result.error.message = str(exc)
            goal_handle.abort()
            self._feedback(goal_handle, 'failed', 1.0, str(exc))
            return result
        finally:
            with self._goal_lock:
                self._goal_active = False

    def _snapshot(self, goal_handle):
        # The task moves the head before it sends DetectObject.  Do not reuse a
        # still-valid buffered frame captured during that motion: require the
        # complete RGBD tuple to have been produced after this action started.
        action_started_ns = self.get_clock().now().nanoseconds
        try:
            return self.frames.wait_snapshot(
                timeout_s=self.sensor_timeout_s,
                sync_tolerance_s=self.sync_tolerance_s,
                max_age_s=self.max_sensor_age_s,
                min_stamp_ns=action_started_ns,
                now_ns=lambda: self.get_clock().now().nanoseconds,
                check_interrupt=lambda: self._check_canceled(goal_handle),
            )
        except TimeoutError as exc:
            raise CapabilityFailure(CapabilityError.TIMEOUT, str(exc)) from exc

    def _target_point(self, camera_point, target_frame):
        if camera_point.header.frame_id == target_frame:
            camera_point.header.frame_id = target_frame
            return camera_point
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                camera_point.header.frame_id,
                Time.from_msg(camera_point.header.stamp),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException as exc:
            raise CapabilityFailure(
                CapabilityError.UNAVAILABLE,
                f'missing timestamped TF {camera_point.header.frame_id}->{target_frame}: {exc}',
            ) from exc
        return transform_point(camera_point, transform)

    @staticmethod
    def _check_canceled(goal_handle):
        if goal_handle.is_cancel_requested:
            raise CapabilityFailure(CapabilityError.CANCELED, 'detect object canceled')

    @staticmethod
    def _feedback(goal_handle, phase, progress, message):
        feedback = DetectObject.Feedback()
        feedback.state.phase = phase
        feedback.state.progress = float(progress)
        feedback.state.message = message
        goal_handle.publish_feedback(feedback)


def main():
    """Run the read-only detection action server."""
    rclpy.init()
    node = DetectObjectNode()
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
