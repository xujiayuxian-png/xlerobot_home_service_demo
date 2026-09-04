"""Read-only nearest-person localization from one synchronized RGBD view."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
import threading
import time

import numpy as np

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
from xlerobot_interfaces.action import ScanForPerson
from xlerobot_interfaces.msg import CapabilityError, PerceptionObservation
from xlerobot_perception.detection.yolo import YoloPersonDetector
from xlerobot_perception.execution_modes import validate_dry_run_mode
from xlerobot_perception.geometry.pinhole import project_pixel_to_point
from xlerobot_perception.geometry.transforms import transform_point
from xlerobot_perception.observations import make_observation
from xlerobot_perception.rgbd.depth import depth_median
from xlerobot_perception.rgbd.frame_buffer import RgbdFrameBuffer
from xlerobot_perception.rgbd.frame_buffer import stamp_ns
from xlerobot_perception.rgbd.images import color_to_bgr


_FRAME_PATTERN = re.compile(r'^[A-Za-z][A-Za-z0-9_/]*$')


@dataclass
class ScanFailure(RuntimeError):
    """Internal failure converted to the stable capability error contract."""

    code: int
    message: str

    def __str__(self):
        return self.message


class ScanForPersonNode(Node):
    """Select the nearest locally detected person without commanding motion."""

    def __init__(
        self, *, parameter_overrides=None, detector_factory=YoloPersonDetector
    ) -> None:
        super().__init__('scan_for_person_detector', parameter_overrides=parameter_overrides)
        self.group = ReentrantCallbackGroup()
        self.backend_enabled = bool(self.declare_parameter('backend_enabled', False).value)
        self.dry_run_mode = str(self.declare_parameter(
            'dry_run_mode', 'contract_only'
        ).value)
        self.supported_recipient = str(self.declare_parameter(
            'supported_recipient', 'nearest_person'
        ).value)
        self.action_name = str(self.declare_parameter(
            'action_name', 'scan_for_person'
        ).value)
        self.target_frame = str(self.declare_parameter('target_frame', 'map').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
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
        self.min_depth_m = float(self.declare_parameter('min_depth_m', 0.20).value)
        self.max_depth_m = float(self.declare_parameter('max_depth_m', 5.0).value)
        model_path = str(self.declare_parameter('model_path', '').value).strip()
        if not model_path:
            model_dir = Path(os.environ.get(
                'XLEROBOT_MODEL_DIR', '.xlerobot/models'
            ))
            model_path = str(model_dir / 'yolov8n.pt')
        self.model_path = os.path.expandvars(os.path.expanduser(model_path))
        self.yolo_conf = float(self.declare_parameter('yolo_conf', 0.60).value)
        self.yolo_imgsz = int(self.declare_parameter('yolo_imgsz', 480).value)
        self.yolo_device = str(self.declare_parameter('yolo_device', 'cpu').value)
        self._validate_parameters()
        self.detector_factory = detector_factory
        self.detector = None
        self.detector_lock = threading.Lock()
        self.detector_ready = threading.Event()
        self.detector_error = None

        self.frames = RgbdFrameBuffer()
        self._goal_lock = threading.Lock()
        self._goal_active = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.observation_publisher = self.create_publisher(
            PerceptionObservation, '/perception/observations', 10
        )
        self.create_subscription(
            Image, self.color_topic, lambda message: self.frames.store('color', message),
            qos_profile_sensor_data, callback_group=self.group,
        )
        self.create_subscription(
            Image, self.depth_topic, lambda message: self.frames.store('depth', message),
            qos_profile_sensor_data, callback_group=self.group,
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic,
            lambda message: self.frames.store('info', message),
            qos_profile_sensor_data, callback_group=self.group,
        )
        self.server = ActionServer(
            self,
            ScanForPerson,
            self.action_name,
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.group,
        )
        mode = 'enabled' if self.backend_enabled else 'disabled (contract-only)'
        self.get_logger().info(
            f'ScanForPerson local YOLO action ready; backend {mode}'
        )
        if self.backend_enabled:
            self.detector_thread = threading.Thread(
                target=self._preload_detector,
                name='person-yolo-preload',
                daemon=True,
            )
            self.detector_thread.start()

    def _validate_parameters(self):
        validate_dry_run_mode(self.dry_run_mode)
        if not self.supported_recipient or not _FRAME_PATTERN.fullmatch(self.target_frame):
            raise ValueError('supported recipient and target frame must be valid')
        if not _FRAME_PATTERN.fullmatch(self.base_frame):
            raise ValueError('base_frame must be a relative ROS frame id')
        positive = (
            self.sync_tolerance_s,
            self.sensor_timeout_s,
            self.max_sensor_age_s,
            self.tf_timeout_s,
            self.min_depth_m,
            self.max_depth_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('scan bounds must be finite and positive')
        if self.max_depth_m <= self.min_depth_m:
            raise ValueError('max_depth_m must be greater than min_depth_m')
        if (
            not self.model_path or not 0.0 < self.yolo_conf <= 1.0
            or self.yolo_imgsz <= 0 or not self.yolo_device
        ):
            raise ValueError('YOLO person detector parameters are invalid')

    def goal_callback(self, request):
        if request.recipient_id.strip() != self.supported_recipient:
            self.get_logger().warning(
                f'Cannot identify recipient: {request.recipient_id!r}; '
                f'only {self.supported_recipient!r} is supported'
            )
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_active:
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        result = ScanForPerson.Result()
        try:
            self._feedback(goal_handle, 'validating', 0.05, 'goal accepted')
            if goal_handle.request.dry_run and self.dry_run_mode == 'contract_only':
                result.person.header.frame_id = self.target_frame
                result.error.code = CapabilityError.NONE
                result.error.message = (
                    'contract-only scan validated; no sensor or VLM request made'
                )
                result.distance_m = float('nan')
                goal_handle.succeed()
                self._feedback(goal_handle, 'complete', 1.0, result.error.message)
                return result
            if not self.backend_enabled:
                raise ScanFailure(
                    CapabilityError.SAFETY_REJECTED,
                    'person VLM backend is disabled; set backend_enabled=true explicitly',
                )

            color, depth, info = self._snapshot(goal_handle)
            stamp = Time.from_msg(depth.header.stamp)
            camera_frame = info.header.frame_id or depth.header.frame_id
            if not camera_frame:
                raise ScanFailure(CapabilityError.UNAVAILABLE, 'camera frame id is empty')

            # Preserve the transforms that belong to this RGBD frame before
            # running the detector.  The first YOLO invocation may take longer
            # than tf2's history window while it loads model state; looking up
            # the transforms after inference would then reject an otherwise
            # current image as too old.
            base_transform = self._lookup(self.target_frame, self.base_frame, stamp)
            camera_transform = None
            if camera_frame != self.target_frame:
                camera_transform = self._lookup(
                    self.target_frame, camera_frame, stamp
                )
            self._feedback(goal_handle, 'detecting', 0.30, 'running local person detector')
            try:
                detections, latency_s = self._detect(color_to_bgr(color))
            except Exception as exc:
                raise ScanFailure(CapabilityError.BACKEND_FAILURE, str(exc)) from exc
            self._check_canceled(goal_handle)
            if not detections:
                raise ScanFailure(CapabilityError.NOT_FOUND, 'no person detected in current view')

            base_x = float(base_transform.transform.translation.x)
            base_y = float(base_transform.transform.translation.y)
            candidates = []
            for detection in detections:
                localized = self._localize(
                    detection, depth, info, camera_frame, camera_transform
                )
                if localized is None:
                    continue
                point, depth_m = localized
                distance_m = math.hypot(point.point.x - base_x, point.point.y - base_y)
                if math.isfinite(distance_m):
                    self.get_logger().info(
                        f'person bbox={detection.bbox_px} '
                        f'confidence={detection.confidence:.3f} '
                        f'depth={depth_m:.3f}m '
                        f'target_map=({point.point.x:.3f}, {point.point.y:.3f}, '
                        f'{point.point.z:.3f}) distance={distance_m:.3f}m'
                    )
                    observation = make_observation(
                        kind='person',
                        label='person',
                        camera_id='head',
                        image=color,
                        bbox=detection.bbox_px,
                        confidence=detection.confidence,
                        target=point,
                    )
                    candidates.append((
                        distance_m, -float(detection.confidence), point, observation
                    ))
            if not candidates:
                raise ScanFailure(
                    CapabilityError.NOT_FOUND,
                    'person detections had no valid depth and timestamped TF',
                )

            distance_m, _negative_confidence, result.person, result.observation = min(
                candidates, key=lambda item: (item[0], item[1])
            )
            self.observation_publisher.publish(result.observation)
            result.distance_m = float(distance_m)
            result.error.code = CapabilityError.NONE
            result.error.message = (
                f'nearest current-view person localized; YOLO latency {latency_s:.2f}s'
            )
            goal_handle.succeed()
            self._feedback(goal_handle, 'complete', 1.0, result.error.message)
            return result
        except ScanFailure as exc:
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
        # PersonSearch sends this view action only after the head trajectory and
        # settle delay have completed.  Require a complete RGBD tuple produced
        # after this action started so a still-valid frame from head motion can
        # never be selected.
        action_started_ns = self.get_clock().now().nanoseconds
        try:
            snapshot = self.frames.wait_snapshot(
                timeout_s=self.sensor_timeout_s,
                sync_tolerance_s=self.sync_tolerance_s,
                max_age_s=self.max_sensor_age_s,
                min_stamp_ns=action_started_ns,
                now_ns=lambda: self.get_clock().now().nanoseconds,
                check_interrupt=lambda: self._check_canceled(goal_handle),
            )
            self.get_logger().info(
                'using newest post-head-settle person RGBD frame: '
                f'color_stamp_ns={stamp_ns(snapshot[0])}, '
                f'depth_stamp_ns={stamp_ns(snapshot[1])}, '
                f'info_stamp_ns={stamp_ns(snapshot[2])}'
            )
            return snapshot
        except TimeoutError as exc:
            raise ScanFailure(CapabilityError.TIMEOUT, str(exc)) from exc

    def _localize(self, detection, depth, info, camera_frame, camera_transform):
        x1, y1, x2, y2 = detection.bbox_px
        u = 0.5 * (x1 + x2)
        v = 0.5 * (y1 + y2)
        depth_m = depth_median(depth, u, v)
        if depth_m is None or not self.min_depth_m <= depth_m <= self.max_depth_m:
            return None
        camera_point = project_pixel_to_point(
            u, v, depth_m, info, frame_id=camera_frame, stamp=depth.header.stamp
        )
        if camera_frame == self.target_frame:
            camera_point.header.frame_id = self.target_frame
            return camera_point, depth_m
        return (
            transform_point(camera_point, camera_transform),
            depth_m,
        )

    def _detect(self, bgr):
        with self.detector_lock:
            if self.detector_error is not None:
                raise RuntimeError(
                    f'person detector failed during startup: {self.detector_error}'
                )
            if self.detector is None:
                self.detector = self._create_detector()
            return self.detector.detect(bgr)

    def _create_detector(self):
        if not Path(self.model_path).is_file():
            raise RuntimeError(f'person model does not exist: {self.model_path}')
        return self.detector_factory(
            self.model_path,
            confidence=self.yolo_conf,
            image_size=self.yolo_imgsz,
            device=self.yolo_device,
        )

    def _preload_detector(self):
        started = time.monotonic()
        try:
            with self.detector_lock:
                if self.detector is None:
                    self.detector = self._create_detector()
                # Run the complete inference path once. Loading weights alone still
                # leaves the first real person view paying framework warm-up cost.
                warmup = np.zeros(
                    (self.yolo_imgsz, self.yolo_imgsz, 3), dtype=np.uint8
                )
                self.detector.detect(warmup)
            elapsed_s = time.monotonic() - started
            self.get_logger().info(
                f'Person YOLO preloaded and warmed in {elapsed_s:.2f}s'
            )
        except Exception as exc:
            with self.detector_lock:
                self.detector = None
                self.detector_error = exc
            self.get_logger().error(f'Person YOLO preload failed: {exc}')
        finally:
            self.detector_ready.set()

    def _lookup(self, target_frame, source_frame, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException as exc:
            raise ScanFailure(
                CapabilityError.UNAVAILABLE,
                f'missing timestamped TF {source_frame}->{target_frame}: {exc}',
            ) from exc

    @staticmethod
    def _check_canceled(goal_handle):
        if goal_handle.is_cancel_requested:
            raise ScanFailure(CapabilityError.CANCELED, 'scan for person canceled')

    @staticmethod
    def _feedback(goal_handle, phase, progress, message):
        feedback = ScanForPerson.Feedback()
        feedback.state.phase = phase
        feedback.state.progress = float(progress)
        feedback.state.message = message
        goal_handle.publish_feedback(feedback)


def main():
    """Run the read-only current-view person localization server."""
    rclpy.init()
    node = ScanForPersonNode()
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
