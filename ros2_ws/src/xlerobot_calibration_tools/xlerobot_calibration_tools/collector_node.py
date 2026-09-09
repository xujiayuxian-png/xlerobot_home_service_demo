"""Read-only TF sampler for offline camera/hand-eye calibration."""

from __future__ import annotations

from pathlib import Path
from dataclasses import replace
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from xlerobot_interfaces.msg import CalibrationTargetObservation
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_interfaces.srv import CaptureCalibrationSample

from .sample_set import transform_to_matrix, TransformSampleSet
from .solver import HEAD_MODEL


class TransformSampleCollector(Node):
    """Capture synchronized TF pairs when explicitly triggered."""

    def __init__(self) -> None:
        super().__init__('transform_sample_collector')
        self.declare_parameter('calibration_id', 'calibration_run')
        self.declare_parameter('model', HEAD_MODEL)
        self.declare_parameter('output', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('moving_frame', 'head_tilt_link')
        self.declare_parameter('camera_frame', 'd455_color_optical_frame')
        self.declare_parameter('target_frame', 'calibration_target')
        self.declare_parameter('max_target_age_sec', 0.25)
        self.declare_parameter('min_translation_m', 0.002)
        self.declare_parameter('min_rotation_deg', 2.0)

        output = self.get_parameter('output').value
        if not output:
            raise ValueError('output parameter is required')
        self._output = Path(output)
        self._max_target_age_sec = float(
            self.get_parameter('max_target_age_sec').value
        )
        if self._max_target_age_sec <= 0:
            raise ValueError('max_target_age_sec must be positive')
        calibration_id = str(self.get_parameter('calibration_id').value)
        model = str(self.get_parameter('model').value)
        frames = {
            'base': str(self.get_parameter('base_frame').value),
            'moving': str(self.get_parameter('moving_frame').value),
            'camera': str(self.get_parameter('camera_frame').value),
            'target': str(self.get_parameter('target_frame').value),
        }
        min_translation_m = float(
            self.get_parameter('min_translation_m').value
        )
        min_rotation_deg = float(
            self.get_parameter('min_rotation_deg').value
        )
        if self._output.exists():
            self._samples = TransformSampleSet.read(
                self._output,
                expected_model=model,
                expected_calibration_id=calibration_id,
                expected_frames=frames,
                min_translation_m=min_translation_m,
                min_rotation_deg=min_rotation_deg,
            )
        else:
            self._samples = TransformSampleSet(
                calibration_id=calibration_id,
                model=model,
                base_frame=frames['base'],
                moving_frame=frames['moving'],
                camera_frame=frames['camera'],
                target_frame=frames['target'],
                min_translation_m=min_translation_m,
                min_rotation_deg=min_rotation_deg,
            )
        self._samples.validate()
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._lock = threading.Lock()
        self._observation_condition = threading.Condition()
        self._observation = None
        self.create_subscription(
            CalibrationTargetObservation,
            '/calibration/target_observation',
            self._on_observation,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=ReentrantCallbackGroup(),
        )
        self._service = self.create_service(
            CaptureCalibrationSample,
            '/calibration/capture_sample',
            self._capture,
        )
        self.get_logger().info(
            'Read-only collector ready with '
            f'{len(self._samples.samples)} restored samples; '
            'call ~/capture after each distinct pose'
        )

    def _on_observation(self, message):
        with self._observation_condition:
            self._observation = message
            self._observation_condition.notify_all()

    def _fresh_snapshot(self):
        """Wait briefly for fresh evidence and exact-stamp TF, never restamp it.

        Observation and TF callbacks remain runnable while the serialized capture
        service reads/writes YAML or waits. A late frame is not a failed pose if
        the next frame arrives within this bounded, stationary capture window.
        """
        deadline = time.monotonic() + 1.0
        detail = 'no accepted calibration target is visible'
        with self._observation_condition:
            while True:
                observation = self._observation
                if observation is not None and observation.accepted:
                    stamp = Time.from_msg(observation.header.stamp)
                    age = (self.get_clock().now() - stamp).nanoseconds / 1.0e9
                    if 0 <= age <= self._max_target_age_sec:
                        try:
                            visual = self._buffer.lookup_transform(
                                self._samples.camera_frame, self._samples.target_frame, stamp)
                            moving = self._buffer.lookup_transform(
                                self._samples.base_frame, self._samples.moving_frame, stamp)
                            return observation, stamp, age, visual, moving
                        except TransformException as error:
                            # TF and observation travel on separate topics.
                            detail = f'waiting for image-time TF: {error}'
                    else:
                        detail = f'target observation age {age:.3f}s exceeds limit'
                else:
                    detail = 'no accepted calibration target is visible'
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ValueError(f'{detail}; no synchronized fresh observation within 1 second')
                self._observation_condition.wait(timeout=min(.02, remaining))

    def _capture(self, request, response):
        with self._lock:
            try:
                # The sequencer may archive a completed/paused session without
                # restarting this read-only collector. Disk is authoritative.
                old = self._samples
                self._samples = TransformSampleSet.read(
                    self._output, expected_model=old.model,
                    expected_calibration_id=old.calibration_id,
                    expected_frames={'base': old.base_frame, 'moving': old.moving_frame,
                                     'camera': old.camera_frame, 'target': old.target_frame},
                    min_translation_m=old.min_translation_m,
                    min_rotation_deg=old.min_rotation_deg,
                ) if self._output.exists() else replace(old, samples=[], stamps_sec=[], qualities=[])
                if not request.job_id.strip():
                    raise ValueError('job_id is required')
                observation, visual_stamp, age_sec, visual, moving = self._fresh_snapshot()
                sample = self._samples.append_and_write_atomic(
                    self._output,
                    transform_to_matrix(moving.transform),
                    transform_to_matrix(visual.transform),
                    visual_stamp.nanoseconds / 1.0e9,
                    {
                        'capture_job_id': request.job_id,
                        'observation_age_sec': age_sec,
                        'tag_count': int(observation.tag_count),
                        'reprojection_rmse_px': float(
                            observation.reprojection_rmse_px
                        ),
                    },
                )
            except (TransformException, OSError, ValueError) as error:
                response.error.code = CapabilityError.INVALID_GOAL
                response.error.message = str(error)
                response.sample_count = len(self._samples.samples)
                return response
        response.error.code = CapabilityError.NONE
        response.error.message = f'captured sample {sample.index}'
        response.sample_count = len(self._samples.samples)
        return response


def main(args=None) -> None:
    """Run the read-only collector."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = TransformSampleCollector()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
