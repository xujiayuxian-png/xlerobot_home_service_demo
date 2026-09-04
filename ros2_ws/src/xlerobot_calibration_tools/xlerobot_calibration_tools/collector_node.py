"""Read-only TF sampler for offline camera/hand-eye calibration."""

from __future__ import annotations

from pathlib import Path
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
        self._observation = None
        self.create_subscription(
            CalibrationTargetObservation,
            '/calibration/target_observation',
            self._on_observation,
            qos_profile_sensor_data,
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
        self._observation = message

    def _capture(self, request, response):
        with self._lock:
            try:
                if not request.job_id.strip():
                    raise ValueError('job_id is required')
                observation = self._observation
                if observation is None or not observation.accepted:
                    raise ValueError('no accepted calibration target is visible')
                visual_stamp = Time.from_msg(observation.header.stamp)
                age_sec = (
                    self.get_clock().now() - visual_stamp
                ).nanoseconds / 1.0e9
                if age_sec < 0 or age_sec > self._max_target_age_sec:
                    raise ValueError(
                        f'target observation age {age_sec:.3f}s exceeds limit'
                    )
                visual = self._buffer.lookup_transform(
                    self._samples.camera_frame,
                    self._samples.target_frame,
                    visual_stamp,
                )
                moving = self._buffer.lookup_transform(
                    self._samples.base_frame,
                    self._samples.moving_frame,
                    visual_stamp,
                )
                sample = self._samples.append_and_write_atomic(
                    self._output,
                    transform_to_matrix(moving.transform),
                    transform_to_matrix(visual.transform),
                    visual_stamp.nanoseconds / 1.0e9,
                    {
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
    try:
        node = TransformSampleCollector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
