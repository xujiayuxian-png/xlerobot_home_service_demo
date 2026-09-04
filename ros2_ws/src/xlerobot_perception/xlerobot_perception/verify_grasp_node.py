"""Read-only post-grasp verification from a fresh wrist-camera frame."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from xlerobot_interfaces.action import VerifyGrasp
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_perception.execution_modes import validate_dry_run_mode
from xlerobot_perception.rgbd.frame_buffer import stamp_ns
from xlerobot_perception.rgbd.images import color_to_bgr
from xlerobot_perception.vlm.lmstudio import LmStudioVlmClient


@dataclass
class VerificationFailure(RuntimeError):
    code: int
    message: str

    def __str__(self):
        return self.message


class VerifyGraspNode(Node):
    """Expose one bounded visual check; this node has no command publishers."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__('verify_grasp_vlm', parameter_overrides=parameter_overrides)
        self.group = ReentrantCallbackGroup()
        self.backend_enabled = bool(self.declare_parameter('backend_enabled', False).value)
        self.dry_run_mode = str(self.declare_parameter(
            'dry_run_mode', 'contract_only'
        ).value)
        self.image_topic = str(self.declare_parameter(
            'image_topic', '/right_wrist_camera/image_raw'
        ).value)
        self.sensor_timeout_s = float(self.declare_parameter(
            'sensor_timeout_s', 3.0
        ).value)
        self.max_sensor_age_s = float(self.declare_parameter(
            'max_sensor_age_s', 1.0
        ).value)
        self.success_confidence = float(self.declare_parameter(
            'success_confidence', 0.70
        ).value)
        validate_dry_run_mode(self.dry_run_mode)
        if self.sensor_timeout_s <= 0.0 or self.max_sensor_age_s <= 0.0:
            raise ValueError('wrist image timeouts must be positive')
        if not 0.0 <= self.success_confidence <= 1.0:
            raise ValueError('success_confidence must be between 0 and 1')
        self.vlm = LmStudioVlmClient(
            base_url=str(self.declare_parameter(
                'vlm_base_url', 'http://127.0.0.1:1234'
            ).value),
            model=str(self.declare_parameter(
                'vlm_model', 'qwen/qwen3-vl-4b'
            ).value),
            timeout_s=float(self.declare_parameter('vlm_timeout_s', 10.0).value),
            bbox_format='norm1000',
            jpeg_quality=int(self.declare_parameter('vlm_jpeg_quality', 90).value),
            request_long_edge_px=0,
        )
        self._condition = threading.Condition()
        self._latest_image = None
        self._goal_lock = threading.Lock()
        self._goal_active = False
        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.server = ActionServer(
            self, VerifyGrasp, 'verify_grasp',
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group,
        )

    def goal_callback(self, request):
        object_id = request.object_id.strip()
        if not object_id or len(object_id) > 128:
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_active:
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _on_image(self, message):
        with self._condition:
            self._latest_image = message
            self._condition.notify_all()

    def execute(self, goal_handle):
        result = VerifyGrasp.Result()
        try:
            self._feedback(goal_handle, 'validating', 0.05, 'goal accepted')
            if goal_handle.request.dry_run and self.dry_run_mode == 'contract_only':
                result.error.code = CapabilityError.NONE
                result.error.message = 'dry-run validated; no image or VLM request made'
                result.grasped = True
                result.reason = result.error.message
                goal_handle.succeed()
                return result
            if not self.backend_enabled:
                raise VerificationFailure(
                    CapabilityError.SAFETY_REJECTED,
                    'grasp verification backend is disabled',
                )
            image = self._fresh_image(goal_handle)
            self._feedback(goal_handle, 'verifying', 0.40, 'checking wrist image')
            try:
                verification, latency_s = self.vlm.verify_grasp(
                    color_to_bgr(image), goal_handle.request.object_id.strip()
                )
            except Exception as exc:
                raise VerificationFailure(
                    CapabilityError.BACKEND_FAILURE, str(exc)
                ) from exc
            result.grasped = bool(verification.grasped)
            result.confidence = float(verification.confidence)
            result.reason = verification.reason
            if not result.grasped or result.confidence < self.success_confidence:
                raise VerificationFailure(
                    CapabilityError.NOT_FOUND,
                    f'grasp verification failed in {latency_s:.2f}s: '
                    f'{verification.reason} (confidence={verification.confidence:.2f})',
                )
            result.error.code = CapabilityError.NONE
            result.error.message = (
                f'grasp visually verified in {latency_s:.2f}s: {verification.reason}'
            )
            goal_handle.succeed()
            self._feedback(goal_handle, 'complete', 1.0, result.error.message)
            return result
        except VerificationFailure as exc:
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
            return result
        finally:
            with self._goal_lock:
                self._goal_active = False

    def _fresh_image(self, goal_handle):
        not_before_ns = self.get_clock().now().nanoseconds
        deadline = time.monotonic() + self.sensor_timeout_s
        with self._condition:
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    raise VerificationFailure(
                        CapabilityError.CANCELED, 'grasp verification canceled'
                    )
                image = self._latest_image
                if image is not None and stamp_ns(image) >= not_before_ns:
                    age_s = (
                        self.get_clock().now().nanoseconds - stamp_ns(image)
                    ) * 1.0e-9
                    if 0.0 <= age_s <= self.max_sensor_age_s:
                        return image
                self._condition.wait(
                    timeout=min(0.05, max(0.0, deadline - time.monotonic()))
                )
        raise VerificationFailure(
            CapabilityError.TIMEOUT, 'timed out waiting for a new wrist-camera frame'
        )

    @staticmethod
    def _feedback(goal_handle, phase, progress, message):
        feedback = VerifyGrasp.Feedback()
        feedback.state.phase = phase
        feedback.state.progress = float(progress)
        feedback.state.message = message
        goal_handle.publish_feedback(feedback)


def main():
    rclpy.init()
    node = VerifyGraspNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
