"""Publish the verified right wrist UVC camera through OpenCV/V4L2.

The reference USB2.0_CAM1 visibly tears at 640x480 when captured as YUYV on
this host.  The original verified demo avoided that device/driver behaviour by
requesting the camera's MJPG mode before capture.
"""

from __future__ import annotations

import sys
import time
import re

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class WristCameraNode(Node):
    """Read one UVC camera without exposing any actuator interface."""

    def __init__(self) -> None:
        super().__init__('right_wrist_camera')
        self.declare_parameter('video_device', '')
        self.declare_parameter(
            'camera_frame_id', 'right_arm_wrist_camera_rgb_optical_frame'
        )
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('pixel_format', 'MJPG')
        self.declare_parameter('topic', '/right_wrist_camera/image_raw')

        self.device = str(self.get_parameter('video_device').value)
        self.frame_id = str(self.get_parameter('camera_frame_id').value)
        self.width = int(self.get_parameter('image_width').value)
        self.height = int(self.get_parameter('image_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.pixel_format = str(self.get_parameter('pixel_format').value).upper()
        topic = str(self.get_parameter('topic').value)
        if not self.device.startswith('/dev/') or re.fullmatch(
            r'/dev/video[0-9]+', self.device
        ):
            raise ValueError(
                'video_device must be a stable /dev link, not /dev/videoN'
            )
        if self.width <= 0 or self.height <= 0 or self.fps <= 0.0:
            raise ValueError('wrist camera dimensions and fps must be positive')

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, topic, 10)
        self.info_publisher = self.create_publisher(CameraInfo, '~/camera_info', 10)
        self.capture = None
        if not self._open_capture():
            self.get_logger().fatal(f'failed to open video device {self.device}')
            raise RuntimeError(f'failed to open video device {self.device}')

        actual_width = round(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = round(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)
        actual_fourcc = self._fourcc_text(
            round(self.capture.get(cv2.CAP_PROP_FOURCC))
        )
        self.get_logger().info(
            f'opened {self.device} at {actual_width}x{actual_height} '
            f'@ {actual_fps:.2f} fps fourcc={actual_fourcc}'
        )
        requested_fourcc = self.pixel_format[:4]
        if actual_fourcc != requested_fourcc:
            self.capture.release()
            raise RuntimeError(
                f'wrist camera requested {requested_fourcc} but opened '
                f'{actual_fourcc}; refusing the tearing-prone fallback'
            )
        self.info = CameraInfo()
        self.info.width = actual_width
        self.info.height = actual_height
        self.info.header.frame_id = self.frame_id
        self.last_warning = 0.0
        self.capture_failures = 0
        self.timer = self.create_timer(1.0 / self.fps, self._publish_frame)

    def _open_capture(self) -> bool:
        self.capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            return False
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        if len(self.pixel_format) >= 4:
            self.capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*self.pixel_format[:4]),
            )
        return True

    @staticmethod
    def _fourcc_text(value: int) -> str:
        return ''.join(chr((value >> (8 * index)) & 0xff) for index in range(4))

    def _publish_frame(self) -> None:
        try:
            success, bgr = self.capture.read()
        except cv2.error as exc:
            self._capture_failed(f'OpenCV capture error: {exc}')
            return
        if not success or bgr is None:
            self._capture_failed('failed to capture wrist frame')
            return
        self.capture_failures = 0
        stamp = self.get_clock().now().to_msg()
        image = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        self.image_publisher.publish(image)
        self.info.header.stamp = stamp
        self.info_publisher.publish(self.info)

    def _capture_failed(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warning >= 2.0:
            self.get_logger().warning(message)
            self.last_warning = now
        self.capture_failures += 1
        if self.capture_failures < max(5, round(self.fps)):
            return
        self.capture_failures = 0
        self.capture.release()
        time.sleep(0.1)
        self._open_capture()

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
        return super().destroy_node()


def main(args=None):
    """Run the read-only wrist camera publisher."""
    rclpy.init(args=args)
    try:
        node = WristCameraNode()
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
