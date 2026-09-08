"""Publish the verified calibration target pose and visual quality evidence."""

from __future__ import annotations

from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from xlerobot_interfaces.msg import CalibrationTargetObservation

from .fiducial import FixedTargetDetector


def latest_image_qos():
    # 1280x720 AprilTag detection can be slower than the 15 Hz camera.
    # Keep only the newest waiting image, not a queue of already-stale frames.
    # Preserve the camera timestamp; never disguise latency by restamping it.
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class CalibrationTargetNode(Node):
    """Detect the fixed board or gripper tag without commanding hardware."""

    def __init__(self):
        super().__init__('calibration_target_detector')
        workflow = str(self.declare_parameter('workflow_id', '').value)
        self.detector = FixedTargetDetector(workflow)
        self.optical_frame = str(self.declare_parameter(
            'optical_frame', 'd455_color_optical_frame'
        ).value)
        self.target_frame = str(self.declare_parameter(
            'target_frame', 'calibration_target'
        ).value)
        image_topic = str(self.declare_parameter(
            'image_topic', '/xlerobot/d455/color/image_raw'
        ).value)
        info_topic = str(self.declare_parameter(
            'camera_info_topic', '/xlerobot/d455/color/camera_info'
        ).value)
        self.bridge = CvBridge()
        self.camera_info = None
        self.broadcaster = TransformBroadcaster(self)
        self.observations = self.create_publisher(
            CalibrationTargetObservation,
            '/calibration/target_observation',
            qos_profile_sensor_data,
        )
        self.debug_images = self.create_publisher(
            Image, '/calibration/target_debug', latest_image_qos(),
        )
        self.create_subscription(
            CameraInfo, info_topic, self.on_camera_info, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, image_topic, self.on_image, latest_image_qos()
        )

    def on_camera_info(self, message):
        self.camera_info = message

    def on_image(self, message):
        observation = CalibrationTargetObservation()
        observation.header = message.header
        observation.header.frame_id = self.optical_frame
        if self.camera_info is None:
            observation.detail = 'waiting for CameraInfo'
            self.observations.publish(observation)
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            matrix = np.asarray(self.camera_info.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(self.camera_info.d, dtype=np.float64)
            estimate, debug = self.detector.detect_with_debug(image, matrix, distortion)
            diagnostic = self.detector.diagnostic
            observation.tag_count = int(diagnostic['tag_count'])
            observation.reprojection_rmse_px = float(diagnostic['reprojection_rmse_px'])
            observation.detail = diagnostic['detail']
            debug_message = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            debug_message.header = observation.header
            self.debug_images.publish(debug_message)
        except Exception as error:
            observation.detail = f'detection error: {error}'
            self.observations.publish(observation)
            return
        if estimate is None:
            self.observations.publish(observation)
            return
        transform = TransformStamped()
        transform.header = observation.header
        transform.child_frame_id = self.target_frame
        transform.transform.translation.x = float(estimate.target_in_camera[0, 3])
        transform.transform.translation.y = float(estimate.target_in_camera[1, 3])
        transform.transform.translation.z = float(estimate.target_in_camera[2, 3])
        quaternion = Rotation.from_matrix(
            estimate.target_in_camera[:3, :3]
        ).as_quat()
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self.broadcaster.sendTransform(transform)
        observation.accepted = True
        observation.tag_count = estimate.tag_count
        observation.reprojection_rmse_px = estimate.reprojection_rmse_px
        observation.detail = f'accepted tags={estimate.tag_count}'
        self.observations.publish(observation)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
