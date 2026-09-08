"""The detector must not buffer slow-to-process camera frames."""

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from xlerobot_calibration_tools.target_node import CalibrationTargetNode, latest_image_qos


def test_latest_image_qos_is_a_single_volatile_best_effort_frame():
    qos = latest_image_qos()
    assert qos.depth == 1
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_detector_uses_latest_only_qos_for_input_and_debug_images():
    rclpy.init(args=['--ros-args', '-p', 'workflow_id:=head_camera'], domain_id=77)
    node = None
    try:
        node = CalibrationTargetNode()
        image_input = next(sub for sub in node.subscriptions
                           if sub.topic_name == '/xlerobot/d455/color/image_raw')
        debug_output = next(pub for pub in node.publishers
                            if pub.topic_name == '/calibration/target_debug')
        assert image_input.qos_profile.depth == 1
        assert debug_output.qos_profile.depth == 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
