"""Software-only driver replay: freshness must follow frames, not heartbeats."""

from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def test_health_tracks_stall_and_recovery_without_any_motor_or_camera_device(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '187')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    executable = Path(get_package_prefix('xlerobot_bringup')) / 'lib/xlerobot_bringup/camera_health'
    rclpy.init()
    node = Node('camera_health_replay_test')
    observed = []
    publisher = node.create_publisher(Image, '/right_wrist_camera/image_raw', 1)
    node.create_subscription(DiagnosticArray, '/camera/health', observed.append, 1)
    process = subprocess.Popen([str(executable)], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    def wait_for(predicate, publish=False, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish:
                image = Image(height=2, width=2, step=6, encoding='bgr8', data=bytes(12))
                image.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(image)
            rclpy.spin_once(node, timeout_sec=0.05)
            if observed:
                status = next(item for item in observed[-1].status if item.name.endswith('/wrist'))
                values = {item.key: item.value for item in status.values}
                if predicate(float(values['age_s'])):
                    return
        raise AssertionError('camera health did not reach expected freshness')

    try:
        wait_for(lambda age: age > 1)
        wait_for(lambda age: age < 0.3, publish=True)
        wait_for(lambda age: age > 1)
        wait_for(lambda age: age < 0.3, publish=True)
    finally:
        process.send_signal(signal.SIGINT)
        process.communicate(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
    assert process.returncode == 0
