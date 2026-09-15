"""Real timestamp interpolation and stale-data checks; synthetic messages only."""

from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from xlerobot_interfaces.srv import LookupRobotTransform


def test_gateway_preserves_measurement_time_and_bounds_missing_transform(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '197')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    executable = Path(get_package_prefix('xlerobot_bringup')) / 'lib/xlerobot_bringup/robot_state_gateway'
    rclpy.init()
    node = Node('gateway_replay')
    process = subprocess.Popen([str(executable)], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    tf_pub = node.create_publisher(TFMessage, '/tf', 10)
    joints_pub = node.create_publisher(JointState, '/joint_states', 10)
    summaries = []
    node.create_subscription(JointState, '/x1/joint_state_summary', summaries.append, 10)
    client = node.create_client(LookupRobotTransform, '/x1/lookup_transform')

    def query(source, stamp, timeout=0.1):
        future = client.call_async(LookupRobotTransform.Request(
            target_frame='map', source_frame=source, stamp=stamp, timeout_s=timeout))
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        assert future.done()
        return future.result()

    try:
        assert client.wait_for_service(timeout_sec=5)
        deadline = time.monotonic() + 5
        while tf_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        end_ns = node.get_clock().now().nanoseconds
        transforms = []
        for delta, x in [(-1_000_000_000, 1.0), (0, 3.0)]:
            t = TransformStamped()
            t.header.frame_id = 'map'
            t.header.stamp = Time(nanoseconds=end_ns + delta).to_msg()
            t.child_frame_id = 'camera'
            t.transform.rotation.w = 1.0
            t.transform.translation.x = x
            transforms.append(t)
        tf_pub.publish(TFMessage(transforms=transforms))
        midpoint = Time(nanoseconds=end_ns - 500_000_000).to_msg()
        response = query('camera', midpoint, 0.5)
        assert response.success, response.error
        assert response.transform.transform.translation.x == pytest.approx(2.0)
        assert response.transform.header.stamp == midpoint
        started = time.monotonic()
        assert not query('missing', midpoint).success
        assert time.monotonic() - started < 1.5
        assert not query('camera', midpoint, float('nan')).success
        assert not query('camera', TimeMessage(sec=-1)).success
        assert not query('camera', TimeMessage(nanosec=1_000_000_000)).success
        assert query('camera', midpoint, 3.0).success
        # Old samples remain old; a summary heartbeat must not freshen them.
        joint = JointState(name=['joint'], position=[1.0])
        joint.header.stamp = midpoint
        joints_pub.publish(joint)
        deadline = time.monotonic() + 2
        while not summaries and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert summaries and summaries[-1].header.stamp == midpoint
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(summaries) == 1
    finally:
        process.send_signal(signal.SIGINT)
        process.communicate(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
    assert process.returncode == 0
