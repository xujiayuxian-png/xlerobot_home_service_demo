"""Hardware-free ROS graph proof for the final base software stop."""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def spin_until(node, predicate, timeout_s: float) -> bool:
    """Spin the test node until a graph condition holds or time expires."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return predicate()


def call_stop(node, client, enabled: bool) -> None:
    """Set the final base software inhibit and require acknowledgement."""
    future = client.call_async(SetBool.Request(data=enabled))
    assert spin_until(node, future.done, 3.0)
    response = future.result()
    assert response is not None and response.success, response


def test_clear_discards_commands_queued_while_latched() -> None:
    """Prove latch clearing cannot replay commands sent under the latch."""
    domain_id = str(20 + (os.getpid() + time.monotonic_ns()) % 200)
    environment = dict(os.environ)
    environment.update({
        'ROS_DOMAIN_ID': domain_id,
        'ROS_LOCALHOST_ONLY': '1',
    })
    original_domain_id = os.environ.get('ROS_DOMAIN_ID')
    original_localhost_only = os.environ.get('ROS_LOCALHOST_ONLY')
    os.environ.update({
        'ROS_DOMAIN_ID': domain_id,
        'ROS_LOCALHOST_ONLY': '1',
    })
    process = None
    node = None
    try:
        prefix = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'xlerobot_base'],
            env=environment,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        executable = Path(prefix) / 'lib/xlerobot_base/drive_safety_node'
        process = subprocess.Popen(
            [str(executable)],
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        rclpy.init()
        node = rclpy.create_node('drive_stop_runtime_test')
        outputs = []
        stop_states = []
        publisher = node.create_publisher(Twist, 'cmd_vel_nav', 10)
        output_subscription = node.create_subscription(
            TwistStamped,
            '/base_controller/cmd_vel',
            lambda message: outputs.append(float(message.twist.linear.x)),
            20,
        )
        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_subscription = node.create_subscription(
            Bool,
            '/drive_safety/stop_latched',
            lambda message: stop_states.append(bool(message.data)),
            stop_qos,
        )
        stop_client = node.create_client(SetBool, '/drive_safety/set_stop')
        assert output_subscription is not None
        assert state_subscription is not None
        assert stop_client.wait_for_service(timeout_sec=5.0)
        assert spin_until(
            node, lambda: publisher.get_subscription_count() > 0, 3.0
        )

        command = Twist()
        command.linear.x = 0.08
        for _ in range(10):
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
        assert spin_until(
            node, lambda: any(value > 0.0 for value in outputs), 2.0
        )

        call_stop(node, stop_client, True)
        assert spin_until(node, lambda: stop_states and stop_states[-1], 2.0)
        stale_command = Twist()
        stale_command.linear.x = -0.08
        for _ in range(2000):
            publisher.publish(stale_command)

        call_stop(node, stop_client, False)
        outputs.clear()
        assert spin_until(node, lambda: len(outputs) >= 10, 1.0)
        assert not any(value < -1.0e-12 for value in outputs)

        assert spin_until(
            node, lambda: publisher.get_subscription_count() > 0, 2.0
        )
        outputs.clear()
        for _ in range(10):
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
        assert spin_until(
            node, lambda: any(value > 0.0 for value in outputs), 2.0
        )
        call_stop(node, stop_client, True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
            if process.returncode not in (0, -signal.SIGINT):
                error = process.stderr.read() if process.stderr else ''
                raise AssertionError(
                    'drive_safety_node exited with '
                    f'{process.returncode}: {error}'
                )
        if original_domain_id is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = original_domain_id
        if original_localhost_only is None:
            os.environ.pop('ROS_LOCALHOST_ONLY', None)
        else:
            os.environ['ROS_LOCALHOST_ONLY'] = original_localhost_only
