"""Hardware-free ROS graph proof for the Leader/Follower enable lease."""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory


SOURCE_JOINTS = [
    'leader_shoulder_pan',
    'leader_shoulder_lift',
    'leader_elbow_flex',
    'leader_wrist_flex',
    'leader_wrist_roll',
    'leader_gripper',
]


def spin_until(node, predicate, timeout_s: float, publish=None) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if publish is not None:
            publish()
        rclpy.spin_once(node, timeout_sec=0.01)
        if predicate():
            return True
    return predicate()


def call_enabled(node, client, enabled: bool, publish) -> None:
    future = client.call_async(SetBool.Request(data=enabled))
    assert spin_until(node, future.done, 2.0, publish)
    response = future.result()
    assert response is not None and response.success, response


def test_enable_heartbeat_expires_and_false_is_immediate() -> None:
    domain_id = str(20 + (os.getpid() + time.monotonic_ns()) % 200)
    environment = dict(os.environ)
    environment.update({
        'ROS_DOMAIN_ID': domain_id,
        'ROS_LOCALHOST_ONLY': '1',
    })
    previous_domain = os.environ.get('ROS_DOMAIN_ID')
    previous_localhost = os.environ.get('ROS_LOCALHOST_ONLY')
    os.environ.update({
        'ROS_DOMAIN_ID': domain_id,
        'ROS_LOCALHOST_ONLY': '1',
    })
    process = None
    node = None
    try:
        prefix = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'xlerobot_leader_teleop'],
            env=environment,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        executable = (
            Path(prefix)
            / 'lib/xlerobot_leader_teleop/leader_follower_teleop'
        )
        process = subprocess.Popen(
            [
                str(executable),
                '--ros-args',
                '-p', 'enable_lease_s:=0.25',
                '-p', 'leader_timeout_s:=1.0',
            ],
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        rclpy.init()
        node = rclpy.create_node('leader_follower_watchdog_runtime_test')
        commands = []
        publisher = node.create_publisher(
            JointState, '/leader/joint_states', 10
        )
        subscription = node.create_subscription(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            lambda message: commands.append(message),
            20,
        )
        client = node.create_client(
            SetBool, '/leader_follower_teleop/set_enabled'
        )
        state = JointState()
        state.name = SOURCE_JOINTS
        state.position = [0.0, 0.1, -0.1, 0.05, 0.0, 0.2]

        def publish_state():
            state.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(state)

        assert subscription is not None
        assert client.wait_for_service(timeout_sec=5.0)
        assert spin_until(
            node,
            lambda: publisher.get_subscription_count() > 0,
            2.0,
            publish_state,
        )
        spin_until(node, lambda: False, 0.15, publish_state)

        call_enabled(node, client, True, publish_state)
        assert spin_until(node, lambda: len(commands) >= 3, 1.0, publish_state)

        # Leader input remains fresh, so command publication can stop here
        # only because the coordinator-style heartbeat lease expired.
        spin_until(node, lambda: False, 0.4, publish_state)
        after_expiry = len(commands)
        spin_until(node, lambda: False, 0.2, publish_state)
        assert len(commands) == after_expiry

        # Repeated true calls refresh the same lease without creating a new
        # owner or interface.
        before_heartbeats = len(commands)
        heartbeat_deadline = time.monotonic() + 0.65
        while time.monotonic() < heartbeat_deadline:
            call_enabled(node, client, True, publish_state)
            spin_until(node, lambda: False, 0.08, publish_state)
        assert len(commands) > before_heartbeats + 5

        call_enabled(node, client, False, publish_state)
        spin_until(node, lambda: False, 0.08, publish_state)
        after_false = len(commands)
        spin_until(node, lambda: False, 0.2, publish_state)
        assert len(commands) == after_false
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
                    'leader_follower_teleop exited with '
                    f'{process.returncode}: {error}'
                )
        if previous_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain
        if previous_localhost is None:
            os.environ.pop('ROS_LOCALHOST_ONLY', None)
        else:
            os.environ['ROS_LOCALHOST_ONLY'] = previous_localhost
