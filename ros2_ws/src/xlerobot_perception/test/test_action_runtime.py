import os
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from xlerobot_interfaces.action import DetectObject
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_perception.detect_object_node import DetectObjectNode


def wait_future(future, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done()
    return future.result()


def send_goal(client, *, dry_run):
    goal = DetectObject.Goal()
    goal.object_id = 'camping_lamp'
    goal.target_frame = 'map'
    goal.grasp_backend = 'act'
    goal.dry_run = dry_run
    handle = wait_future(client.send_goal_async(goal))
    assert handle.accepted
    return wait_future(handle.get_result_async()).result


def test_dry_run_never_needs_sensors_or_backend():
    os.environ['ROS_DOMAIN_ID'] = '81'
    rclpy.init()
    server = DetectObjectNode()
    client_node = Node('detect_object_contract_client')
    client = ActionClient(client_node, DetectObject, 'detect_object')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=2.0)
        dry_result = send_goal(client, dry_run=True)
        assert dry_result.error.code == CapabilityError.NONE
        assert dry_result.target.header.frame_id == 'map'

        gated_result = send_goal(client, dry_run=False)
        assert gated_result.error.code == CapabilityError.SAFETY_REJECTED
        assert 'backend_enabled=true' in gated_result.error.message
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        client.destroy()
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()
