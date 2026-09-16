"""Real action cancellation/timeout tests with no sensor or actuator devices."""

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from xlerobot_interfaces.action import VerifyGrasp
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_perception.verify_grasp_node import VerifyGraspNode
from xlerobot_perception.demand_images import DemandImageExecutor


def wait_for(predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError('action condition timed out')


def test_cancel_timeout_and_contract_goal_all_leave_no_image_subscriptions(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '188')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    server = VerifyGraspNode(parameter_overrides=[
        Parameter('x1_low_load', value=True), Parameter('backend_enabled', value=True),
        Parameter('sensor_timeout_s', value=0.5)])
    client_node = Node('demand_image_test_client')
    client = ActionClient(client_node, VerifyGrasp, 'verify_grasp')
    executor = DemandImageExecutor(num_threads=3)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=3)
        for dry_run, cancel, code in ((True, False, CapabilityError.NONE),
                                     (False, True, CapabilityError.CANCELED),
                                     (False, False, CapabilityError.TIMEOUT)):
            assert not server.images.subscriptions
            goal = VerifyGrasp.Goal(object_id='杯子', dry_run=dry_run)
            future = client.send_goal_async(goal)
            wait_for(future.done)
            handle = future.result()
            assert handle.accepted
            if cancel:
                wait_for(lambda: bool(server.images.subscriptions))
                handle.cancel_goal_async()
            result = handle.get_result_async()
            wait_for(result.done)
            assert result.result().result.error.code == code
            assert not server.images.subscriptions and server._latest_image is None
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        client.destroy()
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()
