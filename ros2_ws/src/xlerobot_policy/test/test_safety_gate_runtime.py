import os
import threading
import time

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from xlerobot_interfaces.action import ExecuteLearnedPolicy
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_policy.act_policy_node import ActPolicyNode


def wait_future(future, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done()
    return future.result()


def test_live_goal_is_rejected_before_sensors_network_or_executor_by_default():
    os.environ['ROS_DOMAIN_ID'] = '85'
    rclpy.init()
    policy = ActPolicyNode()
    policy.http.predict = lambda **_observation: (_ for _ in ()).throw(
        AssertionError('default safety rejection must not call HTTP')
    )
    client_node = Node('act_policy_safety_client')
    client = ActionClient(client_node, ExecuteLearnedPolicy, 'execute_learned_policy')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(policy)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=3.0)
        goal = ExecuteLearnedPolicy.Goal()
        goal.policy_id = 'act_xlerobot_box_wrist_only'
        goal.object_id = 'camping_lamp'
        goal.max_duration.sec = 10
        goal.dry_run = False
        handle = wait_future(client.send_goal_async(goal))
        wrapped = wait_future(handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.error.code == CapabilityError.SAFETY_REJECTED
        assert 'backend is disabled' in wrapped.result.error.message
    finally:
        executor.shutdown(timeout_sec=3.0)
        thread.join(timeout=3.0)
        client.destroy()
        client_node.destroy_node()
        policy.destroy_node()
        rclpy.shutdown()
