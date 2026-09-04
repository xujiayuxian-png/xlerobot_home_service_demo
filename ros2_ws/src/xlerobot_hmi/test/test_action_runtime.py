import os
import threading

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from xlerobot_hmi.fetch_deliver_cli import (
    argument_parser,
    build_goal,
    FetchDeliverClient,
)
from xlerobot_interfaces.action import ExecuteTask
from xlerobot_interfaces.msg import CapabilityError


def test_cli_talks_only_to_execute_task():
    os.environ['ROS_DOMAIN_ID'] = '83'
    rclpy.init()
    backend = Node('hmi_test_task_backend')
    received = []

    def execute(goal_handle):
        received.append(goal_handle.request)
        result = ExecuteTask.Result()
        result.error.code = CapabilityError.NONE
        result.error.message = 'done'
        result.completed_object_id = goal_handle.request.object_id
        goal_handle.succeed()
        return result

    server = ActionServer(backend, ExecuteTask, '/execute_task', execute_callback=execute)
    backend_executor = MultiThreadedExecutor(num_threads=2)
    backend_executor.add_node(backend)
    thread = threading.Thread(target=backend_executor.spin, daemon=True)
    thread.start()
    client = FetchDeliverClient()
    try:
        goal = build_goal(argument_parser().parse_args(['camping_lamp']))
        wrapped = client.submit(goal, 3.0)
        assert wrapped.result.error.code == CapabilityError.NONE
        assert len(received) == 1
        assert received[0].dry_run
    finally:
        client.destroy_node()
        backend_executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        server.destroy()
        backend.destroy_node()
        rclpy.shutdown()
