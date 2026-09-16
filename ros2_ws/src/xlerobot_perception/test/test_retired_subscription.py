"""Reproduce Humble's queued-take/destruction race with real ROS handles only."""
import pytest
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.exceptions import InvalidHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from xlerobot_perception.demand_images import DemandImages, DemandImageExecutor


@pytest.fixture
def stream(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    context.init(domain_id=189)
    node = Node('retired_subscription_test', context=context)
    node.group = ReentrantCallbackGroup()
    images = DemandImages(node, [(JointState, '/test_joints', lambda _: None)],
                          lambda: None, enabled=True)
    executor = DemandImageExecutor(num_threads=2, context=context)
    images.start()
    try:
        yield node, images, executor
    finally:
        images.stop()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_queued_take_after_retirement_is_empty_and_next_generation_works(stream):
    node, images, executor = stream
    retired = images.subscriptions[0]
    images.stop()
    # This is the same real-handle failure seen in the task node traceback.
    with pytest.raises(InvalidHandle):
        MultiThreadedExecutor._take_subscription(executor, retired)
    assert executor._take_subscription(retired) is None
    assert not images.subscriptions
    images.start()
    current = images.subscriptions[0]
    assert current is not retired
    assert not getattr(current, '_xlerobot_retired', False)
    assert executor._take_subscription(current) is None


def test_retirement_between_executor_check_and_take_is_also_safe(stream, monkeypatch):
    node, images, executor = stream
    subscription = images.subscriptions[0]
    original = MultiThreadedExecutor._take_subscription

    def retire_then_take(self, sub):
        images.stop()
        return original(self, sub)

    monkeypatch.setattr(MultiThreadedExecutor, '_take_subscription', retire_then_take)
    assert executor._take_subscription(subscription) is None


def test_unexpected_invalid_handle_is_not_suppressed(stream):
    node, images, executor = stream
    subscription = images.subscriptions.pop()
    node.destroy_subscription(subscription)
    with pytest.raises(InvalidHandle):
        executor._take_subscription(subscription)
