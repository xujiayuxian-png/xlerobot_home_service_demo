"""Action-scoped subscriptions with a generation and acquisition-time barrier."""

import threading

from rclpy.exceptions import InvalidHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy


class DemandImageExecutor(MultiThreadedExecutor):
    """Handle queued takes of explicitly retired subscriptions on ROS Humble.

    Humble can schedule a subscription take before an action destroys that
    subscription. Returning no message here lets its handler finish callback
    group bookkeeping; catching the exception around spin would not do that.
    Other InvalidHandle errors must still propagate.
    """

    def _take_subscription(self, subscription):
        if getattr(subscription, '_xlerobot_retired', False):
            return None
        try:
            return super()._take_subscription(subscription)
        except InvalidHandle:
            if not getattr(subscription, '_xlerobot_retired', False):
                raise
            return None


class DemandImages:
    """Own a bounded set of subscriptions; discard callbacks from retired goals."""

    def __init__(self, node, specs, clear, *, enabled):
        self.node = node
        self.specs = specs
        self.clear = clear
        self.enabled = enabled
        self.lock = threading.RLock()
        self.subscriptions = []
        self.generation = 0
        self.active = False
        if not enabled:
            self._start(barrier=False)

    def start(self):
        """Acquire fresh streams for one active goal."""
        if self.enabled:
            self._start(barrier=True)

    def _start(self, *, barrier):
        with self.lock:
            if self.active:
                return
            self.clear()
            self.generation += 1
            generation = self.generation
            since = self.node.get_clock().now().nanoseconds if barrier else 0
            self.active = True
            try:
                for message_type, topic, callback in self.specs:
                    def receive(message, callback=callback):
                        stamp = message.header.stamp
                        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                        with self.lock:
                            if self.active and generation == self.generation and stamp_ns >= since:
                                callback(message)
                    self.subscriptions.append(self.node.create_subscription(
                        message_type, topic, receive,
                        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                        callback_group=self.node.group))
            except Exception:
                self._stop()
                raise

    def stop(self):
        """Release streams even when the owning action failed or was canceled."""
        if self.enabled:
            self._stop()

    def _stop(self):
        with self.lock:
            self.active = False
            self.generation += 1
            subscriptions, self.subscriptions = self.subscriptions, []
            self.clear()
        for subscription in subscriptions:
            subscription._xlerobot_retired = True
            self.node.destroy_subscription(subscription)
