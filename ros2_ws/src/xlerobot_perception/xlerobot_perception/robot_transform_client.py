"""Bounded, timestamped C++ TF lookups without a Python TF subscription."""

import threading
import time

from tf2_ros import TransformException
from xlerobot_interfaces.srv import LookupRobotTransform


class RobotTransformClient:
    def __init__(self, node, callback_group):
        self.client = node.create_client(
            LookupRobotTransform, '/x1/lookup_transform', callback_group=callback_group)

    def lookup_transform(self, target_frame, source_frame, stamp, *, timeout):
        # Include discovery/transport in one finite budget, with a small RPC margin.
        wait_s = timeout.nanoseconds / 1e9
        if not 0.0 <= wait_s <= 5.0:
            raise TransformException('X1 transform timeout must be between 0 and 5 seconds')
        deadline = time.monotonic() + wait_s + 0.25
        if not self.client.wait_for_service(timeout_sec=max(0.0, deadline - time.monotonic())):
            raise TransformException('X1 robot state gateway unavailable')
        request = LookupRobotTransform.Request(
            target_frame=target_frame, source_frame=source_frame,
            stamp=stamp.to_msg(), timeout_s=wait_s)
        done = threading.Event()
        future = self.client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(max(0.0, deadline - time.monotonic())):
            self.client.remove_pending_request(future)
            future.cancel()
            raise TransformException('X1 timestamped transform request timed out')
        response = future.result()
        if response is None or not response.success:
            raise TransformException(response.error if response else 'empty transform response')
        return response.transform
