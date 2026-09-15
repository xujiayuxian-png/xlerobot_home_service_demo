"""Bounded, read-only startup probe; no images, devices or motion clients."""

import json
import math
import time

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class Readiness:
    def __init__(self, spec):
        self.required = spec.get('diagnostics', [])
        self.services = set(spec.get('services', []))
        self.status = {}

    def receive(self, message, monotonic, ros_now):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        transport_age = ros_now - stamp
        for status in message.status:
            if status.name not in self.required:
                continue
            age = transport_age
            if status.name.startswith('xlerobot/camera/'):
                try:
                    frame_age = float(dict((v.key, v.value) for v in status.values)['age_s'])
                    age += frame_age
                    if not math.isfinite(frame_age) or frame_age < 0:
                        age = math.inf
                except (KeyError, ValueError):
                    age = math.inf
            if stamp <= 0 or transport_age < -0.1:
                age = math.inf
            level = status.level[0] if isinstance(status.level, (bytes, bytearray)) else int(status.level)
            self.status[status.name] = (level, status.message, monotonic, max(0, age))

    def pending(self, monotonic, service_names):
        reasons = []
        for name in self.required:
            status = self.status.get(name)
            limit = 1.0 if name.startswith('xlerobot/camera/') else 3.0
            if status is None:
                reasons.append(name + ': not received')
            elif status[0] != 0 or status[3] + monotonic - status[2] > limit:
                reasons.append(name + ': ' + (status[1] if status[0] else 'stale'))
        reasons.extend(name + ': unavailable' for name in sorted(self.services - set(service_names)))
        return reasons


class StartupProbe(Node):
    def __init__(self):
        super().__init__('x1_startup_probe')
        spec = json.loads(self.declare_parameter('stage_spec', '{}').value)
        if not spec.get('name') or not any(spec.get(key) for key in
                                          ('diagnostics', 'services', 'model_service')):
            raise ValueError('startup probe requires a named, nonempty readiness contract')
        self.phase = spec['name']
        self.timeout = float(self.declare_parameter('timeout_s', 120.0).value)
        if not math.isfinite(self.timeout) or not 0 < self.timeout <= 300:
            raise ValueError('timeout_s must be in (0, 300]')
        self.readiness = Readiness(spec)
        self.subscriptions_owned = [self.create_subscription(
            DiagnosticArray, topic, self.receive, 10)
            for topic in ('/diagnostics', '/camera/health') if self.readiness.required]
        self.model = (self.create_client(Trigger, spec['model_service'])
                      if spec.get('model_service') else None)
        self.future = None
        self.model_ready = self.model is None
        self.next_model_check = 0.0

    def receive(self, message):
        self.readiness.receive(message, time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)

    def run(self):
        started = time.monotonic()
        stable_since = None
        next_log = started
        next_check = started
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            now = time.monotonic()
            if now < next_check:
                continue
            next_check = now + 0.2
            reasons = self.readiness.pending(now, [name for name, _ in self.get_service_names_and_types()])
            if not self.model_ready:
                if self.future is not None and self.future.done():
                    response = self.future.result()
                    self.future = None
                    if response.success:
                        self.model_ready = True
                    elif response.message != 'loading':
                        self.get_logger().error(f'{self.phase}: {response.message}')
                        return 1
                if not self.model_ready:
                    reasons.append('person model warmup pending')
                    if self.future is None and now >= self.next_model_check and self.model.service_is_ready():
                        self.future = self.model.call_async(Trigger.Request())
                        self.next_model_check = now + 1.0
            if not reasons:
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= 0.5:
                    self.get_logger().info(f'{self.phase}: ready after {now - started:.1f}s')
                    return 0
            else:
                stable_since = None
            if now - started >= self.timeout:
                self.get_logger().error(f'{self.phase}: startup timeout: ' + '; '.join(reasons))
                return 1
            if now >= next_log:
                self.get_logger().info(f'{self.phase}: waiting: ' + '; '.join(reasons))
                next_log = now + 5.0
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StartupProbe()
        return node.run()
    except KeyboardInterrupt:
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
