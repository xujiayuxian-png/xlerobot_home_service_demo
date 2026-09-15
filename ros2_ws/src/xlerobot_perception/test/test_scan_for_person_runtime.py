from array import array
import math
import os
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from xlerobot_interfaces.action import ScanForPerson
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_perception.detection.yolo import YoloDetection
from xlerobot_perception.scan_for_person_node import ScanForPersonNode


class EmptyDetector:
    created = 0
    detections = 0

    def __init__(self, *_args, **_kwargs):
        type(self).created += 1

    @classmethod
    def detect(cls, _image):
        cls.detections += 1
        return [], 0.0


def test_model_ready_distinguishes_loading_failure_and_completed_warmup():
    state = SimpleNamespace(backend_enabled=True, detector_ready=threading.Event(),
                            detector=object(), detector_error=None)
    query = lambda: ScanForPersonNode._model_ready(state, None, Trigger.Response())
    assert not query().success
    assert query().message == 'loading'  # Constructed detector is not warmed up yet.
    state.detector_ready.set()
    assert query().success
    state.detector_error = RuntimeError('warmup failed')
    assert not query().success
    assert 'warmup failed' in query().message
    state.detector_error = None
    state.detector = None
    assert not query().success
    state.backend_enabled = False
    assert query().message == 'disabled'


def test_readiness_service_responds_while_warmup_holds_detector_lock(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = '81'
    release = threading.Event()
    warming = threading.Event()
    class SlowDetector(EmptyDetector):
        @classmethod
        def detect(cls, _image):
            warming.set()
            assert release.wait(timeout=5)
            return [], 0.0
    rclpy.init()
    model_path = tmp_path/'fake-model.pt'
    model_path.write_bytes(b'fake detector does not load weights')
    server = ScanForPersonNode(parameter_overrides=[
        Parameter('model_path', value=str(model_path)),
        Parameter('x1_low_load', value=True), Parameter('backend_enabled', value=True),
        Parameter('dry_run_mode', value='observe')], detector_factory=SlowDetector)
    client = server.create_client(Trigger, '/scan_for_person/model_ready')
    try:
        assert warming.wait(timeout=1)
        assert client.wait_for_service(timeout_sec=1)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(server, future, timeout_sec=1)
        assert future.done(), 'readiness query blocked behind model warmup'
        assert not future.result().success
        assert future.result().message == 'loading'
        release.set()
        assert server.detector_ready.wait(timeout=1)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(server, future, timeout_sec=1)
        assert future.done() and future.result().success
    finally:
        release.set()
        server.detector_thread.join(timeout=1)
        server.destroy_node()
        rclpy.shutdown()


def wait_future(future, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done()
    return future.result()


def scan_goal(*, dry_run=True, recipient='nearest_person'):
    goal = ScanForPerson.Goal()
    goal.recipient_id = recipient
    goal.dry_run = dry_run
    return goal


def test_contract_only_gate_and_recipient_rejection():
    os.environ['ROS_DOMAIN_ID'] = '81'
    rclpy.init()
    server = ScanForPersonNode()
    client_node = Node('scan_for_person_contract_client')
    client = ActionClient(client_node, ScanForPerson, 'scan_for_person')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=2.0)
        dry_handle = wait_future(client.send_goal_async(scan_goal()))
        assert dry_handle.accepted
        dry = wait_future(dry_handle.get_result_async())
        assert dry.status == GoalStatus.STATUS_SUCCEEDED
        assert dry.result.error.code == CapabilityError.NONE
        assert math.isnan(dry.result.distance_m)

        live_handle = wait_future(client.send_goal_async(scan_goal(dry_run=False)))
        assert live_handle.accepted
        live = wait_future(live_handle.get_result_async())
        assert live.status == GoalStatus.STATUS_ABORTED
        assert live.result.error.code == CapabilityError.SAFETY_REJECTED

        unsupported = wait_future(
            client.send_goal_async(scan_goal(recipient='specific_person'))
        )
        assert not unsupported.accepted
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        client.destroy()
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def test_observe_mode_selects_nearest_valid_depth_candidate():
    os.environ['ROS_DOMAIN_ID'] = '81'
    rclpy.init()
    server = ScanForPersonNode(
        parameter_overrides=[
            Parameter('backend_enabled', value=True),
            Parameter('dry_run_mode', value='observe'),
            Parameter('sensor_timeout_s', value=2.0),
            Parameter('model_path', value=__file__),
        ],
        detector_factory=EmptyDetector,
    )
    assert server.detector_ready.wait(timeout=1.0)
    assert EmptyDetector.created >= 1
    assert EmptyDetector.detections >= 1
    calls = []

    lookup = server._lookup

    def lookup_before_detection(*args):
        calls.append('tf')
        return lookup(*args)

    server._lookup = lookup_before_detection

    def detect(_image):
        calls.append('person')
        return [
            YoloDetection('person', (15, 15, 25, 25), 0.95),
            YoloDetection('person', (55, 15, 65, 25), 0.90),
        ], 0.02

    server.detector = type('FakeDetector', (), {'detect': staticmethod(detect)})()
    inputs = Node('scan_for_person_observe_inputs')
    color_pub = inputs.create_publisher(
        Image, '/xlerobot/d455/color/image_raw', 10
    )
    depth_pub = inputs.create_publisher(
        Image, '/xlerobot/d455/aligned_depth_to_color/image_raw', 10
    )
    info_pub = inputs.create_publisher(
        CameraInfo, '/xlerobot/d455/color/camera_info', 10
    )
    tf = StaticTransformBroadcaster(inputs)
    transform = TransformStamped()
    transform.header.stamp = inputs.get_clock().now().to_msg()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.transform.rotation.w = 1.0
    tf.sendTransform(transform)

    def publish_rgbd():
        stamp = inputs.get_clock().now().to_msg()
        color = Image()
        color.header.stamp = stamp
        color.header.frame_id = 'map'
        color.height = 40
        color.width = 80
        color.encoding = 'bgr8'
        color.step = 240
        color.data = bytes(40 * 240)
        color_pub.publish(color)

        values = [0] * (40 * 80)
        for y in range(12, 29):
            for x in range(12, 29):
                values[y * 80 + x] = 1000
            for x in range(52, 69):
                values[y * 80 + x] = 500
        depth = Image()
        depth.header = color.header
        depth.height = 40
        depth.width = 80
        depth.encoding = '16UC1'
        depth.step = 160
        depth.data = array('H', values).tobytes()
        depth_pub.publish(depth)

        info = CameraInfo()
        info.header = color.header
        info.height = 40
        info.width = 80
        info.k = [100.0, 0.0, 40.0, 0.0, 100.0, 20.0, 0.0, 0.0, 1.0]
        info_pub.publish(info)

    timer = inputs.create_timer(0.05, publish_rgbd)
    client = ActionClient(inputs, ScanForPerson, 'scan_for_person')
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(server)
    executor.add_node(inputs)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=2.0)
        handle = wait_future(client.send_goal_async(scan_goal()))
        assert handle.accepted
        wrapped = wait_future(handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.error.code == CapabilityError.NONE
        assert calls == ['tf', 'person']
        assert wrapped.result.person.header.frame_id == 'map'
        assert math.isclose(wrapped.result.person.point.x, 0.1, abs_tol=1.0e-6)
        assert math.isclose(wrapped.result.person.point.z, 0.5, abs_tol=1.0e-6)
        assert math.isclose(wrapped.result.distance_m, 0.1, abs_tol=1.0e-6)
    finally:
        inputs.destroy_timer(timer)
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        client.destroy()
        inputs.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def test_cancel_interrupts_rgbd_wait():
    os.environ['ROS_DOMAIN_ID'] = '81'
    rclpy.init()
    server = ScanForPersonNode(
        parameter_overrides=[
            Parameter('backend_enabled', value=True),
            Parameter('dry_run_mode', value='observe'),
            Parameter('sensor_timeout_s', value=5.0),
            Parameter('model_path', value=__file__),
        ],
        detector_factory=EmptyDetector,
    )
    client_node = Node('scan_for_person_cancel_client')
    client = ActionClient(client_node, ScanForPerson, 'scan_for_person')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=2.0)
        handle = wait_future(client.send_goal_async(scan_goal()))
        assert handle.accepted
        canceled = wait_future(handle.cancel_goal_async())
        assert len(canceled.goals_canceling) == 1
        wrapped = wait_future(handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_CANCELED
        assert wrapped.result.error.code == CapabilityError.CANCELED
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        client.destroy()
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()
