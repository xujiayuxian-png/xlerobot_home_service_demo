"""Read-only evidence timing; no motors, controller calls or live ROS graph."""
from types import MethodType, SimpleNamespace
import threading
import time

import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from rclpy.clock import ClockType
from tf2_ros import TransformException

from xlerobot_calibration_tools.collector_node import TransformSampleCollector
from xlerobot_calibration_tools.sample_set import TransformSampleSet
from xlerobot_calibration_tools.solver import HEAD_MODEL
from xlerobot_interfaces.msg import CalibrationTargetObservation, CapabilityError


def observation(age=0, accepted=True):
    msg = CalibrationTargetObservation()
    msg.header.stamp = Time(nanoseconds=time.time_ns() - int(age * 1e9)).to_msg()
    msg.accepted = accepted
    msg.tag_count = 1
    msg.reprojection_rmse_px = .1
    return msg


def collector(tmp_path):
    calls = []

    def lookup(parent, child, stamp):
        calls.append((parent, child, stamp.nanoseconds))
        result = TransformStamped()
        result.transform.rotation.w = 1.0
        return result

    node = SimpleNamespace(
        _lock=threading.Lock(), _observation_condition=threading.Condition(),
        _observation=None, _max_target_age_sec=.25,
        _samples=TransformSampleSet('test', HEAD_MODEL, 'base', 'moving', 'camera', 'target'),
        _output=tmp_path / 'samples.yaml',
        _buffer=SimpleNamespace(lookup_transform=lookup),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(
            nanoseconds=time.time_ns(), clock_type=ClockType.ROS_TIME)),
    )
    node._fresh_snapshot = MethodType(TransformSampleCollector._fresh_snapshot, node)
    node._on_observation = MethodType(TransformSampleCollector._on_observation, node)
    return node, calls


def test_stale_frame_waits_for_next_frame_and_uses_exact_image_tf(tmp_path):
    node, calls = collector(tmp_path)
    node._on_observation(observation(age=.4))
    fresh = observation()
    timer = threading.Timer(.03, node._on_observation, args=(fresh,))
    timer.start()
    try:
        msg, stamp, age, _, _ = node._fresh_snapshot()
    finally:
        timer.join()
    assert msg is fresh
    assert 0 <= age <= .25
    assert calls == [('camera', 'target', stamp.nanoseconds), ('base', 'moving', stamp.nanoseconds)]
    assert stamp == Time.from_msg(fresh.header.stamp)


@pytest.mark.parametrize('age,accepted', [(.4, True), (-2, True), (0, False)])
def test_stale_future_or_rejected_stream_still_fails_bounded(tmp_path, age, accepted):
    node, calls = collector(tmp_path)
    node._on_observation(observation(age, accepted))
    start = time.monotonic()
    with pytest.raises(ValueError, match='no synchronized fresh observation'):
        node._fresh_snapshot()
    assert time.monotonic() - start < 1.5
    assert calls == []


def test_observation_can_arrive_before_its_tf(tmp_path):
    node, calls = collector(tmp_path)
    lookup = node._buffer.lookup_transform
    ready = time.monotonic() + .04

    def delayed(*args):
        if time.monotonic() < ready:
            raise TransformException('not received yet')
        return lookup(*args)

    node._buffer.lookup_transform = delayed
    node._on_observation(observation())
    assert node._fresh_snapshot()[2] <= .25
    assert len(calls) == 2


def test_slow_yaml_read_does_not_freeze_latest_observation(tmp_path, monkeypatch):
    node, calls = collector(tmp_path)
    node._samples.write_atomic(node._output)
    read = TransformSampleSet.read

    def slow_read(*args, **kwargs):
        time.sleep(.35)  # More than the allowed observation age.
        return read(*args, **kwargs)

    monkeypatch.setattr(TransformSampleSet, 'read', slow_read)
    stop = threading.Event()

    def stream():
        while not stop.is_set():
            node._on_observation(observation())
            stop.wait(.02)

    thread = threading.Thread(target=stream)
    thread.start()
    response = SimpleNamespace(error=SimpleNamespace(code=None, message=''), sample_count=0)
    try:
        TransformSampleCollector._capture(node, SimpleNamespace(job_id='slow-disk'), response)
    finally:
        stop.set()
        thread.join()
    assert response.error.code == CapabilityError.NONE, response.error.message
    assert response.sample_count == 1
    stored = read(node._output)
    assert 0 <= stored.qualities[0]['observation_age_sec'] <= .25
    assert calls[0][2] == calls[1][2]
    assert stored.stamps_sec[0] == pytest.approx(calls[0][2] / 1e9, rel=0, abs=1e-6)
