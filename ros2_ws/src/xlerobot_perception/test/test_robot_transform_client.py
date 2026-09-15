from types import SimpleNamespace

import pytest
from rclpy.duration import Duration
from rclpy.task import Future
from rclpy.time import Time
from tf2_ros import TransformException

from xlerobot_perception.robot_transform_client import RobotTransformClient


def test_transform_response_preserves_stamp_and_has_no_latest_fallback():
    calls = []
    def call(request):
        calls.append(request)
        future = Future()
        future.set_result(SimpleNamespace(success=False, error='extrapolation'))
        return future
    client = RobotTransformClient(SimpleNamespace(create_client=lambda *a, **kw:
        SimpleNamespace(wait_for_service=lambda **kw: True, call_async=call)), None)
    with pytest.raises(TransformException, match='extrapolation'):
        client.lookup_transform('map', 'camera', Time(nanoseconds=123), timeout=Duration())
    assert len(calls) == 1
    assert calls[0].stamp.nanosec == 123


def test_timed_out_request_is_removed_and_canceled(monkeypatch):
    removed = []
    future = Future()
    monkeypatch.setattr('xlerobot_perception.robot_transform_client.threading.Event',
        lambda: SimpleNamespace(wait=lambda _: False, set=lambda: None))
    client = RobotTransformClient(SimpleNamespace(create_client=lambda *a, **kw:
        SimpleNamespace(wait_for_service=lambda **kw: True, call_async=lambda _: future,
                        remove_pending_request=removed.append)), None)
    with pytest.raises(TransformException, match='timed out'):
        client.lookup_transform('map', 'camera', Time(), timeout=Duration())
    assert removed == [future] and future.cancelled()
