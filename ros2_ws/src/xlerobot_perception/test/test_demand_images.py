from types import SimpleNamespace

import pytest

from xlerobot_perception.demand_images import DemandImages


def message(stamp):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=stamp)))


class FakeNode:
    group = None
    now = 100

    def __init__(self):
        self.active = []
        self.callbacks = []
        self.fail_after = None

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now))

    def create_subscription(self, kind, topic, callback, qos, **kwargs):
        if self.fail_after == len(self.active):
            raise RuntimeError('subscription creation failed')
        assert qos.depth == 1
        self.active.append(callback)
        self.callbacks.append(callback)
        return callback

    def destroy_subscription(self, subscription):
        self.active.remove(subscription)


def test_no_idle_frames_and_retired_callbacks_cannot_contaminate_next_goal():
    node, received = FakeNode(), []
    images = DemandImages(node, [(object, '/image', received.append)], received.clear, enabled=True)
    assert not node.active
    images.start()
    first = node.callbacks[0]
    first(message(99))
    assert not received
    first(message(100))
    assert len(received) == 1
    images.stop()
    first(message(101))
    assert not received and not node.active
    node.now = 200
    images.start()
    first(message(201))
    node.callbacks[-1](message(199))
    assert not received
    node.callbacks[-1](message(202))
    assert len(received) == 1
    images.stop()
    images.stop()
    assert not received and not node.active


def test_partial_subscription_failure_releases_all_resources():
    node = FakeNode()
    node.fail_after = 1
    images = DemandImages(node, [(object, '/a', lambda msg: None),
                                 (object, '/b', lambda msg: None)], lambda: None, enabled=True)
    with pytest.raises(RuntimeError):
        images.start()
    assert not node.active and not images.active
    node.fail_after = None
    images.start()
    assert len(node.active) == 2
    images.stop()
