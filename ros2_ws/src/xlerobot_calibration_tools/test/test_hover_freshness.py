"""A transient camera frame must not require another operator button press."""
from types import SimpleNamespace

import pytest

from xlerobot_calibration_tools.hover_node import HoverNode


def test_waits_for_next_valid_observation_without_restamping(monkeypatch):
    calls = []
    row = {'stamp': 123.456}

    def observe_once():
        calls.append(True)
        if len(calls) == 1:
            raise ValueError('stale image')
        return row

    monkeypatch.setattr('xlerobot_calibration_tools.hover_node.time.sleep', lambda _: None)
    assert HoverNode.observe(SimpleNamespace(observe_once=observe_once)) is row
    assert len(calls) == 2


def test_persistent_invalid_observation_still_fails_bounded(monkeypatch):
    clock = iter([0., .4, 1.1])
    monkeypatch.setattr('xlerobot_calibration_tools.hover_node.time.monotonic', lambda: next(clock))
    monkeypatch.setattr('xlerobot_calibration_tools.hover_node.time.sleep', lambda _: None)

    def observe_once():
        raise ValueError('image/joint timestamps disagree')

    with pytest.raises(ValueError, match='image/joint timestamps disagree.*1 秒'):
        HoverNode.observe(SimpleNamespace(observe_once=observe_once))
