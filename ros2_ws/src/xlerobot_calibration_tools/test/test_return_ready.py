import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from xlerobot_calibration_tools.head_auto_node import HeadCalibrationNode, SweepPaused


@pytest.mark.parametrize('fails', [False, True])
def test_saved_draft_returns_ready_before_marking_complete(tmp_path, monkeypatch, fails):
    monkeypatch.setattr('rclpy.ok', lambda: True)
    monkeypatch.setattr('xlerobot_calibration_tools.head_auto_node.solve_transform_samples',
                        lambda *args: {'metrics': {'sample_count': 12}})
    order = []
    document = {'sample_count': 12, 'metrics': {}, 'message': '', 'phase': 'IDLE'}
    def phase(name, message, index=None):
        document.update(phase=name, message=message)
    def move(handle, index, *, return_ready=False):
        assert return_ready and document['phase'] == 'RETURNING'
        order.append('return_ready')
        if fails:
            raise SweepPaused('controller unavailable')
    def complete(uri, metrics):
        order.append('complete')
        document['phase'] = 'COMPLETED'
    session = SimpleNamespace(begin=lambda: [], unit='test-unit', sample_path=tmp_path / 'samples.yaml',
        poses=[], document=document, complete=complete, save=Mock(), pause=lambda message: phase('PAUSED', message))
    handle = SimpleNamespace(is_cancel_requested=False, succeed=Mock(), abort=Mock(), canceled=Mock())
    node = SimpleNamespace(lock=threading.RLock(), session=session, handeye=False, required=12,
        predecessors=[], predecessor_bytes=[], workflow='head_camera', move=move, transition=phase,
        store=SimpleNamespace(save_component=lambda *args: order.append('saved'), draft_components=lambda unit: tmp_path),
        publish_status=Mock(), busy=True)
    result = HeadCalibrationNode.execute(node, handle)
    assert order[:2] == ['saved', 'return_ready']
    if fails:
        assert 'complete' not in order and document['phase'] == 'PAUSED'
        assert '草稿已保存' in result.error.message
        handle.abort.assert_called_once()
    else:
        assert order == ['saved', 'return_ready', 'complete']
        assert result.quality_passed
        handle.succeed.assert_called_once()
