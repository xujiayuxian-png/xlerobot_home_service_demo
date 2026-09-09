"""No hardware: archive/restart must preserve evidence and reset sample caches."""
from pathlib import Path
from types import SimpleNamespace
import threading
import hashlib

import pytest

from xlerobot_calibration_tools.head_auto import HeadSession
from xlerobot_calibration_tools.handeye_auto import HandeyeSession
from xlerobot_calibration_tools.sample_set import TransformSampleSet
from xlerobot_calibration_tools.solver import HEAD_MODEL


@pytest.mark.parametrize('kind', [HeadSession, HandeyeSession])
def test_restart_archives_all_evidence_and_preserves_draft(tmp_path, kind):
    directory = tmp_path / 'capture'
    old = kind(directory, 'unit', [], 'poses', servo_hash='servo')
    old.phase('COMPLETED', 'passed')
    for name in ('samples.yaml', 'fit.yaml', 'validation.yaml'):
        (directory / name).write_bytes(b'original evidence')
    draft = tmp_path / 'draft.yaml'
    draft.write_bytes(b'passing draft')
    fresh, archive = old.restart(draft)
    assert fresh.document['sample_count'] == 0
    assert fresh.document['phase'] == 'IDLE'
    assert fresh.document['quality_passed'] is False
    assert not fresh.sample_path.exists()
    assert (archive / 'samples.yaml').read_bytes() == b'original evidence'
    assert (archive / 'validation.yaml').read_bytes() == b'original evidence'
    assert (archive / 'draft_snapshot.yaml').read_bytes() == draft.read_bytes() == b'passing draft'
    restored = kind(directory, 'unit', [], 'poses', servo_hash='servo')
    assert restored.document == fresh.document


def test_collector_does_not_reintroduce_archived_memory(tmp_path):
    from xlerobot_calibration_tools.collector_node import TransformSampleCollector
    samples = TransformSampleSet('unit', HEAD_MODEL, 'base', 'moving', 'camera', 'target')
    samples.samples = [object()]
    samples.stamps_sec = [1.0]
    samples.qualities = [{}]
    collector = SimpleNamespace(_lock=threading.Lock(), _samples=samples,
                                _output=tmp_path / 'new' / 'samples.yaml')
    response = SimpleNamespace(error=SimpleNamespace(code=None, message=''), sample_count=-1)
    TransformSampleCollector._capture(collector, SimpleNamespace(job_id=''), response)
    assert collector._samples.samples == []
    assert collector._samples.stamps_sec == []
    assert collector._samples.qualities == []
    assert response.sample_count == 0


@pytest.mark.parametrize('flag', ['busy', 'motion_unconfirmed', 'capture_unconfirmed'])
def test_restart_refuses_active_or_unconfirmed_work(flag):
    from xlerobot_calibration_tools.head_auto_node import HeadCalibrationNode
    node = SimpleNamespace(lock=threading.RLock(), busy=False,
                           motion_unconfirmed=False, capture_unconfirmed=False)
    setattr(node, flag, True)
    response = SimpleNamespace(success=None, message='')
    assert not HeadCalibrationNode.reset_session(node, None, response).success


@pytest.mark.parametrize('kind', [HeadSession, HandeyeSession])
def test_reset_service_returns_idle_and_repeat_is_noop(tmp_path, kind):
    from xlerobot_calibration_tools.head_auto_node import HeadCalibrationNode
    pose = tmp_path / 'poses.yaml'
    pose.write_bytes(b'pose configuration')
    session = kind(tmp_path / 'work', 'unit', [], hashlib.sha256(pose.read_bytes()).hexdigest())
    session.phase('COMPLETED', 'done')
    node = SimpleNamespace(lock=threading.RLock(), busy=False,
        motion_unconfirmed=False, capture_unconfirmed=False, session=session,
        pose_file=pose, predecessors=[], predecessor_bytes=[],
        draft_path=tmp_path / 'draft.yaml', observations=[], publish_status=lambda: None)
    response = SimpleNamespace(success=None, message='')
    HeadCalibrationNode.reset_session(node, None, response)
    assert response.success
    assert node.session.document['phase'] == 'IDLE'
    archives = list(tmp_path.glob('work.archive-*'))
    assert len(archives) == 1
    HeadCalibrationNode.reset_session(node, None, response)
    assert response.success
    assert list(tmp_path.glob('work.archive-*')) == archives
