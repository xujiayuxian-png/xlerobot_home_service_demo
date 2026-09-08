from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from xlerobot_calibration_tools.bundle import UnitCalibrationStore
from xlerobot_calibration_tools.head_auto import HeadSession, load_poses
from xlerobot_calibration_tools.head_auto_node import HeadCalibrationNode, SweepPaused, stable_observations
from xlerobot_calibration_tools.sample_set import TransformSampleSet
from xlerobot_calibration_tools.solver import HEAD_MODEL
from xlerobot_interfaces.action import CalibrationJob
from xlerobot_interfaces.msg import CapabilityError
from rclpy.action import GoalResponse


REPO = Path(__file__).resolve().parents[4]
POSE_FILE = Path(__file__).resolve().parents[1] / 'config/head_camera_poses.yaml'


def session(directory):
    poses, identity = load_poses(POSE_FILE)
    return HeadSession(directory, 'test-unit', poses, identity)


def append_sample(sweep, index, job, *, bad_job=False):
    if sweep.sample_path.exists():
        samples = TransformSampleSet.read(sweep.sample_path)
    else:
        samples = TransformSampleSet('test-unit', HEAD_MODEL, 'base_link',
                                     'head_tilt_link', 'head_camera_link', 'calibration_target')
    moving = np.eye(4)
    moving[:3, :3] = Rotation.from_euler('xy', [index * 0.1, index * 0.05]).as_matrix()
    samples.append_and_write_atomic(sweep.sample_path, moving, np.eye(4), float(index),
                                    {'capture_job_id': 'wrong' if bad_job else job,
                                     'reprojection_rmse_px': 0.2, 'tag_count': 16})


def test_pose_set_center_then_complete_bounded_grid():
    poses, identity = load_poses(POSE_FILE)
    assert len(poses) == 25 and len(identity) == 64
    assert poses[0] == {'pan': 0.0, 'tilt': 0.8}
    assert {(p['pan'], p['tilt']) for p in poses} == {
        (pan, tilt) for pan in [-0.3, -0.15, 0, 0.15, 0.3]
        for tilt in [0.6, 0.7, 0.8, 0.9, 1.0]}


def test_pause_and_restart_keep_captured_and_retry_only_remaining(tmp_path):
    sweep = session(tmp_path)
    assert sweep.begin() == list(range(25))
    job = sweep.prepare_capture(0)
    append_sample(sweep, 0, job)
    sweep.captured(0)
    sweep.phase('MOVING', 'moving', 1)
    recovered = session(tmp_path)
    assert recovered.document['phase'] == 'PAUSED'
    assert recovered.document['sample_count'] == 1
    assert recovered.document['pose_states'][0] == 'captured'
    assert recovered.begin() == list(range(1, 25))


def test_capture_committed_before_checkpoint_is_reconciled_once(tmp_path):
    sweep = session(tmp_path)
    sweep.begin()
    job = sweep.prepare_capture(0)
    append_sample(sweep, 0, job)
    recovered = session(tmp_path)
    assert recovered.document['phase'] == 'PAUSED'
    assert recovered.document['sample_count'] == 1
    assert recovered.document['pose_states'][0] == 'captured'
    assert 0 not in recovered.begin()
    assert session(tmp_path).document['sample_count'] == 1


@pytest.mark.parametrize('failure', ['unit', 'poses', 'servo', 'changed_samples', 'wrong_job'])
def test_progress_rejects_mismatched_identity(tmp_path, failure):
    sweep = session(tmp_path)
    sweep.begin()
    if failure == 'unit':
        with pytest.raises(ValueError, match='different unit'):
            HeadSession(tmp_path, 'other-unit', sweep.poses, sweep.pose_hash)
    elif failure == 'poses':
        with pytest.raises(ValueError, match='pose set'):
            HeadSession(tmp_path, sweep.unit, sweep.poses, 'changed')
    elif failure == 'servo':
        with pytest.raises(ValueError, match='servo calibration'):
            HeadSession(tmp_path, sweep.unit, sweep.poses, sweep.pose_hash, servo_hash='changed')
    else:
        job = sweep.prepare_capture(0) if failure == 'wrong_job' else 'external-capture'
        append_sample(sweep, 0, job, bad_job=failure == 'wrong_job')
        with pytest.raises(ValueError, match='changed outside'):
            session(tmp_path)


def test_error_retries_skipped_without_recapturing_saved_samples(tmp_path):
    sweep = session(tmp_path)
    sweep.begin()
    job = sweep.prepare_capture(0)
    append_sample(sweep, 0, job)
    sweep.captured(0)
    sweep.skipped(1, 'board not visible')
    sweep.phase('ERROR', 'too few independent samples')
    assert sweep.begin() == list(range(1, 25))
    assert sweep.document['pose_states'][0] == 'captured'


def test_completed_session_restores_quality_and_rejects_new_start(tmp_path):
    sweep = session(tmp_path)
    sweep.complete('file:///draft/head_camera.yaml', {'sample_count': 12, 'translation_rmse_mm': 0.5})
    restored = session(tmp_path)
    assert restored.document['quality_passed']
    assert restored.document['metrics']['sample_count'] == 12
    with pytest.raises(ValueError, match='--fresh'):
        restored.begin()


def fake_node(tmp_path, monkeypatch):
    monkeypatch.setattr('xlerobot_calibration_tools.head_auto_node.rclpy.ok', lambda: True)
    node = object.__new__(HeadCalibrationNode)
    node.session = session(tmp_path / 'capture')
    node.store = UnitCalibrationStore(tmp_path / 'state', REPO)
    node.lock = threading.RLock()
    node.enabled = True
    node.busy = True
    node.motion_unconfirmed = False
    node.publish_status = lambda: None
    node.moves = []
    node.move = lambda handle, index: node.moves.append(index)
    node.wait_target = lambda handle: node.session.document['pose_index'] < 12
    fixture = TransformSampleSet.read(REPO / 'examples/calibration/head-camera/samples.yaml')
    samples = TransformSampleSet('test-unit', HEAD_MODEL, 'base_link', 'head_tilt_link',
                                'head_camera_link', 'calibration_target')

    def capture(request):
        index = len(samples.samples)
        row = fixture.samples[index]
        samples.append_and_write_atomic(node.session.sample_path, row.moving_in_base,
                                        row.target_in_camera, float(index),
                                        dict(fixture.qualities[index], capture_job_id=request.job_id))
        response = SimpleNamespace(error=SimpleNamespace(code=CapabilityError.NONE, message='captured'))
        return SimpleNamespace(done=lambda: True, result=lambda: response)

    node.capture_client = SimpleNamespace(wait_for_service=lambda **_: True, call_async=capture)
    handle = SimpleNamespace(is_cancel_requested=False, terminal='', publish_feedback=lambda _: None)
    handle.succeed = lambda: setattr(handle, 'terminal', 'succeeded')
    handle.abort = lambda: setattr(handle, 'terminal', 'aborted')
    handle.canceled = lambda: setattr(handle, 'terminal', 'canceled')
    return node, handle


def test_automatic_flow_uses_real_strict_solver_and_only_saves_draft(tmp_path, monkeypatch):
    node, handle = fake_node(tmp_path, monkeypatch)
    result = node.execute(handle)
    assert handle.terminal == 'succeeded', result.error.message
    assert result.quality_passed
    assert node.session.document['sample_count'] == 12
    assert node.session.document['pose_states'].count('captured') == 12
    assert node.session.document['pose_states'].count('skipped') == 13
    assert (node.store.draft_components('test-unit') / 'head_camera.yaml').is_file()
    assert not (node.store.unit_root('test-unit') / 'active.yaml').exists()
    assert not node.busy


def test_motion_failure_pauses_without_next_pose_or_capture(tmp_path, monkeypatch):
    node, handle = fake_node(tmp_path, monkeypatch)
    def fail_move(handle, index):
        node.moves.append(index)
        raise SweepPaused('controller reported failure')
    node.move = fail_move
    result = node.execute(handle)
    assert handle.terminal == 'aborted'
    assert node.moves == [0]
    assert node.session.document['phase'] == 'PAUSED'
    assert not node.session.sample_path.exists()
    assert result.error.code == CapabilityError.BACKEND_FAILURE


def test_no_board_fails_quality_without_overwriting_existing_draft(tmp_path, monkeypatch):
    node, handle = fake_node(tmp_path, monkeypatch)
    node.wait_target = lambda _: False
    draft = node.store.draft_components('test-unit') / 'head_camera.yaml'
    draft.parent.mkdir(parents=True)
    draft.write_text('keep existing draft', encoding='utf-8')
    result = node.execute(handle)
    assert handle.terminal == 'aborted'
    assert node.session.document['phase'] == 'ERROR'
    assert node.session.document['pose_states'].count('skipped') == 25
    assert draft.read_text() == 'keep existing draft'
    assert not result.quality_passed
    assert '0/12 independent samples' in result.error.message


def test_cancel_preserves_samples_and_does_not_save_draft(tmp_path, monkeypatch):
    node, handle = fake_node(tmp_path, monkeypatch)
    def wait_target(_):
        if node.session.document['sample_count'] == 1:
            handle.is_cancel_requested = True
            raise SweepPaused('operator paused')
        return True
    node.wait_target = wait_target
    result = node.execute(handle)
    assert handle.terminal == 'canceled'
    assert node.session.document['phase'] == 'PAUSED'
    assert node.session.document['sample_count'] == 1
    assert not node.store.draft_components('test-unit').exists()
    assert result.error.code == CapabilityError.CANCELED


@pytest.mark.parametrize('invalid', ['disabled', 'busy', 'motion_unknown', 'unit', 'workflow', 'manual', 'dry'])
def test_goal_requires_explicit_automatic_idle_enabled_session(tmp_path, monkeypatch, invalid):
    node, _ = fake_node(tmp_path, monkeypatch)
    node.busy = False
    goal = CalibrationJob.Goal(unit_id='test-unit', workflow_id='head_camera', automatic=True, dry_run=False)
    if invalid == 'disabled': node.enabled = False
    elif invalid == 'busy': node.busy = True
    elif invalid == 'motion_unknown': node.motion_unconfirmed = True
    elif invalid == 'unit': goal.unit_id = 'other-unit'
    elif invalid == 'workflow': goal.workflow_id = 'right_handeye'
    elif invalid == 'manual': goal.automatic = False
    elif invalid == 'dry': goal.dry_run = True
    assert node.goal(goal) == GoalResponse.REJECT


def test_duplicate_goal_is_rejected(tmp_path, monkeypatch):
    node, _ = fake_node(tmp_path, monkeypatch)
    node.busy = False
    goal = CalibrationJob.Goal(unit_id='test-unit', workflow_id='head_camera', automatic=True, dry_run=False)
    assert node.goal(goal) == GoalResponse.ACCEPT
    assert node.goal(goal) == GoalResponse.REJECT


@pytest.mark.parametrize('stamps,now,settled,expected', [
    ([10.0, 10.1, 10.2], 10.25, 9.9, True),
    ([10.0, 10.1, 10.2], 10.6, 9.9, False),
    ([9.0, 10.1, 10.2], 10.25, 8.9, False),
    ([10.0, 10.1, 10.5], 10.51, 9.9, False),
    ([10.0, 10.1, 10.2], 10.25, 10.1, False),
    ([10.0, 10.0, 10.2], 10.25, 9.9, False),
    ([10.0, 10.1, 10.3], 10.25, 9.9, False),
    ([10.1, 10.2], 10.25, 9.9, False),
])
def test_stable_observations_reject_stale_repeated_future_and_gapped_frames(stamps, now, settled, expected):
    assert stable_observations(stamps, now, settled) == expected
