"""Hardware-free fit/holdout lifecycle; no serial devices or ROS controllers."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

from xlerobot_calibration_tools.handeye_auto import (
    HandeyeSession, load_poses, validate_heldout, write_yaml, FRAMES,
)
from xlerobot_calibration_tools.sample_set import TransformSampleSet
from xlerobot_calibration_tools.solver import ARM_MODEL
from xlerobot_calibration_tools.transforms import invert

POSES = Path(__file__).resolve().parents[1] / 'config/right_handeye_poses.yaml'


def test_clockwise_mount_offset_preserves_other_joints(tmp_path):
    raw = yaml.safe_load(POSES.read_text())
    poses, _ = load_poses(POSES)
    for actual, reference in zip(poses, raw['poses']):
        assert actual[:4] == reference[:4]
        assert actual[4] == pytest.approx(reference[4] - np.pi / 2)
    raw['wrist_roll_offset_rad'] = 0.2
    invalid = tmp_path / 'invalid.yaml'
    invalid.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match='mounting offset'):
        load_poses(invalid)


def test_lifted_flex_stays_in_bounded_capture_envelope(tmp_path):
    poses, _ = load_poses(POSES)
    assert min(row[3] for row in poses) == pytest.approx(-.9747)
    raw = yaml.safe_load(POSES.read_text())
    raw['poses'][11][3] = -.99
    invalid = tmp_path / 'outside-flex-envelope.yaml'
    invalid.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match='capture envelope'):
        load_poses(invalid)


def session(path):
    poses, identity = load_poses(POSES)
    return HandeyeSession(path, 'test-unit', poses, identity, servo_hash='test-predecessors')


def synthetic_rows():
    rng = np.random.default_rng(2308)
    x, y = np.eye(4), np.eye(4)
    x[:3, :3] = Rotation.from_euler('xyz', [.2, -.3, .4]).as_matrix()
    x[:3, 3] = [.25, -.15, .65]
    y[:3, :3] = Rotation.from_euler('xyz', [.1, .2, -.1]).as_matrix()
    y[:3, 3] = [.02, .01, .06]
    rows = []
    for _ in range(26):
        a = np.eye(4)
        a[:3, :3] = Rotation.from_rotvec(rng.uniform(-.6, .6, 3)).as_matrix()
        a[:3, 3] = rng.uniform([.15, -.35, .2], [.55, .1, .6])
        rows.append((a, invert(x) @ a @ y))
    return rows


def capture(sweep, index, *, perturb=0, checkpoint=True):
    job = sweep.prepare_capture(index)
    if sweep.sample_path.exists():
        samples = TransformSampleSet.read(sweep.sample_path)
    else:
        samples = TransformSampleSet(sweep.unit, ARM_MODEL, *FRAMES.values())
    a, b = synthetic_rows()[index]
    b[0, 3] += perturb
    samples.append_and_write_atomic(sweep.sample_path, a, b, float(index),
                                    {'capture_job_id': job, 'tag_count': 1, 'reprojection_rmse_px': .2})
    if checkpoint:
        sweep.captured(index)


def fitted(path):
    sweep = session(path)
    sweep.begin()
    for index in range(20):
        capture(sweep, index)
    sweep.freeze_fit()
    return sweep


def test_fit_is_frozen_before_unseen_samples_and_never_refitted(tmp_path, monkeypatch):
    sweep = fitted(tmp_path)
    frozen = sweep.fit_path.read_bytes()
    monkeypatch.setattr('xlerobot_calibration_tools.handeye_auto.solve_transform_samples',
                        lambda *_: pytest.fail('must not refit after freezing'))
    for index in range(20, 26):
        capture(sweep, index)
    result = sweep.finish_validation()
    assert sweep.fit_path.read_bytes() == frozen
    assert len(yaml.safe_load(sweep.training_path.read_text())['samples']) == 20
    assert len(yaml.safe_load(sweep.heldout_path.read_text())['samples']) == 6
    assert result['independent_validation']['refitted'] is False
    assert result['independent_validation']['metrics']['translation_rmse_mm'] < 1e-6
    assert len(yaml.safe_load(sweep.report_path.read_text())['per_pose']) == 6
    assert sweep.document['metrics']['fit']['sample_count'] == 20
    assert sweep.document['metrics']['validation']['sample_count'] == 6


def test_bad_holdout_saves_failed_report_without_changing_frozen_fit(tmp_path):
    sweep = fitted(tmp_path)
    frozen = sweep.fit_path.read_bytes()
    for index in range(20, 26):
        capture(sweep, index, perturb=.03)
    with pytest.raises(ValueError, match='independent validation failed'):
        sweep.finish_validation()
    report = yaml.safe_load(sweep.report_path.read_text())
    assert not report['quality_passed']
    assert report['metrics']['translation_rmse_mm'] == pytest.approx(30)
    assert sweep.fit_path.read_bytes() == frozen
    assert not sweep.document['quality_passed']


def test_restart_after_committed_validation_sample_preserves_fit(tmp_path):
    sweep = fitted(tmp_path)
    frozen = sweep.fit_path.read_bytes()
    capture(sweep, 20, checkpoint=False)
    resumed = session(tmp_path)
    assert resumed.begin() == list(range(21, 26))
    resumed.freeze_fit()
    assert resumed.fit_path.read_bytes() == frozen


def test_incomplete_fit_cannot_enter_validation(tmp_path):
    sweep = session(tmp_path)
    sweep.begin()
    capture(sweep, 0)
    with pytest.raises(ValueError, match='20/20'):
        sweep.freeze_fit()
    assert not sweep.fit_path.exists()


@pytest.mark.parametrize('corruption', ['unit', 'frames', 'nan', 'nonrigid', 'reused_pose', 'old_stamp'])
def test_independent_validation_rejects_invalid_or_reused_evidence(tmp_path, corruption):
    sweep = fitted(tmp_path)
    for index in range(20, 26):
        capture(sweep, index)
    sweep.finish_validation()
    document = yaml.safe_load(sweep.heldout_path.read_text())
    if corruption == 'unit': document['calibration_id'] = 'other'
    elif corruption == 'frames': document['frames']['camera'] = 'other_camera'
    elif corruption == 'nan': document['samples'][0]['target_in_camera'][0][0] = float('nan')
    elif corruption == 'nonrigid': document['samples'][0]['target_in_camera'][0][0] = 10.0
    elif corruption == 'old_stamp': document['samples'][0]['stamp_sec'] = 0.0
    else:
        train = yaml.safe_load(sweep.training_path.read_text())['samples'][0]
        document['samples'][0]['moving_in_base'] = deepcopy(train['moving_in_base'])
    write_yaml(sweep.heldout_path, document)
    with pytest.raises(ValueError):
        validate_heldout(yaml.safe_load(sweep.fit_path.read_text()), sweep.training_path, sweep.heldout_path)


@pytest.mark.parametrize('kind', ['training', 'predecessor'])
def test_frozen_fit_rejects_changed_provenance(tmp_path, kind):
    sweep = fitted(tmp_path)
    if kind == 'training': sweep.training_path.write_text('changed: true')
    else: sweep.document['servo_sha256'] = 'new-head-calibration'
    with pytest.raises(ValueError, match='frozen fitting evidence changed'):
        sweep.freeze_fit()


@pytest.mark.parametrize('kind', ['duplicate', 'nan', 'units', 'head'])
def test_pose_configuration_rejects_invalid_values(tmp_path, kind):
    document = yaml.safe_load(POSES.read_text())
    if kind == 'duplicate': document['poses'][-1] = document['poses'][0]
    elif kind == 'nan': document['poses'][0][0] = float('nan')
    elif kind == 'units': document['units'] = 'deg'
    else: document['head_pose'] = [0, .9]
    path = tmp_path / 'poses.yaml'
    write_yaml(path, document)
    with pytest.raises(ValueError): load_poses(path)
