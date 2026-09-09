"""Automatic hand-eye evidence: fit first, freeze, then test unseen poses.

Tag 23 moves with the fixed jaw. Closure is X @ camera_from_tag versus
base_from_jaw @ Y, NOT a stationary tag in the base frame.
"""

from copy import deepcopy
import math
from pathlib import Path

import numpy as np
import yaml

from .head_auto import HeadSession, digest
from .quality import quality_profile, validate_component
from .sample_set import TransformSampleSet
from .solver import ARM_MODEL, residual_metrics
from .transforms import validate_transform
from .workflow import load_yaml, sha256_file, solve_transform_samples, utc_now


JOINTS = ['right_arm_shoulder_pan', 'right_arm_shoulder_lift',
          'right_arm_elbow_flex', 'right_arm_wrist_flex', 'right_arm_wrist_roll']
FIT_COUNT = 20
VALIDATION_COUNT = 6
FRAMES = {'base': 'base_link', 'moving': 'right_arm_fixed_jaw_link',
          'camera': 'd455_color_optical_frame', 'target': 'calibration_target'}


def load_poses(path):
    value = load_yaml(Path(path))
    if value.get('schema') != 'xlerobot_calibration_pose_set/v1' \
            or value.get('workflow') != 'right_handeye' or value.get('units') != 'rad' \
            or value.get('joint_order') != JOINTS or value.get('fit_count') != FIT_COUNT \
            or value.get('head_pose') != [0.0, 0.796136]:
        raise ValueError('expected the reference right_handeye pose set and fixed head pose')
    rows = value.get('poses')
    # This is a bounded reference sweep, not an arbitrary trajectory uploader.
    # Flex envelope includes the per-pose upward visibility adjustments.
    bounds = [(-0.92, 0.36), (-0.56, 1.39), (0.10, 1.19), (-0.98, 0.46), (1.50, 1.51)]
    if not isinstance(rows, list) or len(rows) != FIT_COUNT + VALIDATION_COUNT:
        raise ValueError('hand-eye requires 20 fitting and 6 held-out poses')
    for row in rows:
        if not isinstance(row, list) or len(row) != 5 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not lo <= v <= hi
                for v, (lo, hi) in zip(row, bounds)):
            raise ValueError('hand-eye pose is outside the reference capture envelope')
    if len({tuple(row) for row in rows}) != len(rows):
        raise ValueError('fitting and held-out poses must be distinct')
    offset = value.get('wrist_roll_offset_rad', 0.0)
    if isinstance(offset, bool) or offset not in (0.0, -math.pi / 2):
        raise ValueError('unsupported hand-eye wrist mounting offset')
    rows = [row[:4] + [row[4] + offset] for row in rows]
    return rows, sha256_file(Path(path))


def write_yaml(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_text(yaml.safe_dump(document, sort_keys=False), encoding='utf-8')
    temporary.replace(path)


def validate_heldout(candidate, training_path, heldout_path):
    """Evaluate frozen transforms without invoking a solver or modifying them."""
    validate_component('right_handeye', candidate)
    if candidate['input_sha256'] != sha256_file(training_path):
        raise ValueError('frozen fit no longer matches its training samples')
    training = TransformSampleSet.read(training_path, expected_model=ARM_MODEL,
                                       expected_calibration_id=candidate['calibration_id'],
                                       expected_frames=candidate['frames'])
    validation = TransformSampleSet.read(heldout_path, expected_model=ARM_MODEL,
                                         expected_calibration_id=candidate['calibration_id'],
                                         expected_frames=candidate['frames'])
    if len(validation.samples) < VALIDATION_COUNT:
        raise ValueError('six independent held-out samples are required')
    # Also reject reuse of a near-identical training pose, even with a new ID.
    merged = deepcopy(training)
    for sample, stamp, quality in zip(validation.samples, validation.stamps_sec, validation.qualities):
        if stamp <= max(training.stamps_sec):
            raise ValueError('held-out observations must be acquired after fitting observations')
        merged.append(sample.moving_in_base, sample.target_in_camera, stamp, quality)
    x = validate_transform(candidate['x']['matrix'])
    y = validate_transform(candidate['y']['matrix'])
    errors = residual_metrics(validation.samples, ARM_MODEL, x, y)
    reprojection = np.asarray([q.get('reprojection_rmse_px', np.nan) for q in validation.qualities])
    if not np.all(np.isfinite(reprojection)) or np.any(reprojection < 0):
        raise ValueError('held-out reprojection errors must be finite and nonnegative')
    metrics = {k: v for k, v in errors.items() if not k.startswith('per_sample_')}
    metrics.update(sample_count=len(validation.samples),
                   translation_p95_mm=float(np.percentile(errors['per_sample_translation_m'], 95) * 1000),
                   reprojection_rmse_px=float(np.sqrt(np.mean(reprojection ** 2))))
    profile = quality_profile('right_handeye')
    failures = [name for name in ('translation_rmse_mm', 'translation_p95_mm',
                                  'rotation_rmse_deg', 'reprojection_rmse_px')
                if metrics[name] >= profile['maximum_' + name]]
    return {
        'schema': 'xlerobot_handeye_validation/v1', 'created_at': utc_now(),
        'calibration_id': candidate['calibration_id'], 'frames': candidate['frames'],
        'fit_document_sha256': digest(candidate), 'training_sha256': sha256_file(training_path),
        'heldout_sha256': sha256_file(heldout_path), 'refitted': False,
        'quality_passed': not failures, 'failed_metrics': failures, 'metrics': metrics,
        'thresholds': {name: profile['maximum_' + name] for name in (
            'translation_rmse_mm', 'translation_p95_mm', 'rotation_rmse_deg', 'reprojection_rmse_px')},
        'per_pose': [{'capture_job_id': quality['capture_job_id'],
                      'translation_mm': float(t * 1000), 'rotation_deg': float(np.degrees(r))}
                     for quality, t, r in zip(validation.qualities, errors['per_sample_translation_m'],
                                              errors['per_sample_rotation_rad'])],
        'scope': 'Independent visual/FK closure; not absolute fingertip or grasp accuracy.',
    }


class HandeyeSession(HeadSession):
    workflow = 'right_handeye'
    model = ARM_MODEL
    frames = FRAMES
    schema = 'xlerobot_handeye_capture_progress/v1'

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fit_path = self.directory / 'fit.yaml'
        self.training_path = self.directory / 'training.yaml'
        self.heldout_path = self.directory / 'heldout.yaml'
        self.report_path = self.directory / 'validation.yaml'
        if self.document['sample_count'] and not self.path.exists():
            raise ValueError('untracked hand-eye samples; archive with --fresh before starting')

    def subset(self, role):
        samples = self._samples()
        if samples is None:
            raise ValueError('no hand-eye samples collected')
        selected = []
        seen = set()
        for row in samples['samples']:
            job = row['quality'].get('capture_job_id', '')
            index = next((i for i in range(len(self.poses))
                          if job == f'{self.unit}:{self.workflow}:{self.pose_hash[:12]}:pose:{i}'), None)
            if index is None or index in seen:
                raise ValueError('sample does not belong to a unique pose in this session')
            seen.add(index)
            if (index < FIT_COUNT) == (role == 'fit'):
                selected.append(dict(row, index=len(selected)))
        return dict(samples, samples=selected)

    def freeze_fit(self):
        training = self.subset('fit')
        if len(training['samples']) != FIT_COUNT:
            raise ValueError('20/20 fitting poses required before held-out validation; continue to retry missing poses')
        provenance = {'pose_sha256': self.pose_hash, 'predecessor_sha256': self.document['servo_sha256']}
        if self.fit_path.exists():
            candidate = load_yaml(self.fit_path)
            if candidate.get('capture_provenance') != provenance \
                    or not self.training_path.exists() or load_yaml(self.training_path) != training \
                    or candidate['input_sha256'] != sha256_file(self.training_path):
                raise ValueError('frozen fitting evidence changed; archive and start --fresh')
            validate_component('right_handeye', candidate)
        else:
            if len(self.subset('validation')['samples']):
                raise ValueError('held-out samples exist without a frozen fit')
            write_yaml(self.training_path, training)
            candidate = solve_transform_samples(self.training_path, 'right_handeye')
            candidate['capture_provenance'] = provenance
            write_yaml(self.fit_path, candidate)
        self.document['metrics']['fit'] = candidate['metrics']
        self.save()
        return candidate

    def finish_validation(self):
        candidate = self.freeze_fit()
        write_yaml(self.heldout_path, self.subset('validation'))
        report = validate_heldout(candidate, self.training_path, self.heldout_path)
        report['fit_sha256'] = sha256_file(self.fit_path)
        write_yaml(self.report_path, report)
        self.document['metrics']['validation'] = report['metrics']
        self.document['metrics']['heldout_poses'] = {
            str(int(row['capture_job_id'].rsplit(':', 1)[1]) + 1): {
                'translation_mm': row['translation_mm'], 'rotation_deg': row['rotation_deg']}
            for row in report['per_pose']}
        self.save()
        if not report['quality_passed']:
            raise ValueError('independent validation failed: ' + ', '.join(report['failed_metrics'])
                             + '; report retained, draft unchanged; inspect mounting and use --fresh')
        return dict(candidate, independent_validation={
            'report_sha256': sha256_file(self.report_path),
            'heldout_sha256': report['heldout_sha256'], 'refitted': False,
            'quality_passed': True, 'metrics': report['metrics']})
