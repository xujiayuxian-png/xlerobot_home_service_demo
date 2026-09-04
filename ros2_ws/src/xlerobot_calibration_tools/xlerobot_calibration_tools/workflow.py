"""Pure-software calibration workflow helpers used by the public CLI."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .quality import observability, validate_component
from .sample_set import TransformSampleSet
from .solver import ARM_MODEL, HEAD_MODEL, solve
from .transforms import to_dict


MODEL_BY_WORKFLOW = {
    'head_camera': HEAD_MODEL,
    'right_handeye': ARM_MODEL,
}


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except yaml.YAMLError as error:
        raise ValueError(f'invalid YAML in {path}: {error}') from error
    if not isinstance(document, dict):
        raise ValueError(f'expected a YAML mapping: {path}')
    return document


def solve_transform_samples(path: Path, workflow: str) -> dict[str, Any]:
    """Solve, report robust residuals, and enforce observability plus quality."""
    if workflow not in MODEL_BY_WORKFLOW:
        raise ValueError(f'unsupported transform workflow: {workflow}')
    path = path.expanduser().resolve()
    model = MODEL_BY_WORKFLOW[workflow]
    sample_set = TransformSampleSet.read(path, expected_model=model)
    coverage = observability(sample_set.samples, model)
    solution = solve(sample_set.samples, model)
    translations_mm = np.asarray(
        solution.metrics['per_sample_translation_m'], dtype=np.float64
    ) * 1000.0
    reprojection = np.asarray([
        float(quality.get('reprojection_rmse_px', math.nan))
        for quality in sample_set.qualities
    ])
    if not len(reprojection) or not np.all(np.isfinite(reprojection)):
        raise ValueError(
            'every visual sample must record finite quality.reprojection_rmse_px'
        )
    metrics = {
        'sample_count': len(sample_set.samples),
        'reprojection_rmse_px': float(np.sqrt(np.mean(reprojection**2))),
        'translation_rmse_mm': float(solution.metrics['translation_rmse_mm']),
        'translation_p95_mm': float(np.percentile(translations_mm, 95)),
        'translation_max_mm': float(solution.metrics['translation_max_mm']),
        'rotation_rmse_deg': float(solution.metrics['rotation_rmse_deg']),
        'rotation_max_deg': float(solution.metrics['rotation_max_deg']),
        'observability': coverage,
    }
    result = {
        'schema': 'xlerobot_transform_calibration/v1',
        'model': model,
        'calibration_id': sample_set.calibration_id,
        'created_at': utc_now(),
        'input_sha256': sha256_file(path),
        'frames': {
            'base': sample_set.base_frame,
            'moving': sample_set.moving_frame,
            'camera': sample_set.camera_frame,
            'target': sample_set.target_frame,
        },
        'method': solution.method,
        'quality_policy': 'xlerobot_calibration_quality/v1',
        'metrics': metrics,
        'x_semantics': (
            'mount_from_camera' if model == HEAD_MODEL else 'base_from_camera'
        ),
        'y_semantics': (
            'base_from_target' if model == HEAD_MODEL else 'gripper_from_target'
        ),
        'x': to_dict(solution.x),
        'y': to_dict(solution.y),
    }
    validate_component(workflow, result)
    return result


def fit_base_geometry(path: Path) -> dict[str, Any]:
    """Fit wheel radius/track from manually measured straight and turn trials."""
    path = path.expanduser().resolve()
    source = load_yaml(path)
    if source.get('schema') != 'xlerobot_base_measurements/v1':
        raise ValueError('expected xlerobot_base_measurements/v1')
    nominal_radius = _positive(source.get('nominal_wheel_radius_m'), 'nominal_wheel_radius_m')
    nominal_separation = _positive(
        source.get('nominal_wheel_separation_m'), 'nominal_wheel_separation_m'
    )
    straight = _trials(source.get('straight_trials'), 'commanded_m', 'actual_m')
    rotation = _trials(source.get('rotation_trials'), 'commanded_rad', 'actual_rad')
    straight_scales = [actual / commanded for commanded, actual in straight]
    rotation_scales = [commanded / actual for commanded, actual in rotation]
    straight_scale = float(np.mean(straight_scales))
    separation_scale = float(np.mean(rotation_scales))
    straight_errors = [
        100.0 * (actual / straight_scale - commanded) / commanded
        for commanded, actual in straight
    ]
    rotation_errors = [
        100.0 * (actual * separation_scale - commanded) / commanded
        for commanded, actual in rotation
    ]
    result = {
        'schema': 'xlerobot_base_geometry/v1',
        'created_at': utc_now(),
        'input_sha256': sha256_file(path),
        'wheel_radius_m': nominal_radius * straight_scale,
        'wheel_separation_m': nominal_separation * separation_scale,
        'straight_error_percent': max(abs(value) for value in straight_errors),
        'rotation_error_percent': max(abs(value) for value in rotation_errors),
        'fit': {
            'nominal_wheel_radius_m': nominal_radius,
            'nominal_wheel_separation_m': nominal_separation,
            'straight_trials': source['straight_trials'],
            'rotation_trials': source['rotation_trials'],
        },
    }
    validate_component('base_geometry', result)
    return result


def fit_grasp_alignment(path: Path) -> dict[str, Any]:
    """Fit the final visual/FK offset separately from repeatable gravity sag."""
    path = path.expanduser().resolve()
    source = load_yaml(path)
    if source.get('schema') != 'xlerobot_grasp_alignment_measurements/v1':
        raise ValueError('expected xlerobot_grasp_alignment_measurements/v1')
    if source.get('frame') != 'base_link':
        raise ValueError('grasp alignment measurements must use base_link')
    rows = source.get('samples')
    if not isinstance(rows, list) or len(rows) < 3:
        raise ValueError('at least three grasp alignment samples are required')
    deltas = []
    sag_values = []
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f'sample {index} must be a mapping')
        vision = _vector3(row.get('vision_xyz_m'), f'sample {index}.vision_xyz_m')
        kinematic = _vector3(row.get('fk_xyz_m'), f'sample {index}.fk_xyz_m')
        deltas.append(kinematic - vision)
        sag_values.append(_nonnegative(
            row.get('settled_z_shortfall_m'),
            f'sample {index}.settled_z_shortfall_m',
        ))
    delta_array = np.asarray(deltas)
    compensation = np.mean(delta_array, axis=0)
    residuals = np.linalg.norm(delta_array - compensation, axis=1)
    result = {
        'schema': 'xlerobot_grasp_alignment/v1',
        'created_at': utc_now(),
        'input_sha256': sha256_file(path),
        'frame': 'base_link',
        'vision_fk_compensation_m': compensation.tolist(),
        'gravity_sag_z_m': float(np.mean(sag_values)),
        'head_pose_rad': _head_pose(source.get('head_pose_rad')),
        'workspace_m': _workspace(source.get('workspace_m')),
        'metrics': {
            'sample_count': len(rows),
            'plane_rmse_mm': float(np.sqrt(np.mean(residuals**2)) * 1000.0),
            'gravity_sag_stddev_mm': float(np.std(sag_values) * 1000.0),
        },
    }
    validate_component('grasp_alignment', result)
    return result


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{label} must be finite')
    return result


def _positive(value: Any, label: str) -> float:
    result = _number(value, label)
    if result <= 0:
        raise ValueError(f'{label} must be positive')
    return result


def _nonnegative(value: Any, label: str) -> float:
    result = _number(value, label)
    if result < 0:
        raise ValueError(f'{label} must be nonnegative')
    return result


def _trials(value: Any, command_key: str, actual_key: str) -> list[tuple[float, float]]:
    if not isinstance(value, list) or len(value) < 2:
        raise ValueError(f'at least two {command_key} trials are required')
    trials = []
    for index, row in enumerate(value):
        if not isinstance(row, dict) or set(row) != {command_key, actual_key}:
            raise ValueError(f'trial {index} must contain {command_key} and {actual_key}')
        trials.append((
            _positive(row[command_key], f'trial {index}.{command_key}'),
            _positive(row[actual_key], f'trial {index}.{actual_key}'),
        ))
    return trials


def _vector3(value: Any, label: str) -> np.ndarray:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f'{label} must contain three values')
    return np.asarray([_number(item, f'{label}[{index}]') for index, item in enumerate(value)])


def _head_pose(value: Any) -> dict[str, float]:
    if not isinstance(value, dict) or set(value) != {'pan', 'tilt'}:
        raise ValueError('head_pose_rad must contain pan and tilt')
    return {name: _number(value[name], f'head_pose_rad.{name}') for name in ('pan', 'tilt')}


def _workspace(value: Any) -> dict[str, list[float]]:
    if not isinstance(value, dict) or set(value) != {'min', 'max'}:
        raise ValueError('workspace_m must contain min and max')
    return {
        name: _vector3(value[name], f'workspace_m.{name}').tolist()
        for name in ('min', 'max')
    }
