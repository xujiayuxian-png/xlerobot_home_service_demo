"""Strict reader for the two runtime files required by classical grasping."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml


@dataclass(frozen=True)
class ClassicalRuntimeCalibration:
    """Validated values shared by classical planning and local execution."""

    vision_fk_compensation_m: tuple[float, float, float]
    gravity_sag_z_m: float
    head_pan_rad: float
    head_tilt_rad: float
    workspace_min_m: tuple[float, float, float]
    workspace_max_m: tuple[float, float, float]
    provenance: str


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be a YAML mapping')
    return value


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{label} must be finite')
    return result


def _vector3(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f'{label} must contain three values')
    return tuple(_number(item, f'{label}[{index}]') for index, item in enumerate(value))


def _read(path_value: str, label: str) -> tuple[dict[str, Any], bytes]:
    if not path_value.strip():
        raise ValueError(f'{label} path is required for centroid/gpd')
    path = Path(path_value).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f'{label} file does not exist: {path}')
    raw = path.read_bytes()
    try:
        document = yaml.safe_load(raw)
    except yaml.YAMLError as exc:
        raise ValueError(f'{label} is not valid YAML: {exc}') from exc
    return _mapping(document, label), raw


def _rigid_matrix(value: Any, label: str) -> None:
    document = _mapping(value, label)
    try:
        matrix = np.asarray(document['matrix'], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f'{label}.matrix must be numeric') from exc
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f'{label}.matrix must be a finite 4x4 matrix')
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-8):
        raise ValueError(f'{label}.matrix has an invalid homogeneous row')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-6):
        raise ValueError(f'{label}.matrix rotation is not orthonormal')
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-6):
        raise ValueError(f'{label}.matrix rotation determinant is not +1')


def _transform_component(
    value: Any,
    *,
    name: str,
    model: str,
    x_semantics: str,
    y_semantics: str,
) -> None:
    document = _mapping(value, name)
    if document.get('schema') != 'xlerobot_transform_calibration/v1':
        raise ValueError(f'{name} has an unsupported schema')
    if document.get('model') != model:
        raise ValueError(f'{name} has the wrong calibration model')
    if document.get('x_semantics') != x_semantics or document.get('y_semantics') != y_semantics:
        raise ValueError(f'{name} transform semantics are invalid')
    frames = _mapping(document.get('frames'), f'{name}.frames')
    required_frames = {'base', 'moving', 'camera', 'target'}
    if set(frames) != required_frames or any(
        not isinstance(frames[key], str) or not frames[key]
        for key in required_frames
    ):
        raise ValueError(f'{name}.frames must define four nonempty frame names')
    _rigid_matrix(document.get('x'), f'{name}.x')
    _rigid_matrix(document.get('y'), f'{name}.y')
    digest = document.get('input_sha256')
    if not isinstance(digest, str) or len(digest) != 64 or any(
        character not in '0123456789abcdef' for character in digest
    ):
        raise ValueError(f'{name}.input_sha256 is invalid')
    metrics = _mapping(document.get('metrics'), f'{name}.metrics')
    if _number(metrics.get('sample_count'), f'{name}.metrics.sample_count') <= 0:
        raise ValueError(f'{name}.metrics.sample_count must be positive')


def load_classical_runtime_calibration(
    transforms_file: str, grasp_alignment_file: str
) -> ClassicalRuntimeCalibration:
    """Load a complete activated runtime calibration or fail closed."""
    transforms, transforms_raw = _read(transforms_file, 'transforms_file')
    if transforms.get('schema') != 'xlerobot_runtime_transforms/v1':
        raise ValueError('transforms_file has an unsupported schema')
    _transform_component(
        transforms.get('head_camera'),
        name='head_camera',
        model='moving_camera_fixed_target',
        x_semantics='mount_from_camera',
        y_semantics='base_from_target',
    )
    _transform_component(
        transforms.get('right_handeye'),
        name='right_handeye',
        model='fixed_camera_moving_target',
        x_semantics='base_from_camera',
        y_semantics='gripper_from_target',
    )

    alignment, alignment_raw = _read(grasp_alignment_file, 'grasp_alignment_file')
    if alignment.get('schema') != 'xlerobot_grasp_alignment/v1':
        raise ValueError('grasp_alignment_file has an unsupported schema')
    if alignment.get('frame') != 'base_link':
        raise ValueError('grasp alignment frame must be base_link')
    compensation = _vector3(
        alignment.get('vision_fk_compensation_m'), 'vision_fk_compensation_m'
    )
    sag = _number(alignment.get('gravity_sag_z_m'), 'gravity_sag_z_m')
    if sag < 0.0:
        raise ValueError('gravity_sag_z_m must be nonnegative')
    head = _mapping(alignment.get('head_pose_rad'), 'head_pose_rad')
    if set(head) != {'pan', 'tilt'}:
        raise ValueError('head_pose_rad must contain pan and tilt')
    head_pan = _number(head['pan'], 'head_pose_rad.pan')
    head_tilt = _number(head['tilt'], 'head_pose_rad.tilt')
    workspace = _mapping(alignment.get('workspace_m'), 'workspace_m')
    if set(workspace) != {'min', 'max'}:
        raise ValueError('workspace_m must contain min and max')
    workspace_min = _vector3(workspace['min'], 'workspace_m.min')
    workspace_max = _vector3(workspace['max'], 'workspace_m.max')
    if any(left >= right for left, right in zip(workspace_min, workspace_max)):
        raise ValueError('workspace_m bounds must be ordered')
    metrics = _mapping(alignment.get('metrics'), 'grasp_alignment.metrics')
    if _number(metrics.get('sample_count'), 'grasp_alignment.metrics.sample_count') <= 0:
        raise ValueError('grasp_alignment.metrics.sample_count must be positive')
    if _number(metrics.get('plane_rmse_mm'), 'grasp_alignment.metrics.plane_rmse_mm') < 0:
        raise ValueError('grasp_alignment.metrics.plane_rmse_mm must be nonnegative')

    digest = hashlib.sha256(transforms_raw + b'\0' + alignment_raw).hexdigest()
    return ClassicalRuntimeCalibration(
        vision_fk_compensation_m=compensation,
        gravity_sag_z_m=sag,
        head_pan_rad=head_pan,
        head_tilt_rad=head_tilt,
        workspace_min_m=workspace_min,
        workspace_max_m=workspace_max,
        provenance=f'runtime-calibration-sha256:{digest}',
    )
