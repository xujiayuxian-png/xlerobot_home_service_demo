"""Read the active alignment shared with ACT; extrinsics come from live TF.

Solver quality gates belong to calibration activation. The stored sample region
describes calibration provenance, not a backend-specific robot motion envelope.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class GraspRuntimeCalibration:
    """Shared alignment values; all poses use the live robot TF tree."""

    vision_fk_compensation_m: tuple[float, float, float]
    gravity_sag_z_m: float
    head_pan_rad: float
    head_tilt_rad: float
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


def load_grasp_runtime_calibration(grasp_alignment_file: str) -> GraspRuntimeCalibration:
    """Accept measured or explicitly imported same-unit alignment, as ACT does."""
    if not grasp_alignment_file.strip():
        raise ValueError('grasp_alignment_file is required')
    raw = Path(grasp_alignment_file).expanduser().read_bytes()
    try:
        alignment = _mapping(yaml.safe_load(raw), 'grasp_alignment_file')
    except yaml.YAMLError as exc:
        raise ValueError(f'grasp_alignment_file is not valid YAML: {exc}') from exc
    if alignment.get('schema') != 'xlerobot_grasp_alignment/v1':
        raise ValueError('grasp_alignment_file has an unsupported schema')
    if alignment.get('frame') != 'base_link':
        raise ValueError('grasp alignment frame must be base_link')
    compensation = _vector3(alignment.get('vision_fk_compensation_m'),
                            'vision_fk_compensation_m')
    sag = _number(alignment.get('gravity_sag_z_m'), 'gravity_sag_z_m')
    if sag < 0.0:
        raise ValueError('gravity_sag_z_m must be nonnegative')
    head = _mapping(alignment.get('head_pose_rad'), 'head_pose_rad')
    if set(head) != {'pan', 'tilt'}:
        raise ValueError('head_pose_rad must contain pan and tilt')
    pan = _number(head['pan'], 'head_pose_rad.pan')
    tilt = _number(head['tilt'], 'head_pose_rad.tilt')
    workspace = _mapping(alignment.get('workspace_m'), 'workspace_m')
    lower = _vector3(workspace.get('min'), 'workspace_m.min')
    upper = _vector3(workspace.get('max'), 'workspace_m.max')
    if any(left >= right for left, right in zip(lower, upper)):
        raise ValueError('workspace_m bounds must be ordered')
    imported = alignment.get('validation') == 'existing_unit_runtime'
    if imported:
        if not _mapping(alignment.get('provenance'), 'provenance'):
            raise ValueError('imported alignment requires source provenance')
    else:
        metrics = _mapping(alignment.get('metrics'), 'grasp_alignment.metrics')
        if _number(metrics.get('sample_count'), 'metrics.sample_count') <= 0:
            raise ValueError('metrics.sample_count must be positive')
        if _number(metrics.get('plane_rmse_mm'), 'metrics.plane_rmse_mm') < 0:
            raise ValueError('metrics.plane_rmse_mm must be nonnegative')
    source = 'existing_unit_runtime' if imported else 'measured'
    return GraspRuntimeCalibration(
        compensation, sag, pan, tilt,
        f'runtime-calibration-sha256:{hashlib.sha256(raw).hexdigest()}; source={source}',
    )
