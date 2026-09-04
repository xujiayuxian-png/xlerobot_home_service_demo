"""Validation and quality gates for public calibration artifacts."""

from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from .profiles import quality_profile
from .solver import ARM_MODEL, CalibrationSample, HEAD_MODEL
from .transforms import validate_transform


SERVO_GROUPS = {
    'right_arm': {
        'shoulder_pan': 1, 'shoulder_lift': 2, 'elbow_flex': 3,
        'wrist_flex': 4, 'wrist_roll': 5, 'gripper': 6,
    },
    'left_arm': {
        'shoulder_pan': 1, 'shoulder_lift': 2, 'elbow_flex': 3,
        'wrist_flex': 4, 'wrist_roll': 5, 'gripper': 6,
    },
    'head': {'pan': 7, 'tilt': 8},
}


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{label} must be finite')
    return result


def _integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f'{label} must be an integer')
    return int(value)


def validate_servo(document: dict[str, Any]) -> dict[str, float]:
    """Validate the exact two-arm/head servo layout used by this demo."""
    if not isinstance(document, dict) or document.get('schema') != 'xlerobot_servo_calibration/v1':
        raise ValueError('expected xlerobot_servo_calibration/v1')
    if set(document) - {'schema', *SERVO_GROUPS}:
        raise ValueError('servo calibration contains unsupported top-level fields')
    bus_ids: dict[str, set[int]] = {'right': set(), 'left': set()}
    minimum_coverage = float(quality_profile('servo')['minimum_range_coverage'])
    minimum_seen = float('inf')
    for group, expected in SERVO_GROUPS.items():
        value = document.get(group)
        if not isinstance(value, dict) or set(value) != {'joints'}:
            raise ValueError(f'servo calibration must contain only {group}.joints')
        joints = value['joints']
        if not isinstance(joints, dict) or set(joints) != set(expected):
            raise ValueError(f'{group} must contain exactly the reference joint names')
        bus = 'right' if group == 'right_arm' else 'left'
        for name, expected_id in expected.items():
            label = f'{group}.{name}'
            row = joints[name]
            if not isinstance(row, dict):
                raise ValueError(f'{label} must be a mapping')
            required = {
                'servo_id', 'direction', 'offset', 'raw_min', 'raw_max',
                'limit_min', 'limit_max',
            }
            if set(row) != required:
                raise ValueError(f'{label} fields do not match the servo schema')
            servo_id = _integer(row['servo_id'], f'{label}.servo_id')
            if servo_id != expected_id:
                raise ValueError(f'{label}.servo_id must be {expected_id}')
            if servo_id in bus_ids[bus]:
                raise ValueError(f'duplicate servo_id {servo_id} on {bus} bus')
            bus_ids[bus].add(servo_id)
            direction = _integer(row['direction'], f'{label}.direction')
            if direction not in (-1, 1):
                raise ValueError(f'{label}.direction must be -1 or 1')
            offset = _integer(row['offset'], f'{label}.offset')
            raw_min = _integer(row['raw_min'], f'{label}.raw_min')
            raw_max = _integer(row['raw_max'], f'{label}.raw_max')
            if any(value < 0 or value > 4095 for value in (offset, raw_min, raw_max)):
                raise ValueError(f'{label} raw values must be within 0..4095')
            if raw_min >= raw_max:
                raise ValueError(f'{label}.raw_min must be below raw_max')
            if not raw_min <= offset <= raw_max:
                raise ValueError(f'{label}.offset must be inside the captured raw range')
            lower = _number(row['limit_min'], f'{label}.limit_min')
            upper = _number(row['limit_max'], f'{label}.limit_max')
            if lower >= upper:
                raise ValueError(f'{label} ROS limits must be ordered')
            required_ticks = (upper - lower) * 4096.0 / (2.0 * math.pi)
            coverage = (raw_max - raw_min) / required_ticks
            if coverage < minimum_coverage:
                raise ValueError(
                    f'{label} captured range covers {coverage:.1%}; '
                    f'{minimum_coverage:.0%} is required'
                )
            minimum_seen = min(minimum_seen, coverage)
    return {'joint_count': 14.0, 'minimum_range_coverage': minimum_seen}


def validate_base_geometry(document: dict[str, Any]) -> dict[str, float]:
    if not isinstance(document, dict) or document.get('schema') != 'xlerobot_base_geometry/v1':
        raise ValueError('expected xlerobot_base_geometry/v1')
    radius = _number(document.get('wheel_radius_m'), 'wheel_radius_m')
    separation = _number(document.get('wheel_separation_m'), 'wheel_separation_m')
    if radius <= 0 or separation <= 0:
        raise ValueError('wheel radius and separation must be positive')
    straight = abs(_number(document.get('straight_error_percent'), 'straight_error_percent'))
    rotation = abs(_number(document.get('rotation_error_percent'), 'rotation_error_percent'))
    limit = float(quality_profile('base_geometry')['maximum_residual_percent'])
    if max(straight, rotation) > limit:
        raise ValueError(f'base geometry residual exceeds {limit:g}%')
    return {
        'wheel_radius_m': radius,
        'wheel_separation_m': separation,
        'straight_error_percent': straight,
        'rotation_error_percent': rotation,
    }


def observability(samples: Sequence[CalibrationSample], model: str) -> dict[str, float]:
    """Reject low-residual datasets that do not excite enough independent motion."""
    if not samples:
        raise ValueError('calibration sample set is empty')
    transforms = [validate_transform(sample.moving_in_base) for sample in samples]
    first = transforms[0][:3, :3]
    vectors = np.asarray([
        Rotation.from_matrix(first.T @ value[:3, :3]).as_rotvec()
        for value in transforms
    ])
    centered = vectors - np.mean(vectors, axis=0)
    singular = np.linalg.svd(centered, compute_uv=False)
    rotation_rank = int(np.count_nonzero(singular > 0.05))
    maximum_angle = 0.0
    for index, left in enumerate(transforms):
        for right in transforms[index + 1:]:
            maximum_angle = max(
                maximum_angle,
                float(np.degrees(Rotation.from_matrix(
                    left[:3, :3].T @ right[:3, :3]
                ).magnitude())),
            )
    profile_name = 'head_camera' if model == HEAD_MODEL else 'right_handeye'
    profile = quality_profile(profile_name)['observability']
    if rotation_rank < int(profile['minimum_rotation_rank']):
        raise ValueError('calibration motions are not rotationally observable')
    if maximum_angle < float(profile['minimum_max_pairwise_rotation_deg']):
        raise ValueError('calibration motions do not span enough rotation')
    result = {
        'rotation_rank': float(rotation_rank),
        'max_pairwise_rotation_deg': maximum_angle,
    }
    if model == ARM_MODEL:
        points = np.asarray([value[:3, 3] for value in transforms])
        spans = np.ptp(points, axis=0)
        required_axes = int(profile['minimum_translation_axes'])
        minimum_span = float(profile['minimum_translation_span_m'])
        qualifying = int(np.count_nonzero(spans >= minimum_span))
        if qualifying < required_axes:
            raise ValueError('hand-eye motions do not span enough translation')
        result.update({
            'translation_span_x_m': float(spans[0]),
            'translation_span_y_m': float(spans[1]),
            'translation_span_z_m': float(spans[2]),
        })
    return result


def _transform_matrix(value: Any, label: str) -> np.ndarray:
    if not isinstance(value, dict) or 'matrix' not in value:
        raise ValueError(f'{label} must contain a matrix')
    return validate_transform(np.asarray(value['matrix'], dtype=np.float64), label)


def validate_transform_result(
    workflow: str, document: dict[str, Any]
) -> dict[str, float]:
    if workflow not in {'head_camera', 'right_handeye'}:
        raise ValueError(f'unsupported transform workflow: {workflow}')
    if (
        not isinstance(document, dict)
        or document.get('schema') != 'xlerobot_transform_calibration/v1'
    ):
        raise ValueError('expected xlerobot_transform_calibration/v1')
    expected_model = HEAD_MODEL if workflow == 'head_camera' else ARM_MODEL
    if document.get('model') != expected_model:
        raise ValueError(f'expected calibration model {expected_model}')
    calibration_id = document.get('calibration_id')
    if not isinstance(calibration_id, str) or not calibration_id.strip():
        raise ValueError('calibration_id is required')
    frames = document.get('frames')
    if (
        not isinstance(frames, dict)
        or set(frames) != {'base', 'moving', 'camera', 'target'}
    ):
        raise ValueError('calibration frames must contain base, moving, camera, and target')
    if any(not isinstance(value, str) or not value for value in frames.values()):
        raise ValueError('calibration frame names must be non-empty strings')
    if len(set(frames.values())) != 4:
        raise ValueError('calibration frame names must be distinct')
    expected_semantics = (
        ('mount_from_camera', 'base_from_target')
        if workflow == 'head_camera'
        else ('base_from_camera', 'gripper_from_target')
    )
    if (
        document.get('x_semantics'), document.get('y_semantics')
    ) != expected_semantics:
        raise ValueError(f'{workflow} transform semantics do not match the model')
    _transform_matrix(document.get('x'), 'x')
    _transform_matrix(document.get('y'), 'y')
    digest = document.get('input_sha256')
    if (
        not isinstance(digest, str)
        or len(digest) != 64
        or any(c not in '0123456789abcdef' for c in digest)
    ):
        raise ValueError('input_sha256 must be a lowercase SHA-256 digest')
    profile = quality_profile(workflow)
    metrics_value = document.get('metrics')
    if not isinstance(metrics_value, dict):
        raise ValueError('calibration metrics must be a mapping')
    names = (
        'sample_count', 'reprojection_rmse_px', 'translation_rmse_mm',
        'translation_p95_mm', 'translation_max_mm', 'rotation_rmse_deg',
    )
    metrics = {name: _number(metrics_value.get(name), f'metrics.{name}') for name in names}
    if metrics['sample_count'] < float(profile['minimum_samples']):
        raise ValueError(f'{workflow} has too few samples')
    checks = {
        'reprojection_rmse_px': 'maximum_reprojection_rmse_px',
        'translation_rmse_mm': 'maximum_translation_rmse_mm',
        'translation_p95_mm': 'maximum_translation_p95_mm',
        'rotation_rmse_deg': 'maximum_rotation_rmse_deg',
    }
    for metric, limit_name in checks.items():
        limit = float(profile[limit_name])
        if metrics[metric] >= limit:
            raise ValueError(f'{workflow} {metric} must be below {limit:g}')
    evidence = metrics_value.get('observability')
    if not isinstance(evidence, dict):
        raise ValueError(f'{workflow} metrics.observability is required')
    observability_profile = profile['observability']
    rotation_rank = _number(
        evidence.get('rotation_rank'), 'metrics.observability.rotation_rank'
    )
    maximum_angle = _number(
        evidence.get('max_pairwise_rotation_deg'),
        'metrics.observability.max_pairwise_rotation_deg',
    )
    if rotation_rank < float(observability_profile['minimum_rotation_rank']):
        raise ValueError(f'{workflow} recorded rotation rank is insufficient')
    if maximum_angle < float(
        observability_profile['minimum_max_pairwise_rotation_deg']
    ):
        raise ValueError(f'{workflow} recorded rotation span is insufficient')
    if workflow == 'right_handeye':
        spans = [
            _number(
                evidence.get(f'translation_span_{axis}_m'),
                f'metrics.observability.translation_span_{axis}_m',
            )
            for axis in 'xyz'
        ]
        qualifying = sum(
            value >= float(observability_profile['minimum_translation_span_m'])
            for value in spans
        )
        if qualifying < int(observability_profile['minimum_translation_axes']):
            raise ValueError(f'{workflow} recorded translation span is insufficient')
    return metrics


def validate_grasp_alignment(document: dict[str, Any]) -> dict[str, float]:
    if not isinstance(document, dict) or document.get('schema') != 'xlerobot_grasp_alignment/v1':
        raise ValueError('expected xlerobot_grasp_alignment/v1')
    if document.get('frame') != 'base_link':
        raise ValueError('grasp alignment frame must be base_link')
    compensation = document.get('vision_fk_compensation_m')
    if not isinstance(compensation, list) or len(compensation) != 3:
        raise ValueError('vision_fk_compensation_m must contain three values')
    for index, value in enumerate(compensation):
        _number(value, f'vision_fk_compensation_m[{index}]')
    sag = _number(document.get('gravity_sag_z_m'), 'gravity_sag_z_m')
    if sag < 0:
        raise ValueError('gravity_sag_z_m must be nonnegative')
    head = document.get('head_pose_rad')
    if not isinstance(head, dict) or set(head) != {'pan', 'tilt'}:
        raise ValueError('head_pose_rad must contain pan and tilt')
    _number(head['pan'], 'head_pose_rad.pan')
    _number(head['tilt'], 'head_pose_rad.tilt')
    workspace = document.get('workspace_m')
    if not isinstance(workspace, dict) or set(workspace) != {'min', 'max'}:
        raise ValueError('workspace_m must contain min and max')
    if any(
        not isinstance(workspace[name], list) or len(workspace[name]) != 3
        for name in ('min', 'max')
    ):
        raise ValueError('workspace bounds must contain three values')
    lower = [
        _number(value, f'workspace_m.min[{i}]')
        for i, value in enumerate(workspace['min'])
    ]
    upper = [
        _number(value, f'workspace_m.max[{i}]')
        for i, value in enumerate(workspace['max'])
    ]
    if any(left >= right for left, right in zip(lower, upper)):
        raise ValueError('workspace bounds must be ordered')
    metrics_value = document.get('metrics')
    if not isinstance(metrics_value, dict):
        raise ValueError('grasp alignment metrics must be a mapping')
    sample_count = _number(metrics_value.get('sample_count'), 'metrics.sample_count')
    plane_rmse = _number(metrics_value.get('plane_rmse_mm'), 'metrics.plane_rmse_mm')
    profile = quality_profile('grasp_alignment')
    if sample_count < float(profile['minimum_samples']):
        raise ValueError('grasp alignment has too few samples')
    if plane_rmse >= float(profile['maximum_plane_rmse_mm']):
        raise ValueError('grasp alignment plane RMSE exceeds the quality limit')
    return {'sample_count': sample_count, 'plane_rmse_mm': plane_rmse}


def validate_component(name: str, document: dict[str, Any]) -> dict[str, float]:
    if name == 'servo':
        return validate_servo(document)
    if name == 'base_geometry':
        return validate_base_geometry(document)
    if name in {'head_camera', 'right_handeye'}:
        return validate_transform_result(name, document)
    if name == 'grasp_alignment':
        return validate_grasp_alignment(document)
    raise ValueError(f'unknown calibration component: {name}')
