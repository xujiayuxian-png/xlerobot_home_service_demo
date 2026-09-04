"""Versioned unit-calibration drafts and quality gates."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse

from xlerobot_assets import ArtifactCatalog, ArtifactRef
import yaml

from .site_assets import _atomic_json, _id, _utc_now


WORKFLOWS = ('servo', 'base_geometry', 'head_camera', 'right_handeye')


def _local_path(uri: str) -> Path:
    parsed = urlparse(str(uri))
    if parsed.scheme not in ('', 'file'):
        raise ValueError('calibration source must be a local file URI')
    path = Path(unquote(parsed.path) if parsed.scheme else uri).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f'calibration result does not exist: {path}')
    return path


def _finite_positive(document: dict[str, Any], names: tuple[str, ...]) -> None:
    for name in names:
        value = float(document[name])
        if not (value > 0.0 and value < float('inf')):
            raise ValueError(f'{name} must be finite and positive')


def validate_component(workflow: str, document: dict[str, Any]) -> tuple[bool, dict[str, float]]:
    """Validate one component and calculate its non-negotiable quality result."""
    if workflow not in WORKFLOWS:
        raise ValueError(f'unsupported calibration workflow: {workflow}')
    if not isinstance(document, dict):
        raise ValueError('calibration result must be a YAML mapping')
    metrics: dict[str, float] = {}
    if workflow == 'servo':
        if document.get('schema') != 'xlerobot_servo_calibration/v1':
            raise ValueError('expected xlerobot_servo_calibration/v1')
        groups = ('right_arm', 'left_arm', 'head')
        if any(not isinstance(document.get(group, {}).get('joints'), dict) for group in groups):
            raise ValueError('servo calibration must contain right_arm, left_arm and head joints')
        joints = {
            f'{group}.{name}': row
            for group in groups
            for name, row in document[group]['joints'].items()
        }
        if len(joints) != 14:
            raise ValueError('servo calibration must contain all 14 robot joints')
        for name, row in joints.items():
            if not isinstance(row, dict):
                raise ValueError(f'invalid servo row: {name}')
            for field in ('servo_id', 'direction', 'offset', 'raw_min', 'raw_max',
                          'limit_min', 'limit_max'):
                if field not in row:
                    raise ValueError(f'{name} is missing {field}')
            if int(row['direction']) not in (-1, 1):
                raise ValueError(f'{name} direction must be -1 or 1')
            if float(row['raw_min']) >= float(row['raw_max']):
                raise ValueError(f'{name} raw_min must be below raw_max')
            if float(row['limit_min']) >= float(row['limit_max']):
                raise ValueError(f'{name} limit_min must be below limit_max')
        metrics['joint_count'] = float(len(joints))
        return True, metrics
    if workflow == 'base_geometry':
        if document.get('schema') != 'xlerobot_base_geometry/v1':
            raise ValueError('expected xlerobot_base_geometry/v1')
        _finite_positive(document, ('wheel_radius_m', 'wheel_separation_m'))
        metrics = {
            'straight_error_percent': float(document['straight_error_percent']),
            'rotation_error_percent': float(document['rotation_error_percent']),
        }
        return max(abs(value) for value in metrics.values()) <= 5.0, metrics

    if document.get('schema') != 'xlerobot_transform_calibration/v1':
        raise ValueError('expected xlerobot_transform_calibration/v1')
    raw_metrics = document.get('metrics', {})
    metrics = {
        'sample_count': float(document['sample_count']),
        'reprojection_rmse_px': float(raw_metrics['reprojection_rmse_px']),
        'translation_rmse_mm': float(raw_metrics['translation_rmse_mm']),
        'translation_max_mm': float(raw_metrics['translation_max_mm']),
        'rotation_rmse_deg': float(raw_metrics['rotation_rmse_deg']),
    }
    if not isinstance(document.get('x'), dict) or not isinstance(document.get('y'), dict):
        raise ValueError('transform calibration must contain x and y transforms')
    if workflow == 'head_camera':
        expected_model = 'moving_camera_fixed_target'
        quality = (
            metrics['sample_count'] >= 12
            and metrics['reprojection_rmse_px'] < 1.0
            and metrics['translation_rmse_mm'] < 10.0
            and metrics['rotation_rmse_deg'] < 1.5
        )
    else:
        expected_model = 'fixed_camera_moving_target'
        quality = (
            metrics['sample_count'] >= 20
            and metrics['reprojection_rmse_px'] < 1.2
            and metrics['translation_rmse_mm'] < 10.0
            and metrics['translation_max_mm'] < 15.0
        )
    if document.get('model') != expected_model:
        raise ValueError(f'expected calibration model {expected_model}')
    return quality, metrics


class UnitCalibrationStore:
    """Keep component drafts mutable, publish complete unit bundles immutably."""

    def __init__(self, root: Path | str, software_revision: str = 'development'):
        self.catalog = ArtifactCatalog(root)
        self.root = self.catalog.root / 'units' / '.drafts'
        self.root.mkdir(parents=True, exist_ok=True)
        self.software_revision = software_revision

    def draft(self, unit_id: str) -> Path:
        return self.root / _id(unit_id, 'unit_id')

    def import_result(
        self, unit_id: str, workflow: str, source_uri: str
    ) -> tuple[Path, bool, dict[str, float]]:
        unit_id = _id(unit_id, 'unit_id')
        if workflow not in WORKFLOWS:
            raise ValueError(f'unsupported calibration workflow: {workflow}')
        source = _local_path(source_uri)
        document = yaml.safe_load(source.read_text(encoding='utf-8'))
        return self.save_component(unit_id, workflow, document)

    def save_component(
        self, unit_id: str, workflow: str, document: dict[str, Any]
    ) -> tuple[Path, bool, dict[str, float]]:
        """Validate and atomically replace one component in the unit draft."""
        quality, metrics = validate_component(workflow, document)
        draft = self.draft(unit_id)
        components = draft / 'components'
        components.mkdir(parents=True, exist_ok=True)
        target = components / f'{workflow}.yaml'
        temporary = target.with_suffix('.yaml.tmp')
        temporary.write_text(
            yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
            encoding='utf-8',
        )
        os.replace(temporary, target)
        status_path = draft / 'status.json'
        status = json.loads(status_path.read_text()) if status_path.exists() else {
            'schema': 'xlerobot_unit_calibration_draft/v1', 'unit_id': unit_id,
            'components': {}, 'created_at': _utc_now(),
        }
        status['components'][workflow] = {
            'quality_passed': quality, 'metrics': metrics, 'updated_at': _utc_now(),
        }
        status['updated_at'] = _utc_now()
        _atomic_json(status_path, status)
        return draft, quality, metrics

    def save_base_geometry_measurements(
        self,
        unit_id: str,
        *,
        nominal_radius: float,
        nominal_separation: float,
        straight_commanded: list[float],
        straight_actual: list[float],
        rotation_commanded: list[float],
        rotation_actual: list[float],
    ) -> tuple[Path, bool, dict[str, float]]:
        """Fit wheel radius/separation from two or more measured trials."""
        series = (
            straight_commanded, straight_actual,
            rotation_commanded, rotation_actual,
        )
        if not all(len(values) >= 2 for values in series):
            raise ValueError('two straight and two rotation measurements are required')
        if len(straight_commanded) != len(straight_actual):
            raise ValueError('straight measurement lengths do not match')
        if len(rotation_commanded) != len(rotation_actual):
            raise ValueError('rotation measurement lengths do not match')
        values = [
            nominal_radius,
            nominal_separation,
            *(value for row in series for value in row),
        ]
        if not all(
            math.isfinite(float(value)) and float(value) > 0.0
            for value in values
        ):
            raise ValueError('base geometry measurements must be finite and positive')
        straight_scale = sum(
            actual / commanded
            for commanded, actual in zip(straight_commanded, straight_actual)
        ) / len(straight_commanded)
        separation_scale = sum(
            commanded / actual
            for commanded, actual in zip(rotation_commanded, rotation_actual)
        ) / len(rotation_commanded)
        straight_errors = [
            100.0 * (actual / straight_scale - commanded) / commanded
            for commanded, actual in zip(straight_commanded, straight_actual)
        ]
        rotation_errors = [
            100.0 * (actual * separation_scale - commanded) / commanded
            for commanded, actual in zip(rotation_commanded, rotation_actual)
        ]
        document = {
            'schema': 'xlerobot_base_geometry/v1',
            'wheel_radius_m': float(nominal_radius) * straight_scale,
            'wheel_separation_m': float(nominal_separation) * separation_scale,
            'straight_error_percent': max(abs(value) for value in straight_errors),
            'rotation_error_percent': max(abs(value) for value in rotation_errors),
            'fit': {
                'nominal_wheel_radius_m': float(nominal_radius),
                'nominal_wheel_separation_m': float(nominal_separation),
                'straight_trials': [
                    {'commanded_m': float(commanded), 'actual_m': float(actual)}
                    for commanded, actual in zip(straight_commanded, straight_actual)
                ],
                'rotation_trials': [
                    {'commanded_rad': float(commanded), 'actual_rad': float(actual)}
                    for commanded, actual in zip(rotation_commanded, rotation_actual)
                ],
            },
        }
        return self.save_component(unit_id, 'base_geometry', document)

    def ready(self, unit_id: str) -> bool:
        path = self.draft(unit_id) / 'status.json'
        if not path.exists():
            return False
        status = json.loads(path.read_text(encoding='utf-8'))
        return all(
            status.get('components', {}).get(name, {}).get('quality_passed', False)
            for name in WORKFLOWS
        )

    def finalize_and_activate(self, unit_id: str, version: str = '') -> ArtifactRef:
        unit_id = _id(unit_id, 'unit_id')
        draft = self.draft(unit_id)
        if not self.ready(unit_id):
            raise ValueError('unit calibration draft is incomplete or failed quality')
        files = {
            f'components/{name}.yaml': draft / 'components' / f'{name}.yaml'
            for name in WORKFLOWS
        }
        files['validation.json'] = draft / 'status.json'
        reference = self.catalog.create(
            kind='units', artifact_id=unit_id,
            schema='xlerobot_unit_calibration/v1', version=version,
            files=files,
            metadata={'software_revision': self.software_revision},
        )
        return self.catalog.activate('units', unit_id, reference.version)
