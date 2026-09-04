"""Versioned, atomic storage for transform calibration samples."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .cli import SAMPLE_SCHEMA
from .solver import ARM_MODEL, CalibrationSample, HEAD_MODEL
from .transforms import invert


def _validate_finite_value(value: Any, label: str) -> None:
    """Reject non-finite or non-serializable values in persisted evidence."""
    if value is None or isinstance(value, (bool, str)):
        return
    if isinstance(value, (int, float)):
        try:
            finite = math.isfinite(float(value))
        except OverflowError as error:
            raise ValueError(
                f'{label} must contain only finite numbers'
            ) from error
        if not finite:
            raise ValueError(f'{label} must contain only finite numbers')
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _validate_finite_value(item, f'{label}[{index}]')
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError(f'{label} keys must be strings')
            _validate_finite_value(item, f'{label}.{key}')
        return
    raise ValueError(f'{label} contains an unsupported value')


def _matrix_from_document(value: Any, label: str) -> np.ndarray:
    """Load a strict numeric 4x4 matrix from the YAML representation."""
    if (
        not isinstance(value, list)
        or len(value) != 4
        or any(not isinstance(row, list) or len(row) != 4 for row in value)
    ):
        raise ValueError(f'{label} must be a 4x4 numeric array')
    for row in value:
        for item in row:
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                raise ValueError(f'{label} must contain only finite numbers')
            try:
                finite = math.isfinite(float(item))
            except OverflowError as error:
                raise ValueError(
                    f'{label} must contain only finite numbers'
                ) from error
            if not finite:
                raise ValueError(f'{label} must contain only finite numbers')
    return np.asarray(value, dtype=np.float64)


def transform_to_matrix(transform) -> np.ndarray:
    """Convert a geometry-like Transform object to a 4x4 matrix."""
    translation = transform.translation
    quaternion = transform.rotation
    values = np.asarray([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    if not np.all(np.isfinite(values)) or np.linalg.norm(values) < 1.0e-9:
        raise ValueError('TF quaternion is invalid')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values).as_matrix()
    result[:3, 3] = [translation.x, translation.y, translation.z]
    return result


@dataclass
class TransformSampleSet:
    """Collect distinct transform pairs without owning any motion source."""

    calibration_id: str
    model: str
    base_frame: str
    moving_frame: str
    camera_frame: str
    target_frame: str
    min_translation_m: float = 0.002
    min_rotation_deg: float = 2.0
    samples: list[CalibrationSample] = field(default_factory=list)
    stamps_sec: list[float] = field(default_factory=list)
    qualities: list[dict] = field(default_factory=list)

    def validate(self) -> None:
        if self.model not in {HEAD_MODEL, ARM_MODEL}:
            raise ValueError(f'unsupported sample model: {self.model}')
        if not self.calibration_id.strip():
            raise ValueError('calibration_id is required')
        frames = [
            self.base_frame,
            self.moving_frame,
            self.camera_frame,
            self.target_frame,
        ]
        if any(not frame.strip() for frame in frames) or len(set(frames)) != 4:
            raise ValueError('four distinct non-empty frame names are required')
        if (
            not math.isfinite(self.min_translation_m)
            or not math.isfinite(self.min_rotation_deg)
            or self.min_translation_m < 0
            or self.min_rotation_deg < 0
        ):
            raise ValueError(
                'sample separation limits must be finite and nonnegative'
            )
        if not (
            len(self.samples) == len(self.stamps_sec) == len(self.qualities)
        ):
            raise ValueError('sample metadata lengths do not match')
        for position, (sample, stamp, quality) in enumerate(zip(
            self.samples, self.stamps_sec, self.qualities
        )):
            if sample.index != position:
                raise ValueError('sample indices must be contiguous from zero')
            sample.validate()
            if not math.isfinite(float(stamp)) or float(stamp) < 0:
                raise ValueError(
                    'sample timestamp must be finite and nonnegative'
                )
            if not isinstance(quality, dict):
                raise ValueError('sample quality must be a mapping')
            _validate_finite_value(quality, f'sample {position} quality')

    @classmethod
    def read(
        cls,
        source: Path,
        *,
        expected_model: str | None = None,
        expected_calibration_id: str | None = None,
        expected_frames: dict[str, str] | None = None,
        min_translation_m: float = 0.002,
        min_rotation_deg: float = 2.0,
    ) -> TransformSampleSet:
        """Restore one complete atomic sample document, failing closed."""
        try:
            document = yaml.safe_load(
                source.expanduser().resolve().read_text(encoding='utf-8')
            )
        except yaml.YAMLError as error:
            raise ValueError(f'invalid sample YAML: {error}') from error
        if not isinstance(document, dict):
            raise ValueError('sample document must be a mapping')
        if set(document) != {
            'schema', 'model', 'calibration_id', 'frames', 'samples'
        }:
            raise ValueError('sample document fields do not match the schema')
        if document.get('schema') != SAMPLE_SCHEMA:
            raise ValueError(f'expected schema {SAMPLE_SCHEMA}')
        model = document.get('model')
        if not isinstance(model, str):
            raise ValueError('sample model must be a string')
        if expected_model is not None and model != expected_model:
            raise ValueError(f'expected model {expected_model}')
        calibration_id = document.get('calibration_id')
        if not isinstance(calibration_id, str):
            raise ValueError('calibration_id must be a string')
        if (
            expected_calibration_id is not None
            and calibration_id != expected_calibration_id
        ):
            raise ValueError(
                f'expected calibration_id {expected_calibration_id}'
            )
        frames = document.get('frames')
        if not isinstance(frames, dict) or set(frames) != {
            'base', 'moving', 'camera', 'target'
        }:
            raise ValueError(
                'frames must contain exactly base, moving, camera, and target'
            )
        if any(not isinstance(value, str) for value in frames.values()):
            raise ValueError('frame names must be strings')
        if expected_frames is not None and frames != expected_frames:
            raise ValueError('sample frames do not match the active collector')
        rows = document.get('samples')
        if not isinstance(rows, list):
            raise ValueError('samples must be a list')
        restored = cls(
            calibration_id=calibration_id,
            model=model,
            base_frame=frames['base'],
            moving_frame=frames['moving'],
            camera_frame=frames['camera'],
            target_frame=frames['target'],
            min_translation_m=min_translation_m,
            min_rotation_deg=min_rotation_deg,
        )
        for position, row in enumerate(rows):
            if not isinstance(row, dict):
                raise ValueError(f'sample {position} must be a mapping')
            if set(row) != {
                'index',
                'stamp_sec',
                'moving_in_base',
                'target_in_camera',
                'quality',
            }:
                raise ValueError(
                    f'sample {position} fields do not match the schema'
                )
            index = row.get('index')
            if (
                isinstance(index, bool)
                or not isinstance(index, int)
                or index != position
            ):
                raise ValueError('sample indices must be contiguous from zero')
            if (
                isinstance(row['stamp_sec'], bool)
                or not isinstance(row['stamp_sec'], (int, float))
            ):
                raise ValueError(
                    f'sample {position} stamp_sec must be numeric'
                )
            quality = row.get('quality')
            if not isinstance(quality, dict):
                raise ValueError(f'sample {position} quality must be a mapping')
            _validate_finite_value(quality, f'sample {position} quality')
            try:
                moving = _matrix_from_document(
                    row['moving_in_base'],
                    f'sample {position} moving_in_base',
                )
                target = _matrix_from_document(
                    row['target_in_camera'],
                    f'sample {position} target_in_camera',
                )
                stamp = float(row['stamp_sec'])
            except (OverflowError, TypeError, ValueError) as error:
                raise ValueError(
                    f'sample {position} contains invalid transform data'
                ) from error
            restored.append(moving, target, stamp, quality)
        restored.validate()
        return restored

    def append(
        self,
        moving_in_base: np.ndarray,
        target_in_camera: np.ndarray,
        stamp_sec: float,
        quality: dict | None = None,
    ) -> CalibrationSample:
        """Validate and append a kinematically distinct sample."""
        self.validate()
        sample = CalibrationSample(
            index=len(self.samples),
            moving_in_base=np.asarray(moving_in_base, dtype=np.float64),
            target_in_camera=np.asarray(target_in_camera, dtype=np.float64),
        )
        sample.validate()
        if not np.isfinite(stamp_sec) or stamp_sec < 0:
            raise ValueError('sample timestamp must be finite and nonnegative')
        quality_evidence = dict(quality or {})
        _validate_finite_value(
            quality_evidence, f'sample {sample.index} quality'
        )
        for previous_sample in self.samples:
            delta = invert(
                previous_sample.moving_in_base
            ) @ sample.moving_in_base
            translation = float(np.linalg.norm(delta[:3, 3]))
            rotation_deg = float(
                np.degrees(
                    np.linalg.norm(
                        Rotation.from_matrix(delta[:3, :3]).as_rotvec()
                    )
                )
            )
            if (
                translation < self.min_translation_m
                and rotation_deg < self.min_rotation_deg
            ):
                raise ValueError(
                    'sample is too close to an existing moving-link pose '
                    f'({previous_sample.index})'
                )
        self.samples.append(sample)
        self.stamps_sec.append(float(stamp_sec))
        self.qualities.append(quality_evidence)
        return sample

    def append_and_write_atomic(
        self,
        output: Path,
        moving_in_base: np.ndarray,
        target_in_camera: np.ndarray,
        stamp_sec: float,
        quality: dict | None = None,
    ) -> CalibrationSample:
        """Append one sample only if its atomic persistence succeeds."""
        previous_count = len(self.samples)
        sample = self.append(
            moving_in_base,
            target_in_camera,
            stamp_sec,
            quality,
        )
        try:
            self.write_atomic(output)
        except Exception:
            del self.samples[previous_count:]
            del self.stamps_sec[previous_count:]
            del self.qualities[previous_count:]
            raise
        return sample

    def coverage(self) -> dict:
        """Return factual moving-link pose coverage without pass thresholds."""
        self.validate()
        points = np.asarray(
            [sample.moving_in_base[:3, 3] for sample in self.samples],
            dtype=np.float64,
        )
        spans = np.ptp(points, axis=0) if len(points) else np.zeros(3)
        maximum_angle_deg = 0.0
        for first_index, first in enumerate(self.samples):
            for second in self.samples[first_index + 1:]:
                relative = (
                    first.moving_in_base[:3, :3].T
                    @ second.moving_in_base[:3, :3]
                )
                angle_deg = float(np.degrees(
                    Rotation.from_matrix(relative).magnitude()
                ))
                maximum_angle_deg = max(maximum_angle_deg, angle_deg)
        return {
            'sample_count': len(self.samples),
            'points': [
                {
                    'index': sample.index,
                    'x_m': float(sample.moving_in_base[0, 3]),
                    'y_m': float(sample.moving_in_base[1, 3]),
                    'z_m': float(sample.moving_in_base[2, 3]),
                }
                for sample in self.samples
            ],
            'spans_m': {
                'x': float(spans[0]),
                'y': float(spans[1]),
                'z': float(spans[2]),
            },
            'max_pairwise_pose_angle_deg': maximum_angle_deg,
        }

    def to_dict(self) -> dict:
        """Return the versioned solver input document."""
        self.validate()
        return {
            'schema': SAMPLE_SCHEMA,
            'model': self.model,
            'calibration_id': self.calibration_id,
            'frames': {
                'base': self.base_frame,
                'moving': self.moving_frame,
                'camera': self.camera_frame,
                'target': self.target_frame,
            },
            'samples': [
                {
                    'index': sample.index,
                    'stamp_sec': stamp,
                    'moving_in_base': sample.moving_in_base.tolist(),
                    'target_in_camera': sample.target_in_camera.tolist(),
                    'quality': quality,
                }
                for sample, stamp, quality in zip(
                    self.samples, self.stamps_sec, self.qualities
                )
            ],
        }

    def write_atomic(self, output: Path) -> None:
        """Replace one YAML file only after the new document is complete."""
        output = output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_name(f'.{output.name}.tmp')
        try:
            temporary.write_text(
                yaml.safe_dump(self.to_dict(), sort_keys=False),
                encoding='utf-8',
            )
            temporary.replace(output)
        except Exception:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
            raise
