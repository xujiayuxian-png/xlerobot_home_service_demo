"""Small, testable rigid-transform helpers."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def validate_transform(transform: np.ndarray, label: str = 'transform') -> np.ndarray:
    """Return a validated 4x4 rigid transform."""
    value = np.asarray(transform, dtype=np.float64)
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise ValueError(f'{label} must be a finite 4x4 matrix')
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-8):
        raise ValueError(f'{label} has an invalid homogeneous row')
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-6):
        raise ValueError(f'{label} rotation is not orthonormal')
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-6):
        raise ValueError(f'{label} rotation determinant is not +1')
    return value


def invert(transform: np.ndarray) -> np.ndarray:
    """Invert a rigid transform."""
    value = validate_transform(transform)
    result = np.eye(4)
    result[:3, :3] = value[:3, :3].T
    result[:3, 3] = -value[:3, :3].T @ value[:3, 3]
    return result


def from_parameters(parameters: np.ndarray) -> np.ndarray:
    """Create a transform from rotation-vector xyz and translation xyz."""
    values = np.asarray(parameters, dtype=np.float64)
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ValueError('transform parameters must contain six finite values')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(values[:3]).as_matrix()
    result[:3, 3] = values[3:]
    return result


def to_parameters(transform: np.ndarray) -> np.ndarray:
    """Convert a transform to rotation-vector xyz and translation xyz."""
    value = validate_transform(transform)
    return np.concatenate(
        [Rotation.from_matrix(value[:3, :3]).as_rotvec(), value[:3, 3]]
    )


def to_dict(transform: np.ndarray) -> dict:
    """Serialize a transform with matrix, xyz, quaternion, and RPY."""
    value = validate_transform(transform)
    rotation = Rotation.from_matrix(value[:3, :3])
    return {
        'matrix': value.tolist(),
        'xyz_m': value[:3, 3].tolist(),
        'rpy_rad': rotation.as_euler('xyz').tolist(),
        'quaternion_xyzw': rotation.as_quat().tolist(),
    }
