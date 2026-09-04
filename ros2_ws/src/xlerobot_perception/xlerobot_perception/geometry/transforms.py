"""Small transform helper kept independent of tf2 Python binding details."""

from __future__ import annotations

import math

from geometry_msgs.msg import PointStamped
import numpy as np


def quaternion_to_matrix(quaternion) -> np.ndarray:
    """Convert a finite, nonzero quaternion to a rotation matrix."""
    x, y, z, w = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError('invalid transform quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def transform_point(point, transform):
    """Apply one TransformStamped to a PointStamped."""
    translation = transform.transform.translation
    vector = np.array([point.point.x, point.point.y, point.point.z], dtype=np.float64)
    offset = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
    if not np.all(np.isfinite(vector)) or not np.all(np.isfinite(offset)):
        raise ValueError('transform contains non-finite coordinates')
    output = quaternion_to_matrix(transform.transform.rotation) @ vector + offset
    stamped = PointStamped()
    stamped.header.frame_id = transform.header.frame_id
    stamped.header.stamp = point.header.stamp
    stamped.point.x, stamped.point.y, stamped.point.z = map(float, output)
    return stamped
