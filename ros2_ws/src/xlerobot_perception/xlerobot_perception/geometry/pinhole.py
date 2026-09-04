"""Pinhole projection with explicit validation."""

from __future__ import annotations

import math

from geometry_msgs.msg import PointStamped


def project_pixel_to_point(u, v, depth_m, camera_info, *, frame_id=None, stamp=None):
    """Project a color pixel and aligned depth into the optical frame."""
    if len(camera_info.k) != 9:
        raise ValueError(f'CameraInfo.k must have 9 elements, got {len(camera_info.k)}')
    u = float(u)
    v = float(v)
    depth_m = float(depth_m)
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    if not all(math.isfinite(value) for value in (u, v, depth_m, fx, fy, cx, cy)):
        raise ValueError('pixel, depth, and camera intrinsics must be finite')
    if depth_m <= 0.0:
        raise ValueError('depth must be positive')
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('invalid camera intrinsics')
    point = PointStamped()
    point.header.frame_id = frame_id or camera_info.header.frame_id
    point.header.stamp = stamp or camera_info.header.stamp
    point.point.x = (u - cx) * depth_m / fx
    point.point.y = (v - cy) * depth_m / fy
    point.point.z = depth_m
    return point
