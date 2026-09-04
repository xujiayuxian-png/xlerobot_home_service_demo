"""Robust depth sampling at a localized image point."""

from __future__ import annotations

import numpy as np

from xlerobot_perception.rgbd.images import image_to_array


def valid_depth_values(depth_msg, u: float, v: float, radius: int) -> np.ndarray:
    """Return positive finite depth samples in metres."""
    depth = image_to_array(depth_msg)
    if depth.ndim != 2:
        raise ValueError('depth image must be single-channel')
    u_px = int(round(u))
    v_px = int(round(v))
    if u_px < 0 or v_px < 0 or u_px >= depth.shape[1] or v_px >= depth.shape[0]:
        return np.array([], dtype=np.float64)
    radius = max(0, int(radius))
    window = depth[
        max(0, v_px - radius):min(depth.shape[0], v_px + radius + 1),
        max(0, u_px - radius):min(depth.shape[1], u_px + radius + 1),
    ]
    if depth_msg.encoding == '16UC1':
        return window[window > 0].astype(np.float64) * 0.001
    if depth_msg.encoding == '32FC1':
        return window[np.isfinite(window) & (window > 0)].astype(np.float64)
    raise ValueError(f'unsupported depth encoding: {depth_msg.encoding}')


def depth_median(depth_msg, u: float, v: float, radii=(8, 16, 28)):
    """Return a percentile-trimmed median, expanding until samples exist."""
    for radius in radii:
        valid = valid_depth_values(depth_msg, u, v, radius)
        if not valid.size:
            continue
        low, high = np.percentile(valid, [10.0, 90.0])
        trimmed = valid[(valid >= low) & (valid <= high)]
        return float(np.median(trimmed if trimmed.size else valid))
    return None
