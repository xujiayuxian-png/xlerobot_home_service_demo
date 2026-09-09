"""Offline straight-wall yaw reference, not an automatic mount calibration."""
import math
from pathlib import Path
import numpy as np


def estimate_yaw(points: Path, current_yaw_deg: float, roll_deg: float, wall_normal_deg: float):
    values = np.asarray([current_yaw_deg, roll_deg, wall_normal_deg])
    if not np.isfinite(values).all() or abs(math.sin(math.radians(roll_deg))) > 1e-6:
        raise ValueError('finite angles and a level upright/inverted scanner are required')
    data = np.loadtxt(points, delimiter=',', skiprows=1, ndmin=2)
    if data.shape[1] != 2 or len(data) < 20 or not np.isfinite(data).all():
        raise ValueError('CSV must contain x,y and at least 20 finite wall points in the scan frame')
    data[:, 1] *= math.cos(math.radians(roll_deg))
    center = data.mean(axis=0)
    _, _, vt = np.linalg.svd(data - center, full_matrices=False)
    normal = vt[-1]
    if normal @ center < 0:
        normal = -normal
    rms = float(np.sqrt(np.mean(((data - center) @ normal) ** 2)))
    span = float(np.ptp((data - center) @ vt[0]))
    if span < .5 or rms > .01 or abs(normal @ center) < .2:
        raise ValueError('use one wall spanning >=0.5 m, >=0.2 m away, with line RMS <=10 mm')
    observed = math.degrees(math.atan2(normal[1], normal[0])) + current_yaw_deg
    correction = (wall_normal_deg - observed + 180) % 360 - 180
    if abs(correction) > 15:
        raise ValueError('correction exceeds 15 degrees; check scan frame, inversion and selected wall')
    return {'candidate_lidar_yaw_deg': current_yaw_deg + correction,
            'correction_deg': correction, 'wall_fit_rms_mm': rms * 1000,
            'wall_span_m': span, 'sample_count': len(data),
            'applied': False, 'hardware_verified': False,
            'assumption': 'wall normal in base frame independently measured; stationary level scanner'}
