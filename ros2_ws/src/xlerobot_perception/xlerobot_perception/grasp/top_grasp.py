"""Minimal top-grasp geometry migrated from the frozen prototype.

Both traditional routes consume the same prompted-SAM2 mask and RGB-D point
cloud.  Centroid and GPD differ only in how the grasp XY/yaw candidate is
selected.  GPD selection is strict and contains no centroid fallback.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from xlerobot_perception.grasp.gpd_client import GpdCandidate
from xlerobot_perception.rgbd.images import image_to_array


@dataclass(frozen=True)
class TopGraspGeometry:
    backend: str
    method: str
    score: float
    pregrasp_m: np.ndarray
    grasp_m: np.ndarray
    lift_m: np.ndarray
    table_height_m: float
    object_height_m: float
    grasp_width_m: float
    wrist_yaw_rad: float
    selection_reason: str


@dataclass(frozen=True)
class TablePlane:
    """Fitted table plane in the planning frame: ``normal dot p + d = 0``."""

    normal: np.ndarray
    d: float
    inlier_count: int


@dataclass(frozen=True)
class TopGraspConfig:
    pregrasp_above_top_m: float = 0.08
    grasp_below_top_m: float = 0.012
    lift_above_top_m: float = 0.10
    min_table_clearance_m: float = 0.005
    min_points: int = 80
    workspace_x_m: tuple[float, float] = (-0.06, 0.18)
    workspace_y_m: tuple[float, float] = (-0.10, 0.15)
    workspace_z_m: tuple[float, float] = (0.0, 1.6)
    min_grasp_width_m: float = 0.015
    max_grasp_width_m: float = 0.085
    width_margin_m: float = 0.008
    gpd_max_centroid_distance_m: float = 0.10


def depth_to_u16_mm(depth_message) -> np.ndarray:
    """Normalize supported ROS depth encodings to little-endian millimetres."""
    depth = image_to_array(depth_message)
    if depth_message.encoding == "16UC1":
        return np.ascontiguousarray(depth, dtype="<u2")
    if depth_message.encoding != "32FC1":
        raise ValueError(f"unsupported depth encoding: {depth_message.encoding}")
    valid = np.isfinite(depth) & (depth > 0.0) & (depth < 65.535)
    out = np.zeros(depth.shape, dtype=np.uint16)
    out[valid] = np.rint(depth[valid] * 1000.0).astype(np.uint16)
    return out


def camera_intrinsics(camera_info) -> dict[str, float]:
    fx, fy = float(camera_info.k[0]), float(camera_info.k[4])
    cx, cy = float(camera_info.k[2]), float(camera_info.k[5])
    if min(fx, fy) <= 0.0 or not all(math.isfinite(v) for v in (fx, fy, cx, cy)):
        raise ValueError("camera intrinsics are invalid")
    return {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "depth_scale": 1000.0}


def masked_point_cloud(
    depth_u16: np.ndarray,
    mask: np.ndarray,
    intrinsics: dict[str, float],
    *,
    min_depth_m: float,
    max_depth_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return camera-optical XYZ plus corresponding (u,v) pixels."""
    if depth_u16.shape != mask.shape:
        raise ValueError("depth and SAM2 mask dimensions differ")
    v, u = np.nonzero(mask & (depth_u16 > 0))
    z = depth_u16[v, u].astype(np.float64) / float(intrinsics["depth_scale"])
    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    u, v, z = u[valid], v[valid], z[valid]
    x = (u.astype(np.float64) - intrinsics["cx"]) * z / intrinsics["fx"]
    y = (v.astype(np.float64) - intrinsics["cy"]) * z / intrinsics["fy"]
    return np.column_stack((x, y, z)), np.column_stack((u, v))


def transform_matrix(transform_stamped) -> np.ndarray:
    q = transform_stamped.transform.rotation
    t = transform_stamped.transform.translation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("TF rotation quaternion is invalid")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    matrix[:3, 3] = [float(t.x), float(t.y), float(t.z)]
    if not np.all(np.isfinite(matrix)):
        raise ValueError("TF contains non-finite values")
    return matrix


def transform_cloud(points: np.ndarray, target_from_camera: np.ndarray) -> np.ndarray:
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("point cloud must have shape Nx3")
    return points @ target_from_camera[:3, :3].T + target_from_camera[:3, 3]


def _wrap_pi(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _short_axis(points_xy: np.ndarray) -> tuple[float, float]:
    centered = points_xy - np.median(points_xy, axis=0)
    covariance = centered.T @ centered / max(1, len(centered) - 1)
    values, vectors = np.linalg.eigh(covariance)
    axis = vectors[:, int(np.argmin(values))]
    yaw = math.atan2(float(axis[1]), float(axis[0]))
    projections = centered @ axis
    width = float(np.percentile(projections, 95) - np.percentile(projections, 5))
    return _wrap_pi(yaw), width


def _table_height_at(table_plane: TablePlane, xy: np.ndarray) -> float:
    normal = np.asarray(table_plane.normal, dtype=np.float64)
    if normal.shape != (3,) or not np.all(np.isfinite(normal)):
        raise ValueError("fitted table plane normal is invalid")
    if not math.isfinite(table_plane.d) or normal[2] < 0.70:
        raise ValueError("fitted table plane is not horizontal in the planning frame")
    return float(-(normal[0] * xy[0] + normal[1] * xy[1] + table_plane.d) / normal[2])


def _object_stats(
    points_target: np.ndarray, config: TopGraspConfig, table_plane: TablePlane
):
    if len(points_target) < config.min_points:
        raise ValueError(
            f"object cloud has {len(points_target)} points; need {config.min_points}"
        )
    signed_height = points_target @ table_plane.normal + table_plane.d
    object_height_normal = float(np.percentile(signed_height, 90.0))
    if object_height_normal <= config.min_table_clearance_m:
        raise ValueError("refined object cloud is not observably above the fitted table")
    top = points_target[signed_height >= 0.55 * object_height_normal]
    if len(top) < max(20, config.min_points // 4):
        top = points_target
    return top


def _check_xy(xy: np.ndarray, config: TopGraspConfig) -> None:
    if not (
        config.workspace_x_m[0] <= float(xy[0]) <= config.workspace_x_m[1]
        and config.workspace_y_m[0] <= float(xy[1]) <= config.workspace_y_m[1]
    ):
        raise ValueError(
            f"grasp XY ({xy[0]:.3f}, {xy[1]:.3f}) is outside configured workspace"
        )


def _check_grasp_xyz(xyz: np.ndarray, config: TopGraspConfig) -> None:
    _check_xy(xyz[:2], config)
    if not config.workspace_z_m[0] <= float(xyz[2]) <= config.workspace_z_m[1]:
        raise ValueError(
            f"grasp Z {xyz[2]:.3f} is outside configured workspace"
        )


def _geometry(
    *,
    backend: str,
    method: str,
    score: float,
    xy: np.ndarray,
    yaw: float,
    width: float,
    points_target: np.ndarray,
    config: TopGraspConfig,
    table_plane: TablePlane,
    reason: str,
) -> TopGraspGeometry:
    top_z = float(np.percentile(points_target[:, 2], 90.0))
    table_z = _table_height_at(table_plane, xy)
    object_height = top_z - table_z
    if object_height <= config.min_table_clearance_m:
        raise ValueError("object top is not observably above the fitted table")
    grasp_z = max(
        table_z + config.min_table_clearance_m,
        top_z - min(config.grasp_below_top_m, 0.55 * object_height),
    )
    pregrasp = np.array([xy[0], xy[1], top_z + config.pregrasp_above_top_m])
    grasp = np.array([xy[0], xy[1], grasp_z])
    lift = np.array([xy[0], xy[1], top_z + config.lift_above_top_m])
    _check_grasp_xyz(grasp, config)
    width = float(np.clip(
        width + config.width_margin_m,
        config.min_grasp_width_m,
        config.max_grasp_width_m,
    ))
    return TopGraspGeometry(
        backend=backend,
        method=method,
        score=float(score),
        pregrasp_m=pregrasp,
        grasp_m=grasp,
        lift_m=lift,
        table_height_m=table_z,
        object_height_m=object_height,
        grasp_width_m=width,
        wrist_yaw_rad=_wrap_pi(yaw),
        selection_reason=reason,
    )


def centroid_top_grasp(
    points_target: np.ndarray,
    *,
    confidence: float,
    config: TopGraspConfig,
    table_plane: TablePlane,
) -> TopGraspGeometry:
    top = _object_stats(points_target, config, table_plane)
    xy = np.median(top[:, :2], axis=0)
    yaw, width = _short_axis(top[:, :2])
    return _geometry(
        backend="centroid",
        method="sam2_rgbd_top_centroid",
        score=confidence,
        xy=xy,
        yaw=yaw,
        width=width,
        points_target=points_target,
        config=config,
        table_plane=table_plane,
        reason="SAM2 mask upper-surface median and PCA short axis",
    )


def gpd_top_grasp(
    candidates: list[GpdCandidate],
    *,
    points_target: np.ndarray,
    pixels_uv: np.ndarray,
    mask: np.ndarray,
    intrinsics: dict[str, float],
    target_from_camera: np.ndarray,
    config: TopGraspConfig,
    table_plane: TablePlane,
) -> TopGraspGeometry:
    """Select a valid GPD candidate or raise; never substitutes centroid."""
    del pixels_uv  # Cloud correspondence is intentionally shared, not re-created.
    cloud_center = np.median(points_target, axis=0)
    valid: list[tuple[float, GpdCandidate, np.ndarray, float]] = []
    for candidate in candidates:
        camera = candidate.translation_camera_m
        if camera[2] <= 0.0:
            continue
        u = int(round(intrinsics["fx"] * camera[0] / camera[2] + intrinsics["cx"]))
        v = int(round(intrinsics["fy"] * camera[1] / camera[2] + intrinsics["cy"]))
        if not (0 <= v < mask.shape[0] and 0 <= u < mask.shape[1]):
            continue
        y0, y1 = max(0, v - 8), min(mask.shape[0], v + 9)
        x0, x1 = max(0, u - 8), min(mask.shape[1], u + 9)
        if not np.any(mask[y0:y1, x0:x1]):
            continue
        target = transform_cloud(camera.reshape(1, 3), target_from_camera)[0]
        if np.linalg.norm(target - cloud_center) > config.gpd_max_centroid_distance_m:
            continue
        opening_target = (
            target_from_camera[:3, :3] @ candidate.rotation_camera[:, 1]
        )
        yaw = math.atan2(float(opening_target[1]), float(opening_target[0]))
        valid.append((candidate.score, candidate, target, yaw))
    if not valid:
        raise ValueError("GPD returned no candidate consistent with the SAM2 object cloud")
    score, candidate, target, yaw = max(valid, key=lambda item: item[0])
    return _geometry(
        backend="gpd",
        method="sam2_rgbd_gpd_top",
        score=score,
        xy=target[:2],
        yaw=yaw,
        width=candidate.width_m,
        points_target=points_target,
        config=config,
        table_plane=table_plane,
        reason="highest-score GPD candidate inside the prompted SAM2 cloud",
    )
