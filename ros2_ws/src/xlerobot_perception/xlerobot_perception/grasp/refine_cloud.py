"""Strict table-plane removal and main-body refinement for classical grasping.

This is the minimal vertical slice migrated from the frozen prototype's
``depth/table_cluster_mask.py`` and ``depth/body_seg_mask.py``.  Unlike the
prototype exploration helpers, every unobservable refinement state raises;
the caller never receives the unrefined SAM mask as an implicit fallback.

Frozen migration source: ``2477d789e678468c6cb684f6319b69f749a62bef``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial import cKDTree

from xlerobot_perception.grasp.top_grasp import (
    TablePlane,
    masked_point_cloud,
    transform_cloud,
)


@dataclass(frozen=True)
class RefinementConfig:
    table_roi_y_start_fraction: float = 0.35
    table_plane_distance_m: float = 0.003
    table_ransac_iterations: int = 400
    table_min_inliers: int = 200
    body_cluster_epsilon_m: float = 0.015
    body_cluster_min_points: int = 40
    sampling_stride: int = 2


@dataclass(frozen=True)
class RefinedObjectCloud:
    mask: np.ndarray
    points_camera: np.ndarray
    points_target: np.ndarray
    pixels_uv: np.ndarray
    table_plane: TablePlane
    table_pixels_removed: int
    cluster_points: int


def _sampled_points(
    depth_u16: np.ndarray,
    mask: np.ndarray,
    intrinsics: dict[str, float],
    *,
    stride: int,
    min_depth_m: float,
    max_depth_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    sample_mask = np.zeros(mask.shape, dtype=bool)
    sample_mask[::stride, ::stride] = mask[::stride, ::stride]
    return masked_point_cloud(
        depth_u16,
        sample_mask,
        intrinsics,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
    )


def _fit_table_plane(
    points: np.ndarray, config: RefinementConfig
) -> tuple[np.ndarray, float, int]:
    if len(points) < config.table_min_inliers:
        raise ValueError(
            f'table plane has only {len(points)} ROI points; '
            f'need {config.table_min_inliers}'
        )
    rng = np.random.default_rng(0)
    best_indices = None
    for _ in range(config.table_ransac_iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        length = float(np.linalg.norm(normal))
        if length < 1.0e-9:
            continue
        normal /= length
        if abs(float(normal[2])) < 0.55:
            continue
        d = -float(np.dot(normal, sample[0]))
        indices = np.flatnonzero(
            np.abs(points @ normal + d) <= config.table_plane_distance_m
        )
        if best_indices is None or len(indices) > len(best_indices):
            best_indices = indices
    if best_indices is None or len(best_indices) < config.table_min_inliers:
        count = 0 if best_indices is None else len(best_indices)
        raise ValueError(
            f'table plane has only {count} RANSAC inliers; '
            f'need {config.table_min_inliers}'
        )
    inliers = points[best_indices]
    center = np.mean(inliers, axis=0)
    _, _, right = np.linalg.svd(inliers - center, full_matrices=False)
    normal = right[-1]
    if normal[2] < 0.0:
        normal = -normal
    normal /= np.linalg.norm(normal)
    d = -float(np.dot(normal, center))
    final_count = int(np.count_nonzero(
        np.abs(points @ normal + d) <= config.table_plane_distance_m
    ))
    if final_count < config.table_min_inliers:
        raise ValueError('refitted table plane lost the required inlier support')
    return normal, d, final_count


def _plane_mask(
    depth_u16: np.ndarray,
    intrinsics: dict[str, float],
    normal: np.ndarray,
    d: float,
    distance_m: float,
    *,
    min_depth_m: float,
    max_depth_m: float,
) -> np.ndarray:
    height, width = depth_u16.shape
    z = depth_u16.astype(np.float64) / float(intrinsics['depth_scale'])
    valid = (z >= min_depth_m) & (z <= max_depth_m)
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    x = (u - intrinsics['cx']) * z / intrinsics['fx']
    y = (v - intrinsics['cy']) * z / intrinsics['fy']
    distance = np.abs(normal[0] * x + normal[1] * y + normal[2] * z + d)
    return valid & (distance <= distance_m)


def _cluster_labels(points: np.ndarray, epsilon_m: float, min_points: int) -> np.ndarray:
    labels = np.full(len(points), -1, dtype=np.int32)
    if len(points) < min_points:
        return labels
    neighbors = cKDTree(points).query_ball_point(points, epsilon_m, workers=1)
    visited = np.zeros(len(points), dtype=bool)
    cluster_id = 0
    for start in range(len(points)):
        if visited[start]:
            continue
        stack = [start]
        visited[start] = True
        members = []
        while stack:
            current = stack.pop()
            members.append(current)
            for neighbor in neighbors[current]:
                if not visited[neighbor]:
                    visited[neighbor] = True
                    stack.append(neighbor)
        if len(members) >= min_points:
            labels[members] = cluster_id
            cluster_id += 1
    return labels


def _target_plane(
    normal_camera: np.ndarray, d_camera: float, target_from_camera: np.ndarray,
    inlier_count: int,
) -> TablePlane:
    rotation = target_from_camera[:3, :3]
    translation = target_from_camera[:3, 3]
    normal = rotation @ normal_camera
    d = float(d_camera - np.dot(normal, translation))
    if normal[2] < 0.0:
        normal, d = -normal, -d
    if not np.all(np.isfinite(normal)) or not math.isfinite(d) or normal[2] < 0.70:
        raise ValueError('fitted table is not horizontal in the planning frame')
    return TablePlane(normal=normal, d=d, inlier_count=inlier_count)


def refine_object_cloud(
    depth_u16: np.ndarray,
    sam_mask: np.ndarray,
    intrinsics: dict[str, float],
    target_from_camera: np.ndarray,
    *,
    min_depth_m: float,
    max_depth_m: float,
    config: RefinementConfig,
) -> RefinedObjectCloud:
    """Return one observable object body above a fitted table, or raise."""
    if depth_u16.shape != sam_mask.shape or sam_mask.dtype != np.bool_:
        raise ValueError('SAM mask must be boolean and match depth dimensions')
    height, width = depth_u16.shape
    table_roi = np.zeros((height, width), dtype=bool)
    y0 = int(height * config.table_roi_y_start_fraction)
    table_roi[y0:, int(width * 0.08):int(width * 0.92)] = True
    table_points, _ = _sampled_points(
        depth_u16,
        table_roi,
        intrinsics,
        stride=config.sampling_stride,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
    )
    normal_camera, d_camera, inlier_count = _fit_table_plane(table_points, config)
    plane_pixels = _plane_mask(
        depth_u16,
        intrinsics,
        normal_camera,
        d_camera,
        config.table_plane_distance_m,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
    )
    seed = sam_mask & ~plane_pixels
    sampled, _ = _sampled_points(
        depth_u16,
        seed,
        intrinsics,
        stride=config.sampling_stride,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
    )
    labels = _cluster_labels(
        sampled, config.body_cluster_epsilon_m, config.body_cluster_min_points
    )
    cluster_count = int(labels.max()) + 1 if labels.size and labels.max() >= 0 else 0
    if cluster_count == 0:
        raise ValueError('SAM valid-depth cloud has no observable main 3D body')
    winner = max(range(cluster_count), key=lambda label: int(np.count_nonzero(labels == label)))
    winner_points = sampled[labels == winner]
    if len(winner_points) < config.body_cluster_min_points:
        raise ValueError('main 3D body has insufficient cluster support')

    all_points, all_pixels = masked_point_cloud(
        depth_u16,
        seed,
        intrinsics,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
    )
    distance, _ = cKDTree(winner_points).query(all_points, k=1, workers=1)
    keep = distance <= max(
        config.body_cluster_epsilon_m * 1.35,
        config.body_cluster_epsilon_m + 0.006,
    )
    points_camera = all_points[keep]
    pixels_uv = all_pixels[keep]
    if len(points_camera) < config.body_cluster_min_points:
        raise ValueError('expanded main 3D body has insufficient points')
    refined_mask = np.zeros(sam_mask.shape, dtype=bool)
    refined_mask[pixels_uv[:, 1], pixels_uv[:, 0]] = True
    table_plane = _target_plane(
        normal_camera, d_camera, target_from_camera, inlier_count
    )
    return RefinedObjectCloud(
        mask=refined_mask,
        points_camera=points_camera,
        points_target=transform_cloud(points_camera, target_from_camera),
        pixels_uv=pixels_uv,
        table_plane=table_plane,
        table_pixels_removed=int(np.count_nonzero(sam_mask & plane_pixels)),
        cluster_points=len(winner_points),
    )
