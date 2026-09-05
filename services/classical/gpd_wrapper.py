"""Headless wrapper around atenpas/gpd; deliberately has no fake fallback."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np

from protocol import Camera


SERVICE_ROOT = Path(__file__).resolve().parent


class GpdError(RuntimeError):
    pass


def _gpd_root() -> Path:
    return Path(os.environ.get("GPD_ROOT", SERVICE_ROOT.parents[1] / ".xlerobot/vendor/gpd"))


def _binary(root: Path) -> Path:
    override = os.environ.get("GPD_BINARY")
    if override:
        return Path(override)
    for candidate in (
        root / "build" / "gpd_detect_grasps_json",
        root / "build" / "detect_grasps_json",
    ):
        if candidate.is_file():
            return candidate
    return root / "build" / "gpd_detect_grasps_json"


def _config() -> Path:
    return Path(os.environ.get(
        "GPD_CONFIG", SERVICE_ROOT / "config" / "xlerobot_headless.cfg"
    ))


def health() -> tuple[bool, str]:
    root = _gpd_root()
    binary, config = _binary(root), _config()
    if not binary.is_file():
        return False, f"GPD binary missing: {binary}"
    if not config.is_file():
        return False, f"GPD config missing: {config}"
    return True, "ready"


def point_cloud(depth: np.ndarray, mask: np.ndarray, camera: Camera) -> np.ndarray:
    v, u = np.nonzero(mask & (depth > 0))
    z = depth[v, u].astype(np.float64) / camera.depth_scale
    valid = np.isfinite(z) & (z >= 0.05) & (z <= 4.0)
    u, v, z = u[valid], v[valid], z[valid]
    if len(z) < 80:
        raise GpdError(f"object cloud has only {len(z)} valid points")
    x = (u - camera.cx) * z / camera.fx
    y = (v - camera.cy) * z / camera.fy
    return np.column_stack((x, y, z))


def _write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\n"
        "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA ascii\n"
    )
    with path.open("w", encoding="ascii") as stream:
        stream.write(header)
        np.savetxt(stream, points.astype(np.float32), fmt="%.8f %.8f %.8f")


def _parse(stdout: str, top_k: int) -> list[dict]:
    candidates = []
    for line in stdout.splitlines():
        if not line.lstrip().startswith("{"):
            continue
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if item.get("type") == "gpd_summary":
            continue
        if not all(key in item for key in ("score", "translation", "rotation_matrix")):
            continue
        candidates.append(item)
    candidates.sort(key=lambda item: float(item["score"]), reverse=True)
    return candidates[:top_k]


def infer(
    depth: np.ndarray,
    object_mask: np.ndarray,
    camera: Camera,
    *,
    top_k: int,
    timeout_s: float = 120.0,
) -> dict:
    ready, reason = health()
    if not ready:
        raise GpdError(reason)
    points = point_cloud(depth, object_mask, camera)
    binary, config = _binary(_gpd_root()), _config()
    with tempfile.TemporaryDirectory(prefix="xlerobot_gpd_") as temp:
        cloud = Path(temp) / "object.pcd"
        _write_ascii_pcd(cloud, points)
        environment = os.environ.copy()
        environment.setdefault("QT_QPA_PLATFORM", "offscreen")
        try:
            completed = subprocess.run(
                [str(binary), str(config), str(cloud)],
                cwd=str(binary.parent),
                env=environment,
                capture_output=True,
                text=True,
                timeout=float(timeout_s),
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise GpdError(f"GPD timed out after {timeout_s:.1f}s") from exc
    if completed.returncode != 0:
        detail = (completed.stdout + "\n" + completed.stderr)[-2000:]
        raise GpdError(f"GPD exited {completed.returncode}: {detail}")
    candidates = _parse(completed.stdout, top_k)
    if not candidates:
        raise GpdError("GPD completed but returned no grasp candidates")
    return {
        "top_grasps": candidates,
        "masked_points": int(len(points)),
        "mode": "gpd_json",
    }
