"""Strict GPD client: a GPD request either returns GPD or fails."""

from __future__ import annotations

import base64
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from xlerobot_perception.detection.sam2_client import (
    ClassicalServiceError,
    post_json,
)


@dataclass(frozen=True)
class GpdCandidate:
    score: float
    width_m: float
    translation_camera_m: np.ndarray
    rotation_camera: np.ndarray


def _encode_png(image_bgr: np.ndarray) -> str:
    ok, encoded = cv2.imencode(".png", image_bgr)
    if not ok:
        raise ValueError("could not encode RGB image as PNG")
    return base64.b64encode(encoded.tobytes()).decode("ascii")


def _encode_array(array: np.ndarray, dtype: str) -> str:
    return base64.b64encode(
        np.ascontiguousarray(array, dtype=dtype).tobytes()
    ).decode("ascii")


def _candidate(item: dict[str, Any]) -> GpdCandidate:
    try:
        translation = np.asarray(item["translation"], dtype=np.float64)
        rotation = np.asarray(item["rotation_matrix"], dtype=np.float64)
        score = float(item["score"])
        width = float(item.get("width", 0.08))
    except (KeyError, TypeError, ValueError) as exc:
        raise ClassicalServiceError(f"invalid GPD candidate: {exc}") from exc
    if translation.shape != (3,) or rotation.shape != (3, 3):
        raise ClassicalServiceError("GPD translation/rotation dimensions are invalid")
    if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(rotation)):
        raise ClassicalServiceError("GPD candidate contains non-finite values")
    if not np.isfinite(score) or not np.isfinite(width) or width <= 0.0:
        raise ClassicalServiceError("GPD score/width is invalid")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=0.12):
        raise ClassicalServiceError("GPD rotation matrix is not orthonormal")
    return GpdCandidate(score, width, translation, rotation)


class GpdClient:
    def __init__(self, base_url: str, *, timeout_s: float = 120.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_s = float(timeout_s)

    def infer(
        self,
        image_bgr: np.ndarray,
        depth_u16: np.ndarray,
        mask: np.ndarray,
        intrinsics: dict[str, float],
        *,
        label: str,
        bbox_px: tuple[float, float, float, float],
        top_k: int = 10,
    ) -> list[GpdCandidate]:
        height, width = depth_u16.shape
        response = post_json(
            self.base_url,
            "/api/infer_objects",
            {
                "backend": "gpd",
                "top_k": int(top_k),
                "width": int(width),
                "height": int(height),
                "depth_dtype": "uint16",
                "intrinsics": {key: float(value) for key, value in intrinsics.items()},
                "color_b64": _encode_png(image_bgr),
                "depth_b64": _encode_array(depth_u16, "<u2"),
                "objects": [
                    {
                        "object_id": 0,
                        "class": str(label),
                        "bbox": [float(value) for value in bbox_px],
                        "mask_b64": _encode_array(mask, "u1"),
                    }
                ],
            },
            self.timeout_s,
        )
        if not response.get("ok"):
            raise ClassicalServiceError(str(response.get("error") or "GPD failed"))
        objects = response.get("objects")
        if not isinstance(objects, list) or len(objects) != 1:
            raise ClassicalServiceError("GPD response must contain exactly one object")
        obj = objects[0]
        if not obj.get("ok"):
            raise ClassicalServiceError(str(obj.get("error") or "GPD object failed"))
        raw = obj.get("top_grasps")
        if not isinstance(raw, list) or not raw:
            raise ClassicalServiceError("GPD returned no grasp candidates")
        candidates = [_candidate(item) for item in raw]
        candidates.sort(key=lambda item: item.score, reverse=True)
        return candidates
