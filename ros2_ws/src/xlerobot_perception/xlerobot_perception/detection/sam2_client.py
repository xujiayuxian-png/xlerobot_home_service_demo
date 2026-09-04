"""Prompted SAM2 client for the optional LAN classical service."""

from __future__ import annotations

import base64
import json
import urllib.error
import urllib.request
from typing import Any

import cv2
import numpy as np

from xlerobot_perception.detection.seg_instances import (
    SegInstance,
    decode_prompted_instance,
)


class ClassicalServiceError(RuntimeError):
    """A malformed response, unavailable service, or explicit backend failure."""


def post_json(
    base_url: str, path: str, payload: dict[str, Any], timeout_s: float
) -> dict[str, Any]:
    body = json.dumps(payload, allow_nan=False).encode("utf-8")
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}{path}",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=float(timeout_s)) as response:
            decoded = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        try:
            detail = json.loads(exc.read().decode("utf-8")).get("error", exc.reason)
        except Exception:
            detail = exc.reason
        raise ClassicalServiceError(f"{path} HTTP {exc.code}: {detail}") from exc
    except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        raise ClassicalServiceError(f"{path} unavailable: {exc}") from exc
    if not isinstance(decoded, dict):
        raise ClassicalServiceError(f"{path} returned a non-object JSON value")
    return decoded


def _png_b64(image_bgr: np.ndarray) -> str:
    ok, encoded = cv2.imencode(".png", image_bgr)
    if not ok:
        raise ValueError("could not encode RGB image as PNG")
    return base64.b64encode(encoded.tobytes()).decode("ascii")


def _depth_b64(depth_u16: np.ndarray) -> str:
    depth = np.ascontiguousarray(depth_u16, dtype="<u2")
    return base64.b64encode(depth.tobytes()).decode("ascii")


class PromptedSam2Client:
    def __init__(self, base_url: str, *, timeout_s: float = 90.0) -> None:
        if not base_url.startswith(("http://", "https://")):
            raise ValueError("classical_base_url must use http:// or https://")
        if timeout_s <= 0.0:
            raise ValueError("SAM2 timeout must be positive")
        self.base_url = base_url.rstrip("/")
        self.timeout_s = float(timeout_s)

    def segment(
        self,
        image_bgr: np.ndarray,
        depth_u16: np.ndarray,
        *,
        label: str,
        bbox_px: tuple[int, int, int, int],
    ) -> SegInstance:
        if image_bgr.shape[:2] != depth_u16.shape:
            raise ValueError("color and depth dimensions differ")
        height, width = depth_u16.shape
        response = post_json(
            self.base_url,
            "/api/segment_prompt",
            {
                "width": int(width),
                "height": int(height),
                "depth_dtype": "uint16",
                "color_b64": _png_b64(image_bgr),
                "depth_b64": _depth_b64(depth_u16),
                "refine": True,
                "use_negative_corners": True,
                "objects": [
                    {
                        "object_id": 0,
                        "class": str(label),
                        "bbox": [int(value) for value in bbox_px],
                    }
                ],
            },
            self.timeout_s,
        )
        try:
            return decode_prompted_instance(
                response, width=width, height=height, object_id=0
            )
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            raise ClassicalServiceError(str(exc)) from exc
