"""Validation and codecs for the deliberately small classical HTTP API."""

from __future__ import annotations

import base64
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


MAX_IMAGE_PIXELS = 4096 * 4096


class RequestError(ValueError):
    pass


@dataclass(frozen=True)
class Camera:
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float


def dimensions(payload: dict[str, Any]) -> tuple[int, int]:
    try:
        width, height = int(payload["width"]), int(payload["height"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RequestError("width and height must be integers") from exc
    if width <= 0 or height <= 0 or width * height > MAX_IMAGE_PIXELS:
        raise RequestError("image dimensions are outside the supported range")
    return width, height


def _base64_bytes(value: Any, name: str) -> bytes:
    if not isinstance(value, str) or not value:
        raise RequestError(f"missing {name}")
    try:
        return base64.b64decode(value, validate=True)
    except Exception as exc:
        raise RequestError(f"{name} is not valid base64") from exc


def color_bgr(payload: dict[str, Any]) -> np.ndarray:
    width, height = dimensions(payload)
    raw = _base64_bytes(payload.get("color_b64"), "color_b64")
    image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape != (height, width, 3):
        raise RequestError("decoded color image does not match width/height")
    return image


def depth_u16(payload: dict[str, Any]) -> np.ndarray:
    width, height = dimensions(payload)
    if payload.get("depth_dtype", "uint16") != "uint16":
        raise RequestError("depth_dtype must be uint16")
    raw = _base64_bytes(payload.get("depth_b64"), "depth_b64")
    if len(raw) != width * height * 2:
        raise RequestError("depth_b64 byte count does not match width/height")
    return np.frombuffer(raw, dtype="<u2").reshape(height, width)


def mask(payload: dict[str, Any], *, width: int, height: int) -> np.ndarray:
    raw = _base64_bytes(payload.get("mask_b64"), "mask_b64")
    if len(raw) != width * height:
        raise RequestError("mask_b64 byte count does not match width/height")
    result = np.frombuffer(raw, dtype=np.uint8).reshape(height, width) > 0
    if not np.any(result):
        raise RequestError("object mask is empty")
    return result


def camera(payload: dict[str, Any]) -> Camera:
    intrinsics = payload.get("intrinsics")
    if not isinstance(intrinsics, dict):
        raise RequestError("intrinsics must be an object")
    try:
        values = [
            float(intrinsics[key]) for key in ("fx", "fy", "cx", "cy")
        ]
        scale = float(intrinsics.get("depth_scale", 1000.0))
    except (KeyError, TypeError, ValueError) as exc:
        raise RequestError("intrinsics are incomplete") from exc
    if not np.all(np.isfinite(values + [scale])) or min(values[:2] + [scale]) <= 0:
        raise RequestError("intrinsics are invalid")
    return Camera(*values, scale)


def encode_mask(mask_array: np.ndarray) -> str:
    return base64.b64encode(
        np.ascontiguousarray(mask_array, dtype=np.uint8).tobytes()
    ).decode("ascii")


def object_prompts(payload: dict[str, Any]) -> list[dict[str, Any]]:
    width, height = dimensions(payload)
    raw = payload.get("objects")
    if not isinstance(raw, list) or not raw or len(raw) > 8:
        raise RequestError("objects must contain one to eight prompts")
    prompts = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise RequestError("each object prompt must be an object")
        bbox = item.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            raise RequestError("each object bbox must contain four values")
        try:
            x1, y1, x2, y2 = [float(value) for value in bbox]
        except (TypeError, ValueError) as exc:
            raise RequestError("bbox values must be numeric") from exc
        if not np.all(np.isfinite([x1, y1, x2, y2])):
            raise RequestError("bbox values must be finite")
        x1, x2 = np.clip([x1, x2], 0, width - 1)
        y1, y2 = np.clip([y1, y2], 0, height - 1)
        if x2 <= x1 or y2 <= y1:
            raise RequestError("bbox must have positive area")
        prompts.append({
            "object_id": int(item.get("object_id", index)),
            "class": str(item.get("class") or "object")[:128],
            "bbox": [float(x1), float(y1), float(x2), float(y2)],
        })
    return prompts
