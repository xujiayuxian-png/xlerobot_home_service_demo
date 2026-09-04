"""Small, typed boundary for prompted instance segmentation.

The mask protocol is derived from the frozen prototype at
2477d789e678468c6cb684f6319b69f749a62bef.  Display-only helpers and
automatic-mask experiments were intentionally left behind.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class SegInstance:
    object_id: int
    label: str
    confidence: float
    bbox_px: tuple[float, float, float, float]
    mask: np.ndarray
    valid_depth_pixels: int


def decode_prompted_instance(
    response: dict[str, Any], *, width: int, height: int, object_id: int = 0
) -> SegInstance:
    """Decode one exact-size mask and reject ambiguous backend output."""
    if not response.get("ok"):
        raise RuntimeError(str(response.get("error") or "SAM2 segmentation failed"))
    objects = response.get("objects")
    if not isinstance(objects, list):
        raise RuntimeError("SAM2 response is missing objects")
    matches = [item for item in objects if int(item.get("object_id", -1)) == object_id]
    if len(matches) != 1:
        raise RuntimeError(
            f"SAM2 returned {len(matches)} masks for object_id={object_id}; expected one"
        )
    item = matches[0]
    try:
        raw = base64.b64decode(str(item["mask_b64"]), validate=True)
    except Exception as exc:
        raise RuntimeError(f"SAM2 mask is not valid base64: {exc}") from exc
    expected = int(width) * int(height)
    if len(raw) != expected:
        raise RuntimeError(
            f"SAM2 mask has {len(raw)} bytes; expected {expected} for {width}x{height}"
        )
    mask = np.frombuffer(raw, dtype=np.uint8).reshape(height, width) > 0
    if not np.any(mask):
        raise RuntimeError("SAM2 returned an empty object mask")
    bbox = item.get("bbox")
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        raise RuntimeError("SAM2 object bbox must contain four values")
    return SegInstance(
        object_id=object_id,
        label=str(item.get("class") or "object"),
        confidence=float(item.get("conf", 0.0)),
        bbox_px=tuple(float(value) for value in bbox),
        mask=mask,
        valid_depth_pixels=int(item.get("valid_depth_pixels", 0)),
    )
