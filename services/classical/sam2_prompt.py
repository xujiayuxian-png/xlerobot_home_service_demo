"""Lazy prompted SAM2 inference pinned to the published model snapshot.

The service never resolves the mutable Hugging Face ``main`` revision and it
never downloads model weights while handling a request.  Run
``python3 download_model.py`` once during GPU setup; health then verifies the
cached checkpoint digest before advertising the segmentation backend ready.
"""

from __future__ import annotations

import os
from pathlib import Path
import hashlib
import threading
from typing import Any

import cv2
import numpy as np


MODEL_ID = os.environ.get("SAM2_MODEL_ID", "facebook/sam2.1-hiera-tiny")
MODEL_REVISION = os.environ.get(
    "SAM2_MODEL_REVISION", "de431c4043854a71d8101e17995dfe596bf101a5"
)
MODEL_FILENAME = "sam2.1_hiera_tiny.pt"
MODEL_CONFIG = "configs/sam2.1/sam2.1_hiera_t.yaml"
MODEL_SHA256 = os.environ.get(
    "SAM2_CHECKPOINT_SHA256",
    "7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69",
)
_model = None
_predictor = None
_checkpoint_path: Path | None = None
_load_lock = threading.Lock()


def _digest(path: Path) -> str:
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def resolve_checkpoint(*, allow_download: bool = False) -> Path:
    """Return the exact verified checkpoint, optionally fetching it for setup."""
    global _checkpoint_path
    if _checkpoint_path is not None:
        return _checkpoint_path
    configured = os.environ.get("SAM2_CHECKPOINT_PATH", "").strip()
    if configured:
        path = Path(configured).expanduser().resolve()
    else:
        from huggingface_hub import hf_hub_download

        try:
            path = Path(hf_hub_download(
                repo_id=MODEL_ID,
                filename=MODEL_FILENAME,
                revision=MODEL_REVISION,
                local_files_only=not allow_download,
            ))
        except Exception as exc:
            raise RuntimeError(
                "pinned SAM2 checkpoint is not cached; run "
                "`python3 services/classical/download_model.py` during setup"
            ) from exc
    if not path.is_file():
        raise RuntimeError(f"SAM2 checkpoint does not exist: {path}")
    actual = _digest(path)
    if actual != MODEL_SHA256:
        raise RuntimeError(
            f"SAM2 checkpoint SHA-256 mismatch: expected {MODEL_SHA256}, got {actual}"
        )
    _checkpoint_path = path
    return path


def health() -> tuple[bool, str]:
    try:
        import torch
        import sam2  # noqa: F401
        if not torch.cuda.is_available():
            return False, "CUDA is unavailable"
        resolve_checkpoint(allow_download=False)
        return True, f"ready ({MODEL_ID}@{MODEL_REVISION})"
    except Exception as exc:
        return False, str(exc)


def _load() -> None:
    global _model, _predictor
    if _predictor is not None:
        return
    with _load_lock:
        if _predictor is not None:
            return
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        checkpoint = resolve_checkpoint(allow_download=False)
        _model = build_sam2(MODEL_CONFIG, str(checkpoint), device="cuda")
        _predictor = SAM2ImagePredictor(_model)


def _negative_corners(bbox: list[float]) -> np.ndarray:
    x1, y1, x2, y2 = bbox
    dx, dy = 0.08 * (x2 - x1), 0.08 * (y2 - y1)
    return np.asarray([
        [x1 + dx, y1 + dy], [x2 - dx, y1 + dy],
        [x1 + dx, y2 - dy], [x2 - dx, y2 - dy],
    ], dtype=np.float32)


def _best_mask(masks, scores, bbox):
    x1, y1, x2, y2 = [int(round(value)) for value in bbox]
    ranked = []
    for index, (candidate, score) in enumerate(zip(masks, scores)):
        crop = candidate[y1:y2, x1:x2]
        inside = float(crop.mean()) if crop.size else 0.0
        ranked.append((float(score) * (0.5 + 0.5 * inside), index))
    _, index = max(ranked)
    return masks[index].astype(bool), float(scores[index]), index


def segment(image_bgr: np.ndarray, prompts: list[dict[str, Any]]) -> list[dict[str, Any]]:
    _load()
    assert _predictor is not None
    _predictor.set_image(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))
    height, width = image_bgr.shape[:2]
    results = []
    for prompt in prompts:
        bbox = prompt["bbox"]
        x1, y1, x2, y2 = bbox
        points = np.vstack((
            np.asarray([[(x1 + x2) * 0.5, (y1 + y2) * 0.5]], dtype=np.float32),
            _negative_corners(bbox),
        ))
        labels = np.asarray([1, 0, 0, 0, 0], dtype=np.int32)
        masks, scores, logits = _predictor.predict(
            point_coords=points,
            point_labels=labels,
            box=np.asarray(bbox, dtype=np.float32),
            multimask_output=True,
        )
        selected, confidence, selected_index = _best_mask(masks, scores, bbox)
        refined, refined_scores, _ = _predictor.predict(
            point_coords=points,
            point_labels=labels,
            box=np.asarray(bbox, dtype=np.float32),
            mask_input=logits[selected_index:selected_index + 1],
            multimask_output=False,
        )
        if int(refined[0].sum()) >= max(80, int(selected.sum()) * 0.4):
            selected = refined[0].astype(bool)
            confidence = float(refined_scores[0])
        selected = selected[:height, :width]
        ys, xs = np.where(selected)
        refined_bbox = (
            [float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1)]
            if len(xs) else bbox
        )
        results.append({
            "object_id": prompt["object_id"],
            "class": prompt["class"],
            "conf": confidence,
            "bbox": refined_bbox,
            "mask": selected,
        })
    return results
