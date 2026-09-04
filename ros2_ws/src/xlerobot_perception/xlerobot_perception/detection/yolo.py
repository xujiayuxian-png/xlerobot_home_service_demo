"""Small local YOLO adapter for the verified person-search behavior."""

from __future__ import annotations

from dataclasses import dataclass
import time


@dataclass(frozen=True)
class YoloDetection:
    label: str
    bbox_px: tuple[int, int, int, int]
    confidence: float


class YoloPersonDetector:
    """Load the configured person model and return boxes in source pixels."""

    def __init__(self, model_path: str, *, confidence: float, image_size: int, device: str):
        from ultralytics import YOLO

        self.model = YOLO(model_path)
        self.confidence = float(confidence)
        self.image_size = int(image_size)
        self.device = str(device)

    def detect(self, bgr):
        started = time.monotonic()
        prediction = self.model.predict(
            bgr,
            classes=[0],
            conf=self.confidence,
            imgsz=self.image_size,
            device=self.device,
            verbose=False,
        )[0]
        latency_s = time.monotonic() - started
        detections = []
        if prediction.boxes is None:
            return detections, latency_s
        for box in prediction.boxes:
            if box.xyxy is None or box.conf is None:
                continue
            x1, y1, x2, y2 = (
                int(round(float(value)))
                for value in box.xyxy[0].detach().cpu().tolist()
            )
            if x2 <= x1 or y2 <= y1:
                continue
            detections.append(YoloDetection(
                label='person',
                bbox_px=(x1, y1, x2, y2),
                confidence=float(box.conf[0].detach().cpu().item()),
            ))
        return detections, latency_s
