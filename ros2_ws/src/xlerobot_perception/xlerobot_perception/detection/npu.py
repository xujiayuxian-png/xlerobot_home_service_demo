"""Opt-in person adapter for the separately managed X1 loopback NPU worker."""

import base64
import json
import math
import time
from urllib.parse import urlparse
from urllib.request import Request, urlopen

from .yolo import YoloDetection


class NpuPersonDetector:
    def __init__(self, base_url, *, confidence=0.6, timeout_s=5):
        parsed = urlparse(base_url)
        if parsed.scheme != 'http' or parsed.hostname != '127.0.0.1' or parsed.username or parsed.path not in ('', '/'):
            raise ValueError('NPU person worker must be a literal loopback HTTP URL')
        if not 0.6 <= confidence <= 1 or not 0 < timeout_s <= 10:
            raise ValueError('invalid NPU confidence/timeout')
        self.url, self.confidence, self.timeout = base_url.rstrip('/'), confidence, timeout_s
        with urlopen(self.url+'/healthz', timeout=self.timeout) as response:
            health = json.loads(response.read(65536))
        if health.get('kind') != 'person' or health.get('device') != 'qnn-htp':
            raise RuntimeError('wrong NPU person worker; no CPU fallback')

    def detect(self, bgr):
        import cv2
        import numpy as np
        if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('BGR uint8 image required')
        started = time.monotonic()
        ok, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise RuntimeError('image encoding failed')
        body = json.dumps({'image_base64': base64.b64encode(encoded).decode(),
                           'deadline_monotonic': started+self.timeout}).encode()
        with urlopen(Request(self.url+'/predict', body, {'Content-Type': 'application/json'}), timeout=self.timeout) as response:
            raw = response.read(1_000_001)
        if len(raw) > 1_000_000 or time.monotonic()-started >= self.timeout:
            raise RuntimeError('NPU person response exceeded limits')
        rows = json.loads(raw).get('detections')
        if not isinstance(rows, list) or len(rows) > 100:
            raise ValueError('invalid NPU person detection list')
        detections = []
        height, width = bgr.shape[:2]
        for row in rows:
            if not isinstance(row, dict):
                raise ValueError('invalid NPU person row')
            box, score = row.get('bbox_px'), row.get('confidence')
            if (not isinstance(box, list) or len(box) != 4
                    or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in box)
                    or isinstance(score, bool) or not isinstance(score, (int, float)) or not math.isfinite(score)):
                raise ValueError('invalid NPU person detection')
            x1, y1, x2, y2 = (int(round(v)) for v in box)
            if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height and 0 <= score <= 1):
                raise ValueError('NPU person geometry outside source image')
            if score >= self.confidence:
                detections.append(YoloDetection('person', (x1, y1, x2, y2), score))
        return detections, time.monotonic()-started
