"""Bounded HTTP client for an ACT server; returns proposals, never commands."""

from __future__ import annotations

import base64
import json
import urllib.error
import urllib.request

import cv2
import numpy as np

from xlerobot_policy.postprocess import validate_action_chunk


class ActHttpClient:
    """Encode one observation and parse one finite raw action chunk."""

    def __init__(
        self,
        *,
        predict_url,
        timeout_s=3.0,
        jpeg_quality=95,
        request_width=640,
        max_response_bytes=10_000_000,
        auth_token=None,
    ):
        if not predict_url.startswith(('http://', 'https://')):
            raise ValueError('predict_url must use http or https')
        if timeout_s <= 0.0 or request_width <= 0 or max_response_bytes <= 0:
            raise ValueError('ACT HTTP bounds must be positive')
        self.predict_url = predict_url
        self.timeout_s = float(timeout_s)
        self.jpeg_quality = max(35, min(95, int(jpeg_quality)))
        self.request_width = int(request_width)
        self.max_response_bytes = int(max_response_bytes)
        self.auth_token = str(auth_token or '')

    def predict(self, *, head_bgr, wrist_bgr, state):
        """Return a validated raw chunk in physical joint units."""
        state = [float(value) for value in state]
        if len(state) != 6 or not all(np.isfinite(value) for value in state):
            raise ValueError('ACT state must contain six finite joint values')
        payload = {
            'head_image_base64': self._jpeg(head_bgr),
            'wrist_image_base64': self._jpeg(wrist_bgr),
            'state': state,
        }
        headers = {'Content-Type': 'application/json'}
        if self.auth_token:
            headers['Authorization'] = f'Bearer {self.auth_token}'
        request = urllib.request.Request(
            self.predict_url,
            data=json.dumps(payload).encode('utf-8'),
            headers=headers,
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                raw = response.read(self.max_response_bytes + 1)
        except urllib.error.HTTPError as exc:
            detail = exc.read(1000).decode('utf-8', errors='replace')
            raise RuntimeError(f'ACT HTTP {exc.code}: {detail}') from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f'ACT request failed: {exc.reason}') from exc
        if len(raw) > self.max_response_bytes:
            raise RuntimeError('ACT response exceeded configured byte limit')
        try:
            payload = json.loads(raw.decode('utf-8'))
            actions = payload['action']
        except (json.JSONDecodeError, KeyError, TypeError) as exc:
            raise RuntimeError('ACT response is missing a valid action field') from exc
        return validate_action_chunk(actions)

    def _jpeg(self, image):
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
            raise ValueError('ACT image must be a uint8 array')
        if image.ndim != 3 or image.shape[2] != 3 or not image.size:
            raise ValueError('ACT image must be nonempty BGR with three channels')
        height, width = image.shape[:2]
        if width > self.request_width:
            scale = self.request_width / width
            image = cv2.resize(
                image,
                (self.request_width, max(1, round(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError('ACT observation JPEG encoding failed')
        return base64.b64encode(encoded.tobytes()).decode('ascii')
