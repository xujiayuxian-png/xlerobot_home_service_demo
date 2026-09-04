"""Minimal OpenAI-compatible VLM grounding client for LM Studio."""

from __future__ import annotations

import base64
from dataclasses import dataclass
import json
import re
import time
from typing import Any
import urllib.error
import urllib.request

import cv2
import numpy as np


_BBOX_KEYS = ('bbox_2d', 'bbox', 'box', 'bounding_box')


@dataclass(frozen=True)
class VlmDetection:
    """One parsed 2D detection in source-image pixels."""

    label: str
    bbox_px: tuple[int, int, int, int]
    confidence: float


@dataclass(frozen=True)
class GraspVerification:
    """One bounded visual judgment from the post-grasp wrist image."""

    grasped: bool
    confidence: float
    reason: str


def bbox_to_px(bbox, width: int, height: int, bbox_format: str):
    """Convert one explicit xyxy coordinate convention to pixels."""
    if len(bbox) < 4:
        raise ValueError('bbox must have at least four values')
    x1, y1, x2, y2 = (float(value) for value in bbox[:4])
    if bbox_format == 'norm1':
        values = (x1 * width, y1 * height, x2 * width, y2 * height)
    elif bbox_format == 'norm1000':
        values = (
            x1 * width / 1000.0,
            y1 * height / 1000.0,
            x2 * width / 1000.0,
            y2 * height / 1000.0,
        )
    elif bbox_format == 'px':
        values = (x1, y1, x2, y2)
    else:
        raise ValueError(f'unsupported bbox_format: {bbox_format}')
    if not all(np.isfinite(value) for value in values):
        raise ValueError('bbox values must be finite')
    return tuple(int(round(value)) for value in values)


def _json_payload(text: str):
    cleaned = text.strip()
    fenced = re.search(r'```(?:json)?\s*(.*?)```', cleaned, re.S)
    if fenced:
        cleaned = fenced.group(1).strip()
    object_start = cleaned.find('{')
    array_start = cleaned.find('[')
    if array_start >= 0 and (object_start < 0 or array_start < object_start):
        array_end = cleaned.rfind(']')
        if array_end < array_start:
            raise ValueError('incomplete JSON array in VLM response')
        return {'objects': json.loads(cleaned[array_start:array_end + 1])}
    if object_start >= 0:
        object_end = cleaned.rfind('}')
        if object_end < object_start:
            raise ValueError('incomplete JSON object in VLM response')
        return json.loads(cleaned[object_start:object_end + 1])
    raise ValueError('no JSON in VLM response')


def parse_detections(text: str, *, width: int, height: int, bbox_format='norm1000'):
    """Parse bounded detections; invalid rows are ignored, invalid schema is not."""
    if width <= 0 or height <= 0:
        raise ValueError('image dimensions must be positive')
    if bbox_format not in ('norm1', 'norm1000', 'px'):
        raise ValueError(f'unsupported bbox_format: {bbox_format}')
    payload = _json_payload(text)
    if isinstance(payload, list):
        rows = payload
    elif isinstance(payload, dict):
        rows = payload.get('objects', payload.get('detections', []))
    else:
        raise ValueError('VLM payload must be an object or array')
    if not isinstance(rows, list):
        raise ValueError('objects must be a list')

    detections = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        bbox = next((row.get(key) for key in _BBOX_KEYS if key in row), None)
        if not isinstance(bbox, (list, tuple)) or len(bbox) < 4:
            continue
        try:
            x1, y1, x2, y2 = bbox_to_px(bbox, width, height, bbox_format)
            confidence = float(row.get('confidence', row.get('score', 0.0)))
        except (TypeError, ValueError):
            continue
        x1 = max(0, min(width - 1, x1))
        y1 = max(0, min(height - 1, y1))
        x2 = max(0, min(width, x2))
        y2 = max(0, min(height, y2))
        if x2 <= x1 or y2 <= y1 or not np.isfinite(confidence):
            continue
        detections.append(VlmDetection(
            label=str(row.get('label', row.get('class', 'object'))),
            bbox_px=(x1, y1, x2, y2),
            confidence=max(0.0, min(1.0, confidence)),
        ))
    return detections


def parse_grasp_verification(text: str) -> GraspVerification:
    """Parse the strict post-grasp JSON contract."""
    payload = _json_payload(text)
    if not isinstance(payload, dict):
        raise ValueError('grasp verification must be a JSON object')
    grasped = payload.get('grasp_success')
    if not isinstance(grasped, bool):
        raise ValueError('grasp_success must be boolean')
    confidence = float(payload.get('confidence'))
    if not np.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError('confidence must be between 0 and 1')
    reason = str(payload.get('reason') or '').strip()
    if not reason or len(reason) > 256:
        raise ValueError('reason must contain 1 to 256 characters')
    return GraspVerification(grasped, confidence, reason)


def _message_text(message: dict[str, Any]) -> str:
    content = str(message.get('content') or '').strip()
    return content or str(message.get('reasoning_content') or '').strip()


class LmStudioVlmClient:
    """Issue one bounded HTTP request; this class has no ROS or motor access."""

    def __init__(
        self,
        *,
        base_url: str,
        model: str,
        timeout_s: float,
        bbox_format: str,
        jpeg_quality: int = 90,
        request_long_edge_px: int = 0,
    ) -> None:
        if not base_url.startswith(('http://', 'https://')):
            raise ValueError('base_url must use http or https')
        if not model.strip():
            raise ValueError('model must not be empty')
        if timeout_s <= 0.0:
            raise ValueError('timeout_s must be positive')
        if bbox_format not in ('norm1', 'norm1000', 'px'):
            raise ValueError(f'unsupported bbox_format: {bbox_format}')
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.timeout_s = float(timeout_s)
        self.bbox_format = bbox_format
        self.jpeg_quality = max(30, min(100, int(jpeg_quality)))
        self.request_long_edge_px = int(request_long_edge_px)
        if self.request_long_edge_px < 0:
            raise ValueError('request_long_edge_px must be nonnegative')

    @staticmethod
    def prompt(object_id: str, width: int, height: int) -> str:
        """Preserve the prompt used by the verified prototype grounding path."""
        return f"""这是一张 {width}x{height} 的机器人头相机 RGB 图（宽={width}，高={height}）。
请检测以下物体（若不存在则不要编造）：{object_id}
/no_think

要求：
1. 只输出 JSON 数组，不要 markdown，不要解释。
2. 每个物体：label（中文类别名）、bbox_2d（Qwen 相对坐标 [x1,y1,x2,y2]，范围 0-1000，左上角原点）、confidence（0-1）。
3. bbox_2d 必须基于原图 {width}x{height} 的 0-1000 相对坐标，与输入图像尺寸无关。
4. 格式：[{{"label":"杯子","bbox_2d":[x1,y1,x2,y2],"confidence":0.9}}]"""

    def _request_image(self, bgr: np.ndarray):
        height, width = bgr.shape[:2]
        long_edge = max(width, height)
        if self.request_long_edge_px <= 0 or long_edge >= self.request_long_edge_px:
            return bgr, 1.0, 1.0
        scale = self.request_long_edge_px / float(long_edge)
        request_width = max(1, int(round(width * scale)))
        request_height = max(1, int(round(height * scale)))
        resized = cv2.resize(
            bgr, (request_width, request_height), interpolation=cv2.INTER_CUBIC
        )
        return resized, request_width / float(width), request_height / float(height)

    def detect(self, bgr: np.ndarray, object_id: str):
        """Return parsed detections and request latency in seconds."""
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError('VLM input must be a BGR uint8 image')
        source_height, source_width = bgr.shape[:2]
        request_bgr, scale_x, scale_y = self._request_image(bgr)
        height, width = request_bgr.shape[:2]
        ok, encoded = cv2.imencode(
            '.jpg', request_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        image_data = base64.b64encode(encoded.tobytes()).decode('ascii')
        payload = {
            'model': self.model,
            'messages': [{
                'role': 'user',
                'content': [
                    {'type': 'text', 'text': self.prompt(object_id, width, height)},
                    {
                        'type': 'image_url',
                        'image_url': {'url': f'data:image/jpeg;base64,{image_data}'},
                    },
                ],
            }],
            'temperature': 0.0,
            'max_tokens': 1024,
        }
        request = urllib.request.Request(
            f'{self.base_url}/v1/chat/completions',
            data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        started = time.monotonic()
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                raw = response.read(2_000_001)
        except urllib.error.HTTPError as exc:
            detail = exc.read(1000).decode('utf-8', errors='replace')
            raise RuntimeError(f'VLM HTTP {exc.code}: {detail}') from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f'VLM request failed: {exc.reason}') from exc
        if len(raw) > 2_000_000:
            raise RuntimeError('VLM response exceeded 2 MB')
        try:
            response = json.loads(raw.decode('utf-8'))
            text = _message_text(response['choices'][0]['message'])
            detections = parse_detections(
                text, width=width, height=height, bbox_format=self.bbox_format
            )
        except (KeyError, IndexError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise RuntimeError(f'invalid VLM response: {exc}') from exc
        if scale_x != 1.0 or scale_y != 1.0:
            detections = [
                VlmDetection(
                    label=item.label,
                    bbox_px=(
                        max(0, min(source_width - 1, int(round(item.bbox_px[0] / scale_x)))),
                        max(0, min(source_height - 1, int(round(item.bbox_px[1] / scale_y)))),
                        max(0, min(source_width, int(round(item.bbox_px[2] / scale_x)))),
                        max(0, min(source_height, int(round(item.bbox_px[3] / scale_y)))),
                    ),
                    confidence=item.confidence,
                )
                for item in detections
            ]
        return detections, time.monotonic() - started

    @staticmethod
    def grasp_verification_prompt(object_id: str) -> str:
        return f"""这是机器人完成抓取后，右腕相机拍摄的图像。
目标物体：{object_id}
只有当目标物体已经离开桌面，并且清楚位于夹爪两指之间被夹持时，才可判定成功。
目标仍在桌面、位于夹爪旁边或后方、仅出现在背景中、或无法确认时，必须判定失败。
/no_think

只输出 JSON 对象，不要 markdown，不要解释：
{{"grasp_success":true,"confidence":0.0,"reason":"简短中文依据"}}"""

    def verify_grasp(self, bgr: np.ndarray, object_id: str):
        """Judge one wrist image without accessing ROS or any actuator."""
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError('VLM input must be a BGR uint8 image')
        request_bgr, _scale_x, _scale_y = self._request_image(bgr)
        ok, encoded = cv2.imencode(
            '.jpg', request_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        image_data = base64.b64encode(encoded.tobytes()).decode('ascii')
        payload = {
            'model': self.model,
            'messages': [{
                'role': 'user',
                'content': [
                    {
                        'type': 'text',
                        'text': self.grasp_verification_prompt(object_id),
                    },
                    {
                        'type': 'image_url',
                        'image_url': {'url': f'data:image/jpeg;base64,{image_data}'},
                    },
                ],
            }],
            'temperature': 0.0,
            'max_tokens': 256,
        }
        request = urllib.request.Request(
            f'{self.base_url}/v1/chat/completions',
            data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        started = time.monotonic()
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                raw = response.read(2_000_001)
        except urllib.error.HTTPError as exc:
            detail = exc.read(1000).decode('utf-8', errors='replace')
            raise RuntimeError(f'VLM HTTP {exc.code}: {detail}') from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f'VLM request failed: {exc.reason}') from exc
        if len(raw) > 2_000_000:
            raise RuntimeError('VLM response exceeded 2 MB')
        try:
            response = json.loads(raw.decode('utf-8'))
            text = _message_text(response['choices'][0]['message'])
            verification = parse_grasp_verification(text)
        except (KeyError, IndexError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise RuntimeError(f'invalid VLM response: {exc}') from exc
        return verification, time.monotonic() - started
