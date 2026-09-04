"""Strict fetch-and-deliver intent parsing and bounded LM Studio client."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
from typing import Any
import urllib.error
import urllib.request


GENERIC_OBJECT_NAMES = {
    '拿',
    '拿个',
    '拿一个',
    '一个',
    '这个',
    '那个',
    '东西',
    '物体',
    '物体名',
    '目标',
    '过来',
}

# Deterministic corrections for recurring ASR homophones. This is not an
# object allowlist: names absent from this table continue through unchanged.
COMMON_OBJECT_HOMOPHONES = {
    '鱼毛球': '羽毛球',
    '鱼模球': '羽毛球',
    '羽模球': '羽毛球',
}

SYSTEM_PROMPT = """你是 XLeRobot 家庭服务机器人的语义解析器。
关闭思考模式，不要输出推理、解释或 Markdown，只输出 JSON。
当前只支持 fetch_deliver：从指定地点取物并递送给指定接收者。
source_place 默认 table，recipient 默认 nearest_person。
物体名必须来自用户命令；只可纠正明显的语音同音字，不得按能力清单凭空替换。"""

USER_PROMPT = """/no_think
解析这个语音命令：
{command_text}

只输出 JSON：
{{"intent":"fetch_deliver","object":"物体名","source_place":"table",\
"recipient":"nearest_person","confidence":0.95,"normalized_command":"规范化命令"}}"""


@dataclass(frozen=True)
class IntentResult:
    """Validated task fields proposed by the language backend."""

    intent: str
    object_id: str
    source_place: str
    recipient_id: str
    confidence: float
    normalized_command: str


class IntentParseError(ValueError):
    """The backend response cannot safely become a task goal."""


def extract_json_payload(text: str) -> dict[str, Any]:
    """Extract one object from plain or fenced model output."""
    cleaned = (text or '').strip()
    fence = re.search(r'```(?:json)?\s*(.*?)```', cleaned, re.S)
    if fence:
        cleaned = fence.group(1).strip()
    start = cleaned.find('{')
    end = cleaned.rfind('}')
    if start < 0 or end < start:
        raise IntentParseError('no JSON object in intent response')
    try:
        payload = json.loads(cleaned[start:end + 1])
    except json.JSONDecodeError as exc:
        raise IntentParseError(f'invalid JSON in intent response: {exc}') from exc
    if not isinstance(payload, dict):
        raise IntentParseError('intent JSON is not an object')
    return payload


def _safe_text(payload, key, default, *, max_length):
    value = str(payload.get(key) or default).strip()
    if not value or len(value) > max_length:
        raise IntentParseError(f'{key} must contain 1 to {max_length} characters')
    if any(ord(character) < 32 for character in value):
        raise IntentParseError(f'{key} must not contain control characters')
    return value


def normalize_intent(payload: dict[str, Any], *, min_confidence: float):
    """Validate semantic and scalar constraints before task dispatch."""
    if not 0.0 <= min_confidence <= 1.0:
        raise ValueError('min_confidence must be in [0, 1]')
    intent = _safe_text(payload, 'intent', '', max_length=32)
    object_id = str(payload.get('object') or payload.get('object_id') or '').strip()
    if not object_id or len(object_id) > 128:
        raise IntentParseError('object must contain 1 to 128 characters')
    if any(ord(character) < 32 for character in object_id):
        raise IntentParseError('object must not contain control characters')
    object_id = COMMON_OBJECT_HOMOPHONES.get(object_id, object_id)
    source_place = _safe_text(payload, 'source_place', 'table', max_length=64)
    # The delivered demo has one recipient capability: find the nearest person.
    # Do not let free-form model output leak an unsupported recipient identifier
    # into ExecuteTask and fail only after the object has already been grasped.
    recipient_id = 'nearest_person'
    normalized = str(payload.get('normalized_command') or '').strip()
    try:
        confidence = float(payload.get('confidence', 0.0))
    except (TypeError, ValueError) as exc:
        raise IntentParseError('confidence is not numeric') from exc
    if intent != 'fetch_deliver':
        raise IntentParseError(f'unsupported intent: {intent}')
    if object_id in GENERIC_OBJECT_NAMES:
        raise IntentParseError(f'object is not concrete: {object_id}')
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise IntentParseError('confidence must be finite and in [0, 1]')
    if confidence < min_confidence:
        raise IntentParseError(f'confidence too low: {confidence:.2f}')
    return IntentResult(
        intent=intent,
        object_id=object_id,
        source_place=source_place,
        recipient_id=recipient_id,
        confidence=confidence,
        normalized_command=normalized,
    )


def parse_intent_text(text: str, *, min_confidence: float):
    """Parse and validate one model response."""
    return normalize_intent(extract_json_payload(text), min_confidence=min_confidence)


class LmStudioIntentClient:
    """Small OpenAI-compatible client with response and time bounds."""

    def __init__(self, *, base_url: str, model: str, timeout_s: float):
        if not base_url.startswith(('http://', 'https://')):
            raise ValueError('base_url must use http or https')
        if not model.strip():
            raise ValueError('model must not be empty')
        if timeout_s <= 0.0:
            raise ValueError('timeout_s must be positive')
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.timeout_s = float(timeout_s)

    def parse(self, command_text: str, *, min_confidence: float):
        """Request then validate a structured fetch-and-deliver intent."""
        command_text = command_text.strip()
        if not command_text or len(command_text) > 500:
            raise IntentParseError('command text must contain 1 to 500 characters')
        body = {
            'model': self.model,
            'messages': [
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {
                    'role': 'user',
                    'content': USER_PROMPT.format(command_text=command_text),
                },
            ],
            'temperature': 0.0,
            'max_tokens': 192,
            'stream': False,
        }
        request = urllib.request.Request(
            f'{self.base_url}/v1/chat/completions',
            data=json.dumps(body, ensure_ascii=False).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                raw = response.read(1_000_001)
        except (urllib.error.URLError, TimeoutError) as exc:
            raise IntentParseError(f'LM Studio request failed: {exc}') from exc
        if len(raw) > 1_000_000:
            raise IntentParseError('LM Studio response exceeded 1 MB')
        try:
            data = json.loads(raw.decode('utf-8'))
            message = data['choices'][0]['message']
            content = str(message.get('content') or message.get('reasoning_content') or '')
        except (json.JSONDecodeError, KeyError, IndexError, TypeError) as exc:
            raise IntentParseError('LM Studio response is missing message content') from exc
        return parse_intent_text(content, min_confidence=min_confidence)
