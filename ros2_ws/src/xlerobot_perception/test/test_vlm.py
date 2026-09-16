import io
import json

import cv2
import numpy as np
import pytest

from xlerobot_perception.vlm import lmstudio

from xlerobot_perception.vlm.lmstudio import (
    bbox_to_px,
    LmStudioVlmClient,
    parse_detections,
    parse_grasp_verification,
)


@pytest.mark.parametrize(
    ('bbox_format', 'bbox', 'expected'),
    [
        ('norm1', [0.1, 0.2, 0.5, 0.8], (64, 96, 320, 384)),
        ('norm1000', [100, 200, 500, 800], (64, 96, 320, 384)),
        ('px', [10, 20, 300, 400], (10, 20, 300, 400)),
    ],
)
def test_bbox_formats_are_explicit(bbox_format, bbox, expected):
    assert bbox_to_px(bbox, 640, 480, bbox_format) == expected


def test_deprecated_auto_bbox_format_is_rejected():
    with pytest.raises(ValueError, match='unsupported bbox_format'):
        bbox_to_px([0.1, 0.1, 0.2, 0.2], 640, 480, 'auto')


def test_geniex_preserves_native_png_and_maps_normalized_coordinates(monkeypatch):
    import base64
    import cv2
    captured = []
    def respond(request, **kwargs):
        captured.append(json.loads(request.data))
        return io.BytesIO(json.dumps({'choices': [{'finish_reason': 'stop', 'message': {
            'content': '{"found":true,"box":[100,200,500,800],"confidence":0.9}'}}]}).encode())
    monkeypatch.setattr(lmstudio.urllib.request, 'urlopen', respond)
    client = LmStudioVlmClient(base_url='http://127.0.0.1:18888',
        model='qwen3-vl-4b-instruct-geniex-q4_0-qcs8550', timeout_s=30,
        bbox_format='px', backend='npu')
    detections, _ = client.detect(np.zeros((480, 640, 3), np.uint8), '蓝色打火机')
    assert detections[0].bbox_px == (64, 96, 320, 384)
    url = captured[0]['messages'][0]['content'][1]['image_url']['url']
    assert url.startswith('data:image/png;base64,')
    decoded = cv2.imdecode(np.frombuffer(base64.b64decode(url.split(',')[1]), np.uint8), 1)
    assert decoded.shape == (480, 640, 3)
    resized, _, _ = client._request_image(np.zeros((960, 1280, 3), np.uint8))
    assert resized.shape == (480, 640, 3)


@pytest.mark.parametrize('box', ([100, 200, 500, 800], [[100, 200, 500, 800]]))
def test_qwen3_npu_uses_448_input_and_normalized_boxes(monkeypatch, box):
    import base64
    requests = []
    def respond(request, **kwargs):
        requests.append(json.loads(request.data))
        return io.BytesIO(json.dumps({'choices': [{
            'finish_reason': 'stop', 'message': {'content': json.dumps({
                'found': True, 'box': box, 'confidence': 0.9})}}]}).encode())
    monkeypatch.setattr(lmstudio.urllib.request, 'urlopen', respond)
    client = LmStudioVlmClient(
        base_url='http://127.0.0.1:18888',
        model='qwen3-vl-4b-instruct-448x448-qnn2.40-w4a16-qcs8550',
        timeout_s=5, bbox_format='px', backend='npu')
    detections, _ = client.detect(np.zeros((480, 640, 3), np.uint8), '羽毛球')
    assert detections[0].bbox_px == (64, 96, 320, 384)
    content = requests[0]['messages'][0]['content']
    assert '0到1000归一化' in content[0]['text']
    encoded = content[1]['image_url']['url'].split(',')[1]
    assert cv2.imdecode(np.frombuffer(base64.b64decode(encoded), np.uint8), 1).shape == (448, 448, 3)


def test_grasp_verification_requires_explicit_boolean_and_confidence():
    result = parse_grasp_verification(
        '{"grasp_success":true,"confidence":0.95,"reason":"夹在两指之间"}'
    )
    assert result.grasped is True
    assert result.confidence == 0.95
    with pytest.raises(ValueError, match='boolean'):
        parse_grasp_verification(
            '{"grasp_success":"yes","confidence":0.9,"reason":"看起来成功"}'
        )


def test_grasp_verification_prompt_rejects_tabletop_and_background_objects():
    prompt = LmStudioVlmClient.grasp_verification_prompt('黄色胶棒')
    assert '离开桌面' in prompt
    assert '夹爪两指之间' in prompt
    assert '必须判定失败' in prompt


def test_parse_fenced_array_clamps_and_sorts_no_results():
    text = """```json
    [
      {"label":"lamp","bbox_2d":[-10,100,1100,900],"confidence":1.2},
      {"label":"bad","bbox_2d":[500,500,400,600],"confidence":0.5}
    ]
    ```"""
    detections = parse_detections(text, width=100, height=50)
    assert len(detections) == 1
    assert detections[0].bbox_px == (0, 5, 100, 45)
    assert detections[0].confidence == 1.0


def test_parse_requires_json_and_list_schema():
    with pytest.raises(ValueError, match='no JSON'):
        parse_detections('I saw a lamp', width=100, height=50)
    with pytest.raises(ValueError, match='objects must be a list'):
        parse_detections('{"objects": {}}', width=100, height=50)


def test_verified_object_grounding_request_upscales_long_edge_to_1280():
    client = LmStudioVlmClient(
        base_url='http://127.0.0.1:1234',
        model='test-model',
        timeout_s=1.0,
        bbox_format='norm1000',
        request_long_edge_px=1280,
    )
    request, scale_x, scale_y = client._request_image(
        np.zeros((480, 640, 3), dtype=np.uint8)
    )

    assert request.shape == (960, 1280, 3)
    assert scale_x == 2.0
    assert scale_y == 2.0
    prompt = client.prompt('羽毛球', request.shape[1], request.shape[0])
    assert '1280x960' in prompt
    assert '若不存在则不要编造' in prompt


def test_npu_grounding_two_stage_resize_preserves_source_coordinates(monkeypatch):
    client = LmStudioVlmClient(
        base_url='http://127.0.0.1:18888', model='test-model',
        timeout_s=1.0, bbox_format='px', backend='npu',
    )
    calls = []
    resize = cv2.resize

    def record_resize(image, size, **kwargs):
        calls.append((image.shape[:2], size, kwargs.get('interpolation')))
        return resize(image, size, **kwargs)

    def respond(request, **kwargs):
        payload = json.loads(request.data)
        assert '672×672' in payload['messages'][0]['content'][0]['text']
        return io.BytesIO(json.dumps({'choices': [{
            'finish_reason': 'stop',
            'message': {'content': json.dumps({
                'found': True, 'box': [336, 336, 504, 504], 'confidence': 0.9,
            })},
        }]}).encode())

    monkeypatch.setattr(lmstudio.cv2, 'resize', record_resize)
    monkeypatch.setattr(lmstudio.urllib.request, 'urlopen', respond)
    detections, _ = client.detect(np.zeros((480, 640, 3), dtype=np.uint8), '黄色胶棒')
    assert calls == [
        ((480, 640), (1280, 960), cv2.INTER_CUBIC),
        ((960, 1280), (672, 672), cv2.INTER_CUBIC),
    ]
    assert detections[0].bbox_px == (320, 240, 480, 360)

    calls.clear()
    client._request_image(np.zeros((480, 640, 3), dtype=np.uint8))
    assert calls == [((480, 640), (672, 672), None)]


def test_npu_verification_checks_lift_without_finger_contact_details(monkeypatch):
    client = LmStudioVlmClient(
        base_url='http://127.0.0.1:18888', model='test-model',
        timeout_s=1.0, bbox_format='px', backend='npu',
    )

    def respond(request, **kwargs):
        text = json.loads(request.data)['messages'][0]['content'][0]['text']
        assert '只判断目标物体是否被夹爪带离桌面' in text
        assert '不需要判断两指是否夹紧' in text
        assert '夹爪处没有可见物体' in text
        assert '不能仅因为桌面上找不到物体就判成功' in text
        assert '不要仅因无法识别物体用途或名称就判失败' in text
        return io.BytesIO(json.dumps({'choices': [{
            'finish_reason': 'stop', 'message': {'content': json.dumps({
                'grasp_success': True, 'confidence': .9, 'reason': '物体在夹爪处悬空，已离开桌面',
            })},
        }]}).encode())

    monkeypatch.setattr(lmstudio.urllib.request, 'urlopen', respond)
    result, _ = client.verify_grasp(np.zeros((480, 640, 3), dtype=np.uint8), '羽毛球')
    assert result.grasped
