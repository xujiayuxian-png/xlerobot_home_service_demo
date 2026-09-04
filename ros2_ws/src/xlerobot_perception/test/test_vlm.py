import numpy as np
import pytest

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
