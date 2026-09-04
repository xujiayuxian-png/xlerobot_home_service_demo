import base64
import json

import numpy as np
import pytest

from xlerobot_policy.act_http import ActHttpClient


class FakeResponse:
    def __init__(self, payload):
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self, _limit):
        return self.payload


def test_http_client_sends_two_images_and_returns_validated_chunk(monkeypatch):
    captured = {}
    response = {'action': [[0.1, 0.2, 0.3, 0.4, 0.5, 1.0]]}

    def urlopen(request, timeout):
        captured['url'] = request.full_url
        captured['timeout'] = timeout
        captured['authorization'] = request.get_header('Authorization')
        captured['payload'] = json.loads(request.data)
        return FakeResponse(json.dumps(response).encode())

    monkeypatch.setattr('urllib.request.urlopen', urlopen)
    client = ActHttpClient(
        predict_url='http://127.0.0.1:8766/predict',
        auth_token='test-token',
    )
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    chunk = client.predict(head_bgr=image, wrist_bgr=image, state=[0.0] * 6)
    assert chunk == response['action']
    assert captured['url'].endswith('/predict')
    assert captured['timeout'] == 3.0
    assert captured['authorization'] == 'Bearer test-token'
    assert captured['payload']['state'] == [0.0] * 6
    assert base64.b64decode(captured['payload']['head_image_base64'])[:2] == b'\xff\xd8'
    assert base64.b64decode(captured['payload']['wrist_image_base64'])[:2] == b'\xff\xd8'


def test_http_client_rejects_bad_state_before_network(monkeypatch):
    called = False

    def urlopen(_request, _timeout):
        nonlocal called
        called = True

    monkeypatch.setattr('urllib.request.urlopen', urlopen)
    client = ActHttpClient(predict_url='http://127.0.0.1:8766/predict')
    image = np.zeros((2, 2, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match='six finite'):
        client.predict(head_bgr=image, wrist_bgr=image, state=[0.0] * 5)
    assert not called


@pytest.mark.parametrize(
    'response',
    [b'not-json', json.dumps({}).encode(), json.dumps({'action': [[0.0] * 5]}).encode()],
)
def test_http_response_cannot_bypass_chunk_validation(monkeypatch, response):
    monkeypatch.setattr(
        'urllib.request.urlopen', lambda _request, timeout: FakeResponse(response)
    )
    client = ActHttpClient(predict_url='http://127.0.0.1:8766/predict')
    image = np.zeros((2, 2, 3), dtype=np.uint8)
    with pytest.raises((RuntimeError, ValueError)):
        client.predict(head_bgr=image, wrist_bgr=image, state=[0.0] * 6)
