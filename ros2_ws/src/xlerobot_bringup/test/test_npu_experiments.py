"""Experiment utilities only: no SDK imports, NPU access or robot devices."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'tools/lib/npu_vlm_probe.py').is_file())


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'tools/lib'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


vlm = load('npu_vlm_probe')
act = load('npu_act_export')


def test_boxes_reject_invalid_geometry_and_convert_declared_coordinate_scale():
    assert vlm.box_iou([100, 200, 300, 400], [100, 200, 300, 400]) == 1
    for box in [None, [], [True, 1, 2, 3], [0, 1, float('nan'), 3],
                [0, 0, 1001, 100], [20, 20, 10, 10]]:
        assert vlm.box_iou(box, [0, 0, 100, 100]) == 0
    case = {'kind': 'box', 'reference_box': [0, 0, 500, 500]}
    assert vlm.evaluate(case, {'found': True, 'box': [0, 0, 336, 336]}, 672)['passed']
    assert not vlm.evaluate(case, {'found': False, 'box': [0, 0, 336, 336]}, 672)['passed']


def test_vlm_requires_valid_json_and_explicit_absence_and_object_name():
    assert vlm.parse_object('```json\n{"found":false,"box":null}\n```')['found'] is False
    with pytest.raises(ValueError):
        vlm.parse_object('I think {"found":true}')
    with pytest.raises(ValueError):
        vlm.parse_object('[]')
    case = {'kind': 'box', 'reference_box': None}
    assert vlm.evaluate(case, {'found': False, 'box': None})['passed']
    assert not vlm.evaluate(case, {'found': False, 'box': []})['passed']
    case = {'kind': 'intent', 'expected_intent': 'fetch_deliver', 'object_aliases': ['羽毛球']}
    assert vlm.evaluate(case, {'intent': 'fetch_deliver', 'object': '桌上的羽毛球'})['passed']
    assert not vlm.evaluate(case, {'intent': 'fetch_deliver', 'object': '杯子'})['passed']


def test_grasp_probe_never_counts_unlabeled_or_invalid_results_as_success():
    case = {'kind': 'grasp', 'expected_grasp': False}
    result = {'grasp_success': False, 'confidence': 0.9, 'reason': '目标仍在桌面上'}
    assert vlm.evaluate(case, result)['passed']
    assert vlm.evaluate(case, {**result, 'grasp_success': True})['false_success']
    assert not vlm.evaluate({**case, 'expected_grasp': None}, result)['passed']
    for changes in ({'grasp_success': 'false'}, {'confidence': True},
                    {'confidence': float('nan')}, {'reason': ''}):
        assert not vlm.evaluate(case, {**result, **changes})['protocol_ok']


def test_production_intent_probe_uses_real_task_acceptance_rules():
    case = {'kind': 'production_intent', 'expected_accept': True, 'object_aliases': ['橡皮擦']}
    payload = {'intent': 'fetch_deliver', 'object': '蓝色橡皮擦', 'confidence': 0.95}
    assert vlm.evaluate(case, payload)['passed']
    assert not vlm.evaluate(case, {**payload, 'confidence': 0.1})['accepted']
    assert not vlm.evaluate({**case, 'expected_accept': False}, payload)['passed']


def test_local_worker_rejects_expired_requests_before_inference_and_survives_bad_inputs():
    sys.path.insert(0, str(ROOT))
    from services.npu.server import Handler, Server, decode_bytes
    import threading
    import time
    from urllib.error import HTTPError
    from urllib.request import Request, urlopen
    class Runtime:
        calls = 0
        def predict(self, payload):
            self.calls += 1
            return {'result': 1}
    runtime = Runtime()
    with Server(('127.0.0.1', 0), Handler) as server:
        server.runtime, server.kind, server.load_s = runtime, 'act', 0
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f'http://127.0.0.1:{server.server_port}/predict'
        try:
            for body in (b'[]', b'{', json.dumps({'deadline_monotonic': time.monotonic()-1}).encode()):
                with pytest.raises(HTTPError) as error:
                    urlopen(Request(url, body), timeout=3)
                assert error.value.code == 400
            assert runtime.calls == 0
            with urlopen(Request(url, b'{}'), timeout=3) as response:
                assert json.load(response) == {'result': 1}
            assert runtime.calls == 1
        finally:
            server.shutdown(); thread.join(timeout=3)
    for value in ('???', 'YQ==', 1):
        with pytest.raises(ValueError):
            decode_bytes(value, 0)


def test_act_rejects_incompatible_observation_contract():
    config = {'input_features': {'observation.state': {'type': 'STATE', 'shape': [6]},
                                'observation.images.wrist': {'type': 'VISUAL', 'shape': [3, 480, 640]}},
              'output_features': {'action': {'shape': [6]}}, 'n_obs_steps': 1,
              'chunk_size': 100, 'dim_model': 512, 'latent_dim': 32, 'vision_backbone': 'resnet18'}
    act.validate_config(config)
    for key, value in [('chunk_size', 50), ('n_obs_steps', 2), ('vision_backbone', 'resnet50')]:
        with pytest.raises(ValueError):
            act.validate_config({**config, key: value})


def test_vendor_server_bind_is_restricted_to_loopback(tmp_path):
    library = tmp_path/'loopback.so'
    subprocess.run(['cc', '-shared', '-fPIC', str(ROOT/'tools/lib/npu_loopback_bind.c'),
                    '-ldl', '-o', str(library)], check=True, timeout=15)
    code = ('import socket,json; s=socket.socket(); s.bind(("0.0.0.0",0));'
            'print(json.dumps(s.getsockname())); s.close()')
    result = subprocess.run([sys.executable, '-c', code],
                            env={**os.environ, 'LD_PRELOAD': str(library)},
                            capture_output=True, text=True, check=True, timeout=10)
    assert json.loads(result.stdout)[0] == '127.0.0.1'
