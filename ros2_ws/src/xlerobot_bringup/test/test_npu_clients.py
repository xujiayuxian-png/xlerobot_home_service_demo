"""Loopback protocol tests with fake model outputs; no SDK, camera or motor use."""

import json
import os
from pathlib import Path
import subprocess
import sys
import threading

import numpy as np
import pytest

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'tools/run').is_file())
sys.path.insert(0, str(ROOT))
for package in ('xlerobot_voice', 'xlerobot_perception', 'xlerobot_policy'):
    sys.path.insert(0, str(ROOT/'ros2_ws/src'/package))

from services.npu.server import Handler, Server
from xlerobot_perception.detection.npu import NpuPersonDetector
from xlerobot_voice.npu import NpuTranscriber
from xlerobot_policy.act_http import ActHttpClient


def test_private_aidlite_plugin_redirect_is_process_local_and_exact(tmp_path):
    shim = tmp_path/'shim.so'
    plugin = tmp_path/'plugin.so'
    source = tmp_path/'plugin.c'
    source.write_text('int x1_test_value(void) { return 42; }\n')
    subprocess.run(['cc', '-shared', '-fPIC', str(source), '-o', str(plugin)], check=True)
    subprocess.run(['cc', '-shared', '-fPIC', str(ROOT/'tools/lib/npu_aidlite_backend.c'),
                    '-ldl', '-o', str(shim)], check=True)
    script = '''
import ctypes
assert ctypes.CDLL('/usr/local/lib/libaidlite_qnn240.so').x1_test_value() == 42
assert ctypes.CDLL(None).getpid() > 0
try:
    ctypes.CDLL('/nonexistent/libaidlite_qnn240.so')
except OSError:
    pass
else:
    raise AssertionError('unrelated paths must not be redirected')
'''
    env = {**os.environ, 'LD_PRELOAD': str(shim),
           'XLEROBOT_AIDLITE_QNN240_LIBRARY': str(plugin)}
    subprocess.run([sys.executable, '-c', script], env=env, check=True, timeout=5)


def test_local_vlm_maps_672_pixel_boxes_to_original_camera_and_uses_nonzero_temperature(monkeypatch):
    import base64
    import cv2
    import urllib.request
    from xlerobot_perception.vlm.lmstudio import LmStudioVlmClient
    result = {'found': True, 'box': [67.2, 134.4, 336, 537.6], 'confidence': 0.9}
    requests = []
    class Response:
        def __enter__(self): return self
        def __exit__(self, *_): pass
        def read(self, _):
            return json.dumps({'choices': [{'finish_reason': 'stop', 'message': {
                'content': json.dumps(result)}}]}).encode()
    def request(req, **_kwargs):
        requests.append(json.loads(req.data)); return Response()
    monkeypatch.setattr(urllib.request, 'urlopen', request)
    client = LmStudioVlmClient(base_url='http://127.0.0.1:18888', model='test', timeout_s=10,
                              bbox_format='norm1000', backend='npu')
    rows, _ = client.detect(np.zeros((480, 640, 3), np.uint8), '橡皮擦')
    assert rows[0].bbox_px == (64, 96, 320, 384)
    payload = requests[0]
    assert payload['temperature'] == 0.1 and payload['max_tokens'] == 128
    encoded = payload['messages'][0]['content'][1]['image_url']['url'].split(',')[1]
    assert cv2.imdecode(np.frombuffer(base64.b64decode(encoded), np.uint8), 1).shape == (672, 672, 3)
    result['box'] = [0, 0, 1000, 1000]
    with pytest.raises(RuntimeError, match='coordinates'):
        client.detect(np.zeros((480, 640, 3), np.uint8), '橡皮擦')


def test_local_profile_rejects_a_remote_endpoint_and_demo_requires_hardware():
    from tools.lib.npu_profile import MODEL, MODEL_4B, URLS, validate_profile
    config = {'demo': {**dict.fromkeys(('inference_backend', 'asr_backend', 'person_backend', 'vlm_backend'), 'npu'),
                       'grasp_backend': 'act', 'act_wrist_only': True},
              'models': {'vlm': MODEL}, 'services': dict(URLS)}
    validate_profile(config)
    config['models']['vlm'] = MODEL_4B
    validate_profile(config)
    config['services']['act_url'] = 'http://192.0.2.1:8766'
    with pytest.raises(ValueError, match='remote fallback'):
        validate_profile(config)
    result = subprocess.run([sys.executable, str(ROOT/'tools/lib/npu_stack.py'), 'start',
                             '--demo', '--config', 'unused'], capture_output=True, text=True, timeout=5)
    assert result.returncode != 0 and 'both --demo and --hardware' in result.stderr


@pytest.fixture
def worker():
    class Runtime:
        result = {}
        last_request = None
        def predict(self, payload):
            self.last_request = payload
            return self.result
    with Server(('127.0.0.1', 0), Handler) as server:
        server.runtime, server.kind, server.load_s = Runtime(), 'whisper', 0
        thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
        try:
            yield server, f'http://127.0.0.1:{server.server_port}'
        finally:
            server.shutdown(); thread.join(timeout=3)


def test_npu_asr_requires_decoder_evidence_and_rejects_unfinished_text(worker):
    server, url = worker
    server.runtime.result = {'finished': True, 'accepted': True, 'text': '拿橡皮擦',
                             'language_probability': 0.99, 'mean_token_log_probability': -0.1,
                             'no_speech_probability': 0.01}
    client = NpuTranscriber(url)
    assert client.transcribe(np.zeros(100, np.float32)).text == '拿橡皮擦'
    assert server.runtime.last_request['sample_rate'] == 16000
    assert 'deadline_monotonic' in server.runtime.last_request
    del server.runtime.result['language_probability']
    with pytest.raises(ValueError, match='evidence'):
        client.transcribe(np.zeros(100, np.float32))
    server.runtime.result['finished'] = False
    assert client.transcribe(np.zeros(100, np.float32)).text == ''


def test_person_adapter_preserves_source_geometry_and_rejects_invalid_output(worker):
    server, url = worker
    server.kind = 'person'
    server.runtime.result = {'detections': [{'bbox_px': [1, 2, 10, 20], 'confidence': 0.9}]}
    client = NpuPersonDetector(url)
    detections, _ = client.detect(np.zeros((30, 40, 3), np.uint8))
    assert detections[0].bbox_px == (1, 2, 10, 20)
    server.runtime.result['detections'][0]['bbox_px'] = [1, 2, 41, 20]
    with pytest.raises(ValueError, match='geometry'):
        client.detect(np.zeros((30, 40, 3), np.uint8))


def test_existing_act_client_interoperates_with_worker_and_attaches_local_deadline(worker):
    server, url = worker
    server.kind = 'act'
    server.runtime.result = {'action': [[0.1]*6]*100}
    client = ActHttpClient(predict_url=url+'/predict')
    result = client.predict(head_bgr=None, wrist_bgr=np.zeros((480, 640, 3), np.uint8), state=[0]*6)
    assert len(result) == 100
    assert 'deadline_monotonic' in server.runtime.last_request
    assert 'head_image_base64' not in server.runtime.last_request
    assert not ActHttpClient(predict_url='http://192.0.2.1/predict').local_worker


def test_local_act_client_discards_late_action_even_if_http_succeeds(worker, monkeypatch):
    server, url = worker
    server.runtime.result = {'action': [[0.1]*6]*100}
    from xlerobot_policy import act_http
    # A successful response can arrive after its deadline without a socket timeout.
    now = act_http.time.monotonic()
    clock = iter((now, now+4.0))
    monkeypatch.setattr(act_http, 'time', type('Clock', (), {'monotonic': lambda: next(clock)}))
    with pytest.raises(RuntimeError, match='action discarded'):
        ActHttpClient(predict_url=url+'/predict').predict(
            head_bgr=None, wrist_bgr=np.zeros((480, 640, 3), np.uint8), state=[0]*6)


def test_supervisor_can_be_canceled_during_preflight_without_starting_workers(tmp_path):
    script = '''import os, signal, sys
from pathlib import Path
from tools.lib import npu_stack
npu_stack.STATE = Path(sys.argv[1])
def canceled_settings(_path):
    os.kill(os.getpid(), signal.SIGTERM)
    return {}
npu_stack.settings = canceled_settings
sys.argv = ['probe', 'start', '--config', 'unused']
try:
    npu_stack.main()
except RuntimeError as error:
    assert str(error) == 'inference startup canceled'
else:
    raise AssertionError('cancellation ignored')
assert not (npu_stack.STATE/'processes.json').exists()
assert not list(npu_stack.STATE.glob('*.pid'))
'''
    subprocess.run([sys.executable, '-c', script, str(tmp_path)], cwd=ROOT, check=True, timeout=10)


def test_inference_entry_rejects_hardware_before_starting_any_worker():
    result = subprocess.run([str(ROOT/'tools/run'), 'npu', '--hardware'],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode != 0
    assert 'inference-only' in result.stderr


def test_npu_status_works_without_an_explicit_config_or_loading_models():
    import os
    env = {k: v for k, v in os.environ.items() if k != 'XLEROBOT_CONFIG'}
    result = subprocess.run([str(ROOT/'tools/run'), 'npu', '--status'], env=env,
                            capture_output=True, text=True, check=True, timeout=10)
    assert isinstance(json.loads(result.stdout)['supervisor_running'], bool)


@pytest.mark.parametrize('arguments, message', [
    (['gpu', '--install-npu-device-rule'], 'robot-only'),
    (['robot', '--install-npu-device-rule', '--system-only'], 'must be used alone'),
    (['robot', '--install-npu-device-rule', '--install-demo-service'], 'must be used alone'),
])
def test_npu_device_setup_rejects_unrelated_installations(arguments, message):
    result = subprocess.run([str(ROOT/'tools/setup'), *arguments], capture_output=True,
                            text=True, timeout=10)
    assert result.returncode != 0
    assert message in result.stderr


def test_whisper_worker_warms_feature_extractor_before_readiness(monkeypatch):
    from services.npu.server import Whisper
    from tools.lib import npu_bench
    calls = []
    class FakeWhisper:
        def __init__(self, path):
            calls.append(path)
        def transcribe(self, audio, maximum):
            assert audio.dtype == np.float32 and audio.shape == (16000,)
            assert not np.any(audio)
            calls.append(maximum)
    monkeypatch.setattr(npu_bench, 'QnnWhisper', FakeWhisper)
    Whisper('unused-test-path')
    assert calls == ['unused-test-path', 64]


def test_aidgen_shim_uses_public_setting_after_initialization(tmp_path):
    headers = tmp_path/'aidlux/aidgen'; headers.mkdir(parents=True)
    (headers/'aidgen.hpp').write_text('''#include <string>
#include <memory>
namespace aplux::aidgen {
enum class ErrorCode { SUCCESS=0, BACKEND_NOT_FOUND=-21 };
struct ContextProperties {};
enum class BackendType { DEFAULT=0 };
class Context {public:
 static std::shared_ptr<Context> create_instance(const std::string&,const ContextProperties&,BackendType);
 ErrorCode initialize(void*); ErrorCode set_config(const std::string&,const std::string&);};
}''')
    original = tmp_path/'original.cpp'
    original.write_text('''#include <aidlux/aidgen/aidgen.hpp>
#include <iostream>
namespace aplux::aidgen {
std::shared_ptr<Context> Context::create_instance(const std::string& cfg,const ContextProperties&,BackendType) {
 std::cout << "config=" << cfg << "\\n"; return std::make_shared<Context>(); }
ErrorCode Context::initialize(void*) {std::cout << "initialized\\n"; return ErrorCode::SUCCESS;}
ErrorCode Context::set_config(const std::string& k,const std::string& v) {
 std::cout << k << "=" << v << "\\n"; return ErrorCode::SUCCESS; }
}''')
    main = tmp_path/'main.cpp'
    main.write_text('#include <aidlux/aidgen/aidgen.hpp>\nint main(){using namespace aplux::aidgen; auto c=Context::create_instance("original",{},BackendType::DEFAULT); return int(c->initialize(nullptr));}')
    subprocess.run(['c++', '-shared', '-fPIC', '-std=c++17', '-I'+str(tmp_path), str(original),
                    '-o', str(tmp_path/'liboriginal.so')], check=True, timeout=30)
    subprocess.run(['c++', '-shared', '-fPIC', '-std=c++17', '-I'+str(tmp_path),
                    str(ROOT/'tools/lib/npu_aidgen_config.cpp'), '-L'+str(tmp_path), '-loriginal', '-ldl',
                    '-o', str(tmp_path/'shim.so')], check=True, timeout=30)
    subprocess.run(['c++', '-std=c++17', '-I'+str(tmp_path), str(main), '-L'+str(tmp_path),
                    '-loriginal', '-o', str(tmp_path/'probe')], check=True, timeout=30)
    import os
    result = subprocess.run([str(tmp_path/'probe')], env={**os.environ, 'LD_LIBRARY_PATH': str(tmp_path),
                            'LD_PRELOAD': str(tmp_path/'shim.so')}, capture_output=True, text=True,
                            check=True, timeout=5)
    assert result.stdout.splitlines() == ['config=original', 'initialized', 'generator_n-threads=0']
    result = subprocess.run([str(tmp_path/'probe')], env={**os.environ, 'LD_LIBRARY_PATH': str(tmp_path),
                            'LD_PRELOAD': str(tmp_path/'shim.so'), 'XLEROBOT_AIDGEN_RAW_CONFIG': 'raw.json'},
                            capture_output=True, text=True, check=True, timeout=5)
    assert result.stdout.splitlines() == ['config=raw.json', 'initialized', 'generator_n-threads=0']


def test_experimental_embedding_bridge_checks_size_and_preserves_default(tmp_path):
    declaration = '''#include <cstddef>
namespace aplux::aidgen {
class ContextImpl;
int get_embedding_buff_se(ContextImpl*,const char*&,std::size_t&);
}
'''
    original = tmp_path/'original.cpp'
    original.write_text(declaration + '''namespace aplux::aidgen {
int get_embedding_buff_se(ContextImpl*,const char*& p,std::size_t& n) {p="original"; n=8; return 7;}
}''')
    main = tmp_path/'main.cpp'
    main.write_text(declaration + '''#include <iostream>
int main(){const char* p=nullptr; std::size_t n=0;
int rc=aplux::aidgen::get_embedding_buff_se(nullptr,p,n);
std::cout << rc << ":" << n << ":" << (p ? p[0] : '-');}
''')
    subprocess.run(['c++', '-shared', '-fPIC', str(original), '-o', str(tmp_path/'liboriginal.so')], check=True, timeout=30)
    subprocess.run(['c++', '-shared', '-fPIC', '-std=c++17', str(ROOT/'tools/lib/npu_aidgen_raw_embedding.cpp'),
                    '-ldl', '-o', str(tmp_path/'shim.so')], check=True, timeout=30)
    subprocess.run(['c++', str(main), '-L'+str(tmp_path), '-loriginal', '-o', str(tmp_path/'probe')], check=True, timeout=30)
    import os
    env = {k: v for k, v in os.environ.items() if k != 'XLEROBOT_AIDGEN_VOCAB_EMBEDDING'}
    env.update(LD_LIBRARY_PATH=str(tmp_path), LD_PRELOAD=str(tmp_path/'shim.so'))
    def probe():
        return subprocess.run([str(tmp_path/'probe')], env=env, capture_output=True, text=True,
                              check=True, timeout=5).stdout
    assert probe() == '7:8:o'
    table = tmp_path/'embedding.bin'; table.write_bytes(b'A')
    env['XLEROBOT_AIDGEN_VOCAB_EMBEDDING'] = str(table)
    assert probe() == '-20:0:-'
    expected = 151936 * 2560 * 4
    with table.open('r+b') as sparse:
        sparse.truncate(expected)  # Sparse file; the test touches only its first page.
    assert probe() == f'0:{expected}:A'
