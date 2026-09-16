"""Transport and request isolation checks; fake model, no SDK or devices."""
import base64
import io
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest
from PIL import Image

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'tools/run').is_file())
sys.path.insert(0, str(ROOT))
from services.npu.geniex_server import Engine, Busy, MODEL_ID, prepare_request


def test_profile_and_service_share_model_id():
    from tools.lib.npu_profile import MODEL_GENIEX, MODELS
    assert MODEL_ID == MODEL_GENIEX and MODEL_ID in MODELS


def request():
    out = io.BytesIO()
    Image.new('RGB', (640, 480)).save(out, format='PNG')
    return {'model': MODEL_ID, 'messages': [{'role': 'user', 'content': [
        {'type': 'text', 'text': '找羽毛球'},
        {'type': 'image_url', 'image_url': {'url': 'data:image/png;base64,' + base64.b64encode(out.getvalue()).decode()}}
    ]}], 'max_tokens': 128}


def test_inline_only_and_image_budget(tmp_path):
    payload = request()
    messages, images, _, _ = prepare_request(payload, tmp_path)
    assert messages[0]['content'][0]['type'] == 'image'
    assert Path(images[0]).is_file()
    assert payload['messages'][0]['content'][0]['type'] == 'text'
    payload['messages'][0]['content'][1]['image_url']['url'] = 'http://192.0.2.1/image.png'
    with pytest.raises(ValueError, match='inline'):
        prepare_request(payload, tmp_path)
    out = io.BytesIO()
    Image.new('RGB', (1280, 960)).save(out, format='PNG')
    payload['messages'][0]['content'][1]['image_url']['url'] = 'data:image/png;base64,' + base64.b64encode(out.getvalue()).decode()
    with pytest.raises(ValueError, match='budget'):
        prepare_request(payload, tmp_path)


class FakeModel:
    def __init__(self):
        self.resets = 0
        self.paths = []
        self.tokenizer = self
        self.stop = 'eos'

    def reset(self):
        self.resets += 1

    def apply_chat_template(self, messages, **_):
        return 'prompt'

    def generate(self, prompt, images, **_):
        assert self.resets == len(self.paths) + 1
        assert Path(images[0]).exists()
        self.paths.append(images[0])
        return SimpleNamespace(text='{}', profile=SimpleNamespace(
            stop_reason=self.stop, prompt_tokens=12, generated_tokens=2))


def test_reset_serialization_temporary_cleanup_and_truncation():
    model = FakeModel()
    engine = Engine(model)
    assert engine.complete(request())['choices'][0]['finish_reason'] == 'stop'
    model.stop = 'max_tokens'
    assert engine.complete(request())['choices'][0]['finish_reason'] == 'length'
    assert model.resets == 2
    assert all(not Path(p).exists() for p in model.paths)
    with engine.lock:
        with pytest.raises(Busy):
            engine.complete(request())
    engine.close()


def test_runtime_failure_fails_closed_and_releases_lock():
    model = FakeModel()
    def fail(*_, **__):
        raise RuntimeError('DSP failure')
    model.generate = fail
    engine = Engine(model)
    with pytest.raises(RuntimeError, match='DSP'):
        engine.complete(request())
    assert engine.failed and not engine.lock.locked()
    with pytest.raises(RuntimeError, match='restart'):
        engine.complete(request())
    engine.close()


def test_native_lifetime_uses_one_owner_thread():
    calls = []

    class OwnedModel(FakeModel):
        def __init__(self):
            super().__init__()
            calls.append(('load', threading.get_ident()))

        def reset(self):
            calls.append(('reset', threading.get_ident()))
            super().reset()

        def generate(self, *args, **kwargs):
            calls.append(('generate', threading.get_ident()))
            return super().generate(*args, **kwargs)

        def close(self):
            calls.append(('close', threading.get_ident()))

    engine = Engine(factory=OwnedModel)
    try:
        engine.complete(request())
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(max_workers=1) as caller:
            caller.submit(engine.complete, request()).result()
    finally:
        engine.close()
    assert [name for name, _ in calls] == ['load', 'reset', 'generate', 'reset', 'generate', 'close']
    assert len({ident for _, ident in calls}) == 1
    assert calls[0][1] != threading.get_ident()
