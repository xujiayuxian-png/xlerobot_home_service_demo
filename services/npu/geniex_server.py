"""X1 loopback OpenAI transport for the resident Qwen3-VL Hexagon model.

This worker only returns model data. It never imports ROS or motor drivers.
"""
import argparse
import base64
import binascii
import copy
import faulthandler
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
import math
from pathlib import Path
import signal
import tempfile
import threading
import time

MODEL_ID = 'qwen3-vl-4b-instruct-geniex-q4_0-qcs8550'
MAX_BODY = 8_000_000


def prepare_request(payload, directory):
    """Accept bounded text/data images only; never fetch a URL from a request."""
    if not isinstance(payload, dict) or payload.get('model') != MODEL_ID:
        raise ValueError('unsupported model')
    if payload.get('stream', False) is not False:
        raise ValueError('streaming is not supported')
    limit = payload.get('max_tokens', 192)
    if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= 192:
        raise ValueError('max_tokens must be between 1 and 192')
    temperature = payload.get('temperature', .1)
    if (isinstance(temperature, bool) or not isinstance(temperature, (int, float))
            or not math.isfinite(temperature) or not 0 <= temperature <= 2):
        raise ValueError('invalid temperature')
    messages = copy.deepcopy(payload.get('messages'))
    if not isinstance(messages, list) or not 1 <= len(messages) <= 8:
        raise ValueError('messages must contain 1 to 8 entries')
    images, text_length = [], 0
    for message in messages:
        if not isinstance(message, dict) or message.get('role') not in ('system', 'user', 'assistant'):
            raise ValueError('invalid message role')
        content = message.get('content')
        if isinstance(content, str):
            text_length += len(content)
            continue
        if not isinstance(content, list) or not 1 <= len(content) <= 4:
            raise ValueError('invalid message content')
        for item in content:
            if not isinstance(item, dict):
                raise ValueError('invalid content item')
            if item.get('type') == 'text' and isinstance(item.get('text'), str):
                text_length += len(item['text'])
            elif item.get('type') == 'image_url':
                value = item.get('image_url')
                url = value.get('url') if isinstance(value, dict) else None
                if not isinstance(url, str) or not url.startswith(('data:image/png;base64,', 'data:image/jpeg;base64,')):
                    raise ValueError('only inline PNG/JPEG images are supported')
                if images or len(url) > MAX_BODY - 10000:
                    raise ValueError('at most one bounded image is supported')
                try:
                    raw = base64.b64decode(url.split(',', 1)[1], validate=True)
                except (ValueError, binascii.Error) as exc:
                    raise ValueError('invalid image base64') from exc
                from PIL import Image
                import io
                try:
                    with Image.open(io.BytesIO(raw)) as image:
                        if image.format not in ('PNG', 'JPEG') or not (
                                1 <= image.width <= 640 and 1 <= image.height <= 640
                                and image.width * image.height <= 307200):
                            raise ValueError('image exceeds the X1 visual token budget')
                        image.verify()
                except (OSError, SyntaxError) as exc:
                    raise ValueError('invalid image') from exc
                path = Path(directory) / 'image'
                path.write_bytes(raw)
                images.append(str(path))
                item.clear()
                item.update(type='image', image=str(path))
            else:
                raise ValueError('unsupported message content')
        content.sort(key=lambda item: item['type'] != 'image')
    if not 1 <= text_length <= 8000:
        raise ValueError('text length must be between 1 and 8000')
    return messages, images, limit, max(.1, temperature)


class Busy(RuntimeError):
    pass


class Engine:
    def __init__(self, model=None, *, factory=None):
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='vlm-owner')
        try:
            self.model = self.executor.submit(factory).result() if factory else model
        except BaseException:
            self.executor.shutdown(wait=True)
            raise
        self.lock = threading.Lock()
        self.failed = False

    def _generate(self, messages, images, limit, temperature):
        self.model.reset()
        prompt = self.model.tokenizer.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True)
        return self.model.generate(prompt, images=images, max_new_tokens=limit,
                                   temperature=temperature, top_k=1, seed=42)

    def close(self):
        try:
            self.executor.submit(getattr(self.model, 'close', lambda: None)).result()
        finally:
            self.executor.shutdown(wait=True)

    def complete(self, payload):
        if not self.lock.acquire(blocking=False):
            raise Busy('inference is busy; retry after the active request finishes')
        try:
            if self.failed:
                raise RuntimeError('runtime failed; restart the inference service')
            with tempfile.TemporaryDirectory(prefix='x1-vlm-') as directory:
                messages, images, limit, temperature = prepare_request(payload, directory)
                started = time.monotonic()
                try:
                    print(json.dumps({'vlm_request': 'begin', 'images': len(images),
                                      'max_tokens': limit}), flush=True)
                    output = self.executor.submit(
                        self._generate, messages, images, limit, temperature).result()
                    print(json.dumps({'vlm_request': 'complete',
                                      'seconds': time.monotonic() - started}), flush=True)
                except Exception:
                    self.failed = True
                    raise
                profile = output.profile
                reason = 'stop' if profile.stop_reason == 'eos' else 'length'
                return {'model': MODEL_ID, 'object': 'chat.completion',
                        'choices': [{'index': 0, 'message': {'role': 'assistant', 'content': output.text},
                                     'finish_reason': reason}],
                        'usage': {'prompt_tokens': profile.prompt_tokens,
                                  'completion_tokens': profile.generated_tokens,
                                  'total_tokens': profile.prompt_tokens + profile.generated_tokens},
                        'x1': {'device': 'HTP0', 'cpu_assistance': True,
                               'elapsed_s': time.monotonic() - started}}
        finally:
            self.lock.release()


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def log_message(self, *_):
        pass

    def reply(self, status, value):
        raw = json.dumps(value, ensure_ascii=False).encode()
        try:
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def do_GET(self):
        if self.path not in ('/healthz', '/v1/models'):
            return self.reply(404, {'error': 'not found'})
        if self.server.engine.failed:
            return self.reply(503, {'error': 'runtime failed; restart required'})
        if self.path == '/v1/models':
            return self.reply(200, {'object': 'list', 'data': [{'id': MODEL_ID, 'object': 'model'}]})
        self.reply(200, {'kind': 'vlm', 'model': MODEL_ID, 'device': 'HTP0',
                         'cpu_assistance': True, 'busy': self.server.engine.lock.locked()})

    def do_POST(self):
        if self.path != '/v1/chat/completions':
            return self.reply(404, {'error': 'not found'})
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= MAX_BODY:
                raise ValueError('invalid body length')
            payload = json.loads(self.rfile.read(length))
            self.reply(200, self.server.engine.complete(payload))
        except Busy as exc:
            self.reply(503, {'error': str(exc)})
        except (ValueError, TypeError, KeyError) as exc:
            self.reply(400, {'error': str(exc)})
        except Exception:
            logging.exception('VLM request failed')
            self.reply(500, {'error': 'local VLM inference failed; inspect the service log'})


def main():
    faulthandler.enable(all_threads=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    if config.get('model_id') != MODEL_ID:
        raise ValueError('unsupported model configuration')
    from geniex import AutoModelForVision2Seq
    from PIL import Image  # Verify transport dependency before readiness.
    def create_model():
        model = AutoModelForVision2Seq.from_pretrained(
            config['model_path'], mmproj_path=config['mmproj_path'],
            device_map='llama_cpp:HTP0', n_ctx=2048, n_threads=2, n_threads_batch=2,
            n_batch=256, n_ubatch=128)
        try:
            with tempfile.TemporaryDirectory(prefix='x1-vlm-warmup-') as directory:
                path = str(Path(directory)/'blank.png')
                Image.new('RGB', (640, 480), (127, 127, 127)).save(path)
                messages = [{'role': 'user', 'content': [
                    {'type': 'image', 'image': path},
                    {'type': 'text', 'text': '图中有物体吗？只回答有或无。'}]}]
                prompt = model.tokenizer.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
                output = model.generate(prompt, images=[path], max_new_tokens=8,
                                        temperature=.1, top_k=1, seed=42)
                if not output.text.strip():
                    raise RuntimeError('visual warmup returned no output')
                model.reset()
            print('HTP0 visual warmup complete', flush=True)
            return model
        except BaseException:
            model.close()
            raise
    engine = Engine(factory=create_model)
    try:
        server = ThreadingHTTPServer(('127.0.0.1', 18888), Handler)
    except BaseException:
        engine.close()
        raise
    server.engine = engine
    def stop(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, stop)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        engine.close()


if __name__ == '__main__':
    main()
