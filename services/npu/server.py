"""Bounded loopback workers for the verified X1 NPU artifacts."""

import argparse
import base64
import binascii
from http.server import BaseHTTPRequestHandler, HTTPServer
import io
import json
from pathlib import Path
import signal
import time

import numpy as np

from services.act.protocol import validate_actions, validate_request


def decode_bytes(value, limit):
    if not isinstance(value, str) or len(value) > (limit + 2) // 3 * 4:
        raise ValueError('invalid encoded input size')
    try:
        raw = base64.b64decode(value, validate=True)
    except (ValueError, binascii.Error) as error:
        raise ValueError('invalid base64') from error
    if not raw or len(raw) > limit:
        raise ValueError('invalid decoded input size')
    return raw


def decode_image(value, expected_size=None):
    from PIL import Image
    raw = decode_bytes(value, 2_000_000)
    with Image.open(io.BytesIO(raw)) as image:
        if expected_size and image.size != expected_size:
            raise ValueError('image size does not match the checkpoint')
        if not 0 < image.width <= 1920 or not 0 < image.height <= 1080:
            raise ValueError('image dimensions out of bounds')
        return np.asarray(image.convert('RGB')).copy()


class Act:
    def __init__(self, model):
        from PIL import Image  # Fail before advertising readiness if transport dependencies are absent.
        import onnxruntime as ort
        import onnxruntime_qnn as qnn
        ort.register_execution_provider_library('QNNExecutionProvider', qnn.get_library_path())
        devices = [d for d in ort.get_ep_devices() if d.ep_name == 'QNNExecutionProvider']
        if not devices:
            raise RuntimeError('NPU was not discovered; CPU fallback is forbidden')
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        options.add_session_config_entry('session.disable_cpu_ep_fallback', '1')
        options.add_provider_for_devices(devices, {
            'backend_path': qnn.get_qnn_htp_path(), 'htp_arch': '73', 'soc_model': '603',
            'enable_htp_fp16_precision': '1', 'htp_performance_mode': 'burst'})
        self.session = ort.InferenceSession(str(model), sess_options=options)
        actual = {t.name: (t.shape, t.type) for t in self.session.get_inputs()}
        if actual != {'image': ([1, 3, 480, 640], 'tensor(float)'),
                      'state': ([1, 6], 'tensor(float)')}:
            raise ValueError('unexpected ACT input contract')
        outputs = self.session.get_outputs()
        if len(outputs) != 1 or outputs[0].shape != [1, 100, 6]:
            raise ValueError('unexpected ACT output contract')
        self.run_options = ort.RunOptions()
        self.run_options.add_run_config_entry('qnn.perf_mode', 'burst')
        self.infer(np.zeros((1, 3, 480, 640), np.float32), np.zeros((1, 6), np.float32))

    def infer(self, image, state):
        result = self.session.run(None, {'image': image, 'state': state}, self.run_options)[0]
        if result.shape != (1, 100, 6) or not np.isfinite(result).all():
            raise RuntimeError('invalid ACT output')
        return result

    def predict(self, payload):
        request = validate_request(payload, image_fields=('wrist_image_base64',))
        rgb = decode_image(request['wrist_image_base64'], (640, 480))
        image = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255
        state = np.asarray([request['state']], np.float32)
        started = time.monotonic()
        rows = self.infer(image, state)[0].tolist()
        return {'action': validate_actions(rows), 'inference_latency_s': time.monotonic()-started}

    def close(self):
        del self.session


class Person:
    def __init__(self, model):
        from PIL import Image
        import aidlite
        import cv2
        from tools.lib.npu_bench import load_qnn
        cv2.setNumThreads(1)
        self.model = load_qnn(model, '236', (
            [[1, 640, 640, 3]], aidlite.DataType.TYPE_FLOAT32,
            [[1, 80, 8400], [1, 4, 8400]], aidlite.DataType.TYPE_FLOAT32))
        self.detect(np.zeros((480, 640, 3), np.uint8))

    def detect(self, rgb):
        import cv2
        from tools.lib.npu_bench import check, person_boxes
        length = max(rgb.shape[:2])
        square = np.zeros((length, length, 3), np.uint8)
        square[:rgb.shape[0], :rgb.shape[1]] = rgb
        tensor = cv2.resize(square, (640, 640)).astype(np.float32)[None] / 255
        check(self.model.set_input_tensor(0, tensor))
        check(self.model.invoke())
        return person_boxes(self.model.get_output_tensor(0), self.model.get_output_tensor(1),
                            rgb.shape, length / 640)

    def predict(self, payload):
        rgb = decode_image(payload.get('image_base64'))
        started = time.monotonic()
        return {'detections': self.detect(rgb), 'inference_latency_s': time.monotonic()-started}

    def close(self):
        self.model.destroy()


class Whisper:
    def __init__(self, model):
        from tools.lib.npu_bench import QnnWhisper
        self.model = QnnWhisper(model)
        # Initialize the host feature extractor before advertising readiness.
        # Its first import/JIT otherwise delays the first spoken command by seconds.
        self.model.transcribe(np.zeros(16000, np.float32), 64)

    def predict(self, payload):
        from tools.lib.npu_bench import asr_candidate_passes
        if payload.get('sample_rate') != 16000:
            raise ValueError('16 kHz mono float32 little-endian audio required')
        raw = decode_bytes(payload.get('audio_base64'), 480000 * 4)
        if len(raw) % 4:
            raise ValueError('unaligned audio')
        audio = np.frombuffer(raw, dtype='<f4')
        if not np.isfinite(audio).all() or np.max(np.abs(audio)) > 1:
            raise ValueError('audio must be finite and normalized')
        result = self.model.transcribe(audio, 64)
        result['accepted'] = asr_candidate_passes(result)
        return result

    def close(self):
        self.model.close()


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def respond(self, status, value):
        body = json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path not in ('/healthz', '/health'):
            self.respond(404, {'error': 'not found'})
            return
        self.respond(200, {'status': 'ok', 'kind': self.server.kind, 'device': 'qnn-htp',
                           'cpu_fallback': False, 'load_s': self.server.load_s,
                           'image_features': ['observation.images.wrist'] if self.server.kind == 'act' else []})

    def do_POST(self):
        if self.path not in ('/predict', '/reset'):
            self.respond(404, {'error': 'not found'})
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4_000_000:
                raise ValueError('invalid request size')
            raw = self.rfile.read(length)
            if len(raw) != length:
                raise ValueError('incomplete request')
            payload = json.loads(raw)
            if not isinstance(payload, dict):
                raise ValueError('JSON object required')
            # Same-host monotonic deadline includes waiting in the server queue.
            deadline = payload.get('deadline_monotonic')
            if deadline is not None and (not isinstance(deadline, (int, float))
                                        or not np.isfinite(deadline) or time.monotonic() >= deadline):
                raise ValueError('expired or invalid request deadline')
            result = ({'status': 'ok'} if self.path == '/reset'
                      else self.server.runtime.predict(payload))
            if deadline is not None and time.monotonic() >= deadline:
                self.respond(408, {'error': 'inference exceeded deadline; result discarded'})
            else:
                self.respond(200, result)
        except (ValueError, TypeError, KeyError) as error:
            self.respond(400, {'error': str(error)})
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
        except Exception as error:
            print(f'inference failed: {type(error).__name__}: {error}', flush=True)
            self.respond(500, {'error': 'NPU inference failed; no fallback'})

    def log_message(self, *_args):
        pass


class Server(HTTPServer):
    # One in-flight inference, small socket backlog; no unbounded inference threads.
    request_queue_size = 1
    allow_reuse_address = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=('act', 'person', 'whisper'), required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--port', type=int, required=True)
    args = parser.parse_args()
    if not args.model.exists() or not 1024 <= args.port <= 65535:
        parser.error('existing local model and unprivileged port required')
    started = time.monotonic()
    runtime = {'act': Act, 'person': Person, 'whisper': Whisper}[args.kind](args.model)
    stopping = False
    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        with Server(('127.0.0.1', args.port), Handler) as server:
            server.runtime, server.kind = runtime, args.kind
            server.load_s = time.monotonic()-started
            server.timeout = 0.5
            print(json.dumps({'ready': args.kind, 'port': args.port, 'load_s': server.load_s}), flush=True)
            while not stopping:
                server.handle_request()
    finally:
        runtime.close()


if __name__ == '__main__':
    main()
