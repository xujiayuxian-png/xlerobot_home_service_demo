#!/usr/bin/env python3
"""Authenticated, bounded LeRobot ACT inference service for the GPU host."""

from __future__ import annotations

import argparse
import base64
import binascii
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import os
from pathlib import Path
import threading
import time

from .protocol import authorized, ProtocolError, validate_actions, validate_request


class ActRuntime:
    """Load one checkpoint and serialize access to its policy runtime."""

    def __init__(self, checkpoint: Path, *, device: str):
        import numpy as np
        from PIL import Image
        import torch
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors

        self.np = np
        self.Image = Image
        self.torch = torch
        self.checkpoint = checkpoint.resolve()
        self.device = device
        self.lock = threading.Lock()
        self.policy = ACTPolicy.from_pretrained(
            self.checkpoint,
            local_files_only=True,
        )
        self.policy.eval()
        self.policy.to(device)
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config,
            pretrained_path=str(self.checkpoint),
        )
        self.image_shapes = {
            key: tuple(feature.shape)
            for key, feature in self.policy.config.input_features.items()
            if key.startswith('observation.images.')
        }
        supported = {
            'observation.images.head',
            'observation.images.wrist',
        }
        if not self.image_shapes or not set(self.image_shapes) <= supported:
            raise RuntimeError(
                f'checkpoint image contract is {sorted(self.image_shapes)}, '
                f'expected a nonempty subset of {sorted(supported)}'
            )

    def _decode_image(self, encoded: str, feature: str):
        try:
            raw = base64.b64decode(encoded, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise ProtocolError(f'{feature} is not valid base64') from exc
        if not raw or len(raw) > 2_000_000:
            raise ProtocolError(f'{feature} encoded image size is invalid')
        try:
            image = self.Image.open(io.BytesIO(raw))
            image.verify()
            image = self.Image.open(io.BytesIO(raw)).convert('RGB')
        except Exception as exc:
            raise ProtocolError(f'{feature} is not a valid image') from exc
        channels, height, width = self.image_shapes[feature]
        if channels != 3 or image.size != (width, height):
            raise ProtocolError(
                f'{feature} must be RGB {width}x{height}, '
                f'got {image.size[0]}x{image.size[1]}'
            )
        array = self.np.asarray(image, dtype=self.np.uint8)
        return self.torch.from_numpy(array.copy()).permute(2, 0, 1).float() / 255.0

    def predict(self, payload: dict) -> tuple[list[list[float]], float]:
        """Preprocess, infer, postprocess, and return physical joint units."""
        payload_keys = {
            feature: f'{feature.rsplit(".", 1)[-1]}_image_base64'
            for feature in self.image_shapes
        }
        request = validate_request(
            payload,
            image_fields=tuple(payload_keys.values()),
        )
        batch = {
            feature: self._decode_image(request[payload_key], feature)
            for feature, payload_key in payload_keys.items()
        }
        batch['observation.state'] = self.torch.tensor(
            request['state'], dtype=self.torch.float32
        )
        started = time.monotonic()
        with self.lock, self.torch.inference_mode():
            processed = self.preprocessor(batch)
            actions = self.policy.predict_action_chunk(processed)
            actions = self.postprocessor(actions)
        rows = actions.detach().cpu().squeeze(0).tolist()
        return validate_actions(rows), time.monotonic() - started

    def reset(self):
        """Reset optional policy-local queues under the serialization lock."""
        with self.lock:
            reset = getattr(self.policy, 'reset', None)
            if callable(reset):
                reset()


class ActServer(ThreadingHTTPServer):
    """HTTP server carrying immutable runtime and authentication settings."""

    daemon_threads = True

    def __init__(
        self,
        address,
        runtime: ActRuntime,
        *,
        token: str,
        allow_unauthenticated: bool,
        max_request_bytes: int,
    ):
        super().__init__(address, ActHandler)
        self.runtime = runtime
        self.token = token
        self.allow_unauthenticated = allow_unauthenticated
        self.max_request_bytes = max_request_bytes
        self.started_at = time.monotonic()


class ActHandler(BaseHTTPRequestHandler):
    """Serve health, reset, and inference without exposing tracebacks."""

    server: ActServer

    def _json(self, status: HTTPStatus, payload: dict):
        body = json.dumps(payload, separators=(',', ':')).encode('utf-8')
        self.send_response(status.value)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _authorized(self) -> bool:
        return self.server.allow_unauthenticated or authorized(
            self.headers.get('Authorization'), self.server.token
        )

    def _payload(self):
        value = self.headers.get('Content-Length')
        try:
            length = int(value) if value is not None else -1
        except ValueError as exc:
            raise ProtocolError('invalid Content-Length') from exc
        if length <= 0 or length > self.server.max_request_bytes:
            raise ProtocolError('request body size is outside the configured limit')
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ProtocolError('request body is not valid UTF-8 JSON') from exc

    def do_GET(self):
        if self.path != '/healthz':
            self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})
            return
        runtime = self.server.runtime
        self._json(
            HTTPStatus.OK,
            {
                'status': 'ok',
                'service': 'xlerobot-act-inference',
                'device': runtime.device,
                'checkpoint': runtime.checkpoint.name,
                'image_features': sorted(runtime.image_shapes),
                'authenticated_predict': not self.server.allow_unauthenticated,
                'uptime_s': round(time.monotonic() - self.server.started_at, 3),
            },
        )

    def do_POST(self):
        if self.path not in {'/predict', '/reset'}:
            self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})
            return
        if not self._authorized():
            self._json(HTTPStatus.UNAUTHORIZED, {'error': 'unauthorized'})
            return
        try:
            if self.path == '/reset':
                self.server.runtime.reset()
                self._json(HTTPStatus.OK, {'status': 'ok'})
                return
            actions, latency_s = self.server.runtime.predict(self._payload())
            self._json(
                HTTPStatus.OK,
                {'action': actions, 'inference_latency_s': latency_s},
            )
        except ProtocolError as exc:
            self._json(HTTPStatus.BAD_REQUEST, {'error': str(exc)})
        except Exception:
            self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {'error': 'inference failed'})

    def log_message(self, format_string, *args):
        print(f'{self.client_address[0]} - {format_string % args}', flush=True)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--device', default='cuda')
    parser.add_argument('--token-env', default='XLEROBOT_ACT_TOKEN')
    parser.add_argument('--allow-unauthenticated', action='store_true')
    parser.add_argument('--max-request-bytes', type=int, default=8_000_000)
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.checkpoint.is_dir():
        raise SystemExit(f'checkpoint directory does not exist: {args.checkpoint}')
    if not 1 <= args.port <= 65535 or args.max_request_bytes <= 0:
        raise SystemExit('port and request limit must be positive and bounded')
    token = os.environ.get(args.token_env, '')
    if not args.allow_unauthenticated and not token:
        raise SystemExit(
            f'{args.token_env} must be set unless unauthenticated mode is explicit'
        )
    runtime = ActRuntime(args.checkpoint, device=args.device)
    server = ActServer(
        (args.host, args.port),
        runtime,
        token=token,
        allow_unauthenticated=args.allow_unauthenticated,
        max_request_bytes=args.max_request_bytes,
    )
    print(
        f'ACT inference ready on {args.host}:{args.port}; '
        f'device={args.device}; authenticated={not args.allow_unauthenticated}',
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()

