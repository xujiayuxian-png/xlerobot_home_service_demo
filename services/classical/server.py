#!/usr/bin/env python3
"""Prompted SAM2 + strict GPD HTTP service for a LAN GPU workstation."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


SERVICE_ROOT = Path(__file__).resolve().parent
if str(SERVICE_ROOT) not in sys.path:
    sys.path.insert(0, str(SERVICE_ROOT))

import gpd_wrapper  # noqa: E402
import protocol  # noqa: E402
import sam2_prompt  # noqa: E402


GPU_LOCK = threading.Lock()
MAX_BODY_BYTES = 96 * 1024 * 1024


class ServiceUnavailable(RuntimeError):
    pass


def segment_prompted(payload: dict[str, Any]) -> dict[str, Any]:
    image = protocol.color_bgr(payload)
    depth = protocol.depth_u16(payload)
    prompts = protocol.object_prompts(payload)
    ready, reason = sam2_prompt.health()
    if not ready:
        raise ServiceUnavailable(f"prompted SAM2 is not ready: {reason}")
    if not GPU_LOCK.acquire(blocking=False):
        raise ServiceUnavailable("GPU service is busy")
    started = time.monotonic()
    try:
        raw = sam2_prompt.segment(image, prompts)
    finally:
        GPU_LOCK.release()
    objects = []
    valid_depth = depth > 0
    for item in raw:
        object_mask = item.pop("mask")
        objects.append({
            **item,
            "valid_depth_pixels": int(np_count(valid_depth & object_mask)),
            "mask_b64": protocol.encode_mask(object_mask),
        })
    return {
        "ok": True,
        "backend": "sam2_prompt",
        "objects": objects,
        "total_s": time.monotonic() - started,
    }


def np_count(mask) -> int:
    # Kept behind a tiny function so server import/health stays easy to test.
    return int(mask.sum())


def infer_objects(payload: dict[str, Any]) -> dict[str, Any]:
    if str(payload.get("backend", "")) != "gpd":
        raise protocol.RequestError("backend must be exactly 'gpd'")
    protocol.color_bgr(payload)  # Validate the shared RGB-D request completely.
    depth = protocol.depth_u16(payload)
    camera = protocol.camera(payload)
    width, height = protocol.dimensions(payload)
    objects = payload.get("objects")
    if not isinstance(objects, list) or len(objects) != 1:
        raise protocol.RequestError("GPD requires exactly one segmented object")
    top_k = int(payload.get("top_k", 10))
    if top_k <= 0 or top_k > 100:
        raise protocol.RequestError("top_k must be in [1, 100]")
    ready, reason = gpd_wrapper.health()
    if not ready:
        raise ServiceUnavailable(reason)
    if not GPU_LOCK.acquire(blocking=False):
        raise ServiceUnavailable("GPU service is busy")
    started = time.monotonic()
    item = objects[0]
    try:
        object_mask = protocol.mask(item, width=width, height=height)
        summary = gpd_wrapper.infer(
            depth, object_mask, camera, top_k=top_k
        )
    except gpd_wrapper.GpdError as exc:
        # Preserve backend identity in the response.  There is intentionally
        # no generated candidate and no centroid substitution here.
        raise ServiceUnavailable(str(exc)) from exc
    finally:
        GPU_LOCK.release()
    return {
        "ok": True,
        "backend": "gpd",
        "objects": [{
            "object_id": int(item.get("object_id", 0)),
            "class": str(item.get("class") or "object"),
            "ok": True,
            **summary,
        }],
        "total_s": time.monotonic() - started,
    }


def health_payload() -> dict[str, Any]:
    segment_ready, segment_reason = sam2_prompt.health()
    gpd_ready, gpd_reason = gpd_wrapper.health()
    return {
        "ok": True,
        "service": "xlerobot_classical",
        "segment_backend": "sam2_prompt",
        "segment_ready": segment_ready,
        "segment_reason": segment_reason,
        "gpd_ready": gpd_ready,
        "gpd_reason": gpd_reason,
        "backends": [{
            "backend": "gpd", "ready": gpd_ready, "reason": gpd_reason,
        }],
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "XLeRobotClassical/1"

    def log_message(self, message: str, *args) -> None:
        sys.stderr.write(
            "%s - - [%s] %s\n" % (
                self.client_address[0], self.log_date_time_string(), message % args
            )
        )

    def _json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _payload(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise protocol.RequestError("invalid Content-Length") from exc
        if length <= 0 or length > MAX_BODY_BYTES:
            raise protocol.RequestError("request body size is invalid")
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise protocol.RequestError("request body must be UTF-8 JSON") from exc
        if not isinstance(payload, dict):
            raise protocol.RequestError("request body must be a JSON object")
        return payload

    def do_GET(self) -> None:
        if urlparse(self.path).path == "/health":
            self._json(200, health_payload())
            return
        self._json(404, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:
        try:
            payload = self._payload()
            path = urlparse(self.path).path
            if path == "/api/segment_prompt":
                self._json(200, segment_prompted(payload))
                return
            if path == "/api/infer_objects":
                self._json(200, infer_objects(payload))
                return
            self._json(404, {"ok": False, "error": "not found"})
        except protocol.RequestError as exc:
            self._json(400, {"ok": False, "error": str(exc)})
        except ServiceUnavailable as exc:
            self._json(503, {"ok": False, "error": str(exc)})
        except Exception as exc:
            self._json(500, {"ok": False, "error": str(exc)})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    arguments = parser.parse_args()
    server = ThreadingHTTPServer((arguments.host, arguments.port), Handler)
    print(json.dumps({
        "listening": f"http://{arguments.host}:{arguments.port}",
        "endpoints": ["/health", "/api/segment_prompt", "/api/infer_objects"],
    }), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
