import base64
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import protocol  # noqa: E402
import server  # noqa: E402
import sam2_prompt  # noqa: E402


def encoded_payload(width=4, height=3):
    color = np.zeros((height, width, 3), dtype=np.uint8)
    ok, png = cv2.imencode('.png', color)
    assert ok
    depth = np.full((height, width), 500, dtype='<u2')
    mask = np.ones((height, width), dtype=np.uint8)
    return {
        'width': width,
        'height': height,
        'depth_dtype': 'uint16',
        'color_b64': base64.b64encode(png.tobytes()).decode('ascii'),
        'depth_b64': base64.b64encode(depth.tobytes()).decode('ascii'),
        'mask_b64': base64.b64encode(mask.tobytes()).decode('ascii'),
        'intrinsics': {
            'fx': 100.0, 'fy': 100.0, 'cx': 2.0, 'cy': 1.5,
            'depth_scale': 1000.0,
        },
    }


def test_protocol_round_trip_is_exact_size():
    payload = encoded_payload()
    assert protocol.color_bgr(payload).shape == (3, 4, 3)
    assert protocol.depth_u16(payload).shape == (3, 4)
    assert protocol.mask(payload, width=4, height=3).all()


def test_protocol_rejects_truncated_depth():
    payload = encoded_payload()
    payload['depth_b64'] = base64.b64encode(b'bad').decode('ascii')
    with pytest.raises(protocol.RequestError, match='byte count'):
        protocol.depth_u16(payload)


def test_health_names_each_backend(monkeypatch):
    monkeypatch.setattr(server.sam2_prompt, 'health', lambda: (True, 'ok'))
    monkeypatch.setattr(server.gpd_wrapper, 'health', lambda: (False, 'missing'))
    health = server.health_payload()
    assert health['segment_ready'] is True
    assert health['gpd_ready'] is False
    assert health['backends'] == [
        {'backend': 'gpd', 'ready': False, 'reason': 'missing'}
    ]


def test_gpd_failure_never_returns_a_centroid_candidate(monkeypatch):
    payload = encoded_payload(width=12, height=10)
    payload.update({
        'backend': 'gpd',
        'top_k': 3,
        'objects': [{
            'object_id': 0,
            'class': 'ball',
            'bbox': [0, 0, 11, 9],
            'mask_b64': payload['mask_b64'],
        }],
    })
    monkeypatch.setattr(server.gpd_wrapper, 'health', lambda: (True, 'ok'))
    monkeypatch.setattr(
        server.gpd_wrapper,
        'infer',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            server.gpd_wrapper.GpdError('no candidates')
        ),
    )
    with pytest.raises(server.ServiceUnavailable, match='no candidates'):
        server.infer_objects(payload)


def test_sam2_checkpoint_is_snapshot_pinned_and_checksum_verified(tmp_path, monkeypatch):
    checkpoint = tmp_path / 'sam2.pt'
    checkpoint.write_bytes(b'public-test-checkpoint')
    digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    monkeypatch.setenv('SAM2_CHECKPOINT_PATH', str(checkpoint))
    monkeypatch.setattr(sam2_prompt, 'MODEL_SHA256', digest)
    monkeypatch.setattr(sam2_prompt, '_checkpoint_path', None)
    assert sam2_prompt.MODEL_REVISION == (
        'de431c4043854a71d8101e17995dfe596bf101a5'
    )
    assert sam2_prompt.resolve_checkpoint(allow_download=False) == checkpoint

    monkeypatch.setattr(sam2_prompt, '_checkpoint_path', None)
    monkeypatch.setattr(sam2_prompt, 'MODEL_SHA256', '0' * 64)
    with pytest.raises(RuntimeError, match='SHA-256 mismatch'):
        sam2_prompt.resolve_checkpoint(allow_download=False)
