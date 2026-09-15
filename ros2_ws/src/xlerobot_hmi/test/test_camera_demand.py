import asyncio
import threading
from types import SimpleNamespace

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
import numpy as np
from sensor_msgs.msg import Image

from xlerobot_hmi.operator_console import ConsoleApplication, OperatorConsoleNode


def fake_node():
    created, destroyed = [], []
    node = SimpleNamespace(
        x1_low_load=True, _camera_demand_lock=threading.RLock(),
        _camera_lock=threading.Lock(), _camera_viewers={}, _camera_subscriptions={},
        _camera_generation={}, _camera_frames={}, _camera_last_encoded={},
        parameter=lambda name: 3.0 if name == 'camera_max_fps' else '/' + name,
        destroy_subscription=destroyed.append,
    )

    def subscribe(kind, topic, callback, qos):
        assert qos.depth == 1
        created.append(callback)
        return callback

    node.create_subscription = subscribe
    node._on_image = lambda camera, msg: OperatorConsoleNode._on_image(node, camera, msg)
    node.acquire_camera = lambda camera: OperatorConsoleNode.acquire_camera(node, camera)
    node.release_camera = lambda camera: OperatorConsoleNode.release_camera(node, camera)
    node.camera_frame = lambda camera: OperatorConsoleNode.camera_frame(node, camera)
    return node, created, destroyed


def test_viewers_share_encoder_and_last_disconnect_retires_callbacks():
    node, created, destroyed = fake_node()
    assert not created and not node._camera_frames
    node.acquire_camera('head')
    node.acquire_camera('head')
    assert len(created) == 1
    image = Image(height=2, width=2, step=6, encoding='bgr8', data=np.zeros(12, dtype=np.uint8).tobytes())
    created[0](image)
    assert node.camera_frame('head')[0] == 1
    created[0](image)
    assert node.camera_frame('head')[0] == 1  # One shared rate limit.
    node.release_camera('head')
    assert not destroyed
    node.release_camera('head')
    assert len(destroyed) == 1 and node.camera_frame('head') is None
    node.acquire_camera('head')
    created[0](image)
    assert node.camera_frame('head') is None
    created[1](image)
    assert node.camera_frame('head')[0] == 1
    node.release_camera('head')


def test_http_disconnect_releases_subscription_even_without_camera_frames():
    async def exercise():
        node, created, destroyed = fake_node()
        app = web.Application()
        handler = SimpleNamespace(node=node)
        app.router.add_get('/{camera_id}', lambda request: ConsoleApplication.camera_stream(handler, request))
        async with TestClient(TestServer(app)) as client:
            response = await client.get('/head')
            assert len(created) == 1
            response.close()
            for _ in range(40):
                if destroyed:
                    break
                await asyncio.sleep(0.02)
            assert len(destroyed) == 1
            assert not node._camera_viewers
    asyncio.run(exercise())
