"""The X1 UI cannot reactivate engineering image/TF consumers via HTTP."""
import asyncio
from types import SimpleNamespace

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
import rclpy

from xlerobot_hmi.operator_console import ConsoleApplication, OperatorConsoleNode


def test_demo_routes_reject_camera_mapping_and_manual_control(monkeypatch):
    async def health(_request):
        return web.json_response({'demo_only': True})

    app = ConsoleApplication(SimpleNamespace(x1_low_load=True))
    app.health = health
    monkeypatch.setattr(app, '_static_root', lambda: None)

    async def exercise():
        async with TestClient(TestServer(app.build())) as client:
            assert (await client.get('/api/v1/health')).status == 200
            for path in ('cameras/head/stream', 'mapping/state', 'teleop/base', 'sites'):
                assert (await client.get('/api/v1/' + path)).status == 404
            assert (await client.post('/api/v1/operator/preset')).status == 404
    asyncio.run(exercise())


def test_demo_constructor_has_no_image_map_tf_or_teleop_endpoints(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '196')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=['--ros-args', '-p', 'x1_low_load:=true', '-p',
                    f'artifact_root:={tmp_path}/artifacts', '-p',
                    f'task_history_root:={tmp_path}/history'])
    node = None
    try:
        node = OperatorConsoleNode()
        assert node.tf_buffer is None and node.tf_listener is None
        assert node._teleop_publisher is None
        assert not hasattr(node, 'maintenance_preset_client')
        topics = {sub.topic_name for sub in node.subscriptions}
        assert not topics.intersection({'/tf', '/tf_static', '/map', '/scan', '/plan',
            '/xlerobot/d455/color/image_raw', '/right_wrist_camera/image_raw'})
        assert '/camera/health' in topics
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
