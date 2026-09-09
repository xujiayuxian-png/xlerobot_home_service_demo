"""Construct a real ROS node on an isolated domain; never open motor devices."""
import asyncio
import numpy as np
import rclpy
import yaml
from aiohttp.test_utils import TestClient, TestServer

from xlerobot_calibration_tools.hover_node import HoverNode


def test_real_node_constructs_and_http_runs_without_hardware(tmp_path, monkeypatch):
    folder = tmp_path / 'units/test-unit/draft/components'
    folder.mkdir(parents=True)
    for name in ('servo', 'head_camera', 'right_handeye'):
        (folder / f'{name}.yaml').write_text(yaml.safe_dump({
            'x': {'matrix': np.eye(4).tolist()}, 'y': {'matrix': np.eye(4).tolist()}}))
    # Solver validation has its own replay tests; this checks ROS construction.
    monkeypatch.setattr('xlerobot_calibration_tools.hover_node.validate_component', lambda *args: {})
    rclpy.init(args=['--ros-args', '-p', f'state_root:={tmp_path}', '-p', 'unit_id:=test-unit'], domain_id=79)
    node = None
    try:
        node = HoverNode()
        assert node.handle is not None  # rclpy's native handle must not be overwritten.
        assert node.motion_handle is None
        assert node.get_parameter('execution_enabled').value is False
        async def check():
            async with TestClient(TestServer(node.app())) as client:
                response = await client.get('/api/v1/hover/status')
                assert response.status == 200
                assert (await response.json())['phase'] == 'IDLE'
                response = await client.post('/api/v1/hover/prepare', json={'confirmed': True})
                assert response.status == 409
                assert 'disabled' in await response.text()
        asyncio.run(check())
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
