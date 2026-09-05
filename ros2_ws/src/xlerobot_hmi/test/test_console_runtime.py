"""Exercise the production gateway scheduler without any robot connections."""
import asyncio
import sys

from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer
from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import Reset

import xlerobot_hmi.operator_console as console


def test_console_keeps_teleop_alive_during_ros_map_updates_and_stops_on_timeout(
    tmp_path, monkeypatch,
):
    # Separate ROS domain, no device nodes, and an in-memory velocity sink.
    monkeypatch.setenv('ROS_DOMAIN_ID', '84')
    monkeypatch.setattr(sys, 'argv', [
        'operator_console', '--ros-args', '-p', 'workspace:=mapping',
        '-p', 'mapping_phase:=build', '-p', 'enable_engineering_tools:=true',
        '-p', f'artifact_root:={tmp_path}/assets',
        '-p', f'task_history_root:={tmp_path}/history',
    ])
    original_node = console.OperatorConsoleNode
    original_executor = console.SingleThreadedExecutor
    executors = []
    nodes = []
    commands = []
    ticks = []
    resets = []

    def make_executor():
        executor = original_executor()
        executors.append(executor)
        return executor

    def make_node():
        node = original_node()
        node._teleop_publisher = type('Sink', (), {
            'publish': staticmethod(lambda value: commands.append(value)),
        })()
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width, grid.info.height = 160, 320
        grid.info.resolution = .03
        grid.data = [0] * (160 * 320)

        def update_map():
            ticks.append(1)
            node._on_map(grid)

        node.create_timer(.1, update_map)

        def reset_map(request, response):
            resets.append(request.pause_new_measurements)
            response.result = Reset.Response.RESULT_SUCCESS
            return response

        node.create_service(Reset, '/slam_toolbox/reset', reset_map)
        nodes.append(node)
        return node

    async def exercise(app):
        async with TestClient(TestServer(app)) as client:
            ws = await client.ws_connect('/api/v1/teleop/base')
            for _ in range(12):
                await ws.send_json({'armed': True, 'linear': .08, 'angular': 0})
                response = await client.get('/api/v1/mapping/state')
                assert response.status == 200
                await response.read()
                await asyncio.sleep(.08)
                assert not ws.closed
            assert len(commands) >= 12
            assert commands[-1].linear.x == .08
            assert len(ticks) >= 3
            # Keep the original 350 ms watchdog: no heartbeat must still stop.
            message = await ws.receive(timeout=2.0)
            assert message.type in {WSMsgType.CLOSE, WSMsgType.CLOSED}
            await ws.close()
            await asyncio.sleep(.05)
            assert commands[-1].linear.x == 0
            assert not nodes[0].teleop_active()
            assert len(executors) == 1
            assert isinstance(executors[0], original_executor)
            response = await client.post('/api/v1/mapping/reset', json={'confirm': True})
            assert response.status == 200
            assert (await response.json())['saved_assets_preserved'] is True
            assert resets == [False]
            assert not nodes[0].manual_action_active()

    monkeypatch.setattr(console, 'OperatorConsoleNode', make_node)
    monkeypatch.setattr(console, 'SingleThreadedExecutor', make_executor)
    monkeypatch.setattr(console.web, 'run_app', lambda app, **_: asyncio.run(exercise(app)))
    console.main()
