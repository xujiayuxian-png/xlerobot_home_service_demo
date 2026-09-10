import asyncio
import hashlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
import pytest
import yaml

from xlerobot_calibration_tools.workbench import Workbench, application, file_hash


def test_changed_visual_predecessor_is_diagnosed_before_device_start(tmp_path, monkeypatch):
    value = manager(tmp_path, True)
    draft = value.store.draft_components(value.unit)
    draft.mkdir(parents=True)
    (draft / 'servo.yaml').write_text('servo')
    (draft / 'head_camera.yaml').write_text('head-old')
    progress = value.root / 'capture/calibration_work/right_handeye/progress.yaml'
    progress.parent.mkdir(parents=True)
    progress.write_text(yaml.safe_dump({
        'unit_id': value.unit, 'servo_sha256': hashlib.sha256(b'servohead-old').hexdigest(),
        'pose_sha256': file_hash(REPO / 'ros2_ws/src/xlerobot_calibration_tools/config/right_handeye_poses.yaml'),
    }))
    assert value.resume_issue('right_handeye') == ''
    (draft / 'head_camera.yaml').write_text('head-new')
    issue = value.resume_issue('right_handeye')
    assert '前序' in issue
    monkeypatch.setattr(value, 'snapshot', lambda: {'stages': {'right_handeye': {'missing': [], 'resume_issue': issue}}})
    check = AsyncMock()
    monkeypatch.setattr(value, '_check_devices', check)
    with pytest.raises(ValueError, match='前序'):
        asyncio.run(value.start('right_handeye'))
    check.assert_not_called()
    assert progress.exists()


@pytest.mark.parametrize('stage', ['servo', 'leader'])
def test_restart_archives_via_existing_fresh_capture_and_preserves_stage(tmp_path, stage):
    async def check():
        value = manager(tmp_path, True)
        value.stage = stage
        order = []
        async def stop():
            order.append('stop')
        async def start(selected, fresh=False):
            order.append((selected, fresh))
            return {'ok': True}
        value.stop, value.start = stop, start
        async with TestClient(TestServer(application(value))) as client:
            response = await client.post('/workbench-api/restart-servo', json={'confirmed': True, 'stage': stage})
            assert response.status == 200
            assert order == ['stop', (stage, True)]
            response = await client.post('/workbench-api/restart-servo', json={'confirmed': True, 'stage': 'right_handeye'})
            assert response.status == 409
    asyncio.run(check())


REPO = Path(__file__).resolve().parents[4]


def test_local_documents_follow_selected_language_without_device_start(tmp_path):
    async def check():
        value = manager(tmp_path)
        value.start = AsyncMock()
        async with TestClient(TestServer(application(value))) as client:
            for language, directory in [('zh', 'zh-CN'), ('en', 'en')]:
                response = await client.get(f'/workbench-docs/calibration-versions.md?lang={language}')
                assert response.status == 200
                assert await response.text() == (REPO / 'docs' / directory / 'calibration-versions.md').read_text()
            response = await client.get('/workbench-docs/calibration-versions.md?lang=../../')
            assert response.status == 400
            response = await client.get('/workbench-docs/not-allowed.md?lang=en')
            assert response.status == 404
        value.start.assert_not_called()
    asyncio.run(check())


def manager(tmp_path, hardware=False):
    config = tmp_path / 'local.yaml'
    config.write_text(yaml.safe_dump({'robot': {'devices': {}}, 'schema': 'xlerobot_demo/v1'}))
    return Workbench(REPO, config, tmp_path / 'state', 'test-unit', hardware)


def test_status_is_readonly_and_no_hardware_start(tmp_path, monkeypatch):
    value = manager(tmp_path)
    def unexpected(*args, **kwargs):
        pytest.fail('read-only status opened a process or device')
    monkeypatch.setattr('subprocess.run', unexpected)
    assert value.snapshot()['session']['stage'] is None
    assert not value.root.exists()
    with pytest.raises(ValueError, match='--hardware'):
        asyncio.run(value.start('servo'))
    assert not value.root.exists()


def test_hover_result_survives_device_stop_and_marks_old_running_session_interrupted(tmp_path):
    value = manager(tmp_path)
    assert value.hover_result() == {'report': None, 'stale': False}
    result = value.root / 'workbench/hover_latest.yaml'
    result.parent.mkdir(parents=True)
    result.write_text(yaml.safe_dump({'id': 'run', 'status': 'RUNNING', 'device_generation': 'old',
                                    'predecessors': {'servo': 'old'}, 'arrivals': []}))
    reply = value.hover_result()
    assert reply['report']['status'] == 'INTERRUPTED' and reply['stale']
    assert yaml.safe_load(result.read_text())['status'] == 'RUNNING'  # GET did not write anything.


def test_hover_report_http_without_device_session(tmp_path):
    async def check():
        value = manager(tmp_path)
        async with TestClient(TestServer(application(value))) as client:
            response = await client.get('/workbench-api/hover-result')
            assert response.status == 200
            assert (await response.json())['report'] is None
            response = await client.post('/workbench-api/auto-hover', json={'confirmed': True})
            assert response.status == 409  # hardware disabled; never opens devices
            assert value.process is None
    asyncio.run(check())


def test_auto_hover_handoff_waits_for_readiness_and_submits_one_robot_side_job(tmp_path):
    async def check():
        requests = []
        polls = []
        child = web.Application()
        async def status(request):
            polls.append(True)
            return web.json_response({'ready': len(polls) >= 2})
        async def auto(request):
            requests.append(await request.json())
            return web.json_response({'accepted': True}, status=202)
        child.router.add_get('/api/v1/hover/status', status)
        child.router.add_post('/api/v1/hover/auto', auto)
        async with TestServer(child) as server:
            value = manager(tmp_path, True)
            value.process = SimpleNamespace(returncode=None)
            value.stage, value.hover_port = 'hover', server.port
            value.stop = AsyncMock()
            async with TestClient(TestServer(application(value))) as client:
                response = await client.post('/workbench-api/auto-hover', json={'confirmed': False})
                assert response.status == 409 and not polls
                response = await client.post('/workbench-api/auto-hover', json={'confirmed': True})
                assert response.status == 200 and (await response.json())['accepted']
                assert requests == [{'confirmed': True}] and len(polls) == 2
    asyncio.run(check())


def test_initial_pose_wait_is_visible_until_controller_spawner_continues(tmp_path):
    value = manager(tmp_path)
    value.process = SimpleNamespace(returncode=None)
    value.log_path = tmp_path / 'capture.log'
    value.log_path.write_text('reposition joints within limits: right_arm_shoulder_lift. State feedback remains available.\n')
    assert 'right_arm_shoulder_lift' in value.startup_notice()
    with value.log_path.open('a') as stream:
        stream.write('Measured pose ready; starting standard controller spawner\n')
    assert value.startup_notice() == ''


def test_start_never_stops_existing_owner(tmp_path):
    value = manager(tmp_path, True)
    value.process = SimpleNamespace(returncode=None)
    with pytest.raises(ValueError, match='end the existing'):
        asyncio.run(value.start('leader'))


def test_stage_prerequisites_and_unknown_stage(tmp_path):
    value = manager(tmp_path, True)
    for stage in ('head_camera', 'right_handeye', 'hover'):
        with pytest.raises(ValueError, match='passing drafts'):
            asyncio.run(value.start(stage))
    with pytest.raises(ValueError, match='unknown'):
        asyncio.run(value.start('../servo'))


def test_start_uses_single_existing_capture_command(tmp_path, monkeypatch):
    value = manager(tmp_path, True)
    monkeypatch.setattr(value, '_check_devices', lambda stage: None)
    spawn = AsyncMock(return_value=SimpleNamespace(returncode=None, pid=12345))
    monkeypatch.setattr(asyncio, 'create_subprocess_exec', spawn)
    asyncio.run(value.start('leader'))
    args = spawn.call_args.args
    assert args[:3] == (str(REPO / 'tools/calibrate'), 'capture', 'servo')
    assert '--leader' in args and '--resume' in args and '--hardware' in args
    assert args[args.index('--web-host') + 1] == '127.0.0.1'
    assert spawn.call_args.kwargs['start_new_session'] is True
    assert value.snapshot()['session']['stage'] == 'leader'
    value._release()


def test_failed_device_check_releases_owner_without_launch(tmp_path, monkeypatch):
    value = manager(tmp_path, True)
    def fail(stage):
        raise ValueError('device already open')
    monkeypatch.setattr(value, '_check_devices', fail)
    spawn = AsyncMock()
    monkeypatch.setattr(asyncio, 'create_subprocess_exec', spawn)
    with pytest.raises(ValueError, match='already open'):
        asyncio.run(value.start('servo'))
    assert value.session_lock is None
    spawn.assert_not_called()


def test_cross_workbench_device_ownership(tmp_path):
    first, second = manager(tmp_path, True), manager(tmp_path, True)
    first._claim()
    try:
        with pytest.raises(ValueError, match='another workbench'):
            second._claim()
    finally:
        first._release()
    second._claim()
    second._release()


def test_changed_predecessor_marks_saved_result_stale(tmp_path, monkeypatch):
    value = manager(tmp_path)
    hashes = {'servo': 's1', 'head_camera': 'h1', 'right_handeye': 'r1'}
    monkeypatch.setattr(value, 'hashes', lambda: dict(hashes))
    path = value.root / 'workbench/hover.yaml'
    path.parent.mkdir(parents=True)
    path.write_text(yaml.safe_dump({'predecessors': dict(hashes)}))
    assert value.snapshot()['stages']['hover']['freshness'] == 'current'
    hashes['servo'] = 's2'
    assert value.snapshot()['stages']['hover']['freshness'] == 'needs_validation'
    assert path.is_file()  # Never delete old evidence.


def test_http_readonly_shell_and_confirmation_guards(tmp_path):
    async def check():
        value = manager(tmp_path)
        async with TestClient(TestServer(application(value))) as client:
            response = await client.get('/')
            assert response.status == 200
            assert 'xlerobot-workbench' in await response.text()
            assert (await client.get('/workbench-api/status')).status == 200
            assert (await client.get('/api/v1/bootstrap')).status == 503
            assert (await client.post('/workbench-api/start', json={'stage': 'servo'})).status == 409
            assert (await client.post('/workbench-api/start', json={'stage': 'servo', 'confirmed': True})).status == 409
            assert (await client.post('/workbench-api/start', json={'confirmed': True},
                                      headers={'Origin': 'https://unrelated.example'})).status == 403
            assert (await client.post('/workbench-api/start', data='confirmed=true')).status == 400
            assert not value.root.exists()
    asyncio.run(check())


def test_apply_refused_during_device_session(tmp_path):
    async def check():
        value = manager(tmp_path)
        value.process = SimpleNamespace(returncode=None)
        value.stop = AsyncMock()
        async with TestClient(TestServer(application(value))) as client:
            response = await client.post('/workbench-api/switch', json={'confirmed': True, 'version': 'old'})
            assert response.status == 409
            assert 'end device session' in await response.text()
    asyncio.run(check())


def test_existing_http_api_and_camera_stream_share_the_workbench_origin(tmp_path):
    async def check():
        child = web.Application()
        async def state(request):
            return web.json_response({'phase': 'IDLE'})
        async def camera(request):
            response = web.StreamResponse(headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame'})
            await response.prepare(request)
            await response.write(b'--frame\r\n')
            await response.write(b'image-test-data')
            await response.write_eof()
            return response
        child.router.add_get('/api/v1/calibrations/head/status', state)
        child.router.add_get('/api/v1/cameras/head/stream', camera)
        async with TestServer(child) as upstream:
            value = manager(tmp_path)
            value.process = SimpleNamespace(returncode=None)
            value.child_port = upstream.port
            value.stop = AsyncMock()
            async with TestClient(TestServer(application(value))) as client:
                response = await client.get('/api/v1/calibrations/head/status')
                assert await response.json() == {'phase': 'IDLE'}
                response = await client.get('/api/v1/cameras/head/stream')
                assert response.headers['Content-Type'].startswith('multipart/')
                assert await response.read() == b'--frame\r\nimage-test-data'
    asyncio.run(check())
