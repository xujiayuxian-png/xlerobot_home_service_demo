"""One web entry point supervising the existing, mutually exclusive captures.

This process never talks to a motor. Each explicit session owns one existing
ROS launch process group; browsing and inspecting saved results are read-only.
"""
from __future__ import annotations

import argparse
import asyncio
import fcntl
import hashlib
import json
import os
import re
from pathlib import Path
import signal
import socket
import subprocess
import uuid

from aiohttp import ClientError, ClientSession, ClientTimeout, web
import yaml

from .bundle import UnitCalibrationStore, _atomic_yaml
from .public_cli import _defaults, _repo_root
from .workflow import load_yaml


STAGES = ('servo', 'leader', 'head_camera', 'right_handeye', 'hover')
CLIENT = web.AppKey('client', ClientSession)
DEPENDENCIES = {
    'servo': (), 'leader': (), 'head_camera': ('servo',),
    'right_handeye': ('servo', 'head_camera'),
    'hover': ('servo', 'head_camera', 'right_handeye'),
}


def file_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


class Workbench:
    def __init__(self, repo, config, state, unit, hardware=False):
        self.repo, self.config, self.unit = Path(repo), Path(config), unit
        self.store = UnitCalibrationStore(state, repo)
        self.root = self.store.unit_root(unit)
        self.hardware = hardware
        self.process = None
        self.stage = None
        self.generation = ''
        self.child_port = None
        self.hover_port = None
        self.lock = asyncio.Lock()
        self.command_lock = asyncio.Lock()
        self.log_path = None
        self.last_error = ''
        self.session_lock = None
        self.hover_cache = (None, None)

    def hover_result(self):
        path = self.root / 'workbench/hover_latest.yaml'
        if not path.is_file():
            return {'report': None, 'stale': False}
        signature = (path.stat().st_mtime_ns, path.stat().st_size)
        if signature != self.hover_cache[0]:
            self.hover_cache = (signature, load_yaml(path))
        report = self.hover_cache[1]
        if report.get('status') == 'RUNNING' and (self.process is None or self.stage != 'hover'
                or self.process.returncode is not None or report.get('device_generation') != self.generation):
            report = dict(report, status='INTERRUPTED', message='原设备会话已结束；显示已保存的数据，未自动恢复运动')
        stale = bool(report.get('predecessors')) and report['predecessors'] != self.hashes()
        return {'report': report, 'stale': stale}

    def hashes(self):
        return {name: file_hash(self.store.draft_components(self.unit) / f'{name}.yaml')
                for name in ('servo', 'head_camera', 'right_handeye')}

    def resume_issue(self, stage):
        if stage not in ('head_camera', 'right_handeye'):
            return ''
        progress = self.root / 'capture/calibration_work' / stage / 'progress.yaml'
        if not progress.is_file():
            return ''
        saved = load_yaml(progress)
        pose_file = self.repo / 'ros2_ws/src/xlerobot_calibration_tools/config' / f'{stage}_poses.yaml'
        inputs = [self.store.draft_components(self.unit) / f'{n}.yaml' for n in DEPENDENCIES[stage]]
        if not all(p.is_file() for p in inputs):
            return '前序标定草稿缺失，不能恢复旧采样。'
        digest = hashlib.sha256(b''.join(p.read_bytes() for p in inputs)).hexdigest()
        if saved.get('unit_id') != self.unit:
            return '旧采样属于其他机器人，请归档后重新开始。'
        if saved.get('servo_sha256') != digest:
            return '前序舵机或头部相机标定已变化，旧采样不能续采。请归档旧记录并开始新一轮。'
        if saved.get('pose_sha256') != file_hash(pose_file):
            return '采样姿态集已更新，旧采样不能续采。请归档旧记录并开始新一轮。'
        return ''

    def snapshot(self):
        status = self.store.status(self.unit)
        hashes = self.hashes()
        rows = {}
        for stage in STAGES:
            component = status['components'].get(stage, {})
            receipt_path = self.root / 'workbench' / f'{stage}.yaml'
            receipt = load_yaml(receipt_path) if receipt_path.is_file() else {}
            deps = DEPENDENCIES[stage]
            dependencies_match = bool(receipt) and all(
                receipt.get('predecessors', {}).get(name) == hashes[name]
                for name in deps)
            result_matches = receipt.get('result_sha256') == hashes.get(stage) if stage in hashes else True
            # Old results without recorded provenance stay visible, not certified.
            freshness = ('current' if dependencies_match and result_matches
                         else 'needs_validation' if component.get('present') or receipt else 'missing')
            rows[stage] = {**component, 'freshness': freshness,
                           'resume_issue': self.resume_issue(stage),
                           'missing': [n for n in deps
                                       if not status['components'][n].get('quality_passed')],
                           'provenance': receipt}
        leader = self.root / 'capture/calibration_work/leader_servo/result.yaml'
        rows['leader'].update(present=leader.is_file(), result_path=str(leader))
        status.update(
            stages=rows, hardware_enabled=self.hardware,
            session={'stage': self.stage, 'running': self.process is not None and self.process.returncode is None,
                     'generation': self.generation, 'error': self.last_error,
                     'log_path': str(self.log_path) if self.log_path else None,
                     'startup_notice': self.startup_notice()},
            versions=sorted(p.name for p in (self.root / 'versions').glob('*')
                            if p.is_dir() and not p.name.startswith('.')),
            state_path=str(self.root),
        )
        return status

    def startup_notice(self):
        if not self.process or self.process.returncode is not None or not self.log_path or not self.log_path.is_file():
            return ''
        with self.log_path.open('rb') as stream:
            stream.seek(max(0, self.log_path.stat().st_size - 64000))
            log = stream.read().decode('utf-8', errors='replace')
        waiting = log.rfind('reposition joints within limits:')
        ready = log.rfind('Measured pose ready; starting standard controller spawner')
        if waiting > ready:
            match = re.search(r'reposition joints within limits: (.*?)\. State feedback', log[waiting:])
            joints = match.group(1) if match else '请查看启动日志'
            return f'启动等待：关节超出命令限位（{joints}）。请托住相应部位，手动摆回正常活动范围；回到范围后会自动继续加载控制器。'
        return ''

    def _claim(self):
        path = self.store.state_root / 'calibration-device-owner.lock'
        path.parent.mkdir(parents=True, exist_ok=True)
        stream = path.open('a')
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            stream.close()
            raise ValueError('another workbench owns the calibration devices')
        self.session_lock = stream

    def _release(self):
        if self.session_lock:
            self.session_lock.close()
            self.session_lock = None

    def _check_devices(self, stage):
        devices = load_yaml(self.config)['robot']['devices']
        names = ('leader_arm',) if stage == 'leader' else (
            ('left_arm',) if stage == 'head_camera' else ('right_arm', 'left_arm'))
        for name in names:
            path = Path(devices.get(name, '/dev/' + name))
            if not path.exists():
                raise ValueError(f'missing device {path}; check power, USB and device aliases')
            # Do not kill or attach to another tool's controller.
            result = subprocess.run(['fuser', str(path.resolve())], capture_output=True, timeout=3)
            if result.returncode == 0:
                raise ValueError(f'{path} is already open; stop the current Demo/calibration tool first')
            if result.returncode != 1:
                raise ValueError(f'cannot check ownership of {path}')

    async def start(self, stage, fresh=False):
        async with self.lock:
            if not self.hardware:
                raise ValueError('restart tools/calibrate web with --hardware to open devices')
            if stage not in STAGES:
                raise ValueError('unknown calibration stage')
            if self.process is not None:
                raise ValueError('end the existing device session before starting another')
            status = self.snapshot()
            if status['stages'][stage]['missing']:
                raise ValueError('passing drafts required: ' + ', '.join(status['stages'][stage]['missing']))
            if not fresh and status['stages'][stage]['resume_issue']:
                raise ValueError(status['stages'][stage]['resume_issue'])
            self._claim()
            try:
                await asyncio.to_thread(self._check_devices, stage)
                with socket.socket() as sock:
                    sock.bind(('127.0.0.1', 0))
                    self.child_port = sock.getsockname()[1]
                    with socket.socket() as hover_sock:
                        hover_sock.bind(('127.0.0.1', 0))
                        self.hover_port = hover_sock.getsockname()[1]
                self.generation = uuid.uuid4().hex
                self.log_path = self.root / 'workbench/logs' / f'{self.generation}.log'
                self.log_path.parent.mkdir(parents=True, exist_ok=True)
                workflow = 'servo' if stage == 'leader' else stage.replace('_', '-')
                command = [str(self.repo / 'tools/calibrate'), 'capture', workflow,
                           '--hardware', '--config', str(self.config),
                           '--web-port', str(self.child_port), '--web-host', '127.0.0.1',
                           '--fresh' if fresh else '--resume']
                if stage == 'leader':
                    command.append('--leader')
                if stage == 'hover':
                    command.extend(['--hover-web-port', str(self.hover_port)])
                _atomic_yaml(self.root / 'workbench/session.yaml', {
                    'stage': stage, 'generation': self.generation,
                    'predecessors': {n: self.hashes()[n] for n in DEPENDENCIES[stage]},
                })
                with self.log_path.open('wb') as log:
                    self.process = await asyncio.create_subprocess_exec(
                        *command, cwd=self.repo, stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True)
                self.stage = stage
                self.last_error = ''
            except Exception:
                self._release()
                raise
        return self.snapshot()

    async def stop(self):
        async with self.lock:
            if self.process:
                # Kill only our own process group, never by executable name.
                pid = self.process.pid
                try:
                    os.killpg(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    await asyncio.wait_for(self.process.wait(), 20)
                except asyncio.TimeoutError:
                    self.last_error = 'shutdown timed out; device session retained; inspect its log before retrying'
                    raise ValueError(self.last_error)
                # A launch parent exiting is not proof that all children exited.
                for _ in range(30):
                    try:
                        os.killpg(pid, 0)
                    except ProcessLookupError:
                        break
                    await asyncio.sleep(.1)
                else:
                    raise ValueError('capture descendants still exist; not starting another owner')
                self.process = None
                self.stage = None
                self._release()
        return self.snapshot()

    async def save_result(self):
        """Explicitly accept a completed existing tool result, recording its inputs."""
        async with self.lock:
            if not self.stage or not self.process or self.process.returncode is not None:
                raise ValueError('no live session to accept')
            stage = self.stage
            session = load_yaml(self.root / 'workbench/session.yaml')
            if any(self.hashes()[n] != h for n, h in session['predecessors'].items()):
                raise ValueError('predecessor changed during capture; archive and repeat this stage')
            capture = self.root / 'capture/calibration_work'
            if stage == 'servo':
                self.store.save_component(self.unit, 'servo', load_yaml(capture / 'servo/result.yaml'))
            elif stage in ('head_camera', 'right_handeye'):
                progress = load_yaml(capture / stage / 'progress.yaml')
                if progress.get('phase') != 'COMPLETED' or not progress.get('quality_passed'):
                    raise ValueError('automatic capture has not completed with passing quality')
                predecessor_bytes = b''.join((self.store.draft_components(self.unit) / f'{n}.yaml').read_bytes()
                                             for n in DEPENDENCIES[stage])
                if progress.get('servo_sha256') != hashlib.sha256(predecessor_bytes).hexdigest():
                    raise ValueError('capture used a different servo draft')
            elif stage == 'leader':
                # Preserve separate attachment identity, never import into Follower.
                result = load_yaml(capture / 'leader_servo/result.yaml')
                if result.get('attachment') != 'right_leader':
                    raise ValueError('not a Leader calibration result')
            else:
                raise ValueError('hover records are saved by the measurement tool, not a calibration component')
            if stage in self.hashes() and not self.store.status(self.unit)['components'][stage]['quality_passed']:
                raise ValueError('result did not pass component validation')
            _atomic_yaml(self.root / 'workbench' / f'{stage}.yaml', {
                **session, 'result_sha256': self.hashes().get(stage),
                'hardware_acceptance': 'operator_accepted',
            })
        return self.snapshot()

    def reuse(self, stage):
        if self.process is not None:
            raise ValueError('end device session before replacing a draft')
        if stage not in ('servo', 'head_camera', 'right_handeye'):
            raise ValueError('only structural components can be reused')
        self.store.verify_runtime(self.unit)
        version = self.store.active_version(self.unit)
        source = version / 'components' / f'{stage}.yaml'
        if source.is_file():
            document = load_yaml(source)
        elif stage == 'servo':
            values = load_yaml(self.root / 'runtime/servos.yaml')
            document = {'schema': 'xlerobot_servo_calibration/v1',
                        **{n: values[n] for n in ('right_arm', 'left_arm', 'head')}}
        else:
            document = load_yaml(self.root / 'runtime/transforms.yaml').get(stage)
            if not document:
                raise ValueError('active runtime has only transform values, not a validated solver result; capture this stage first')
        from .quality import validate_component
        validate_component(stage, document)
        old = self.store.draft_components(self.unit) / f'{stage}.yaml'
        if old.is_file():
            archive = self.root / 'workbench/archive' / f'{stage}-{uuid.uuid4().hex}.yaml'
            archive.parent.mkdir(parents=True, exist_ok=True)
            archive.write_bytes(old.read_bytes())
        self.store.save_component(self.unit, stage, document)
        # A reusable component is not proof of compatibility with other drafts.
        _atomic_yaml(self.root / 'workbench' / f'{stage}.yaml', {
            'source': 'active_version', 'version': version.name,
            'result_sha256': self.hashes()[stage], 'predecessors': {},
            'hardware_acceptance': 'reused_not_remeasured'})
        return self.snapshot()


@web.middleware
async def errors(request, handler):
    try:
        if request.method == 'POST':
            # LAN UI only: reject cross-site browser writes, including form posts.
            origin = request.headers.get('Origin')
            if origin and origin != f'{request.scheme}://{request.host}':
                raise web.HTTPForbidden(text='cross-origin operation rejected')
            if request.content_type != 'application/json':
                raise web.HTTPBadRequest(text='application/json required')
        return await handler(request)
    except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as error:
        return web.json_response({'error': str(error)}, status=409)


def application(manager):
    app = web.Application(middlewares=[errors], client_max_size=65536)
    static = manager.repo / 'ros2_ws/src/xlerobot_hmi/web/dist'

    async def status(request):
        return web.json_response(await asyncio.to_thread(manager.snapshot))

    async def hover_result(request):
        return web.json_response(await asyncio.to_thread(manager.hover_result))

    async def operation(request):
        # Serialize whole restart transactions, including stop -> start, across tabs.
        async with manager.command_lock:
            return await operation_impl(request)

    async def operation_impl(request):
        body = await request.json()
        if not isinstance(body, dict):
            raise ValueError('JSON object required')
        if body.get('confirmed') is not True:
            raise ValueError('explicit confirmation required')
        name = request.match_info['operation']
        if name == 'start':
            result = await manager.start(body.get('stage'), body.get('fresh') is True)
        elif name == 'auto-hover':
            if manager.process is None:
                await manager.start('hover')
            elif manager.stage != 'hover' or manager.process.returncode is not None:
                raise ValueError('请先结束当前设备会话，再开始悬停自动验证')
            # The robot-side background job survives browser reload/disconnect.
            deadline = asyncio.get_running_loop().time() + 30
            url = f'http://127.0.0.1:{manager.hover_port}/api/v1/hover/'
            while True:
                try:
                    async with app[CLIENT].get(url + 'status', timeout=ClientTimeout(total=2)) as response:
                        ready = response.status == 200 and (await response.json()).get('ready')
                    if ready:
                        async with app[CLIENT].post(url + 'auto', json={'confirmed': True},
                                                  timeout=ClientTimeout(total=3)) as response:
                            result = await response.json()
                            if response.status != 202:
                                raise ValueError(result.get('error', '自动验证未接受'))
                        break
                except (ClientError, asyncio.TimeoutError):
                    pass
                if asyncio.get_running_loop().time() >= deadline:
                    raise ValueError('设备启动尚未完成，请查看启动提示后重试；未发送自动运动请求')
                await asyncio.sleep(.25)
        elif name == 'stop':
            result = await manager.stop()
        elif name == 'restart-servo':
            stage = body.get('stage')
            if stage not in ('servo', 'leader') or manager.stage != stage:
                raise ValueError('only the currently selected servo session can be restarted')
            await manager.stop()
            result = await manager.start(stage, fresh=True)
        elif name == 'accept':
            result = await manager.save_result()
        elif name == 'reuse':
            async with manager.lock:
                result = await asyncio.to_thread(manager.reuse, body.get('stage'))
        elif name in ('replace', 'switch', 'activate'):
            async with manager.lock:
                if manager.process is not None:
                    raise ValueError('end device session before applying or restoring runtime configuration')
                version = body.get('version', '')
                if name == 'replace':
                    components = body.get('components', [])
                    if not components:
                        raise ValueError('select at least one component')
                    result = await asyncio.to_thread(manager.store.replace_from_draft,
                        manager.unit, version, components, dry_run=body.get('preview') is True)
                elif name == 'switch':
                    result = {'runtime': str(await asyncio.to_thread(
                        manager.store.switch, manager.unit, version))}
                else:
                    selected = await asyncio.to_thread(manager.store.activate, manager.unit, version)
                    await asyncio.to_thread(manager.store.render_active, manager.unit)
                    result = {'version': selected}
        else:
            raise web.HTTPNotFound()
        return web.json_response(result)

    async def proxy(request):
        if manager.process is None or manager.process.returncode is not None:
            raise web.HTTPServiceUnavailable(text='设备会话未启动或已经退出，请查看会话日志')
        port = manager.hover_port if request.path.startswith('/api/v1/hover/') and manager.stage == 'hover' else manager.child_port
        url = f'http://127.0.0.1:{port}{request.rel_url}'
        try:
            async with app[CLIENT].request(request.method, url, data=await request.read(),
                    headers={'Content-Type': request.content_type}, allow_redirects=False) as upstream:
                response = web.StreamResponse(status=upstream.status, headers={
                    'Content-Type': upstream.headers.get('Content-Type', 'application/octet-stream'),
                    'Cache-Control': 'no-store'})
                await response.prepare(request)
                async for chunk in upstream.content.iter_any():
                    await response.write(chunk)
                return response
        except (ClientError, ConnectionError, asyncio.TimeoutError) as error:
            raise web.HTTPServiceUnavailable(text='标定服务正在启动，请稍候') from error

    async def index(request):
        html = (static / 'index.html').read_text()
        if request.path == '/':
            html = html.replace('<head>', '<head><meta name="xlerobot-workbench" content="v1">')
            html = html.replace('<title>XLeRobot Operator Console</title>', '<title>XLeRobot · 机器人标定</title>')
        return web.Response(text=html, content_type='text/html', headers={'Cache-Control': 'no-store'})

    async def log(request):
        text = ''
        if manager.log_path and manager.log_path.is_file():
            with manager.log_path.open('rb') as stream:
                stream.seek(max(0, manager.log_path.stat().st_size - 12000))
                text = stream.read().decode('utf-8', errors='replace')
        return web.json_response({'text': text})

    async def lifecycle(app):
        app[CLIENT] = ClientSession(timeout=ClientTimeout(total=None, sock_connect=3))
        yield
        await app[CLIENT].close()
        await manager.stop()

    async def zero_reference(request):
        return web.FileResponse(static / 'calibration-zero.png')

    async def documentation(request):
        name = request.match_info['name']
        if name not in {'calibration-workbench.md', 'calibration-followup.md', 'calibration-versions.md'}:
            raise web.HTTPNotFound()
        language = request.query.get('lang', 'zh')
        if language not in {'zh', 'en'}:
            raise web.HTTPBadRequest(text='Unsupported language; use zh or en')
        directory = 'en' if language == 'en' else 'zh-CN'
        return web.Response(text=(manager.repo / 'docs' / directory / name).read_text(), content_type='text/plain')

    app.cleanup_ctx.append(lifecycle)
    app.router.add_get('/workbench-api/status', status)
    app.router.add_get('/workbench-api/hover-result', hover_result)
    app.router.add_get('/workbench-api/log', log)
    app.router.add_get('/workbench-docs/{name}', documentation)
    app.router.add_post('/workbench-api/{operation}', operation)
    app.router.add_route('*', '/api/{tail:.*}', proxy)
    app.router.add_get('/calibration-zero.png', zero_reference)
    app.router.add_static('/assets/', static / 'assets')
    app.router.add_get('/', index)
    app.router.add_get('/capture', index)
    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description='Unified robot calibration web workspace')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--web-port', type=int, default=8080)
    parser.add_argument('--web-host', default='0.0.0.0')
    args = parser.parse_args(argv)
    args.state_root = args.unit = None
    repo = _repo_root()
    state, unit = _defaults(args, repo)
    config = (args.config or Path(os.environ.get('XLEROBOT_CONFIG', 'config/local.yaml')))
    if not config.is_absolute():
        config = repo / config
    manager = Workbench(repo, config.resolve(), state, unit, args.hardware)
    print('Calibration workbench: no devices open until you start a session in the page.', flush=True)
    web.run_app(application(manager), host=args.web_host, port=args.web_port)


if __name__ == '__main__':
    main()
