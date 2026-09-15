"""Manage the experimental, inference-only X1 stack as one foreground process."""

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
from urllib.request import urlopen

import yaml

ROOT = Path(__file__).resolve().parents[2]
STATE = ROOT / '.xlerobot/npu/stack'


def process_identity(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return None if stat[0] == 'Z' else stat[19]
    except (OSError, IndexError):
        return None


def alive(row):
    return isinstance(row, dict) and row.get('start_ticks') is not None and (
        process_identity(row.get('pid')) == row['start_ticks'])


def settings(path):
    config = yaml.safe_load(Path(path).read_text())
    npu = config.get('npu', {})
    keys = ('act_model', 'person_model', 'whisper_model', 'vlm_config', 'manifest')
    result = {}
    for key in keys:
        value = npu.get(key)
        if not isinstance(value, str) or not value:
            raise ValueError(f'npu.{key} must be configured in local.yaml')
        target = Path(os.path.expandvars(value)).expanduser()
        result[key] = target if target.is_absolute() else ROOT/target
        if not result[key].exists():
            raise ValueError(f'missing npu.{key}')
    manifest = json.loads(result['manifest'].read_text())
    if not isinstance(manifest, dict) or not manifest:
        raise ValueError('nonempty local artifact SHA256 manifest required')
    # Require direct model artifacts and every Whisper component to be pinned.
    required = {str(result['act_model'].resolve()), str(result['person_model'].resolve())}
    required |= {str((result['whisper_model']/name).resolve()) for name in
                 ('encoder_model_htp.bin.aidem', 'decoder_model_htp.bin.aidem', 'tokenizer.json')}
    vlm = json.loads(result['vlm_config'].read_text())
    model_id = 'qwen2.5-vl-3b-instruct-672x672-qnn2.36-w4a16-qcs8550'
    if vlm.get('default_model_id') != model_id or len(vlm.get('model_cfg_list', [])) != 1:
        raise ValueError('only the experimentally verified VLM is supported')
    model_dir = Path(vlm['res_folder'])/'models'/model_id
    required |= {str(p.resolve()) for p in model_dir.rglob('*') if p.is_file()}
    if not required <= set(manifest):
        raise ValueError('manifest does not cover the selected worker artifacts')
    for filename, expected in manifest.items():
        digest = hashlib.sha256()
        with Path(filename).open('rb') as source:
            for chunk in iter(lambda: source.read(1024*1024), b''):
                digest.update(chunk)
        if digest.hexdigest() != expected:
            raise ValueError(f'NPU artifact checksum mismatch: {Path(filename).name}')
    result['low_idle_cpu'] = npu.get('low_idle_cpu', True)
    if not isinstance(result['low_idle_cpu'], bool):
        raise ValueError('npu.low_idle_cpu must be boolean')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'stop', 'status'))
    parser.add_argument('--config', required=True)
    parser.add_argument('--demo', action='store_true', help='manage the configured ROS demo after model readiness')
    parser.add_argument('--hardware', action='store_true')
    args = parser.parse_args()
    if args.demo != args.hardware or (args.demo and args.action != 'start'):
        parser.error('demo execution requires both --demo and --hardware on start')
    if args.demo:
        import sys
        sys.path.insert(0, str(ROOT))
        from tools.lib.npu_profile import validate_profile
        validate_profile(yaml.safe_load(Path(args.config).read_text()))
    STATE.mkdir(parents=True, exist_ok=True)
    state_path = STATE/'processes.json'
    if args.action != 'start':
        state = json.loads(state_path.read_text()) if state_path.exists() else {}
        supervisor = state.get('supervisor', {})
        if args.action == 'status':
            print(json.dumps({'supervisor_running': alive(supervisor),
                              'workers': {k: alive(v) for k, v in state.get('workers', {}).items()}}, indent=2))
            return
        if alive(supervisor):
            os.kill(supervisor['pid'], signal.SIGTERM)
            deadline = time.monotonic()+15
            while alive(supervisor) and time.monotonic() < deadline:
                time.sleep(0.1)
            if alive(supervisor):
                raise RuntimeError('inference supervisor did not stop within 15 seconds')
        if any(alive(row) for row in state.get('workers', {}).values()):
            for row in state['workers'].values():
                if alive(row) and os.getpgid(row['pid']) == row['pid']:
                    os.killpg(row['pid'], signal.SIGTERM)
            time.sleep(1)
            for row in state['workers'].values():
                if alive(row) and os.getpgid(row['pid']) == row['pid']:
                    os.killpg(row['pid'], signal.SIGKILL)
            state_path.unlink(missing_ok=True)
        print('NPU inference stack stopped')
        return
    with (STATE/'lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('NPU inference stack is already managed')
        previous = json.loads(state_path.read_text()) if state_path.exists() else {}
        if alive(previous.get('supervisor')) or any(alive(row) for row in previous.get('workers', {}).values()):
            raise SystemExit('live NPU processes remain; stop the existing stack first')
        children, logs = {}, []
        state = {'supervisor': {'pid': os.getpid(), 'start_ticks': process_identity(os.getpid())}, 'workers': {}}
        def save_state():
            temporary = STATE/'processes.tmp'
            temporary.write_text(json.dumps(state)); temporary.chmod(0o600)
            temporary.replace(state_path)
        stopping = False
        def stop(_signum, _frame):
            nonlocal stopping
            stopping = True
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        save_state()
        def launch(kind, command, env):
            log = (STATE/f'{kind}.log').open('w'); logs.append(log)
            child = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            children[kind] = child
            state['workers'][kind] = {'pid': child.pid, 'start_ticks': process_identity(child.pid)}
            save_state()
            (STATE/f'{kind}.pid').write_text(str(child.pid))
            return child
        def ready(child, url):
            deadline = time.monotonic()+120
            while not stopping and time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError('worker exited during startup; inspect private stack logs')
                try:
                    with urlopen(url, timeout=1) as response:
                        if response.status == 200:
                            return
                except OSError:
                    pass
                time.sleep(0.5)
            raise RuntimeError('inference startup canceled or timed out')
        try:
            if args.demo:
                from datetime import datetime, timezone
                run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+f'-{os.getpid()}'
                report = ROOT/'.xlerobot/performance'/run_id
                report.parent.mkdir(parents=True, exist_ok=True)
                latest = report.parent/'latest.tmp'
                latest.unlink(missing_ok=True)
                latest.symlink_to(report.name)
                latest.replace(report.parent/'latest')
                local_config = yaml.safe_load(Path(args.config).read_text())
                port = int(local_config.get('demo', {}).get('web_port', 18080))
                launch('resources', [str(ROOT/'.venv/robot/bin/python'),
                       str(ROOT/'tools/lib/demo_resources.py'), '--root-pid', str(os.getpid()),
                       '--output', str(report), '--workers-state', str(state_path),
                       '--events-url', f'http://127.0.0.1:{port}/api/v1/events'],
                       dict(os.environ))
                print(f'Automatic resource recording: {report}', flush=True)
            config = settings(args.config)
            if stopping:
                raise RuntimeError('inference startup canceled')
            for port in (18888, 18901, 18902, 18903):
                with socket.socket() as probe:
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    probe.bind(('127.0.0.1', port))
            if not Path('/dev/fastrpc-cdsp').exists():
                raise SystemExit('QNN device discovery alias missing; see the NPU experiment documentation')
            library = STATE/'loopback_bind.so'
            subprocess.run(['cc', '-shared', '-fPIC', str(ROOT/'tools/lib/npu_loopback_bind.c'),
                            '-ldl', '-o', str(library)], check=True, timeout=30)
            config_library = STATE/'aidgen_config.so'
            if config['low_idle_cpu']:
                subprocess.run(['c++', '-shared', '-fPIC', '-std=c++17',
                                str(ROOT/'tools/lib/npu_aidgen_config.cpp'), '-I/usr/local/include',
                                '-L/usr/local/lib', '-laidgen', '-ldl', '-o', str(config_library)],
                               check=True, timeout=30)
            if stopping:
                raise RuntimeError('inference startup canceled')
            env = {**os.environ, 'OMP_NUM_THREADS': '2', 'OPENBLAS_NUM_THREADS': '1'}
            vendor_env = {**env, 'LD_PRELOAD': str(library),
                          'LD_LIBRARY_PATH': '/opt/aidlux/app/aid-openai-api/lib:'+env.get('LD_LIBRARY_PATH', '')}
            if config['low_idle_cpu']:
                vendor_env['LD_PRELOAD'] += ':'+str(config_library)
            vlm_config = json.loads(config['vlm_config'].read_text())
            if not any(row.get('port') == 18888 for row in vlm_config.get('http_cfg', [])):
                raise ValueError('experimental VLM config must use port 18888')
            # Start sequentially to avoid competing cold compilation peaks.
            child = launch('vlm', ['/opt/aidlux/app/aid-openai-api/api', '--config', str(config['vlm_config'])], vendor_env)
            ready(child, 'http://127.0.0.1:18888/v1/models')
            for kind, port in [('person', 18902), ('whisper', 18903), ('act', 18901)]:
                python = ROOT/('.xlerobot/npu/ort-env/bin/python' if kind == 'act' else '.venv/robot/bin/python')
                child = launch(kind, [str(python), '-m', 'services.npu.server', '--kind', kind,
                                      '--model', str(config[f'{kind}_model']), '--port', str(port)], env)
                ready(child, f'http://127.0.0.1:{port}/healthz')
            print('All four NPU workers ready on loopback', flush=True)
            if args.demo:
                demo_env = {**env, 'XLEROBOT_NPU_MANAGED': '1'}
                demo_env.pop('XLEROBOT_ACT_TOKEN', None)
                launch('demo', [str(ROOT/'tools/run'), 'demo', '--hardware', '--config', args.config], demo_env)
            while not stopping:
                for kind, child in children.items():
                    if child.poll() is not None:
                        raise RuntimeError(f'{kind} exited; stopping the complete inference stack')
                time.sleep(0.5)
        finally:
            for kind, child in children.items():
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT if kind == 'demo' else signal.SIGTERM)
            for child in children.values():
                try:
                    child.wait(timeout=10 if args.demo else 2)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL); child.wait(timeout=2)
            for log in logs:
                log.close()
            state_path.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
