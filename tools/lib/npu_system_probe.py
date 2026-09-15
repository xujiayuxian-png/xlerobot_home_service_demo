"""Exercise resident X1 inference workers with saved inputs; never start ROS or motors."""

import argparse
import base64
import io
import json
import subprocess
from pathlib import Path
import threading
import time
from urllib.request import Request, urlopen
import wave

import numpy as np


def request(port, path, payload=None, timeout=30):
    body = None if payload is None else json.dumps(payload, allow_nan=False).encode()
    with urlopen(Request(f'http://127.0.0.1:{port}{path}', body,
                         {'Content-Type': 'application/json'}), timeout=timeout) as response:
        raw = response.read(2_000_001)
    if len(raw) > 2_000_000:
        raise ValueError('response too large')
    return json.loads(raw)


def counters(pids):
    cpu = [int(v) for v in Path('/proc/stat').read_text().splitlines()[0].split()[1:9]]
    mem = {r.split(':')[0]: int(r.split()[1]) for r in Path('/proc/meminfo').read_text().splitlines()}
    processes = {}
    for name, pid in pids.items():
        try:
            rows = Path(f'/proc/{pid}/status').read_text().splitlines()
            values = {r.split(':')[0]: r.split(':')[1].strip() for r in rows}
            stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
            processes[name] = {'rss_kib': int(values.get('VmRSS', '0 kB').split()[0]),
                               'cpu_ticks': int(stat[11])+int(stat[12]), 'state': values['State']}
        except (OSError, ValueError):
            processes[name] = {'missing': True}
    temperatures = {}
    for sensor in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:
            temperatures[sensor.parent.name] = int(sensor.read_text()) / 1000
        except (OSError, ValueError):
            pass
    return {'monotonic_s': time.monotonic(), 'cpu_total': sum(cpu), 'cpu_idle': cpu[3]+cpu[4],
            'mem_available_kib': mem['MemAvailable'], 'swap_free_kib': mem['SwapFree'],
            'processes': processes, 'temperatures_c': temperatures}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, required=True, help='private reports and worker PID files')
    parser.add_argument('--sample', type=Path, required=True)
    parser.add_argument('--person-image', type=Path, required=True)
    parser.add_argument('--audio', type=Path, required=True)
    parser.add_argument('--duration', type=int, default=180)
    parser.add_argument('--mode', choices=['mixed', 'idle'], default='mixed')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.duration <= 7200:
        parser.error('duration must be 1..7200 seconds')
    service = subprocess.run(['systemctl', 'is-active', 'xlerobot-demo.service'],
                             capture_output=True, text=True, timeout=5).stdout.strip()
    if service not in ('inactive', 'failed', 'unknown'):
        parser.error('stop the demo service before the isolated inference experiment')
    from PIL import Image
    pids = {kind: int((args.directory/f'{kind}.pid').read_text())
            for kind in ('vlm', 'act', 'person', 'whisper')}
    health = {kind: request(port, '/healthz') for kind, port in
              [('act', 18901), ('person', 18902), ('whisper', 18903)]}
    for kind, value in health.items():
        if value.get('kind') != kind or value.get('device') != 'qnn-htp':
            raise RuntimeError('wrong worker identity')
    models = request(18888, '/v1/models')['data']
    model = 'qwen2.5-vl-3b-instruct-672x672-qnn2.36-w4a16-qcs8550'
    if not any(row['id'] == model for row in models):
        raise RuntimeError('expected VLM not loaded')
    with np.load(args.sample, allow_pickle=False) as data:
        image, state, reference = (data[k].copy() for k in ('image', 'state', 'reference'))
    if image.shape != (1, 3, 480, 640) or state.shape != (1, 6) or reference.shape != (1, 100, 6):
        raise ValueError('invalid ACT sample')
    rgb = np.rint(image[0].transpose(1, 2, 0)*255).clip(0, 255).astype(np.uint8)
    buffer = io.BytesIO(); Image.fromarray(rgb).save(buffer, format='PNG')
    act_payload = {'wrist_image_base64': base64.b64encode(buffer.getvalue()).decode(), 'state': state[0].tolist()}
    person_payload = {'image_base64': base64.b64encode(args.person_image.read_bytes()).decode()}
    with wave.open(str(args.audio)) as audio:
        if audio.getnchannels() != 1 or audio.getframerate() != 16000 or audio.getsampwidth() != 2:
            raise ValueError('saved input must be 16 kHz mono PCM16 WAV')
        samples = np.frombuffer(audio.readframes(audio.getnframes()), '<i2').astype('<f4') / 32768
    asr_payload = {'sample_rate': 16000, 'audio_base64': base64.b64encode(samples.tobytes()).decode()}
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from services.npu.prompts import INTENT_SYSTEM, grasp_prompt
    buffer = io.BytesIO(); Image.fromarray(rgb).resize((672, 672)).save(buffer, format='PNG')
    vlm_image = 'data:image/png;base64,' + base64.b64encode(buffer.getvalue()).decode()
    # This AidGen/QNN240 sampler asserts and aborts at temperature=0.
    text_request = {'model': model, 'max_tokens': 128, 'temperature': 0.1,
                    'messages': [{'role': 'system', 'content': INTENT_SYSTEM},
                                 {'role': 'user', 'content': '帮我拿桌上的羽毛球'}]}
    vision_request = {'model': model, 'max_tokens': 128, 'temperature': 0.1,
                      'messages': [{'role': 'user', 'content': [
                          {'type': 'image_url', 'image_url': {'url': vlm_image}},
                          {'type': 'text', 'text': grasp_prompt('黄色圆筒')}]}]}
    usage, records = [], []
    stop = threading.Event()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    def monitor():
        last_progress = started
        while not stop.is_set():
            sample = counters(pids)
            usage.append(sample)
            if sample['monotonic_s']-last_progress >= 60:
                progress = {'elapsed_s': sample['monotonic_s']-started,
                            'requests': len(records),
                            'errors': sum(not r['transport_ok'] for r in list(records)),
                            'processes': sample['processes'],
                            'mem_available_kib': sample['mem_available_kib']}
                progress_path = args.output.with_suffix('.progress.json')
                progress_path.write_text(json.dumps(progress)); progress_path.chmod(0o600)
                print(json.dumps({'progress': progress}), flush=True)
                last_progress = sample['monotonic_s']
            stop.wait(1)
    thread = threading.Thread(target=monitor, daemon=True); thread.start()
    try:
        while time.monotonic()-started < args.duration:
            if args.mode == 'idle':
                time.sleep(0.5)
                continue
            stages = [('whisper', 18903, asr_payload), ('intent', 18888, text_request)]
            stages += [('act', 18901, act_payload)]*9
            stages += [('vision', 18888, vision_request)]
            stages += [('person', 18902, person_payload)]*9
            for kind, port, payload in stages:
                if time.monotonic()-started >= args.duration:
                    break
                before = time.monotonic()
                row = {'kind': kind, 'start_s': before-started}
                try:
                    data = dict(payload)
                    if port != 18888:
                        data['deadline_monotonic'] = before+20
                    result = request(port, '/v1/chat/completions' if port == 18888 else '/predict', data)
                    row['transport_ok'] = True
                    if kind == 'act':
                        actions = np.asarray(result['action'])
                        if actions.shape != (100, 6) or not np.isfinite(actions).all():
                            raise ValueError('invalid action block')
                        row['max_abs_error'] = float(np.max(np.abs(actions-reference[0])))
                    elif kind == 'whisper':
                        row.update(text=result['text'], accepted=result['accepted'])
                    elif kind == 'person':
                        row['detections'] = len(result['detections'])
                    else:
                        row['text'] = result['choices'][0]['message']['content']
                        row['finish_reason'] = result['choices'][0]['finish_reason']
                except Exception as error:
                    row.update(transport_ok=False, error=f'{type(error).__name__}: {error}')
                row['elapsed_ms'] = (time.monotonic()-before)*1000
                records.append(row)
                if kind in ('act', 'person'):
                    time.sleep(max(0, before+1/3-time.monotonic()))
    finally:
        stop.set(); thread.join(timeout=2)
        usage.append(counters(pids))
        a, b = usage[0], usage[-1]
        cpu = 100*(1-(b['cpu_idle']-a['cpu_idle'])/(b['cpu_total']-a['cpu_total']))
        summary = {}
        for kind in sorted({r['kind'] for r in records}):
            rows = [r for r in records if r['kind'] == kind]
            latencies = [r['elapsed_ms'] for r in rows if r['transport_ok']]
            summary[kind] = {'requests': len(rows), 'errors': sum(not r['transport_ok'] for r in rows),
                             'mean_ms': float(np.mean(latencies)) if latencies else None,
                             'p95_ms': float(np.percentile(latencies, 95)) if latencies else None}
        windows = []
        j = 0
        for first in usage:
            while j < len(usage) and usage[j]['monotonic_s']-first['monotonic_s'] < 10:
                j += 1
            if j == len(usage):
                break
            last = usage[j]
            windows.append(100*(1-(last['cpu_idle']-first['cpu_idle'])/(last['cpu_total']-first['cpu_total'])))
        report = {'mode': args.mode, 'duration_s': time.monotonic()-started, 'demo_service_state': service,
                  'scope': 'saved-input inference only', 'semantic_quality_evaluated': False,
                  'hardware_executed': False, 'cpu_normalization': 'all online CPUs = 100%',
                  'max_10s_cpu_percent': max(windows) if windows else None,
                  'whole_cpu_percent': cpu, 'min_mem_available_kib': min(r['mem_available_kib'] for r in usage),
                  'health': health, 'summary': summary, 'records': records, 'system_samples': usage}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)); args.output.chmod(0o600)
        print(json.dumps({k: v for k, v in report.items() if k not in ('records', 'system_samples')}, ensure_ascii=False))
    if any(not r['transport_ok'] for r in records) or any(
            v.get('missing') or v.get('state', '').startswith('Z')
            for row in usage for v in row['processes'].values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
