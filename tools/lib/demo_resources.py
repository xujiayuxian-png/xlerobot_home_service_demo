"""Read-only, bounded-memory demo resource recorder; no ROS or motor access."""
import argparse
import asyncio
from collections import deque
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import time

import aiohttp

try:
    from .profile_robot import snapshot, interval, rolling_cpu_summary
    from .dsp_metrics import DspMetrics
except ImportError:
    from profile_robot import snapshot, interval, rolling_cpu_summary
    from dsp_metrics import DspMetrics


def utc():
    return datetime.now(timezone.utc).isoformat()


def descendants(processes, root):
    selected = {str(root)}
    while True:
        added = {pid for pid, row in processes.items() if row['ppid'] in selected} - selected
        if not added:
            return selected
        selected.update(added)


class Context:
    def __init__(self):
        self.connected = False
        self.task = None
        self.voice = 'UNKNOWN'

    def accept(self, event):
        kind, data = event.get('type'), event.get('data')
        if kind == 'snapshot':
            self.task = data.get('task')
            self.voice = data.get('health', {}).get('voice_state', 'UNKNOWN')
        elif kind == 'task':
            self.task = data
        elif kind == 'voice':
            self.voice = data.get('state', 'UNKNOWN')
        else:
            return False
        return True

    def value(self):
        task = self.task or {}
        active = task.get('status') not in (None, 'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED')
        return {'event_stream_connected': self.connected,
                'task_id': task.get('task_id'), 'task_status': task.get('status'),
                'capability': task.get('current_capability') if active else None,
                'phase': task.get('phase') if active else None,
                'task_error_code': task.get('error_code'),
                'voice_state': self.voice,
                'stage': 'startup_or_telemetry_unavailable' if not self.connected else
                         ((task.get('current_capability') or 'task_starting') if active else 'idle')}


class Summary:
    def __init__(self):
        self.groups = {}
        self.recent = deque()
        self.max_window = None

    def add(self, sample):
        self.recent.append(sample)
        while len(self.recent) > 1 and sum(r['elapsed_s'] for r in list(self.recent)[1:]) >= 10:
            self.recent.popleft()
        window = rolling_cpu_summary(list(self.recent))['max_cpu_busy_percent']
        if window is not None:
            self.max_window = max(self.max_window or 0, window)
        context = sample['context']
        # Aggregate by stage and voice state, not task id: memory stays bounded.
        for key in ('all', 'stage:'+context['stage'], 'voice:'+context['voice_state']):
            row = self.groups.setdefault(key, {'samples': 0, 'seconds': 0., 'cpu_integral': 0.,
                                               'demo_cpu_integral': 0., 'peak_cpu_percent': 0.,
                                               'peak_demo_rss_mib': 0., 'min_available_mib': float('inf')})
            row['samples'] += 1
            row['seconds'] += sample['elapsed_s']
            row['cpu_integral'] += sample['cpu_busy_percent']*sample['elapsed_s']
            row['demo_cpu_integral'] += sample['demo_cpu_percent']*sample['elapsed_s']
            row['peak_cpu_percent'] = max(row['peak_cpu_percent'], sample['cpu_busy_percent'])
            row['peak_demo_rss_mib'] = max(row['peak_demo_rss_mib'], sample['demo_rss_mib'])
            row['min_available_mib'] = min(row['min_available_mib'], sample['mem_available_mib'])
            dsp = sample.get('dsp', {})
            if dsp.get('available'):
                row['dsp_seconds'] = row.get('dsp_seconds', 0)+sample['elapsed_s']
                for name in ('qdsp6_percent', 'hvx_percent', 'hmx_percent'):
                    value = dsp['metrics'][name]
                    row[name+'_integral'] = row.get(name+'_integral', 0)+value*sample['elapsed_s']
                    row['peak_'+name] = max(row.get('peak_'+name, 0), value)

    def value(self):
        groups = {}
        for key, row in self.groups.items():
            groups[key] = {k: v for k, v in row.items() if not k.endswith('_integral')}
            groups[key]['mean_cpu_percent'] = row['cpu_integral']/row['seconds']
            groups[key]['mean_demo_cpu_percent'] = row['demo_cpu_integral']/row['seconds']
            for name in ('qdsp6_percent', 'hvx_percent', 'hmx_percent'):
                groups[key]['mean_'+name] = row[name+'_integral']/row['dsp_seconds'] if row.get('dsp_seconds') else None
        return {'groups': groups, 'max_10s_cpu_percent': self.max_window}


async def record(args):
    os.umask(0o077)
    directory = args.output
    directory.mkdir(parents=True, exist_ok=False)
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    context, summary = Context(), Summary()
    metadata = {'started_at': utc(), 'root_pid': args.root_pid, 'sampler_pid': os.getpid(),
                'cpu_normalization': 'all online CPUs = 100%; process cpu_one_core_percent = one core',
                'sample_interval_s': 1, 'dsp_monitor_enabled': not args.no_dsp_monitor,
                'npu_note': 'dsp_mon reports QDSP6, HVX and HMX separately; do not sum them or call CPU NPU utilization.',
                'rss_note': 'RSS sum includes shared pages; use whole-system MemAvailable alongside it.',
                'stage_note': 'Samples carry interval-end context; events preserve shorter transitions.'}
    (directory/'metadata.json').write_text(json.dumps(metadata, indent=2))
    with (directory/'events.jsonl').open('w', buffering=1) as events, (directory/'samples.jsonl').open('w', buffering=1) as samples:
        def event_row(kind):
            events.write(json.dumps({'at': utc(), 'monotonic': time.monotonic(), 'type': kind,
                                     'context': context.value()}, ensure_ascii=False)+'\n')

        async def event_reader():
            timeout = aiohttp.ClientTimeout(total=None, sock_connect=1, sock_read=25)
            async with aiohttp.ClientSession(timeout=timeout) as session:
                while not stop.is_set():
                    try:
                        async with session.get(args.events_url) as response:
                            response.raise_for_status()
                            context.connected = True
                            event_row('connected')
                            async for line in response.content:
                                if line.startswith(b'data: '):
                                    event = json.loads(line[6:])
                                    if context.accept(event):
                                        event_row(event['type'])
                    except (aiohttp.ClientError, asyncio.TimeoutError, ValueError):
                        pass
                    finally:
                        if context.connected:
                            context.connected = False
                            event_row('disconnected')
                    await asyncio.sleep(1)

        reader = asyncio.create_task(event_reader())
        dsp = DspMetrics()
        dsp_reader = None if args.no_dsp_monitor else asyncio.create_task(dsp.run(directory/'dsp-mon.log'))
        cache = {}
        before = snapshot(cache)
        started = time.monotonic()

        def save_summary(finished=False):
            target = directory/'summary.tmp'
            target.write_text(json.dumps({**summary.value(), 'updated_at': utc(), 'finished': finished}, indent=2))
            target.replace(directory/'summary.json')

        try:
            while not stop.is_set():
                try:
                    await asyncio.wait_for(stop.wait(), timeout=1)
                    break
                except asyncio.TimeoutError:
                    pass
                after = snapshot(cache)
                row = interval(before, after)
                selected = descendants(after['processes'], args.root_pid)
                all_processes = row['processes']
                members = [p for p in all_processes if p['pid'] in selected]
                if args.workers_state:
                    try:
                        workers = json.loads(args.workers_state.read_text()).get('workers', {})
                    except (OSError, ValueError):
                        workers = {}
                    for name, worker in workers.items():
                        pid = str(worker['pid'])
                        if after['processes'].get(pid, {}).get('start') != worker.get('start_ticks'):
                            continue
                        family = descendants(after['processes'], pid)
                        for process in members:
                            if process['pid'] in family:
                                process['component'] = name
                # Normalize process ticks by the measured whole-machine tick capacity.
                tick_capacity = row['cpu_total_ticks']/os.sysconf('SC_CLK_TCK')/row['elapsed_s']
                row['demo_cpu_percent'] = sum(p['cpu_one_core_percent'] for p in members)/max(tick_capacity, 1e-9)
                row['demo_rss_mib'] = sum(p['rss_bytes'] for pid, p in after['processes'].items() if pid in selected)/2**20
                row['processes'] = members
                row['other_top_processes'] = [p for p in all_processes if p['pid'] not in selected][:10]
                memory = dict(line.split(':', 1) for line in after['meminfo'].splitlines())
                row['mem_available_mib'] = int(memory['MemAvailable'].split()[0])/1024
                row['swap_used_mib'] = (int(memory['SwapTotal'].split()[0])-int(memory['SwapFree'].split()[0]))/1024
                row.update({key: after[key] for key in ('temperature_mC', 'frequency_kHz', 'cpu_pressure', 'control_threads', 'loadavg', 'telemetry_age_s')})
                row.update(at=utc(), monotonic=after['monotonic'], context=context.value(), dsp=dsp.value())
                samples.write(json.dumps(row, ensure_ascii=False)+'\n')
                summary.add(row)
                save_summary()
                before = after
                if args.duration and time.monotonic()-started >= args.duration:
                    break
        finally:
            reader.cancel()
            await asyncio.gather(reader, return_exceptions=True)
            if dsp_reader:
                dsp_reader.cancel()
                await asyncio.gather(dsp_reader, return_exceptions=True)
            event_row('recorder_stopped')
            save_summary(finished=True)
    print(f'Resource report: {directory}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--root-pid', type=int, default=os.getpid())
    parser.add_argument('--workers-state', type=Path)
    parser.add_argument('--no-dsp-monitor', action='store_true', help='skip FastRPC telemetry for software-only tests')
    parser.add_argument('--events-url', default='http://127.0.0.1:18080/api/v1/events')
    parser.add_argument('--duration', type=float, default=0, help='seconds; zero records until stopped')
    args = parser.parse_args()
    if args.duration < 0 or args.root_pid < 1:
        parser.error('invalid duration or root PID')
    from urllib.parse import urlparse
    if urlparse(args.events_url).hostname != '127.0.0.1':
        parser.error('event source must use loopback')
    asyncio.run(record(args))


if __name__ == '__main__':
    main()
