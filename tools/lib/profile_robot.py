"""Read-only CPU and scheduler sampler. Never opens robot devices."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
import urllib.request


def read(path):
    try:
        return Path(path).read_text().strip()
    except (OSError, ValueError):
        return ''


def snapshot(telemetry_cache=None):
    cpu = [int(value) for value in read('/proc/stat').splitlines()[0].split()[1:9]]
    processes = {}
    for directory in Path('/proc').glob('[0-9]*'):
        stat = read(directory / 'stat')
        fields = stat.rpartition(') ')[2].split()
        if len(fields) < 39:
            continue
        # stat already contains comm; avoid another /proc open for every PID.
        name = stat.partition(' (')[2].rpartition(') ')[0]
        processes[directory.name] = {
            'name': name, 'start': fields[19], 'ticks': int(fields[11]) + int(fields[12]),
            'rss_bytes': int(fields[21]) * os.sysconf('SC_PAGE_SIZE'),
            'threads': int(fields[17]), 'policy': int(fields[38]), 'rt_priority': int(fields[37]),
        }
    controllers = {}
    for pid, item in processes.items():
        if 'ros2_control' not in item['name']:
            continue
        for path in Path(f'/proc/{pid}/task').glob('*/stat'):
            fields = read(path).rpartition(') ')[2].split()
            if len(fields) >= 39:
                controllers[path.parent.name] = {
                    'policy': int(fields[38]), 'rt_priority': int(fields[37]),
                    'ticks': int(fields[11]) + int(fields[12]), 'cpu': int(fields[36]),
                }
    now = time.monotonic()
    cache = {} if telemetry_cache is None else telemetry_cache
    if not cache or now - cache['sampled_at'] >= 5.0:
        cache.update(sampled_at=now, values={
            'cpu_pressure': read('/proc/pressure/cpu'), 'meminfo': read('/proc/meminfo'),
            'temperature_mC': {read(path.parent / 'type'): read(path)
                               for path in Path('/sys/class/thermal').glob('thermal_zone*/temp')},
            'frequency_kHz': {path.parent.name: read(path)
                              for path in Path('/sys/devices/system/cpu/cpufreq').glob('policy*/scaling_cur_freq')},
        })
    return {
        'monotonic': time.monotonic(), 'cpu': cpu, 'processes': processes,
        'control_threads': controllers, 'loadavg': read('/proc/loadavg'),
        'telemetry_age_s': max(0.0, now - cache['sampled_at']),
        **cache['values'],
    }


def interval(before, after):
    differences = [b - a for a, b in zip(before['cpu'], after['cpu'])]
    total = sum(differences)
    elapsed = after['monotonic'] - before['monotonic']
    result = {
        'elapsed_s': elapsed,
        'cpu_total_ticks': total,
        'cpu_busy_ticks': total - differences[3] - differences[4],
        'cpu_busy_percent': 100 * (total - differences[3] - differences[4]) / max(total, 1),
        'cpu_system_percent': 100 * differences[2] / max(total, 1),
        'cpu_irq_percent': 100 * (differences[5] + differences[6]) / max(total, 1),
        'processes': [],
    }
    for pid, item in after['processes'].items():
        previous = before['processes'].get(pid)
        if previous and previous['start'] == item['start']:
            percent = 100 * (item['ticks'] - previous['ticks']) / os.sysconf('SC_CLK_TCK') / elapsed
            result['processes'].append({'pid': pid, **item, 'cpu_one_core_percent': percent})
    result['processes'].sort(key=lambda item: item['cpu_one_core_percent'], reverse=True)
    return result


def rolling_cpu_summary(samples, window_s=10.0, target_percent=60.0):
    """Windows end at each sample; prorate only the oldest boundary interval."""
    windows = []
    elapsed = 0.0
    for end, sample in enumerate(samples):
        elapsed += sample['elapsed_s']
        if elapsed < window_s - 1e-6:
            continue
        remaining, total, busy = window_s, 0.0, 0.0
        for previous_index in range(end, -1, -1):
            item = samples[previous_index]
            seconds = item['elapsed_s']
            fraction = min(remaining, seconds) / seconds
            # Backward compatibility for reports without raw jiffies.
            ticks = item.get('cpu_total_ticks', seconds)
            total += ticks * fraction
            busy += item.get('cpu_busy_ticks', ticks * item['cpu_busy_percent'] / 100) * fraction
            remaining -= seconds * fraction
            if remaining <= 1e-6:
                break
        windows.append({'end_s': elapsed, 'cpu_busy_percent': 100 * busy / max(total, 1e-9)})
    maximum = max((w['cpu_busy_percent'] for w in windows), default=None)
    return {'window_s': window_s, 'target_percent': target_percent,
            'max_cpu_busy_percent': maximum,
            'windows_at_or_above_target': sum(w['cpu_busy_percent'] >= target_percent for w in windows),
            'passed': maximum is not None and maximum < target_percent,
            'windows': windows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=int, default=60)
    parser.add_argument('--label', default='idle')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--health-url', default='')
    args = parser.parse_args()
    if not 1 <= args.duration <= 7200:
        parser.error('duration must be 1..7200 seconds')
    telemetry_cache = {}
    before = snapshot(telemetry_cache)
    samples = []
    started = time.monotonic()
    for index in range(args.duration):
        time.sleep(max(0, started + index + 1 - time.monotonic()))
        after = snapshot(telemetry_cache)
        sample = interval(before, after)
        sample.update({key: value for key, value in after.items() if key not in ('cpu', 'processes')})
        if args.health_url:
            try:
                with urllib.request.urlopen(args.health_url, timeout=0.3) as response:
                    sample['health'] = json.load(response)
            except (OSError, ValueError) as exc:
                sample['health_unavailable'] = type(exc).__name__
        samples.append(sample)
        before = after
        if (index + 1) % 10 == 0:
            print(f'{index + 1}s CPU={sample["cpu_busy_percent"]:.1f}%', flush=True)
    elapsed = sum(item['elapsed_s'] for item in samples)
    average = sum(item['cpu_busy_percent'] * item['elapsed_s'] for item in samples) / elapsed
    report = {'label': args.label, 'time': datetime.now(timezone.utc).isoformat(),
              'online_cpus': read('/sys/devices/system/cpu/online'),
              'mean_cpu_busy_percent': average, 'samples': samples}
    report['rolling_cpu'] = rolling_cpu_summary(samples)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    args.output.chmod(0o600)
    print(f'mean CPU={average:.2f}%; report: {args.output}')


if __name__ == '__main__':
    main()
