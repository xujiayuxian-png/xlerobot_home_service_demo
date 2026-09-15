import importlib.util
from pathlib import Path


source = next(parent / 'tools/lib/profile_robot.py' for parent in Path(__file__).resolve().parents
              if (parent / 'tools/lib/profile_robot.py').is_file())
spec = importlib.util.spec_from_file_location('profile_robot', source)
profile = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profile)


def sample(seconds, percent):
    return {'elapsed_s': seconds, 'cpu_busy_percent': percent,
            'cpu_total_ticks': seconds * 600, 'cpu_busy_ticks': seconds * 6 * percent}


def test_rolling_windows_expose_overload_hidden_by_whole_run_mean():
    result = profile.rolling_cpu_summary([sample(1, 10)] * 50 + [sample(1, 80)] * 10)
    assert result['max_cpu_busy_percent'] == 80
    assert not result['passed']
    assert len(result['windows']) == 51


def test_partial_boundary_uses_time_and_no_complete_window_cannot_pass():
    result = profile.rolling_cpu_summary([sample(6, 100), sample(6, 0)])
    assert result['max_cpu_busy_percent'] == 40
    assert result['passed']
    assert not profile.rolling_cpu_summary([sample(1, 0)])['passed']
    assert not profile.rolling_cpu_summary([sample(10, 60)])['passed']


def test_snapshot_caches_only_slow_telemetry_and_handles_parentheses(monkeypatch):
    clock = [10.0]
    calls = []
    fields = ['0'] * 39
    fields[19] = '123'
    fields[11] = '20'

    def fake_read(path):
        path = str(path)
        calls.append(path)
        if path == '/proc/stat':
            return 'cpu 10 0 20 30 0 0 0 0'
        if path == '/proc/123/stat':
            return '123 (worker (a) b) ' + ' '.join(fields)
        return ''

    monkeypatch.setattr(profile, 'read', fake_read)
    monkeypatch.setattr(profile.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(profile.Path, 'glob', lambda path, pattern:
                        [Path('/proc/123')] if str(path) == '/proc' else [])
    cache = {}
    first = profile.snapshot(cache)
    fields[11] = '25'
    clock[0] += 1
    second = profile.snapshot(cache)
    assert first['processes']['123']['name'] == 'worker (a) b'
    assert second['processes']['123']['ticks'] == 25
    assert second['telemetry_age_s'] == 1
    assert calls.count('/proc/stat') == 2
    assert calls.count('/proc/meminfo') == 1
    assert not any(path.endswith('/comm') for path in calls)
    clock[0] += 4
    assert profile.snapshot(cache)['telemetry_age_s'] == 0
    assert calls.count('/proc/meminfo') == 2
