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
