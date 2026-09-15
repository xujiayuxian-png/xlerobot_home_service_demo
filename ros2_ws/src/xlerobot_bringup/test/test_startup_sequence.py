"""Software-only startup checks; never include the hardware launch files."""

import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, TimerAction
from launch.events import Shutdown
import pytest
import rclpy
from std_srvs.srv import Trigger

from xlerobot_bringup.startup_probe import Readiness
from xlerobot_bringup.startup_sequence import advance_after_probe, chain_phases


@pytest.fixture(autouse=True)
def isolated_ros_domain(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '229')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')


def diagnostic(name, stamp=100, age='0.1', level=0):
    msg = DiagnosticArray()
    msg.header.stamp.sec = stamp
    msg.status = [DiagnosticStatus(name=name, level=bytes([level]),
                                   values=[KeyValue(key='age_s', value=age)])]
    return msg


def test_readiness_rejects_missing_stale_future_and_invalid_camera_data():
    name = 'xlerobot/camera/wrist'
    gate = Readiness({'diagnostics': [name], 'services': ['/ready']})
    assert len(gate.pending(10, [])) == 2
    gate.receive(diagnostic(name), 10, 100.1)
    assert not gate.pending(10.7, ['/ready'])
    assert gate.pending(10.9, ['/ready'])  # Frame + transit + receipt age.
    for stamp, age, level in [(0, '0', 0), (101, '0', 0), (100, 'nan', 0),
                              (100, '-1', 0), (100, 'bad', 0), (100, '0', 2)]:
        gate.receive(diagnostic(name, stamp, age, level), 10, 100.1)
        assert gate.pending(10, ['/ready'])
    gate.receive(diagnostic(name), 11, 100.1)
    assert not gate.pending(11, ['/ready'])  # Recovery after reconnect.
    assert gate.pending(11, [])


def test_probe_exit_does_not_advance_on_failure_or_shutdown():
    context = LaunchContext()
    assert advance_after_probe(SimpleNamespace(returncode=0), context, ['next'], 'platform') == ['next']
    with pytest.raises(RuntimeError, match='platform failed'):
        advance_after_probe(SimpleNamespace(returncode=1), context, ['next'], 'platform')
    context._set_is_shutdown(True)
    assert advance_after_probe(SimpleNamespace(returncode=0), context, ['next'], 'platform') is None


@pytest.mark.parametrize('outcome', ['ready', 'failed', 'stopped'])
def test_real_launch_process_order_failure_and_cleanup(tmp_path, outcome):
    marker = tmp_path / 'next'
    pid_file = tmp_path / 'pid'
    stopped = tmp_path / 'stopped'
    worker_code = (
        'import os,signal,time; from pathlib import Path; '
        f'Path({str(pid_file)!r}).write_text(str(os.getpid())); '
        f'signal.signal(signal.SIGINT, lambda *args: '
        f'(Path({str(stopped)!r}).touch(), exit(0))); time.sleep(30)')
    worker = ExecuteProcess(cmd=[sys.executable, '-c', worker_code])
    final = ExecuteProcess(cmd=[sys.executable, '-c',
                               f'from pathlib import Path; Path({str(marker)!r}).touch()'])
    def factory(_name, _spec):
        delay = 5 if outcome == 'stopped' else 0.5
        code = 1 if outcome == 'failed' else 0
        return ExecuteProcess(cmd=[sys.executable, '-c',
                                   f'import time; time.sleep({delay}); exit({code})'])
    actions = chain_phases([('platform', [worker]), ('application', [final])], factory)
    # Always terminate owned processes, including the success case's fake daemon.
    actions.append(TimerAction(period=1.5, actions=[EmitEvent(event=Shutdown(reason='test stop'))]))
    service = LaunchService()
    service.include_launch_description(LaunchDescription(actions))
    result = service.run()
    assert (result == 0) == (outcome != 'failed')
    assert marker.exists() == (outcome == 'ready')
    assert stopped.exists()
    with pytest.raises(ProcessLookupError):
        os.kill(int(pid_file.read_text()), 0)


@pytest.mark.parametrize('outcome', ['ready', 'model_failed', 'timeout'])
def test_probe_process_with_synthetic_ros_data(outcome):
    # A separate domain is set by the test command. No real publishers or devices.
    rclpy.init()
    node = rclpy.create_node('synthetic_startup_test')
    publisher = node.create_publisher(DiagnosticArray, '/camera/health', 10)
    requests = []
    def model_ready(_request, response):
        requests.append(1)
        response.success = outcome == 'ready' and len(requests) > 1
        response.message = 'model load failed' if outcome == 'model_failed' else 'loading'
        return response
    node.create_service(Trigger, '/test/model_ready', model_ready)
    spec = {'name': 'synthetic', 'diagnostics': ['xlerobot/camera/wrist'],
            'services': ['/test/model_ready'], 'model_service': '/test/model_ready'}
    script = Path(__file__).resolve().parents[1] / 'scripts/startup_probe'
    child = subprocess.Popen([sys.executable, str(script), '--ros-args', '-p',
                              'stage_spec:=' + json.dumps(json.dumps(spec)), '-p',
                              'timeout_s:=4.0'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 12
        while child.poll() is None and time.monotonic() < deadline:
            if outcome != 'timeout':
                msg = diagnostic('xlerobot/camera/wrist', age='0.0')
                msg.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
        assert child.poll() is not None, 'startup probe did not honor its timeout'
        output = child.communicate(timeout=1)[0]
        assert (child.returncode == 0) == (outcome == 'ready'), output
        assert ('startup timeout' in output) == (outcome == 'timeout'), output
        if outcome == 'ready':
            assert len(requests) >= 2  # An initial loading result must not advance.
        if outcome == 'model_failed':
            assert 'model load failed' in output
    finally:
        if child.poll() is None:
            child.kill()
        child.communicate()
        node.destroy_node()
        rclpy.try_shutdown()
