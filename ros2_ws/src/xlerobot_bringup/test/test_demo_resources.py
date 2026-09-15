"""Resource arithmetic and stage events, without ROS or device access."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import pytest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'tools/run').is_file())
sys.path.insert(0, str(ROOT))
from tools.lib.demo_resources import Context, Summary, descendants
from tools.lib.dsp_metrics import DspMetrics


def test_dsp_metrics_require_complete_fresh_valid_samples(monkeypatch):
    from tools.lib import dsp_metrics
    clock = [10.]
    monkeypatch.setattr(dsp_metrics.time, 'monotonic', lambda: clock[0])
    dsp = DspMetrics()
    dsp.running = True
    dsp.feed('dsp current freq: 1171.2 (MHz), current sample time(ms): 1000.9')
    dsp.feed('qdsp6 utilization: 3.2 (%)')
    dsp.feed('hvx utilization: 5.4 (%)')
    assert not dsp.value()['available']
    dsp.feed('hmx utilization: 72.1 (%)')
    assert dsp.value()['metrics']['hmx_percent'] == 72.1
    clock[0] += 4
    assert not dsp.value()['available'] and dsp.value()['metrics'] is None
    dsp.feed('dsp current freq: 1000 (MHz), current sample time(ms): 1000')
    dsp.feed('qdsp6 utilization: nan (%)')
    dsp.feed('hvx utilization: 0 (%)')
    dsp.feed('hmx utilization: 0 (%)')
    assert not dsp.value()['available']


def test_process_tree_and_disconnected_context_do_not_claim_idle():
    assert descendants({'1': {'ppid': '0'}, '2': {'ppid': '1'}, '3': {'ppid': '2'},
                        '4': {'ppid': '0'}}, 1) == {'1', '2', '3'}
    context = Context()
    assert context.value()['stage'] == 'startup_or_telemetry_unavailable'
    context.connected = True
    context.accept({'type': 'task', 'data': {'task_id': 'test', 'status': 'RUNNING',
                                            'current_capability': 'grasp', 'phase': 'closing'}})
    assert context.value()['stage'] == 'grasp'
    context.connected = False
    assert context.value()['stage'] == 'startup_or_telemetry_unavailable'
    context.connected = True
    context.accept({'type': 'task', 'data': {'task_id': 'test', 'status': 'FAILED', 'error_code': 8}})
    assert context.value()['stage'] == 'idle'
    assert context.value()['task_error_code'] == 8


def test_summary_keeps_ten_second_peaks_and_bounded_samples():
    summary = Summary()
    for i in range(100):
        summary.add({'elapsed_s': 1, 'cpu_busy_percent': 80 if i < 10 else 10,
                     'demo_cpu_percent': 5, 'demo_rss_mib': 200, 'mem_available_mib': 1000,
                     'context': {'stage': 'grasp' if i < 10 else 'idle', 'voice_state': 'PAUSED'}})
    result = summary.value()
    assert result['max_10s_cpu_percent'] == pytest.approx(80)
    assert result['groups']['stage:grasp']['mean_cpu_percent'] == 80
    assert result['groups']['all']['mean_cpu_percent'] == 17
    assert len(summary.recent) == 10


def test_recorder_receives_short_events_and_flushes_on_stop(tmp_path):
    connected = threading.Event()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def do_GET(self):
            assert self.path == '/api/v1/events'
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.end_headers()
            for event in [
                {'type': 'snapshot', 'data': {'health': {'voice_state': 'LISTENING'}, 'task': None}},
                {'type': 'voice', 'data': {'state': 'TRANSCRIBING'}},
                {'type': 'task', 'data': {'task_id': 'sample', 'status': 'RUNNING', 'current_capability': 'detect_object'}},
                {'type': 'task', 'data': {'task_id': 'sample', 'status': 'RUNNING', 'current_capability': 'grasp'}},
            ]:
                self.wfile.write(('data: '+json.dumps(event)+'\n\n').encode())
                self.wfile.flush()
            connected.set()
            time.sleep(4)
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    output = tmp_path/'run'
    child = subprocess.Popen([sys.executable, str(ROOT/'tools/lib/demo_resources.py'),
                              '--no-dsp-monitor',
                              '--output', str(output), '--events-url',
                              f'http://127.0.0.1:{server.server_port}/api/v1/events'])
    try:
        assert connected.wait(5)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and not (output/'summary.json').exists():
            time.sleep(.05)
        child.send_signal(signal.SIGTERM)
        assert child.wait(timeout=5) == 0
        summary = json.loads((output/'summary.json').read_text())
        assert summary['finished'] and summary['groups']['all']['samples'] >= 1
        events = [json.loads(line) for line in (output/'events.jsonl').read_text().splitlines()]
        assert {'detect_object', 'grasp'} <= {r['context']['stage'] for r in events}
        samples = [json.loads(line) for line in (output/'samples.jsonl').read_text().splitlines()]
        assert samples[0]['demo_rss_mib'] > 0
        assert 0 <= samples[0]['cpu_busy_percent'] <= 100
    finally:
        if child.poll() is None:
            child.kill(); child.wait()
        server.shutdown()
        server.server_close()
