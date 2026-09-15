"""Offline ACT ONNX/QNN benchmark; reads saved tensors and never sends actions."""
import argparse
import json
import os
from pathlib import Path
import resource
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--samples', type=Path, nargs='+', required=True)
    parser.add_argument('--backend', choices=['cpu', 'qnn'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=int, default=60)
    parser.add_argument('--rate', type=float, default=3)
    args = parser.parse_args()
    if not 1 <= args.duration <= 600 or not 0 < args.rate <= 30:
        parser.error('duration 1..600, rate (0,30] required')
    import numpy as np
    import onnxruntime as ort
    samples = []
    for path in args.samples:
        with np.load(path, allow_pickle=False) as data:
            row = {k: data[k] for k in ('image', 'state', 'reference')}
        for key, shape in [('image', (1, 3, 480, 640)), ('state', (1, 6)), ('reference', (1, 100, 6))]:
            if row[key].shape != shape or row[key].dtype != np.float32 or not np.isfinite(row[key]).all():
                raise ValueError(f'invalid sample {path}: {key}')
        samples.append((path.name, row))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 1
    options.enable_profiling = True
    options.profile_file_prefix = str(args.output.with_suffix('')) + '-ort'
    run_options = ort.RunOptions()
    if args.backend == 'qnn':
        import onnxruntime_qnn as qnn
        ort.register_execution_provider_library('QNNExecutionProvider', qnn.get_library_path())
        devices = [d for d in ort.get_ep_devices() if d.ep_name == 'QNNExecutionProvider']
        if not devices:
            raise RuntimeError('QNN did not discover the X1 NPU; no CPU fallback is allowed')
        options.add_session_config_entry('session.disable_cpu_ep_fallback', '1')
        options.add_provider_for_devices(devices, {
            'backend_path': qnn.get_qnn_htp_path(), 'htp_arch': '73', 'soc_model': '603',
            'enable_htp_fp16_precision': '1', 'htp_performance_mode': 'burst'})
        run_options.add_run_config_entry('qnn.perf_mode', 'burst')
    start = time.monotonic()
    session = ort.InferenceSession(str(args.model), sess_options=options,
                                   **({'providers': ['CPUExecutionProvider']} if args.backend == 'cpu' else {}))
    load_s = time.monotonic()-start
    error_rows = []
    for name, row in samples:
        output = session.run(None, {k: row[k] for k in ('image', 'state')}, run_options)[0]
        if output.shape != (1, 100, 6) or not np.isfinite(output).all():
            raise RuntimeError('invalid action block')
        error = np.abs(output-row['reference'])
        error_rows.append({'sample': name, 'max_abs': float(error.max()),
                           'mean_abs': float(error.mean()),
                           'per_joint_max_abs': error.max(axis=(0, 1)).tolist()})
    profile_path = session.end_profiling()
    profile = json.loads(Path(profile_path).read_text())
    providers = sorted({e.get('args', {}).get('provider') for e in profile
                        if e.get('args', {}).get('provider')})
    if args.backend == 'qnn' and providers != ['QNNExecutionProvider']:
        raise RuntimeError(f'profile does not confirm exclusive QNN execution: {providers}')
    start = time.monotonic()
    usage = resource.getrusage(resource.RUSAGE_SELF)
    cpu_start = usage.ru_utime + usage.ru_stime
    latencies = []
    while time.monotonic()-start < args.duration:
        _, row = samples[len(latencies) % len(samples)]
        before = time.monotonic()
        output = session.run(None, {k: row[k] for k in ('image', 'state')}, run_options)[0]
        latencies.append((time.monotonic()-before)*1000)
        if not np.isfinite(output).all():
            raise RuntimeError('non-finite actions during repeated inference')
        time.sleep(max(0, start+len(latencies)/args.rate-time.monotonic()))
    elapsed = time.monotonic()-start
    usage = resource.getrusage(resource.RUSAGE_SELF)
    cpu_s = usage.ru_utime + usage.ru_stime-cpu_start
    libraries = sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
                        if 'Qnn' in line or 'cdsprpc' in line})
    report = {'backend': args.backend, 'precision': 'HTP FP16' if args.backend == 'qnn' else 'CPU FP32',
              'ort_version': ort.__version__, 'load_s': load_s, 'duration_s': elapsed,
              'inferences': len(latencies), 'actual_rate_hz': len(latencies)/elapsed,
              'cpu_one_core_percent': 100*cpu_s/elapsed, 'cpu_s': cpu_s,
              'mean_ms': float(np.mean(latencies)), 'p95_ms': float(np.percentile(latencies, 95)),
              'max_ms': max(latencies), 'max_rss_kib': usage.ru_maxrss,
              'errors': error_rows, 'executed_providers': providers, 'runtime_libraries': libraries,
              'profile': profile_path, 'online_cpus': os.cpu_count()}
    args.output.write_text(json.dumps(report, indent=2))
    args.output.chmod(0o600)
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
