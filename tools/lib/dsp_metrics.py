"""Read X1's dsp_mon counters; never treat missing telemetry as zero load."""
import asyncio
import math
import re
import shutil
import time


class DspMetrics:
    def __init__(self):
        self.pending = {}
        self.latest = None
        self.updated = 0.
        self.error = 'not_started'
        self.running = False

    def feed(self, line):
        frequency = re.search(r'dsp current freq:\s*([\d.]+).*sample time\(ms\):\s*([\d.]+)', line)
        if frequency:
            self.pending = {'frequency_mhz': float(frequency[1]), 'sample_time_ms': float(frequency[2])}
        metric = re.search(r'\b(qdsp6|hvx|hmx) utilization:\s*([-+\w.]+)', line)
        if metric:
            try:
                value = float(metric[2])
            except ValueError:
                self.pending = {}
                return
            if not math.isfinite(value) or not 0 <= value <= 100:
                self.pending = {}
                return
            self.pending[metric[1]+'_percent'] = value
            if metric[1] == 'hmx' and len(self.pending) == 5:
                self.latest = dict(self.pending)
                self.updated = time.monotonic()
                self.pending = {}

    def value(self):
        age = time.monotonic()-self.updated if self.latest else None
        available = self.running and age is not None and age < 3
        return {'available': available, 'source': 'dsp_mon', 'age_s': age,
                'error': None if available else (self.error or 'stale_or_incomplete'),
                'metrics': self.latest if available else None}

    async def run(self, log_path):
        tool, stdbuf = shutil.which('dsp_mon'), shutil.which('stdbuf')
        if not tool or not stdbuf:
            self.error = 'tool_missing'
            return
        process = None
        try:
            with log_path.open('w', buffering=1) as log:
                process = await asyncio.create_subprocess_exec(
                    stdbuf, '-oL', '-eL', tool, '--all', '--follow', '--sample_time', '1000',
                    '--sample_interval_time', '100', stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.STDOUT)
                self.running = True
                self.error = None
                async for raw in process.stdout:
                    line = raw.decode(errors='replace')
                    log.write(line)
                    self.feed(line)
                self.error = f'tool_exited_{await process.wait()}'
        except OSError as error:
            self.error = type(error).__name__
        finally:
            self.running = False
            if process and process.returncode is None:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass
                try:
                    await asyncio.wait_for(process.wait(), timeout=2)
                except asyncio.TimeoutError:
                    process.kill()
                    await process.wait()
