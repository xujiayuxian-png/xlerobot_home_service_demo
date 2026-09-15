"""Opt-in ASR client; confidence comes from the local NPU decoder logits."""

import base64
import json
import math
import time
from urllib.parse import urlparse
from urllib.request import Request, urlopen

import numpy as np
from .audio import Transcript


class NpuTranscriber:
    def __init__(self, base_url, timeout_s=10):
        parsed = urlparse(base_url)
        if parsed.scheme != 'http' or parsed.hostname != '127.0.0.1' or parsed.username or parsed.path not in ('', '/'):
            raise ValueError('NPU ASR requires a literal loopback HTTP URL')
        if not 0 < timeout_s <= 30:
            raise ValueError('invalid ASR timeout')
        self.url, self.timeout = base_url.rstrip('/'), timeout_s
        with urlopen(self.url+'/healthz', timeout=timeout_s) as response:
            health = json.loads(response.read(65536))
        if health.get('kind') != 'whisper' or health.get('device') != 'qnn-htp':
            raise RuntimeError('wrong NPU ASR worker; no CPU fallback')

    def transcribe(self, audio):
        if not isinstance(audio, np.ndarray) or audio.ndim != 1 or not 0 < audio.size <= 480000:
            return Transcript('', 0, float('-inf'), 1)
        if not np.isfinite(audio).all() or np.max(np.abs(audio)) > 1:
            raise ValueError('audio must be finite and normalized')
        started = time.monotonic()
        body = json.dumps({'audio_base64': base64.b64encode(audio.astype('<f4').tobytes()).decode(),
                           'sample_rate': 16000, 'deadline_monotonic': started+self.timeout}).encode()
        with urlopen(Request(self.url+'/predict', body, {'Content-Type': 'application/json'}), timeout=self.timeout) as response:
            raw = response.read(65537)
        if len(raw) > 65536 or time.monotonic()-started >= self.timeout:
            raise RuntimeError('NPU ASR response exceeded limits')
        value = json.loads(raw)
        if not isinstance(value, dict):
            raise ValueError('NPU ASR response must be an object')
        if value.get('finished') is not True or value.get('accepted') is not True:
            return Transcript('', 0, float('-inf'), 1)
        keys = ('language_probability', 'mean_token_log_probability', 'no_speech_probability')
        evidence = [value.get(k) for k in keys]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in evidence):
            raise ValueError('NPU ASR confidence evidence missing or invalid')
        lang, logprob, silence = evidence
        text = value.get('text')
        if not 0 <= lang <= 1 or not 0 <= silence <= 1 or logprob > 0 or not isinstance(text, str) or len(text) > 1000:
            raise ValueError('invalid NPU transcript')
        return Transcript(text.strip(), lang, logprob, silence)
