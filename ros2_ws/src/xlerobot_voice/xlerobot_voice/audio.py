"""Lazy optional local KWS, microphone, and ASR adapters."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
from threading import Event
import time
from typing import Any
import wave

import numpy as np


@dataclass(frozen=True)
class KwsConfig:
    """All files and tuning required by sherpa-onnx keyword spotting."""

    tokens: str
    encoder: str
    decoder: str
    joiner: str
    keywords_file: str
    num_threads: int
    provider: str
    max_active_paths: int
    num_trailing_blanks: int
    keywords_score: float
    keywords_threshold: float
    sample_rate: int
    capture_sample_rate: int
    chunk_s: float
    input_device: Any


@dataclass(frozen=True)
class RecorderConfig:
    """Voice activity thresholds and bounded command duration."""

    sample_rate: int
    capture_sample_rate: int
    chunk_s: float
    timeout_s: float
    silence_timeout_s: float
    min_record_s: float
    start_rms: float
    silence_rms: float
    input_device: Any


@dataclass(frozen=True)
class Recording:
    """One captured command and the signal levels used by the VAD."""

    samples: np.ndarray
    duration_s: float
    mean_rms: float
    peak_rms: float


@dataclass(frozen=True)
class Transcript:
    """Whisper text with model-derived confidence evidence."""

    text: str
    language_probability: float
    average_log_probability: float
    no_speech_probability: float


def expand_path(value: str) -> str:
    """Expand user and environment markers in configured model paths."""
    return os.path.expandvars(os.path.expanduser(value or ''))


def default_kws_paths(model_dir: str, keywords_file: str):
    """Return the filenames verified for the selected Chinese KWS model."""
    root = Path(expand_path(model_dir))
    return {
        'tokens': str(root / 'tokens.txt'),
        'encoder': str(root / 'encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx'),
        'decoder': str(root / 'decoder-epoch-13-avg-2-chunk-16-left-64.onnx'),
        'joiner': str(root / 'joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx'),
        'keywords_file': expand_path(keywords_file),
    }


def require_files(paths):
    """Fail before opening a microphone if any model artifact is absent."""
    missing = [path for path in paths if not Path(path).is_file()]
    if missing:
        raise FileNotFoundError('missing voice model/config files: ' + ', '.join(missing))


def select_input_device(
    sounddevice,
    requested: str,
    *,
    sample_rate: int | None = None,
    channels: int = 1,
    alsa_capture_accessible: bool | None = None,
):
    """
    Resolve ``auto`` to a physical capture device, never a null monitor.

    Desktop PipeWire creates ``auto_null.monitor`` when the process cannot open
    ALSA.  PortAudio considers that a valid input, so accepting the default
    device would make the voice state look healthy while every sample is zero.
    """
    configured = (requested or '').strip()
    if configured and configured.lower() != 'auto':
        sounddevice.query_devices(configured, 'input')
        return configured

    rejected_markers = (
        'auto_null',
        'monitor',
        'pipewire',
        'pulse',
        'null',
        'hdmi',
    )
    if alsa_capture_accessible is None:
        alsa_capture_accessible = any(
            os.access(path, os.R_OK | os.W_OK)
            for path in Path('/dev/snd').glob('pcm*C*c')
        )
    candidates = []
    for index, device in enumerate(sounddevice.query_devices()):
        if int(device.get('max_input_channels', 0)) < 1:
            continue
        name = str(device.get('name', ''))
        lowered = name.lower()
        if any(marker in lowered for marker in rejected_markers):
            continue
        if 'default' in lowered and not alsa_capture_accessible:
            continue
        score = 0
        if '(hw:' in lowered:
            score += 8
        if 'mic' in lowered or 'analog' in lowered:
            score += 4
        if 'default' in lowered or 'sysdefault' in lowered:
            score -= 2
        candidates.append((score, index, name))
    if not candidates:
        raise RuntimeError(
            'no physical audio input is available; check /dev/snd access and '
            'the service account audio group'
        )
    candidates.sort(key=lambda item: (item[0], -item[1]), reverse=True)
    failures = []
    for _score, index, name in candidates:
        if sample_rate is None:
            return index
        try:
            sounddevice.check_input_settings(
                device=index,
                channels=channels,
                dtype='float32',
                samplerate=sample_rate,
            )
            return index
        except Exception as exc:
            failures.append(f'{name}: {exc}')
    raise RuntimeError(
        f'no physical audio input supports {sample_rate} Hz/{channels} channel; '
        + '; '.join(failures)
    )


def select_capture_sample_rate(sounddevice, device, desired_rate: int, channels: int = 1):
    """Use the model rate when possible, otherwise the physical device native rate."""
    try:
        sounddevice.check_input_settings(
            device=device,
            channels=channels,
            dtype='float32',
            samplerate=desired_rate,
        )
        return int(desired_rate)
    except Exception as desired_error:
        info = sounddevice.query_devices(device, 'input')
        native_rate = int(round(float(info.get('default_samplerate', 0.0))))
        if native_rate <= 0:
            raise RuntimeError(
                f'audio input does not report a usable native sample rate: {desired_error}'
            ) from desired_error
        try:
            sounddevice.check_input_settings(
                device=device,
                channels=channels,
                dtype='float32',
                samplerate=native_rate,
            )
        except Exception as native_error:
            raise RuntimeError(
                f'audio input supports neither {desired_rate} Hz nor its reported '
                f'native rate {native_rate} Hz: {native_error}'
            ) from native_error
        return native_rate


def resample_audio(samples, source_rate: int, target_rate: int):
    """Return mono float32 audio at the model rate using polyphase filtering."""
    chunk = np.asarray(samples, dtype=np.float32).reshape(-1)
    if source_rate == target_rate or chunk.size == 0:
        return chunk
    if source_rate <= 0 or target_rate <= 0:
        raise ValueError('audio sample rates must be positive')
    from scipy.signal import resample_poly

    divisor = math.gcd(int(source_rate), int(target_rate))
    converted = resample_poly(
        chunk,
        int(target_rate) // divisor,
        int(source_rate) // divisor,
    )
    return np.asarray(converted, dtype=np.float32)


class SherpaOnnxWakeDetector:
    """Block on a local microphone until the configured wake word is detected."""

    def __init__(self, config: KwsConfig):
        try:
            import sherpa_onnx
            import sounddevice
        except ImportError as exc:
            raise RuntimeError('wake word requires sounddevice and sherpa-onnx') from exc
        require_files([
            config.tokens,
            config.encoder,
            config.decoder,
            config.joiner,
            config.keywords_file,
        ])
        self.sounddevice = sounddevice
        self.config = config
        self.spotter = sherpa_onnx.KeywordSpotter(
            tokens=config.tokens,
            encoder=config.encoder,
            decoder=config.decoder,
            joiner=config.joiner,
            num_threads=config.num_threads,
            max_active_paths=config.max_active_paths,
            keywords_file=config.keywords_file,
            keywords_score=config.keywords_score,
            keywords_threshold=config.keywords_threshold,
            num_trailing_blanks=config.num_trailing_blanks,
            provider=config.provider,
        )

    def wait(self, stop_event: Event):
        """Return detected text or None after shutdown."""
        count = max(1, int(self.config.capture_sample_rate * self.config.chunk_s))
        stream = self.spotter.create_stream()
        with self.sounddevice.InputStream(
            device=self.config.input_device or None,
            channels=1,
            dtype='float32',
            samplerate=self.config.capture_sample_rate,
        ) as microphone:
            while not stop_event.is_set():
                samples, _overflow = microphone.read(count)
                chunk = resample_audio(
                    samples,
                    self.config.capture_sample_rate,
                    self.config.sample_rate,
                )
                stream.accept_waveform(self.config.sample_rate, chunk)
                while self.spotter.is_ready(stream):
                    self.spotter.decode_stream(stream)
                result = self.spotter.get_result(stream)
                if result:
                    self.spotter.reset_stream(stream)
                    return str(result)
        return None


class CommandRecorder:
    """Capture one bounded utterance after wake acknowledgement."""

    def __init__(self, config: RecorderConfig):
        try:
            import sounddevice
        except ImportError as exc:
            raise RuntimeError('command recording requires sounddevice') from exc
        self.sounddevice = sounddevice
        self.config = config

    def open_stream(self):
        """Create the capture stream so callers can open it during the cue."""
        return self.sounddevice.InputStream(
            device=self.config.input_device or None,
            channels=1,
            dtype='float32',
            samplerate=self.config.capture_sample_rate,
        )

    @staticmethod
    def discard_buffered_input(microphone):
        """Discard audio captured before the listening cue completed."""
        discarded = 0
        for _ in range(8):
            available = max(0, int(getattr(microphone, 'read_available', 0)))
            if available <= 0:
                break
            microphone.read(available)
            discarded += available
        return discarded

    def record(self, stop_event: Event, *, microphone=None):
        """Return mono float32 audio, optionally reusing an open stream."""
        if microphone is None:
            with self.open_stream() as opened:
                return self._record_from_stream(stop_event, opened)
        return self._record_from_stream(stop_event, microphone)

    def _record_from_stream(self, stop_event: Event, microphone):
        """Capture one utterance from an already started input stream."""
        count = max(1, int(self.config.capture_sample_rate * self.config.chunk_s))
        deadline = time.monotonic() + self.config.timeout_s
        started_at = None
        silence_started_at = None
        frames = []
        rms_values = []
        while not stop_event.is_set() and time.monotonic() < deadline:
            samples, _overflow = microphone.read(count)
            chunk = resample_audio(
                samples,
                self.config.capture_sample_rate,
                self.config.sample_rate,
            )
            rms = math.sqrt(float(np.mean(np.square(chunk)))) if chunk.size else 0.0
            now = time.monotonic()
            if started_at is None:
                if rms < self.config.start_rms:
                    continue
                started_at = now
            frames.append(chunk.copy())
            rms_values.append(rms)
            if rms < self.config.silence_rms and now - started_at >= self.config.min_record_s:
                silence_started_at = silence_started_at or now
                if now - silence_started_at >= self.config.silence_timeout_s:
                    break
            else:
                silence_started_at = None
        if not frames:
            samples = np.zeros(0, dtype=np.float32)
        else:
            samples = np.concatenate(frames).astype(np.float32, copy=False)
        return Recording(
            samples=samples,
            duration_s=float(samples.size) / float(self.config.sample_rate),
            mean_rms=float(np.mean(rms_values)) if rms_values else 0.0,
            peak_rms=max(rms_values, default=0.0),
        )


def save_recording_wav(recording: Recording, sample_rate: int, path: Path):
    """Store bounded debug audio as mono 16-bit PCM without another dependency."""
    path.parent.mkdir(parents=True, exist_ok=True)
    clipped = np.clip(recording.samples, -1.0, 1.0)
    pcm = (clipped * 32767.0).astype('<i2', copy=False)
    with wave.open(str(path), 'wb') as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(sample_rate)
        output.writeframes(pcm.tobytes())


class FasterWhisperTranscriber:
    """Lazy CPU-only faster-whisper wrapper with explicit profile context."""

    def __init__(
        self,
        *,
        model,
        compute_type,
        beam_size,
        cpu_threads,
        workers,
        hotwords='',
        initial_prompt='',
    ):
        try:
            from faster_whisper import WhisperModel
        except ImportError as exc:
            raise RuntimeError('ASR requires faster-whisper') from exc
        self.model = WhisperModel(
            model,
            device='cpu',
            compute_type=compute_type,
            cpu_threads=max(0, int(cpu_threads)),
            num_workers=max(1, int(workers)),
        )
        self.beam_size = max(1, int(beam_size))
        self.hotwords = hotwords.strip()
        self.initial_prompt = initial_prompt.strip()

    def transcribe(self, audio):
        """Transcribe one Chinese command without cross-utterance context."""
        if not audio.size:
            return Transcript('', 0.0, float('-inf'), 1.0)
        segments, info = self.model.transcribe(
            audio,
            language='zh',
            beam_size=self.beam_size,
            best_of=1,
            temperature=0.0,
            condition_on_previous_text=False,
            initial_prompt=self.initial_prompt or None,
            without_timestamps=True,
            vad_filter=False,
            hotwords=self.hotwords or None,
        )
        segments = list(segments)
        text = ''.join(segment.text.strip() for segment in segments).strip()
        weights = [
            max(0.01, float(segment.end - segment.start))
            for segment in segments
        ]
        total = sum(weights)
        average_log_probability = (
            sum(
                float(segment.avg_logprob) * weight
                for segment, weight in zip(segments, weights)
            )
            / total
            if segments
            else float('-inf')
        )
        no_speech_probability = (
            sum(
                float(segment.no_speech_prob) * weight
                for segment, weight in zip(segments, weights)
            )
            / total
            if segments
            else 1.0
        )
        return Transcript(
            text=text,
            language_probability=float(getattr(info, 'language_probability', 0.0)),
            average_log_probability=average_log_probability,
            no_speech_probability=no_speech_probability,
        )
