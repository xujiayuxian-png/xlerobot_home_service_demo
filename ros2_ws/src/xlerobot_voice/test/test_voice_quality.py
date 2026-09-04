"""Regression tests for voice signal and intent quality boundaries."""

from pathlib import Path
import wave

import numpy as np

from xlerobot_interfaces.msg import CapabilityError
from xlerobot_voice.audio import (
    CommandRecorder,
    Recording,
    resample_audio,
    save_recording_wav,
    select_capture_sample_rate,
    select_input_device,
    Transcript,
)
from xlerobot_voice.voice_assistant_node import (
    task_failure_prompts,
    transcript_is_acceptable,
)


def test_whisper_evidence_rejects_low_confidence_or_no_speech():
    """Only model-supported speech evidence reaches semantic parsing."""
    accepted = Transcript('拿羽毛球给我', 0.98, -0.25, 0.02)
    low_confidence = Transcript('拿这里跑去后面', 0.98, -1.2, 0.02)
    no_speech = Transcript('鱼毛球给我', 0.98, -0.25, 0.85)

    def evaluate(transcript):
        return transcript_is_acceptable(
            transcript,
            minimum_language_probability=0.5,
            minimum_log_probability=-0.8,
            maximum_no_speech_probability=0.6,
        )

    assert evaluate(accepted)
    assert not evaluate(low_confidence)
    assert not evaluate(no_speech)


def test_task_failure_prompt_names_stage_and_ignores_operator_cancel():
    assert task_failure_prompts(
        'navigate_to_named_place', CapabilityError.TIMEOUT
    ) == ('导航到桌边失败', '执行超时')
    assert task_failure_prompts(
        'grasp_object', CapabilityError.BACKEND_FAILURE
    ) == ('抓取失败', '后端服务执行异常')
    assert task_failure_prompts(
        '', CapabilityError.INTERNAL_ERROR
    ) == ('任务失败了', '系统内部发生错误')
    assert task_failure_prompts('grasp_object', CapabilityError.CANCELED) == ()


def test_debug_recording_is_bounded_mono_pcm(tmp_path):
    """Local diagnostic WAVs are valid, bounded mono PCM files."""
    recording = Recording(
        samples=np.array([-1.2, -0.5, 0.0, 0.5, 1.2], dtype=np.float32),
        duration_s=5 / 16000,
        mean_rms=0.5,
        peak_rms=1.2,
    )
    path = tmp_path / 'command.wav'
    save_recording_wav(recording, 16000, path)
    with wave.open(str(path), 'rb') as captured:
        assert captured.getnchannels() == 1
        assert captured.getsampwidth() == 2
        assert captured.getframerate() == 16000
        assert captured.getnframes() == 5


class _FakeSoundDevice:
    def __init__(self, devices):
        self.devices = devices

    def query_devices(self, device=None, kind=None):
        if device is None:
            return self.devices
        return self.devices[device] if isinstance(device, int) else {'name': device}


class _RateAwareSoundDevice:
    def query_devices(self, device=None, kind=None):
        return {
            'name': 'UACDemoV1.0: USB Audio (hw:1,0)',
            'default_samplerate': 48000.0,
        }

    def check_input_settings(self, *, samplerate, **_kwargs):
        if int(samplerate) != 48000:
            raise RuntimeError('invalid sample rate')


def test_auto_input_prefers_physical_microphone_over_null_monitor():
    sounddevice = _FakeSoundDevice([
        {'name': 'auto_null.monitor', 'max_input_channels': 2},
        {'name': 'pipewire', 'max_input_channels': 64},
        {'name': 'HDA Intel PCH: ALC287 Analog (hw:0,0)', 'max_input_channels': 2},
    ])

    assert select_input_device(sounddevice, 'auto') == 2


def test_auto_input_fails_when_only_virtual_silence_is_available():
    sounddevice = _FakeSoundDevice([
        {'name': 'auto_null.monitor', 'max_input_channels': 2},
        {'name': 'pipewire', 'max_input_channels': 64},
        {'name': 'default', 'max_input_channels': 64},
    ])

    try:
        select_input_device(
            sounddevice, 'auto', alsa_capture_accessible=False
        )
    except RuntimeError as exc:
        assert 'no physical audio input' in str(exc)
    else:
        raise AssertionError('virtual silence must not satisfy voice readiness')


def test_usb_microphone_uses_native_capture_rate_and_resamples_to_model_rate():
    sounddevice = _RateAwareSoundDevice()
    assert select_capture_sample_rate(
        sounddevice, 'UACDemoV1.0', 16000
    ) == 48000
    source = np.linspace(-0.5, 0.5, 4800, dtype=np.float32)
    converted = resample_audio(source, 48000, 16000)
    assert converted.dtype == np.float32
    assert converted.shape == (1600,)


def test_wake_keyword_uses_central_tuning_without_hidden_per_phrase_override():
    keyword_file = Path(__file__).resolve().parents[1] / 'config' / 'xiaole_keywords.txt'
    keyword = keyword_file.read_text(encoding='utf-8').strip()
    assert keyword == 'x iǎo l è x iǎo l è @小乐小乐'
    assert ':' not in keyword
    assert '#' not in keyword


def test_listening_cue_prebuffer_is_discarded_without_reopening_microphone():
    class BufferedMicrophone:
        def __init__(self):
            self.buffered = [4800, 960]
            self.reads = []

        @property
        def read_available(self):
            return self.buffered.pop(0) if self.buffered else 0

        def read(self, count):
            self.reads.append(count)
            return np.zeros((count, 1), dtype=np.float32), False

    microphone = BufferedMicrophone()
    assert CommandRecorder.discard_buffered_input(microphone) == 5760
    assert microphone.reads == [4800, 960]
