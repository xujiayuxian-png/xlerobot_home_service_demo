from pathlib import Path
from concurrent.futures import Future
from types import SimpleNamespace
import threading
import wave

import pytest
import rclpy
from rclpy.parameter import Parameter
import yaml

from xlerobot_interfaces.msg import CapabilityError
from xlerobot_voice.speak_text_node import SpeakTextNode, SpeechFailure
from xlerobot_voice.voice_assistant_node import VoiceAssistantNode


@pytest.fixture(autouse=True)
def isolated_ros_domain(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '189')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')


def fixed_parameters(directory):
    path = Path(__file__).parents[1] / 'config/speak_fixed.yaml'
    config = yaml.safe_load(path.read_text())['speak_text_server']['ros__parameters']
    config['preset_directory'] = str(directory)
    config['backend_enabled'] = True
    for filename in config['preset_files']:
        with wave.open(str(directory / filename), 'wb') as audio:
            audio.setparams((2, 2, 48000, 0, 'NONE', 'not compressed'))
            audio.writeframes(bytes(480))
    return [Parameter(name, value=value) for name, value in config.items()]


def test_six_pcm_prompts_play_directly_and_unknown_text_never_generates(tmp_path):
    rclpy.init()
    node = SpeakTextNode(parameter_overrides=fixed_parameters(tmp_path))
    calls = []
    node._run_process = lambda goal, argv: calls.append(argv)
    try:
        assert len(node.presets) == 6
        for text in node.presets:
            node._play_preset(None, text)
        assert len(calls) == 6
        assert all(call[0] == 'paplay' and call[-1].endswith('.wav') for call in calls)
        with pytest.raises(SpeechFailure) as failure:
            node._play_preset(None, '好的，我去拿羽毛球')
        assert failure.value.code == CapabilityError.UNAVAILABLE
        assert len(calls) == 6
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_missing_or_wrong_format_pcm_fails_startup_validation(tmp_path):
    rclpy.init()
    node = SpeakTextNode(parameter_overrides=fixed_parameters(tmp_path))
    try:
        path = tmp_path / next(iter(node.presets.values()))
        with wave.open(str(path), 'wb') as audio:
            audio.setparams((1, 2, 16000, 0, 'NONE', 'not compressed'))
            audio.writeframes(bytes(160))
        with pytest.raises(ValueError, match='48 kHz'):
            node._validate_parameters()
        path.unlink()
        with pytest.raises(FileNotFoundError):
            node._validate_parameters()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_low_load_voice_keeps_open_intent_but_only_fixed_acceptance():
    rclpy.init()
    node = VoiceAssistantNode(parameter_overrides=[Parameter('x1_low_load', value=True)])
    calls = []
    node._speak = calls.append
    node._execute_task = lambda intent: intent.object_id
    try:
        assert node.whisper_threads == 2
        assert node.get_parameter('whisper_cpu_threads').value == 2
        assert not node.audio_debug_enabled
        for name in ('羽毛球', '杯子', '任意的新物品'):
            assert node._execute_task_with_parallel_ack(SimpleNamespace(object_id=name)) == name
        assert calls == ['好的，我去拿。'] * 3
        assert node.listening_cue_text == ''
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('late_acceptance', [False, True])
def test_task_timeout_keeps_listening_paused_until_terminal_result(monkeypatch, late_acceptance):
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    sent, terminal = Future(), Future()
    cancel_requested = threading.Event()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: terminal,
                             cancel_goal_async=cancel_requested.set)
    if not late_acceptance:
        sent.set_result(handle)
    client = SimpleNamespace(wait_for_server=lambda **kwargs: True,
                             send_goal_async=lambda *args, **kwargs: sent)
    node = SimpleNamespace(x1_low_load=True, execute_client=client, _task_not_started=False,
                           _active_goal_lock=threading.Lock(), _active_goals=[],
                           stop_event=threading.Event(),
                           get_logger=lambda: SimpleNamespace(warning=lambda text: None),
                           _wait_future=lambda future, timeout: future.done())
    node._wait_task_stopped = lambda future: VoiceAssistantNode._wait_task_stopped(node, future)
    results = []
    thread = threading.Thread(target=lambda: results.append(VoiceAssistantNode._call_action(
        node, client, object(), server_timeout_s=.01, result_timeout_s=.01)))
    thread.start()
    try:
        if late_acceptance:
            assert thread.is_alive()
            sent.set_result(handle)
        assert cancel_requested.wait(1)
        assert thread.is_alive() and not results
        terminal.set_result(SimpleNamespace(result='confirmed stopped'))
        thread.join(1)
        assert results == ['confirmed stopped']
        assert not node._active_goals and not node._task_not_started
    finally:
        node.stop_event.set()
        thread.join(1)


def test_rejected_task_is_distinguished_from_a_task_that_owns_its_terminal_prompt():
    future = Future()
    future.set_result(SimpleNamespace(accepted=False))
    client = SimpleNamespace(wait_for_server=lambda **kwargs: True,
                             send_goal_async=lambda *args, **kwargs: future)
    node = SimpleNamespace(x1_low_load=True, execute_client=client, _task_not_started=False,
                           get_logger=lambda: SimpleNamespace(warning=lambda text: None),
                           _wait_future=lambda future, timeout: True)
    assert VoiceAssistantNode._call_action(node, client, object(),
                                          server_timeout_s=1, result_timeout_s=1) is None
    assert node._task_not_started
