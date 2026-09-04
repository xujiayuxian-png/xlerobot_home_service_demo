import os
from pathlib import Path
import sys
import threading
import time

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from xlerobot_interfaces.action import SpeakText
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_voice.speak_text_node import (
    build_audio_player_command,
    build_command,
    SpeakTextNode,
    validate_text,
)


def wait_future(future, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done()
    return future.result()


def speech_goal(text, *, dry_run):
    goal = SpeakText.Goal()
    goal.text = text
    goal.dry_run = dry_run
    return goal


def run_graph(server, callback):
    client_node = Node('speak_text_runtime_client')
    client = ActionClient(client_node, SpeakText, 'speak_text')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=2.0)
        callback(client)
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        client.destroy()
        client_node.destroy_node()
        server.destroy_node()


def test_validation_and_argv_never_parse_text_as_shell():
    assert validate_text('  给你  ', 10) == '给你'
    assert build_command(['/bin/echo'], 'hello; touch /tmp/nope') == [
        '/bin/echo',
        'hello; touch /tmp/nope',
    ]
    assert build_audio_player_command(
        ['mpg123', '-q'], Path('cue.mp3'), 'plughw:0,0'
    ) == ['mpg123', '-q', '-a', 'plughw:0,0', 'cue.mp3']


def test_default_dry_run_and_live_gate():
    os.environ['ROS_DOMAIN_ID'] = '82'
    rclpy.init()
    server = SpeakTextNode()

    def check(client):
        dry_handle = wait_future(client.send_goal_async(speech_goal('给你', dry_run=True)))
        assert dry_handle.accepted
        dry = wait_future(dry_handle.get_result_async())
        assert dry.status == GoalStatus.STATUS_SUCCEEDED
        assert dry.result.error.code == CapabilityError.NONE

        live_handle = wait_future(client.send_goal_async(speech_goal('给你', dry_run=False)))
        assert live_handle.accepted
        live = wait_future(live_handle.get_result_async())
        assert live.status == GoalStatus.STATUS_ABORTED
        assert live.result.error.code == CapabilityError.SAFETY_REJECTED

        invalid = wait_future(client.send_goal_async(speech_goal(' ', dry_run=True)))
        assert not invalid.accepted

    try:
        run_graph(server, check)
    finally:
        rclpy.shutdown()


def test_enabled_process_success_cancel_and_timeout():
    os.environ['ROS_DOMAIN_ID'] = '82'
    rclpy.init()
    code = (
        'import sys,time; '
        'time.sleep(5.0 if sys.argv[1] == "stall" else 0.05)'
    )
    server = SpeakTextNode(
        parameter_overrides=[
            Parameter('backend_enabled', value=True),
            Parameter('command_argv', value=[sys.executable, '-c', code]),
            Parameter('timeout_s', value=0.4),
            Parameter('terminate_grace_s', value=0.2),
        ]
    )

    def check(client):
        success_handle = wait_future(
            client.send_goal_async(speech_goal('hello', dry_run=False))
        )
        success = wait_future(success_handle.get_result_async())
        assert success.status == GoalStatus.STATUS_SUCCEEDED
        assert success.result.error.code == CapabilityError.NONE

        cancel_handle = wait_future(
            client.send_goal_async(speech_goal('stall', dry_run=False))
        )
        time.sleep(0.1)
        canceled = wait_future(cancel_handle.cancel_goal_async())
        assert len(canceled.goals_canceling) == 1
        cancel_result = wait_future(cancel_handle.get_result_async())
        assert cancel_result.status == GoalStatus.STATUS_CANCELED
        assert cancel_result.result.error.code == CapabilityError.CANCELED

        timeout_handle = wait_future(
            client.send_goal_async(speech_goal('stall', dry_run=False))
        )
        timeout = wait_future(timeout_handle.get_result_async())
        assert timeout.status == GoalStatus.STATUS_ABORTED
        assert timeout.result.error.code == CapabilityError.TIMEOUT

    try:
        run_graph(server, check)
    finally:
        rclpy.shutdown()


def test_preset_backend_plays_packaged_file_and_rejects_unknown_text(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = '82'
    audio = tmp_path / 'handover.mp3'
    audio.write_bytes(b'prerecorded audio fixture')
    player = (
        'import pathlib,sys; '
        'path=pathlib.Path(sys.argv[-1]); '
        'sys.exit(0 if path.read_bytes() == b"prerecorded audio fixture" else 2)'
    )
    rclpy.init()
    server = SpeakTextNode(
        parameter_overrides=[
            Parameter('backend_enabled', value=True),
            Parameter('backend', value='preset'),
            Parameter('audio_player_argv', value=[sys.executable, '-c', player]),
            Parameter('preset_directory', value=str(tmp_path)),
            Parameter('preset_texts', value=['给你']),
            Parameter('preset_files', value=['handover.mp3']),
        ]
    )

    def check(client):
        success_handle = wait_future(
            client.send_goal_async(speech_goal('给你', dry_run=False))
        )
        success = wait_future(success_handle.get_result_async())
        assert success.status == GoalStatus.STATUS_SUCCEEDED
        assert success.result.error.code == CapabilityError.NONE

        missing_handle = wait_future(
            client.send_goal_async(speech_goal('未知提示', dry_run=False))
        )
        missing = wait_future(missing_handle.get_result_async())
        assert missing.status == GoalStatus.STATUS_ABORTED
        assert missing.result.error.code == CapabilityError.UNAVAILABLE

    try:
        run_graph(server, check)
    finally:
        rclpy.shutdown()


def test_hybrid_backend_plays_prerecorded_prefix_then_object(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = '82'
    (tmp_path / 'prefix.mp3').write_bytes(b'prefix')
    (tmp_path / 'object.mp3').write_bytes(b'object')
    playback_log = tmp_path / 'playback.log'
    player = (
        'import pathlib,sys; '
        f'log=pathlib.Path({str(playback_log)!r}); '
        'path=pathlib.Path(sys.argv[-1]); '
        'log.write_text(log.read_text() + path.name + "\\n" if log.exists() '
        'else path.name + "\\n")'
    )
    rclpy.init()
    server = SpeakTextNode(
        parameter_overrides=[
            Parameter('backend_enabled', value=True),
            Parameter('backend', value='hybrid'),
            Parameter('voice_id', value='test-voice'),
            Parameter('audio_player_argv', value=[sys.executable, '-c', player]),
            Parameter('preset_directory', value=str(tmp_path)),
            Parameter('preset_texts', value=['好的，我去拿', '剪刀']),
            Parameter('preset_files', value=['prefix.mp3', 'object.mp3']),
            Parameter('hybrid_prefix_texts', value=['好的，我去拿']),
        ]
    )

    def check(client):
        handle = wait_future(
            client.send_goal_async(speech_goal('好的，我去拿剪刀', dry_run=False))
        )
        result = wait_future(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        assert result.result.error.code == CapabilityError.NONE
        assert playback_log.read_text().splitlines() == ['prefix.mp3', 'object.mp3']

    try:
        run_graph(server, check)
    finally:
        rclpy.shutdown()


def test_hybrid_backend_generates_only_unknown_suffix_once(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = '82'
    (tmp_path / 'prefix.mp3').write_bytes(b'prefix')
    cache = tmp_path / 'cache'
    generation_log = tmp_path / 'generation.log'
    generator = (
        'import pathlib,sys; '
        'args=sys.argv[1:]; '
        'text=args[args.index("--text")+1]; '
        'output=pathlib.Path(args[args.index("--write-media")+1]); '
        'output.write_bytes(("generated:" + text).encode()); '
        f'log=pathlib.Path({str(generation_log)!r}); '
        'log.write_text(log.read_text() + text + "\\n" if log.exists() '
        'else text + "\\n")'
    )
    player = 'import pathlib,sys; assert pathlib.Path(sys.argv[-1]).stat().st_size > 0'
    rclpy.init()
    server = SpeakTextNode(
        parameter_overrides=[
            Parameter('backend_enabled', value=True),
            Parameter('backend', value='hybrid'),
            Parameter('voice_id', value='test-voice'),
            Parameter('audio_player_argv', value=[sys.executable, '-c', player]),
            Parameter('edge_tts_argv', value=[sys.executable, '-c', generator]),
            Parameter('edge_cache_dir', value=str(cache)),
            Parameter('preset_directory', value=str(tmp_path)),
            Parameter('preset_texts', value=['好的，我去拿']),
            Parameter('preset_files', value=['prefix.mp3']),
            Parameter('hybrid_prefix_texts', value=['好的，我去拿']),
        ]
    )

    def check(client):
        for _ in range(2):
            handle = wait_future(
                client.send_goal_async(speech_goal('好的，我去拿水杯', dry_run=False))
            )
            result = wait_future(handle.get_result_async())
            assert result.status == GoalStatus.STATUS_SUCCEEDED
            assert result.result.error.code == CapabilityError.NONE
        assert generation_log.read_text().splitlines() == ['水杯']
        assert len(list(cache.glob('*.mp3'))) == 1

    try:
        run_graph(server, check)
    finally:
        rclpy.shutdown()
