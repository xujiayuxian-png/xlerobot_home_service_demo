"""Gated, cancelable text-to-speech action without shell execution."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from xlerobot_interfaces.action import SpeakText
from xlerobot_interfaces.msg import CapabilityError


@dataclass
class SpeechFailure(RuntimeError):
    """Internal failure converted at the action boundary."""

    code: int
    message: str
    canceled: bool = False

    def __str__(self):
        return self.message


def validate_text(text: str, max_characters: int) -> str:
    """Return stripped speech text after strict bounded validation."""
    normalized = text.strip()
    if not normalized or len(normalized) > max_characters:
        raise ValueError(f'text must contain 1 to {max_characters} characters')
    if any(ord(character) < 32 and character not in ('\t', '\n') for character in normalized):
        raise ValueError('text contains unsupported control characters')
    return normalized


def build_command(base_argv, text: str):
    """Append text as one argv value; never parse it as shell syntax."""
    argv = [str(value) for value in base_argv]
    if not argv or any(not value for value in argv):
        raise ValueError('command_argv must contain nonempty values')
    return [*argv, text]


def build_audio_player_command(base_argv, path: Path, device: str = ''):
    """Build an mpg123 playback command with optional direct ALSA output."""
    argv = [str(value) for value in base_argv]
    if not argv or any(not value for value in argv):
        raise ValueError('audio_player_argv must contain nonempty values')
    selected = device.strip()
    if selected:
        if Path(argv[0]).name != 'mpg123':
            raise ValueError('audio_player_device is supported only with mpg123')
        argv.extend(['-a', selected])
    return [*argv, str(path)]


class SpeakTextNode(Node):
    """Own one bounded TTS subprocess while exposing only SpeakText."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__('speak_text_server', parameter_overrides=parameter_overrides)
        self.group = ReentrantCallbackGroup()
        self.backend_enabled = bool(self.declare_parameter('backend_enabled', False).value)
        self.backend = str(self.declare_parameter('backend', 'command').value)
        self.command_argv = [
            str(value)
            for value in self.declare_parameter('command_argv', ['spd-say', '-w']).value
        ]
        self.voice_id = str(self.declare_parameter('voice_id', '').value)
        self.audio_player_argv = [
            str(value)
            for value in self.declare_parameter(
                'audio_player_argv', ['mpg123', '-q']
            ).value
        ]
        self.audio_player_device = str(self.declare_parameter(
            'audio_player_device', ''
        ).value)
        self.edge_tts_argv = [
            str(value)
            for value in self.declare_parameter(
                'edge_tts_argv', [sys.executable, '-m', 'edge_tts']
            ).value
        ]
        self.edge_cache_dir = Path(os.path.expandvars(os.path.expanduser(str(
            self.declare_parameter(
                'edge_cache_dir', '.xlerobot/cache/tts'
            ).value
        ))))
        preset_directory = str(self.declare_parameter('preset_directory', '').value)
        self.preset_directory = Path(preset_directory) if preset_directory else Path(
            get_package_share_directory('xlerobot_voice')
        ) / 'audio'
        preset_texts = [
            str(value)
            for value in (self.declare_parameter(
                'preset_texts', Parameter.Type.STRING_ARRAY
            ).value or [])
        ]
        preset_files = [
            str(value)
            for value in (self.declare_parameter(
                'preset_files', Parameter.Type.STRING_ARRAY
            ).value or [])
        ]
        self.presets = self._build_presets(preset_texts, preset_files)
        self.hybrid_prefix_texts = [
            str(value).strip()
            for value in (self.declare_parameter(
                'hybrid_prefix_texts', Parameter.Type.STRING_ARRAY
            ).value or [])
        ]
        self.max_characters = int(self.declare_parameter('max_characters', 256).value)
        self.timeout_s = float(self.declare_parameter('timeout_s', 15.0).value)
        self.terminate_grace_s = float(self.declare_parameter('terminate_grace_s', 0.5).value)
        self._validate_parameters()
        self._goal_lock = threading.Lock()
        self._goal_active = False
        self._process_lock = threading.Lock()
        self._process = None
        self.server = ActionServer(
            self,
            SpeakText,
            'speak_text',
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group,
        )
        mode = 'enabled' if self.backend_enabled else 'disabled (dry-run only)'
        self.get_logger().info(f'SpeakText server ready; backend {mode}')

    def _validate_parameters(self):
        if self.backend not in ('command', 'preset', 'hybrid'):
            raise ValueError('backend must be command, preset, or hybrid')
        build_command(self.command_argv, 'validation')
        build_audio_player_command(
            self.audio_player_argv, Path('validation.mp3'), self.audio_player_device
        )
        if self.backend == 'preset' and not self.presets:
            raise ValueError('preset backend requires at least one text/file pair')
        if self.backend == 'hybrid':
            if not self.voice_id:
                raise ValueError('hybrid backend requires voice_id')
            if not self.edge_tts_argv or any(not value for value in self.edge_tts_argv):
                raise ValueError('edge_tts_argv must contain nonempty values')
            if any(not value for value in self.hybrid_prefix_texts):
                raise ValueError('hybrid_prefix_texts must contain nonempty values')
            missing = [
                prefix for prefix in self.hybrid_prefix_texts if prefix not in self.presets
            ]
            if missing:
                raise ValueError(f'hybrid prefixes require prerecorded presets: {missing}')
        if self.max_characters <= 0:
            raise ValueError('max_characters must be positive')
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0.0:
            raise ValueError('timeout_s must be finite and positive')
        if not math.isfinite(self.terminate_grace_s) or self.terminate_grace_s < 0.0:
            raise ValueError('terminate_grace_s must be finite and nonnegative')

    def goal_callback(self, request):
        try:
            validate_text(request.text, self.max_characters)
        except ValueError as exc:
            self.get_logger().warning(f'Rejecting SpeakText goal: {exc}')
            return GoalResponse.REJECT
        if request.voice and request.voice != self.voice_id:
            self.get_logger().warning(f'Unsupported voice id: {request.voice!r}')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_active:
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self._terminate_active_process()
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        result = SpeakText.Result()
        try:
            text = validate_text(goal_handle.request.text, self.max_characters)
            self._feedback(goal_handle, 'validate', 0.10, 'speech request valid')
            if goal_handle.request.dry_run:
                result.error.code = CapabilityError.NONE
                result.error.message = 'dry-run speech contract valid; no process started'
                goal_handle.succeed()
                self._feedback(goal_handle, 'complete', 1.0, result.error.message)
                return result
            if not self.backend_enabled:
                raise SpeechFailure(
                    CapabilityError.SAFETY_REJECTED,
                    'speech backend is disabled; set backend_enabled=true explicitly',
                )
            self._feedback(goal_handle, 'speaking', 0.30, text)
            if self.backend == 'preset':
                self._play_preset(goal_handle, text)
            elif self.backend == 'hybrid':
                self._play_hybrid(goal_handle, text)
            else:
                executable = self.command_argv[0]
                if shutil.which(executable) is None:
                    raise SpeechFailure(
                        CapabilityError.UNAVAILABLE,
                        f'speech command is unavailable: {executable}',
                    )
                self._run_process(
                    goal_handle, build_command(self.command_argv, text)
                )
            result.error.code = CapabilityError.NONE
            result.error.message = 'speech playback completed'
            goal_handle.succeed()
            self._feedback(goal_handle, 'complete', 1.0, result.error.message)
            return result
        except SpeechFailure as exc:
            result.error.code = int(exc.code)
            result.error.message = exc.message
            if exc.canceled or goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:
            result.error.code = CapabilityError.INTERNAL_ERROR
            result.error.message = str(exc)
            goal_handle.abort()
            return result
        finally:
            self._terminate_active_process()
            with self._goal_lock:
                self._goal_active = False

    @staticmethod
    def _build_presets(texts, files):
        if len(texts) != len(files):
            raise ValueError('preset_texts and preset_files must have equal lengths')
        presets = {}
        for text, filename in zip(texts, files):
            normalized = text.strip()
            path = Path(filename)
            if not normalized or not filename:
                raise ValueError('preset text and file values must be nonempty')
            if path.is_absolute() or '..' in path.parts:
                raise ValueError('preset files must be relative to preset_directory')
            if normalized in presets:
                raise ValueError(f'duplicate preset text: {normalized!r}')
            presets[normalized] = path
        return presets

    def _play_preset(self, goal_handle, text):
        player = self.audio_player_argv[0]
        if shutil.which(player) is None:
            raise SpeechFailure(
                CapabilityError.UNAVAILABLE,
                f'audio player is unavailable: {player}',
            )
        relative_path = self.presets.get(text)
        if relative_path is None:
            raise SpeechFailure(
                CapabilityError.UNAVAILABLE,
                f'no prerecorded speech preset for: {text}',
            )
        path = self.preset_directory / relative_path
        if not path.is_file() or path.stat().st_size == 0:
            raise SpeechFailure(
                CapabilityError.UNAVAILABLE,
                f'prerecorded speech file is unavailable: {path}',
            )
        self._run_process(
            goal_handle,
            build_audio_player_command(
                self.audio_player_argv, path, self.audio_player_device
            ),
        )

    def _play_hybrid(self, goal_handle, text):
        """Prefer fixed audio and synthesize only an uncached variable suffix."""
        if text in self.presets:
            self._play_preset(goal_handle, text)
            return
        prefixes = sorted(self.hybrid_prefix_texts, key=len, reverse=True)
        prefix = next(
            (candidate for candidate in prefixes if text.startswith(candidate)),
            None,
        )
        if prefix is None:
            self._play_generated(goal_handle, text)
            return
        suffix = text[len(prefix):].strip()
        if not suffix:
            self._play_preset(goal_handle, prefix)
            return
        self._play_preset(goal_handle, prefix)
        if suffix in self.presets:
            self._play_preset(goal_handle, suffix)
        else:
            self._play_generated(goal_handle, suffix)

    def _play_generated(self, goal_handle, text):
        """Generate one bounded fragment once, then reuse its local MP3 cache."""
        cache_key = hashlib.sha256(
            f'{self.voice_id}\0{text}'.encode('utf-8')
        ).hexdigest()
        path = self.edge_cache_dir / f'{cache_key}.mp3'
        if not path.is_file() or path.stat().st_size == 0:
            try:
                self.edge_cache_dir.mkdir(parents=True, exist_ok=True)
            except OSError as exc:
                raise SpeechFailure(CapabilityError.UNAVAILABLE, str(exc)) from exc
            temporary = self.edge_cache_dir / f'.{cache_key}.{os.getpid()}.tmp.mp3'
            try:
                self._run_process(goal_handle, [
                    *self.edge_tts_argv,
                    '--voice', self.voice_id,
                    '--text', text,
                    '--write-media', str(temporary),
                ])
                if not temporary.is_file() or temporary.stat().st_size == 0:
                    raise SpeechFailure(
                        CapabilityError.BACKEND_FAILURE,
                        'speech generation produced no audio',
                    )
                os.replace(temporary, path)
            finally:
                temporary.unlink(missing_ok=True)
        player = self.audio_player_argv[0]
        if shutil.which(player) is None:
            raise SpeechFailure(
                CapabilityError.UNAVAILABLE,
                f'audio player is unavailable: {player}',
            )
        self._run_process(
            goal_handle,
            build_audio_player_command(
                self.audio_player_argv, path, self.audio_player_device
            ),
        )

    def _run_process(self, goal_handle, argv):
        try:
            process = subprocess.Popen(
                argv,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError as exc:
            raise SpeechFailure(CapabilityError.UNAVAILABLE, str(exc)) from exc
        with self._process_lock:
            self._process = process
        deadline = time.monotonic() + self.timeout_s
        while process.poll() is None:
            if goal_handle.is_cancel_requested:
                self._terminate_active_process()
                raise SpeechFailure(
                    CapabilityError.CANCELED, 'speech canceled', canceled=True
                )
            if time.monotonic() >= deadline:
                self._terminate_active_process()
                raise SpeechFailure(CapabilityError.TIMEOUT, 'speech process timed out')
            time.sleep(0.02)
        if goal_handle.is_cancel_requested:
            raise SpeechFailure(
                CapabilityError.CANCELED, 'speech canceled', canceled=True
            )
        with self._process_lock:
            if self._process is process:
                self._process = None
        if process.returncode != 0:
            raise SpeechFailure(
                CapabilityError.BACKEND_FAILURE,
                f'speech command exited with code {process.returncode}',
            )

    def _terminate_active_process(self):
        with self._process_lock:
            process = self._process
        if process is None:
            return
        if process.poll() is not None:
            with self._process_lock:
                if self._process is process:
                    self._process = None
            return
        try:
            try:
                process.terminate()
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=self.terminate_grace_s)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
                process.wait(timeout=1.0)
        finally:
            with self._process_lock:
                if self._process is process:
                    self._process = None

    @staticmethod
    def _feedback(goal_handle, phase, progress, message):
        feedback = SpeakText.Feedback()
        feedback.state.phase = phase
        feedback.state.progress = float(progress)
        feedback.state.message = message
        goal_handle.publish_feedback(feedback)


def main():
    """Run the gated speech action server."""
    rclpy.init()
    node = SpeakTextNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
