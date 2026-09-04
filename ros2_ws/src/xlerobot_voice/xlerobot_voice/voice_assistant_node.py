"""Gated wake-ASR-intent loop that dispatches only stable task actions."""

from __future__ import annotations

from enum import Enum
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from xlerobot_interfaces.action import ExecuteTask, SpeakText
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_voice.audio import (
    CommandRecorder,
    default_kws_paths,
    expand_path,
    FasterWhisperTranscriber,
    KwsConfig,
    RecorderConfig,
    save_recording_wav,
    select_capture_sample_rate,
    select_input_device,
    SherpaOnnxWakeDetector,
    Transcript,
)
from xlerobot_voice.intent import IntentParseError, LmStudioIntentClient
from xlerobot_voice.task_adapter import execute_task_goal


TASK_FAILURE_PROMPTS = {
    'auto_localize': '自动定位失败',
    'navigate_to_named_place': '导航到桌边失败',
    'detect_object': '物体识别失败',
    'grasp_object': '抓取失败',
    'scan_for_person': '寻找人员失败',
    'approach_target': '接近人员失败',
    'handover_object': '递送失败',
}

TASK_FAILURE_REASONS = {
    CapabilityError.INVALID_GOAL: '任务参数无效',
    CapabilityError.UNAVAILABLE: '对应服务没有启动',
    CapabilityError.TIMEOUT: '执行超时',
    CapabilityError.NOT_FOUND: '没有找到目标',
    CapabilityError.SAFETY_REJECTED: '当前运动条件不满足',
    CapabilityError.BACKEND_FAILURE: '后端服务执行异常',
    CapabilityError.INTERNAL_ERROR: '系统内部发生错误',
}


def task_failure_prompts(capability: str, error_code: int) -> tuple[str, ...]:
    """Return fixed stage and cause prompts for a failed task."""
    if int(error_code) == CapabilityError.CANCELED:
        return ()
    prompts = [TASK_FAILURE_PROMPTS.get(capability, '任务失败了')]
    reason = TASK_FAILURE_REASONS.get(int(error_code))
    if reason:
        prompts.append(reason)
    return tuple(prompts)


def transcript_is_acceptable(
    transcript: Transcript,
    *,
    minimum_language_probability,
    minimum_log_probability,
    maximum_no_speech_probability,
):
    """Use Whisper evidence rather than LM self-reported confidence."""
    return bool(transcript.text) and all([
        transcript.language_probability >= minimum_language_probability,
        transcript.average_log_probability >= minimum_log_probability,
        transcript.no_speech_probability <= maximum_no_speech_probability,
    ])


class VoiceState(str, Enum):
    """Stable coarse state intended for a thin HMI."""

    DISABLED = 'DISABLED'
    INITIALIZING = 'INITIALIZING'
    WAITING_FOR_WAKE = 'WAITING_FOR_WAKE'
    RECORDING = 'RECORDING'
    TRANSCRIBING = 'TRANSCRIBING'
    PARSING = 'PARSING'
    EXECUTING_TASK = 'EXECUTING_TASK'
    ERROR = 'ERROR'


class VoiceAssistantNode(Node):
    """Own voice I/O and task dispatch, but no controller or motor interfaces."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__('voice_assistant', parameter_overrides=parameter_overrides)
        self.group = ReentrantCallbackGroup()
        self.stop_event = threading.Event()
        self._active_goal_lock = threading.Lock()
        self._active_goals = []
        self._task_feedback_lock = threading.Lock()
        self._last_task_capability = ''
        self.worker = None

        self.audio_enabled = bool(self.declare_parameter('audio_enabled', False).value)
        self.intent_backend_enabled = bool(self.declare_parameter(
            'intent_backend_enabled', False
        ).value)
        self.task_dry_run = bool(self.declare_parameter('task_dry_run', True).value)
        self.speech_enabled = bool(self.declare_parameter('speech_enabled', False).value)
        self.speech_dry_run = bool(self.declare_parameter('speech_dry_run', True).value)
        self.execute_action = str(self.declare_parameter(
            'execute_task_action', '/execute_task'
        ).value)
        self.speak_action = str(self.declare_parameter('speak_text_action', '/speak_text').value)
        self.task_timeout_s = float(self.declare_parameter('task_timeout_s', 180.0).value)
        self.speech_timeout_s = float(self.declare_parameter('speech_timeout_s', 8.0).value)
        self.intent_threshold = float(self.declare_parameter(
            'intent_confidence_threshold', 0.60
        ).value)
        self.lmstudio_url = str(self.declare_parameter(
            'lmstudio_url', 'http://127.0.0.1:1234'
        ).value)
        self.lmstudio_model = str(self.declare_parameter(
            'lmstudio_model', 'qwen/qwen3-vl-4b'
        ).value)
        self.lmstudio_timeout_s = float(self.declare_parameter(
            'lmstudio_timeout_s', 6.0
        ).value)
        self.wake_ack_text = str(self.declare_parameter(
            'wake_ack_text', '我在，请问需要什么帮助'
        ).value)
        self.listening_cue_text = str(self.declare_parameter(
            'listening_cue_text', '叮'
        ).value)
        self.accepted_text = str(self.declare_parameter(
            'accepted_text_template', '好的，我去拿{object_id}'
        ).value)
        self.parse_failed_text = str(self.declare_parameter(
            'parse_failed_text', '我没有听清要拿什么东西'
        ).value)
        self.post_ack_delay_s = float(self.declare_parameter(
            'post_wake_ack_listen_delay_s', 0.0
        ).value)
        self._declare_audio_parameters()
        self._validate_parameters()

        self.state_publisher = self.create_publisher(String, '/voice/state', 10)
        self.transcript_publisher = self.create_publisher(
            String, '/voice/transcript', 10
        )
        self.execute_client = ActionClient(
            self, ExecuteTask, self.execute_action, callback_group=self.group
        )
        self.speak_client = ActionClient(
            self, SpeakText, self.speak_action, callback_group=self.group
        )
        if self.audio_enabled:
            self.worker = threading.Thread(
                target=self._run_loop,
                name='voice_assistant_loop',
                daemon=True,
            )
            self.worker.start()
        else:
            self._set_state(VoiceState.DISABLED)
            self.get_logger().info(
                'Voice audio is disabled; no microphone, model, or HTTP backend was opened'
            )

    def _declare_audio_parameters(self):
        self.kws_model_dir = str(self.declare_parameter(
            'kws_model_dir',
            '$HOME/.cache/xlerobot_models/'
            'sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20',
        ).value)
        self.kws_tokens = str(self.declare_parameter('kws_tokens', '').value)
        self.kws_encoder = str(self.declare_parameter('kws_encoder', '').value)
        self.kws_decoder = str(self.declare_parameter('kws_decoder', '').value)
        self.kws_joiner = str(self.declare_parameter('kws_joiner', '').value)
        self.kws_keywords_file = str(self.declare_parameter('kws_keywords_file', '').value)
        self.kws_threads = int(self.declare_parameter('kws_num_threads', 1).value)
        self.kws_provider = str(self.declare_parameter('kws_provider', 'cpu').value)
        self.kws_max_paths = int(self.declare_parameter('kws_max_active_paths', 4).value)
        self.kws_trailing_blanks = int(self.declare_parameter(
            'kws_num_trailing_blanks', 1
        ).value)
        self.kws_score = float(self.declare_parameter('kws_keywords_score', 2.5).value)
        self.kws_threshold = float(self.declare_parameter(
            'kws_keywords_threshold', 0.15
        ).value)
        self.sample_rate = int(self.declare_parameter('audio_sample_rate', 16000).value)
        self.audio_chunk_s = float(self.declare_parameter('audio_chunk_sec', 0.1).value)
        self.input_device = str(self.declare_parameter('audio_input_device', '').value)
        self.record_timeout_s = float(self.declare_parameter(
            'command_record_timeout_s', 4.0
        ).value)
        self.silence_timeout_s = float(self.declare_parameter(
            'command_silence_timeout_s', 0.65
        ).value)
        self.min_record_s = float(self.declare_parameter(
            'command_min_record_sec', 0.5
        ).value)
        self.start_rms = float(self.declare_parameter('command_start_rms', 0.035).value)
        self.silence_rms = float(self.declare_parameter(
            'command_silence_rms', 0.025
        ).value)
        self.audio_debug_enabled = bool(self.declare_parameter(
            'command_audio_debug_enabled', True
        ).value)
        self.audio_debug_dir = Path(expand_path(str(self.declare_parameter(
            'command_audio_debug_dir', '$HOME/.ros/xlerobot_voice/commands'
        ).value)))
        self.audio_debug_max_files = int(self.declare_parameter(
            'command_audio_debug_max_files', 20
        ).value)
        self.whisper_model = str(self.declare_parameter('whisper_model', 'small').value)
        self.whisper_compute_type = str(self.declare_parameter(
            'whisper_compute_type', 'int8'
        ).value)
        self.whisper_beam_size = int(self.declare_parameter('whisper_beam_size', 3).value)
        self.whisper_threads = int(self.declare_parameter('whisper_cpu_threads', 4).value)
        self.whisper_workers = int(self.declare_parameter('whisper_num_workers', 1).value)
        self.whisper_hotwords = str(self.declare_parameter(
            'whisper_hotwords', ''
        ).value)
        self.whisper_initial_prompt = str(self.declare_parameter(
            'whisper_initial_prompt', '这是家庭服务机器人取物命令。'
        ).value)
        self.whisper_min_language_probability = float(self.declare_parameter(
            'whisper_min_language_probability', 0.50
        ).value)
        self.whisper_min_log_probability = float(self.declare_parameter(
            'whisper_min_average_log_probability', -0.80
        ).value)
        self.whisper_max_no_speech_probability = float(self.declare_parameter(
            'whisper_max_no_speech_probability', 0.60
        ).value)

    def _validate_parameters(self):
        if not 0.0 <= self.intent_threshold <= 1.0:
            raise ValueError('intent_confidence_threshold must be in [0, 1]')
        positive = {
            'task_timeout_s': self.task_timeout_s,
            'speech_timeout_s': self.speech_timeout_s,
            'lmstudio_timeout_s': self.lmstudio_timeout_s,
            'post_wake_ack_listen_delay_s': self.post_ack_delay_s,
            'audio_sample_rate': self.sample_rate,
            'audio_chunk_sec': self.audio_chunk_s,
            'command_record_timeout_s': self.record_timeout_s,
            'command_silence_timeout_s': self.silence_timeout_s,
            'command_min_record_sec': self.min_record_s,
        }
        invalid = [
            name for name, value in positive.items()
            if value <= 0 and name != 'post_wake_ack_listen_delay_s'
        ]
        if self.post_ack_delay_s < 0:
            invalid.append('post_wake_ack_listen_delay_s')
        if invalid:
            raise ValueError(f'voice parameters must be positive: {invalid}')
        if self.start_rms <= self.silence_rms:
            raise ValueError('command_start_rms must be greater than command_silence_rms')
        if self.audio_debug_max_files <= 0:
            raise ValueError('command_audio_debug_max_files must be positive')
        probabilities = {
            'whisper_min_language_probability': self.whisper_min_language_probability,
            'whisper_max_no_speech_probability': self.whisper_max_no_speech_probability,
        }
        invalid_probabilities = [
            name for name, value in probabilities.items() if not 0.0 <= value <= 1.0
        ]
        if invalid_probabilities:
            raise ValueError(f'voice probabilities must be in [0, 1]: {invalid_probabilities}')
        try:
            self.accepted_text.format(object_id='test')
        except (KeyError, ValueError) as exc:
            raise ValueError(
                'accepted_text_template must be a valid template using {object_id}'
            ) from exc
        if '{object_id}' not in self.accepted_text:
            raise ValueError('accepted_text_template must contain {object_id}')

    def destroy_node(self):
        self.stop_event.set()
        with self._active_goal_lock:
            active_goals = list(self._active_goals)
        for active_goal in active_goals:
            active_goal.cancel_goal_async()
        if self.worker and self.worker.is_alive():
            self.worker.join(timeout=2.0)
        super().destroy_node()

    def _set_state(self, state):
        message = String()
        message.data = state.value
        self.state_publisher.publish(message)
        self.get_logger().info(f'voice_state={state.value}')

    def _build_kws_config(self, input_device=None, capture_sample_rate=None):
        package_keywords = (
            f'{get_package_share_directory("xlerobot_voice")}/config/xiaole_keywords.txt'
        )
        defaults = default_kws_paths(
            self.kws_model_dir, self.kws_keywords_file or package_keywords
        )
        return KwsConfig(
            tokens=expand_path(self.kws_tokens) or defaults['tokens'],
            encoder=expand_path(self.kws_encoder) or defaults['encoder'],
            decoder=expand_path(self.kws_decoder) or defaults['decoder'],
            joiner=expand_path(self.kws_joiner) or defaults['joiner'],
            keywords_file=expand_path(self.kws_keywords_file) or defaults['keywords_file'],
            num_threads=self.kws_threads,
            provider=self.kws_provider,
            max_active_paths=self.kws_max_paths,
            num_trailing_blanks=self.kws_trailing_blanks,
            keywords_score=self.kws_score,
            keywords_threshold=self.kws_threshold,
            sample_rate=self.sample_rate,
            capture_sample_rate=(
                self.sample_rate
                if capture_sample_rate is None
                else capture_sample_rate
            ),
            chunk_s=self.audio_chunk_s,
            input_device=self.input_device if input_device is None else input_device,
        )

    def _save_debug_recording(self, recording):
        if not self.audio_debug_enabled:
            return None
        path = self.audio_debug_dir / f'command_{time.time_ns()}.wav'
        try:
            save_recording_wav(recording, self.sample_rate, path)
            files = sorted(self.audio_debug_dir.glob('command_*.wav'))
            for stale in files[:-self.audio_debug_max_files]:
                stale.unlink(missing_ok=True)
            return path
        except OSError as exc:
            self.get_logger().warning(f'Unable to save command audio: {exc}')
            return None

    def _run_loop(self):
        self._set_state(VoiceState.INITIALIZING)
        if not self.intent_backend_enabled:
            self.get_logger().error(
                'Audio was enabled but intent_backend_enabled is false; '
                'refusing to open microphone'
            )
            self._set_state(VoiceState.ERROR)
            return
        try:
            import sounddevice

            input_device = select_input_device(
                sounddevice,
                self.input_device,
                channels=1,
            )
            capture_sample_rate = select_capture_sample_rate(
                sounddevice, input_device, self.sample_rate, channels=1
            )
            device_info = sounddevice.query_devices(input_device, 'input')
            self.get_logger().info(
                f'voice microphone={device_info["name"]} '
                f'capture_rate={capture_sample_rate}Hz model_rate={self.sample_rate}Hz'
            )
            intent_client = LmStudioIntentClient(
                base_url=self.lmstudio_url,
                model=self.lmstudio_model,
                timeout_s=self.lmstudio_timeout_s,
            )
            wake_detector = SherpaOnnxWakeDetector(
                self._build_kws_config(
                    input_device=input_device,
                    capture_sample_rate=capture_sample_rate,
                )
            )
            recorder = CommandRecorder(RecorderConfig(
                sample_rate=self.sample_rate,
                capture_sample_rate=capture_sample_rate,
                chunk_s=self.audio_chunk_s,
                timeout_s=self.record_timeout_s,
                silence_timeout_s=self.silence_timeout_s,
                min_record_s=self.min_record_s,
                start_rms=self.start_rms,
                silence_rms=self.silence_rms,
                input_device=input_device,
            ))
            transcriber = FasterWhisperTranscriber(
                model=self.whisper_model,
                compute_type=self.whisper_compute_type,
                beam_size=self.whisper_beam_size,
                cpu_threads=self.whisper_threads,
                workers=self.whisper_workers,
                hotwords=self.whisper_hotwords,
                initial_prompt=self.whisper_initial_prompt,
            )
            device = recorder.sounddevice.query_devices(
                input_device, 'input'
            )
            self.get_logger().info(
                f'Voice input device: {device["name"]}; '
                f'start_rms={self.start_rms:.4f} silence_rms={self.silence_rms:.4f}'
            )
        except Exception as exc:
            self.get_logger().error(f'Voice initialization failed: {exc}')
            self._set_state(VoiceState.ERROR)
            return

        while rclpy.ok() and not self.stop_event.is_set():
            self._set_state(VoiceState.WAITING_FOR_WAKE)
            wake_text = wake_detector.wait(self.stop_event)
            if not wake_text or self.stop_event.is_set():
                continue
            wake_detected_at = time.monotonic()
            self.get_logger().info(f'Wake detected: {wake_text}')
            self._speak(self.wake_ack_text)
            # Open ALSA while the short prerecorded cue is playing.  Once the
            # cue action completes, flush only the already-buffered cue audio
            # and begin reading from the same stream without reopening it.
            with recorder.open_stream() as microphone:
                self._speak(self.listening_cue_text)
                if self.post_ack_delay_s:
                    self.stop_event.wait(timeout=self.post_ack_delay_s)
                if self.stop_event.is_set():
                    break
                discarded = recorder.discard_buffered_input(microphone)
                self._set_state(VoiceState.RECORDING)
                recording_started_at = time.monotonic()
                self.get_logger().info(
                    'voice_latency wake_to_recording='
                    f'{recording_started_at - wake_detected_at:.3f}s '
                    f'prebuffer_discarded={discarded}'
                )
                recording = recorder.record(
                    self.stop_event, microphone=microphone
                )
            if self.stop_event.is_set():
                break
            debug_path = self._save_debug_recording(recording)
            self.get_logger().info(
                f'Command audio duration={recording.duration_s:.2f}s '
                f'mean_rms={recording.mean_rms:.4f} peak_rms={recording.peak_rms:.4f} '
                f'file={debug_path or "disabled"}'
            )
            self._set_state(VoiceState.TRANSCRIBING)
            asr_started_at = time.monotonic()
            transcript = transcriber.transcribe(recording.samples)
            asr_elapsed_s = time.monotonic() - asr_started_at
            self.get_logger().info(
                f'ASR text={transcript.text!r} '
                f'latency={asr_elapsed_s:.3f}s '
                f'language_probability={transcript.language_probability:.3f} '
                f'average_log_probability={transcript.average_log_probability:.3f} '
                f'no_speech_probability={transcript.no_speech_probability:.3f}'
            )
            self.transcript_publisher.publish(String(data=transcript.text.strip()))
            if not transcript_is_acceptable(
                transcript,
                minimum_language_probability=self.whisper_min_language_probability,
                minimum_log_probability=self.whisper_min_log_probability,
                maximum_no_speech_probability=self.whisper_max_no_speech_probability,
            ):
                self.get_logger().warning('ASR evidence rejected the command')
                self._speak(self.parse_failed_text)
                continue
            self._set_state(VoiceState.PARSING)
            intent_started_at = time.monotonic()
            try:
                intent = intent_client.parse(
                    transcript.text, min_confidence=self.intent_threshold
                )
            except IntentParseError as exc:
                self.get_logger().warning(f'Intent parse failed: {exc}')
                self._speak(self.parse_failed_text)
                continue
            self.get_logger().info(
                f'intent latency={time.monotonic() - intent_started_at:.3f}s '
                f'object_id={intent.object_id!r}'
            )

            self._set_state(VoiceState.EXECUTING_TASK)
            result = self._execute_task_with_parallel_ack(intent)
            if result is None or result.error.code != CapabilityError.NONE:
                message = 'no result' if result is None else result.error.message
                self.get_logger().warning(f'ExecuteTask failed: {message}')
                error_code = (
                    CapabilityError.INTERNAL_ERROR if result is None else result.error.code
                )
                with self._task_feedback_lock:
                    capability = self._last_task_capability
                for prompt in task_failure_prompts(capability, error_code):
                    self._speak(prompt)

    def _execute_task_with_parallel_ack(self, intent):
        """Start task navigation while the prerecorded acceptance is playing."""
        result = []
        failure = []

        def execute():
            try:
                result.append(self._execute_task(intent))
            except BaseException as exc:  # Preserve worker exceptions for the owner thread.
                failure.append(exc)

        worker = threading.Thread(
            target=execute,
            name='voice_execute_task',
            daemon=True,
        )
        started_at = time.monotonic()
        worker.start()
        self.get_logger().info(
            'ExecuteTask dispatch started in parallel with acceptance prompt'
        )
        self._speak(self.accepted_text.format(object_id=intent.object_id))
        while worker.is_alive():
            worker.join(timeout=0.05)
        self.get_logger().info(
            f'ExecuteTask completed after {time.monotonic() - started_at:.3f}s'
        )
        if failure:
            raise failure[0]
        return result[0] if result else None

    def _execute_task(self, intent):
        with self._task_feedback_lock:
            self._last_task_capability = ''
        goal = execute_task_goal(intent, dry_run=self.task_dry_run)
        return self._call_action(
            self.execute_client,
            goal,
            server_timeout_s=5.0,
            result_timeout_s=self.task_timeout_s,
            feedback_callback=self._on_task_feedback,
        )

    def _on_task_feedback(self, message):
        capability = str(message.feedback.current_capability).strip()
        if capability and capability != 'complete':
            with self._task_feedback_lock:
                self._last_task_capability = capability

    def _speak(self, text):
        if not self.speech_enabled or not text:
            return None
        goal = SpeakText.Goal()
        goal.text = text
        goal.voice = ''
        goal.dry_run = self.speech_dry_run
        return self._call_action(
            self.speak_client,
            goal,
            server_timeout_s=2.0,
            result_timeout_s=self.speech_timeout_s,
        )

    def _call_action(
        self,
        client,
        goal,
        *,
        server_timeout_s,
        result_timeout_s,
        feedback_callback=None,
    ):
        if not client.wait_for_server(timeout_sec=server_timeout_s):
            self.get_logger().warning('Action server is unavailable')
            return None
        send_future = client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        if not self._wait_future(send_future, server_timeout_s):
            self.get_logger().warning('Action goal response timed out')
            return None
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Action goal was rejected')
            return None
        with self._active_goal_lock:
            self._active_goals.append(goal_handle)
        result_future = goal_handle.get_result_async()
        try:
            if not self._wait_future(result_future, result_timeout_s):
                goal_handle.cancel_goal_async()
                self.get_logger().warning('Action result timed out and cancellation was requested')
                return None
            return result_future.result().result
        finally:
            with self._active_goal_lock:
                self._active_goals = [
                    active for active in self._active_goals
                    if active is not goal_handle
                ]

    def _wait_future(self, future, timeout_s):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not self.stop_event.is_set():
            if event.wait(timeout=0.02):
                return True
            if time.monotonic() >= deadline:
                return False
        return False


def main():
    """Run the gated voice adapter."""
    rclpy.init()
    node = VoiceAssistantNode()
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
