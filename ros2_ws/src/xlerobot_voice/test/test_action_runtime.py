import os
import threading

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from xlerobot_interfaces.action import ExecuteTask
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_voice.intent import IntentResult
from xlerobot_voice.voice_assistant_node import VoiceAssistantNode


def test_disabled_audio_dispatches_typed_dry_run_without_models_or_network():
    os.environ['ROS_DOMAIN_ID'] = '82'
    rclpy.init()
    backend = Node('voice_test_task_backend')
    received = []

    def execute(goal_handle):
        received.append(goal_handle.request)
        feedback = ExecuteTask.Feedback()
        feedback.current_capability = 'grasp_object'
        feedback.state.phase = 'policy'
        goal_handle.publish_feedback(feedback)
        result = ExecuteTask.Result()
        result.error.code = CapabilityError.NONE
        result.error.message = 'test task complete'
        result.completed_object_id = goal_handle.request.object_id
        goal_handle.succeed()
        return result

    server = ActionServer(backend, ExecuteTask, '/execute_task', execute_callback=execute)
    voice = VoiceAssistantNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(backend)
    executor.add_node(voice)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    intent = IntentResult(
        intent='fetch_deliver',
        object_id='camping_lamp',
        source_place='table',
        recipient_id='nearest_person',
        confidence=0.95,
        normalized_command='bring the camping lamp',
    )
    try:
        result = voice._execute_task(intent)
        assert result.error.code == CapabilityError.NONE
        assert result.completed_object_id == 'camping_lamp'
        assert len(received) == 1
        assert received[0].dry_run
        assert voice._last_task_capability == 'grasp_object'
        assert not voice.audio_enabled
        assert not voice.intent_backend_enabled
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        server.destroy()
        voice.destroy_node()
        backend.destroy_node()
        rclpy.shutdown()
