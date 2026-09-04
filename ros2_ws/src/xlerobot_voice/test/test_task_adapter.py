import threading

from xlerobot_voice.intent import IntentResult
from xlerobot_voice.task_adapter import execute_task_goal
from xlerobot_voice.voice_assistant_node import VoiceAssistantNode


def test_current_execute_task_contract_has_no_legacy_json_fields():
    intent = IntentResult(
        intent='fetch_deliver',
        object_id='露营灯',
        source_place='table',
        recipient_id='nearest_person',
        confidence=0.95,
        normalized_command='把露营灯拿过来',
    )
    goal = execute_task_goal(intent, dry_run=True, grasp_backend='centroid')
    assert goal.object_id == '露营灯'
    assert goal.source_place == 'table'
    assert goal.recipient_id == 'nearest_person'
    assert goal.grasp_backend == 'centroid'
    assert goal.dry_run
    assert not hasattr(goal, 'command_text')
    assert not hasattr(goal, 'task_json')


def test_voice_backend_is_configuration_not_language_intent():
    intent = IntentResult(
        intent='fetch_deliver', object_id='羽毛球', source_place='table',
        recipient_id='nearest_person', confidence=0.9,
        normalized_command='拿羽毛球',
    )
    for backend in ('act', 'centroid', 'gpd'):
        assert execute_task_goal(
            intent, dry_run=True, grasp_backend=backend
        ).grasp_backend == backend


def test_acceptance_prompt_overlaps_task_dispatch():
    intent = IntentResult(
        intent='fetch_deliver',
        object_id='羽毛球',
        source_place='table',
        recipient_id='nearest_person',
        confidence=0.95,
        normalized_command='拿羽毛球',
    )

    class Logger:
        def info(self, _message):
            pass

    class Harness:
        accepted_text = '好的，我去拿{object_id}'

        def __init__(self):
            self.task_started = threading.Event()
            self.prompt_started = threading.Event()

        def get_logger(self):
            return Logger()

        def _execute_task(self, _intent):
            self.task_started.set()
            assert self.prompt_started.wait(timeout=1.0)
            return 'task-result'

        def _speak(self, text):
            assert text == '好的，我去拿羽毛球'
            assert self.task_started.wait(timeout=1.0)
            self.prompt_started.set()

    harness = Harness()
    result = VoiceAssistantNode._execute_task_with_parallel_ack(harness, intent)
    assert result == 'task-result'
