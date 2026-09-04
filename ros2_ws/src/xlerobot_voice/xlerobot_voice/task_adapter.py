"""Map validated language intent to the stable task action contract."""

from xlerobot_interfaces.action import ExecuteTask
from xlerobot_voice.intent import IntentResult


def execute_task_goal(intent: IntentResult, *, dry_run: bool):
    """Create only task-level fields; no backend or controller details leak in."""
    goal = ExecuteTask.Goal()
    goal.object_id = intent.object_id
    goal.source_place = intent.source_place
    goal.recipient_id = intent.recipient_id
    goal.dry_run = bool(dry_run)
    return goal
