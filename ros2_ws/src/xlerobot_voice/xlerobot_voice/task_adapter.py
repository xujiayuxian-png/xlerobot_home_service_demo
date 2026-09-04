"""Map validated language intent to the stable task action contract."""

from xlerobot_interfaces.action import ExecuteTask
from xlerobot_voice.intent import IntentResult


def execute_task_goal(intent: IntentResult, *, dry_run: bool, grasp_backend: str):
    """Create task-level fields using the operator-configured grasp route."""
    if grasp_backend not in {'act', 'centroid', 'gpd'}:
        raise ValueError('grasp_backend must be one of: act, centroid, gpd')
    goal = ExecuteTask.Goal()
    goal.object_id = intent.object_id
    goal.source_place = intent.source_place
    goal.recipient_id = intent.recipient_id
    goal.grasp_backend = grasp_backend
    goal.dry_run = bool(dry_run)
    return goal
