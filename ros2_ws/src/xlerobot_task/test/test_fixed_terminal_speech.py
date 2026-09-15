from types import SimpleNamespace
import threading

import pytest
from geometry_msgs.msg import PointStamped
from xlerobot_interfaces.action import ExecuteTask
from xlerobot_interfaces.msg import CapabilityError, TopGraspPlan, PerceptionObservation
from xlerobot_task.fetch_deliver_task_node import FetchDeliverTaskNode, TaskFailure


@pytest.mark.parametrize('outcome,text', [
    ('success', '任务完成。'), ('failed', '任务失败，请查看页面。'), ('cancel', '任务已取消。'),
])
def test_terminal_prompt_follows_handover_or_confirmed_child_stop(outcome, text):
    node = object.__new__(FetchDeliverTaskNode)
    node.x1_low_load = True
    node.speech_enabled = True
    node.task_timeout_s = 10
    node.default_standoff_m = 0.5
    node._lock = threading.Lock()
    node._task_event_lock = threading.Lock()
    node._blocked_reason = ''
    node._goal_active = True
    events = []
    request = ExecuteTask.Goal(object_id='杯子', source_place='table',
                               recipient_id='nearest_person', grasp_backend='act', dry_run=True)
    goal = SimpleNamespace(request=request, is_cancel_requested=False,
                           succeed=lambda: events.append('success'),
                           abort=lambda: events.append('failed'),
                           canceled=lambda: events.append('cancel'))
    node._localization_ready = lambda: True
    node._feedback = lambda *args: None
    node._publish_task_event = lambda *args: None
    node._set_task_observation = lambda *args: None
    node._clear_task_observation = lambda: None
    node._check_parent = lambda *args: None
    node._speak_terminal = lambda phrase, dry: events.append(phrase)
    capabilities = ['auto_localize', 'navigate_to_named_place', 'detect_object', 'grasp_object',
                    'scan_for_person', 'approach_target', 'handover_object']
    node.stage_timeouts = {name: 1.0 for name in capabilities}
    for name in ['localize', 'navigate', 'detect', 'grasp', 'scan', 'approach', 'handover']:
        setattr(node, name + '_client', object())

    def call(client, request, label, *args):
        events.append(label)
        if label == 'handover_object' and outcome != 'success':
            if outcome == 'cancel':
                goal.is_cancel_requested = True
                raise TaskFailure(CapabilityError.CANCELED, 'child stopped')
            raise TaskFailure(CapabilityError.BACKEND_FAILURE, 'handover failed')
        return SimpleNamespace(error=CapabilityError(code=CapabilityError.NONE), backend_used='act',
                               target=PointStamped(), grasp_plan=TopGraspPlan(),
                               observation=PerceptionObservation(), person=PointStamped())

    node._call = call
    result = node.execute(goal)
    assert events[-2:] == [outcome, text]
    assert events.count(text) == 1
    assert events.index('handover_object') < events.index(text)
    assert not node._goal_active
    assert result.error.code == {'success': CapabilityError.NONE,
                                 'failed': CapabilityError.BACKEND_FAILURE,
                                 'cancel': CapabilityError.CANCELED}[outcome]
