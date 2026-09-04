import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
import pytest
from rclpy.action import CancelResponse, GoalResponse

from xlerobot_commissioning import collection_node
from xlerobot_commissioning.collection_node import (
    CollectionCanceled,
    CollectionNode,
    LEADER_JOINTS,
    TARGET_JOINTS,
)
from xlerobot_interfaces.action import (
    CollectEpisode, DetectObject, PrepareGrasp, RecordEpisode,
)
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_interfaces.srv import FinalizeEpisode, MarkEpisodeEvent


class CompletedFuture:
    def __init__(self, value):
        self.value = value

    @staticmethod
    def done():
        return True

    def result(self):
        return self.value


class PendingFuture:
    def __init__(self):
        self.value = None
        self.complete = False

    def done(self):
        return self.complete

    def result(self):
        return self.value

    def set_result(self, value):
        self.value = value
        self.complete = True


class RecorderHandle:
    accepted = True

    def __init__(self, events):
        self.events = events
        self.result_future = PendingFuture()

    def get_result_async(self):
        return self.result_future

    def finish(self):
        result = RecordEpisode.Result()
        result.error.code = CapabilityError.NONE
        result.error.message = 'episode finalized atomically'
        result.episode_uri = 'file:///datasets/raw/episode-001'
        self.result_future.set_result(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=result,
        ))

    def fail(self, message):
        result = RecordEpisode.Result()
        result.error.code = CapabilityError.BACKEND_FAILURE
        result.error.message = message
        self.result_future.set_result(SimpleNamespace(
            status=GoalStatus.STATUS_ABORTED,
            result=result,
        ))

    def cancel_goal_async(self):
        self.events.append('recorder_cancel')
        result = RecordEpisode.Result()
        result.error.code = CapabilityError.CANCELED
        result.error.message = 'incomplete data retained'
        self.result_future.set_result(SimpleNamespace(
            status=GoalStatus.STATUS_CANCELED,
            result=result,
        ))
        return CompletedFuture(SimpleNamespace())


class RecorderClient:
    def __init__(self, handle, ready=True):
        self.handle = handle
        self.ready = ready

    def wait_for_server(self, timeout_sec):
        return self.ready and timeout_sec > 0.0

    def send_goal_async(self, _goal, feedback_callback):
        self.goal = _goal
        self.feedback_callback = feedback_callback
        self.handle.events.append('recorder_admitted')
        feedback = RecordEpisode.Feedback()
        feedback.state.phase = 'READY'
        feedback_callback(SimpleNamespace(feedback=feedback))
        return CompletedFuture(self.handle)

    def emit_first_sample(self):
        self.handle.events.append('recorder_first_sample')
        feedback = RecordEpisode.Feedback()
        feedback.state.phase = 'RECORDING'
        feedback.frame_count = 1
        self.feedback_callback(SimpleNamespace(feedback=feedback))


class ReadyAction:
    @staticmethod
    def wait_for_server(timeout_sec):
        return timeout_sec > 0.0


class ReadyService:
    @staticmethod
    def wait_for_service(timeout_sec):
        return timeout_sec > 0.0


class CollectionHandle:
    def __init__(self, node, events, ending):
        self.node = node
        self.events = events
        self.ending = ending
        self.is_cancel_requested = False
        self.request = CollectEpisode.Goal()
        self.request.template_id = 'manual'
        self.request.dataset_id = 'dataset-001'
        self.request.episode_id = 'episode-001'
        self.request.collection_profile_id = 'two_wheel_pick'
        self.request.language_instruction = 'pick the object'
        self.request.max_duration_s = 30.0

    def publish_feedback(self, feedback):
        self.events.append(feedback.state.phase)
        if (
            feedback.state.phase == 'RECORDER_ADMISSION'
            and self.ending == 'abort_admission'
        ):
            assert (
                CollectionNode.cancel(self.node, self)
                == CancelResponse.ACCEPT
            )
            self.is_cancel_requested = True
            return
        if feedback.state.phase != 'RECORDING':
            return
        if self.ending == 'finish':
            response = CollectionNode.request_finalize(
                self.node,
                FinalizeEpisode.Request(
                    dataset_id=self.request.dataset_id,
                    episode_id=self.request.episode_id,
                ),
                FinalizeEpisode.Response(),
            )
            self.events.append(f'finish_response:{response.error.code}')
        elif self.ending == 'abort':
            assert (
                CollectionNode.cancel(self.node, self)
                == CancelResponse.ACCEPT
            )
            self.is_cancel_requested = True

    def succeed(self):
        self.events.append('action_succeeded')

    def canceled(self):
        self.events.append('action_canceled')

    def abort(self):
        self.events.append('action_aborted')


def coordinator(events):
    node = object.__new__(CollectionNode)
    node._session_lock = threading.Lock()
    node._goal_active = True
    node._cleanup_blocked = False
    node._active_dataset_id = ''
    node._active_episode_id = ''
    node._phase = ''
    node._finish_requested = threading.Event()
    node._joint_state_timeout_s = 0.5
    node._control_enable_lease_s = 1.0
    node._control_heartbeat_period_s = 0.2
    node._recorder_first_sample_timeout_s = 2.0
    node._state_lock = threading.Lock()
    node._states = {
        'leader': {
            name: float(index) / 10.0
            for index, name in enumerate(LEADER_JOINTS)
        },
        'follower': {
            name: float(index) / 10.0
            for index, name in enumerate(TARGET_JOINTS)
        },
    }
    node._state_received_at = {
        'leader': time.monotonic(),
        'follower': time.monotonic(),
    }
    node._leader_frame_count = 10
    recorder_handle = RecorderHandle(events)
    node._recorder_handle = recorder_handle
    node.recorder = RecorderClient(recorder_handle)
    node.align = ReadyAction()
    node.teleop = ReadyService()
    node.torque = ReadyService()
    node.finalize = ReadyService()
    node.episode_event = ReadyService()
    node._set_torque = lambda enabled: events.append(
        f'torque_{"on" if enabled else "off"}'
    )
    node._set_teleop = lambda enabled: events.append(
        f'teleop_{"on" if enabled else "off"}'
    )
    node._align_leader = lambda _positions, _handle: events.append(
        'leader_aligned'
    )

    def service(client, _request, timeout=5.0):
        assert timeout > 0.0
        if client is node.episode_event:
            assert _request.dataset_id == 'dataset-001'
            response = MarkEpisodeEvent.Response()
            response.error.code = CapabilityError.NONE
            if _request.event == 'teleop_enabled':
                events.append('recorder_start')
                node.recorder.emit_first_sample()
            else:
                assert _request.event == 'teleop_disabled'
                events.append('recorder_stop')
            return response
        assert client is node.finalize
        assert _request.dataset_id == 'dataset-001'
        events.append('recorder_finalize')
        recorder_handle.finish()
        response = FinalizeEpisode.Response()
        response.error.code = CapabilityError.NONE
        return response

    node._service = service
    return node


def test_manual_end_disables_teleop_then_finalizes_and_succeeds(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.NONE
    assert result.episode_uri.endswith('/episode-001')
    assert events.index('recorder_admitted') < events.index('torque_on')
    assert events.index('recorder_start') < events.index('teleop_on')
    assert events.index('recorder_first_sample') < events.index('teleop_on')
    assert events.index('teleop_off') < events.index('recorder_finalize')
    assert events.index('teleop_off') < events.index('recorder_stop')
    assert events.index('recorder_stop') < events.index('recorder_finalize')
    assert events.index('recorder_finalize') < events.index('action_succeeded')
    assert 'recorder_cancel' not in events
    assert 'finish_response:0' in events
    assert node.recorder.goal.max_duration.sec == 73
    assert node.recorder.goal.max_duration.nanosec == 0


def test_stop_marker_failure_cancels_incomplete_before_finalize(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    service = node._service

    def fail_stop_marker(client, request, timeout=5.0):
        if (
            client is node.episode_event
            and request.event == 'teleop_disabled'
        ):
            events.append('recorder_stop_failed')
            response = MarkEpisodeEvent.Response()
            response.error.code = CapabilityError.BACKEND_FAILURE
            response.error.message = 'stop marker persistence failed'
            return response
        return service(client, request, timeout)

    node._service = fail_stop_marker
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert 'stop marker persistence failed' in result.error.message
    assert events.index('teleop_off') < events.index('recorder_stop_failed')
    assert events.index('recorder_stop_failed') < events.index(
        'recorder_cancel'
    )
    assert 'recorder_finalize' not in events


def test_live_goal_rejects_identifier_or_profile_before_admission():
    node = object.__new__(CollectionNode)
    node._session_lock = threading.Lock()
    node._goal_active = False
    node._cleanup_blocked = False
    goal = CollectEpisode.Goal()
    goal.template_id = 'manual'
    goal.dataset_id = 'bad dataset'
    goal.episode_id = 'episode-001'
    goal.collection_profile_id = 'two_wheel_pick'
    goal.language_instruction = 'pick the object'
    goal.max_duration_s = 30.0

    assert CollectionNode.goal(node, goal) == GoalResponse.REJECT
    assert node._goal_active is False

    goal.dataset_id = 'dataset-001'
    goal.collection_profile_id = 'unknown'
    assert CollectionNode.goal(node, goal) == GoalResponse.REJECT
    assert node._goal_active is False

    goal.collection_profile_id = 'two_wheel_pick'
    goal.max_duration_s = 120.01
    assert CollectionNode.goal(node, goal) == GoalResponse.REJECT
    assert node._goal_active is False


def test_unknown_recorder_goal_response_blocks_collection():
    node = object.__new__(CollectionNode)
    node._session_lock = threading.Lock()
    node._cleanup_blocked = False
    node.recorder = SimpleNamespace(
        send_goal_async=lambda *_args, **_kwargs: PendingFuture()
    )
    parent = SimpleNamespace(is_cancel_requested=False)

    with pytest.raises(RuntimeError, match='bounded fail-safe'):
        CollectionNode._start_recorder(
            node,
            RecordEpisode.Goal(),
            parent,
            None,
            response_timeout=0.0,
        )
    assert node._cleanup_blocked is True


def test_preflight_failure_happens_before_any_motion(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    node.recorder.ready = False

    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert 'RecordEpisode action is unavailable' in result.error.message
    assert 'torque_on' not in events
    assert 'leader_aligned' not in events
    assert 'teleop_on' not in events
    assert 'action_aborted' in events


def test_pick_recorder_admission_precedes_prepare_grasp_motion(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    node.detect = ReadyAction()
    node.prepare = ReadyAction()

    def action(client, *_args, **_kwargs):
        if client is node.detect:
            events.append('detect')
            detected = DetectObject.Result()
            detected.error.code = CapabilityError.NONE
            return detected
        assert client is node.prepare
        events.append('prepare_motion')
        prepared = PrepareGrasp.Result()
        prepared.error.code = CapabilityError.NONE
        prepared.start_context.joint_names = list(TARGET_JOINTS)
        prepared.start_context.positions = [0.1] * len(TARGET_JOINTS)
        return prepared

    node._action = action
    handle = CollectionHandle(node, events, ending='finish')
    handle.request.template_id = 'pick'
    handle.request.object_id = 'shuttlecock'

    result = CollectionNode.execute(node, handle)

    assert result.error.code == CapabilityError.NONE
    assert events.index('detect') < events.index('recorder_admitted')
    assert events.index('recorder_admitted') < events.index('prepare_motion')
    assert events.index('prepare_motion') < events.index('torque_on')


def test_manual_collection_rejects_stale_follower_before_motion(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    node._state_received_at['follower'] = time.monotonic() - 2.0

    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert 'fresh complete Follower joint state' in result.error.message
    assert 'torque_on' not in events
    assert 'leader_aligned' not in events
    assert 'action_aborted' in events


def test_abort_cancels_recorder_and_keeps_episode_incomplete(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='abort')
    )

    assert result.error.code == CapabilityError.CANCELED
    assert 'incomplete' in result.error.message
    assert events.index('teleop_off') < events.index('recorder_cancel')
    assert events.index('recorder_cancel') < events.index('action_canceled')
    assert 'recorder_finalize' not in events
    assert 'action_succeeded' not in events


def test_abort_with_unconfirmed_teleop_disable_aborts_and_blocks_reentry(
    monkeypatch,
):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)

    def set_teleop(enabled):
        events.append(f'teleop_{"on" if enabled else "off"}')
        if not enabled:
            raise RuntimeError('teleop disable transport failed')

    node._set_teleop = set_teleop
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='abort')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert 'ownership unconfirmed' in result.error.message
    assert 'teleop disable failed' in result.error.message
    assert 'action_aborted' in events
    assert 'action_canceled' not in events
    assert node._cleanup_blocked is True


def test_abort_while_recorder_is_admitting_never_enables_teleop(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='abort_admission')
    )

    assert result.error.code == CapabilityError.CANCELED
    assert 'teleop_on' not in events
    assert 'recorder_cancel' in events
    assert 'action_canceled' in events


def test_recorder_early_terminal_never_enables_teleop(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    send_goal = node.recorder.send_goal_async

    def fail_after_ready(goal, feedback_callback):
        future = send_goal(goal, feedback_callback)
        node._recorder_handle.fail('camera input became stale')
        return future

    node.recorder.send_goal_async = fail_after_ready
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert 'terminated early while admission' in result.error.message
    assert 'camera input became stale' in result.error.message
    assert 'torque_on' not in events
    assert 'leader_aligned' not in events
    assert 'teleop_on' not in events
    assert 'recorder_cancel' not in events
    assert 'action_aborted' in events


def test_child_action_parent_cancel_is_forwarded_and_drained():
    events = []

    class ParentHandle:
        def __init__(self):
            self.reads = 0

        @property
        def is_cancel_requested(self):
            self.reads += 1
            return self.reads >= 3

    class ChildHandle:
        accepted = True

        def __init__(self):
            self.result_future = PendingFuture()

        def get_result_async(self):
            return self.result_future

        def cancel_goal_async(self):
            events.append('child_cancel')
            self.result_future.set_result(SimpleNamespace(
                status=GoalStatus.STATUS_CANCELED,
                result=SimpleNamespace(),
            ))
            return CompletedFuture(SimpleNamespace())

    child_handle = ChildHandle()
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: timeout_sec > 0.0,
        send_goal_async=lambda _goal, feedback_callback=None: CompletedFuture(
            child_handle
        ),
    )
    node = object.__new__(CollectionNode)
    parent = ParentHandle()

    with pytest.raises(CollectionCanceled):
        CollectionNode._action(
            node,
            client,
            object(),
            parent,
            label='PrepareGrasp',
            timeout=60.0,
        )
    assert events == ['child_cancel']
    assert child_handle.result_future.done()


def test_child_action_does_not_start_after_parent_cancel():
    events = []
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: timeout_sec > 0.0,
        send_goal_async=lambda *_args, **_kwargs: events.append('child_sent'),
    )
    node = object.__new__(CollectionNode)
    parent = SimpleNamespace(is_cancel_requested=True)

    with pytest.raises(CollectionCanceled):
        CollectionNode._action(
            node,
            client,
            object(),
            parent,
            label='DetectObject',
            timeout=30.0,
        )
    assert events == []


def test_child_action_timeout_cancels_and_drains():
    events = []

    class ChildHandle:
        accepted = True

        def __init__(self):
            self.result_future = PendingFuture()

        def get_result_async(self):
            return self.result_future

        def cancel_goal_async(self):
            events.append('child_cancel')
            self.result_future.set_result(SimpleNamespace(
                status=GoalStatus.STATUS_CANCELED,
                result=SimpleNamespace(),
            ))
            return CompletedFuture(SimpleNamespace())

    child_handle = ChildHandle()
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: timeout_sec > 0.0,
        send_goal_async=lambda _goal, feedback_callback=None: CompletedFuture(
            child_handle
        ),
    )
    node = object.__new__(CollectionNode)
    parent = SimpleNamespace(is_cancel_requested=False)

    with pytest.raises(TimeoutError, match='timed out and was canceled'):
        CollectionNode._action(
            node,
            client,
            object(),
            parent,
            label='Leader alignment',
            timeout=0.0,
        )
    assert events == ['child_cancel']
    assert child_handle.result_future.done()


def test_child_action_unconfirmed_drain_blocks_collection(monkeypatch):
    child_handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: PendingFuture(),
    )
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: timeout_sec > 0.0,
        send_goal_async=lambda _goal, feedback_callback=None: CompletedFuture(
            child_handle
        ),
    )
    node = object.__new__(CollectionNode)
    node._session_lock = threading.Lock()
    node._cleanup_blocked = False
    parent = SimpleNamespace(is_cancel_requested=False)
    monkeypatch.setattr(
        node,
        '_drain_child_cancel',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError('child never reached terminal state')
        ),
    )

    with pytest.raises(RuntimeError, match='never reached terminal'):
        CollectionNode._action(
            node,
            client,
            object(),
            parent,
            label='PrepareGrasp',
            timeout=0.0,
        )
    assert node._cleanup_blocked is True


def test_lost_torque_enable_response_still_attempts_torque_off(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)

    def set_torque(enabled):
        events.append(f'torque_{"on" if enabled else "off"}')
        if enabled:
            raise RuntimeError('torque enable response lost')

    node._set_torque = set_torque
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert events.index('torque_on') < events.index('torque_off')
    assert 'action_aborted' in events


def test_lost_teleop_enable_response_still_disables_and_cancels_recorder(
    monkeypatch,
):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)

    def set_teleop(enabled):
        events.append(f'teleop_{"on" if enabled else "off"}')
        if enabled:
            raise RuntimeError('teleop enable response lost')

    node._set_teleop = set_teleop
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert events.index('teleop_on') < events.index('teleop_off')
    assert events.index('teleop_off') < events.index('recorder_cancel')
    assert 'action_aborted' in events


def test_lost_finalize_response_uses_successful_recorder_terminal(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    service = node._service

    def lose_finalize_response(client, request, timeout=5.0):
        if client is node.finalize:
            events.append('recorder_finalize')
            node._recorder_handle.finish()
            raise RuntimeError('finalize response lost')
        return service(client, request, timeout)

    node._service = lose_finalize_response
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.NONE
    assert result.episode_uri.endswith('/episode-001')
    assert 'recorder_cancel' not in events
    assert 'action_succeeded' in events


def test_finalize_request_is_fail_closed_and_repeat_is_idempotent():
    node = object.__new__(CollectionNode)
    node._session_lock = threading.Lock()
    node._goal_active = True
    node._active_dataset_id = 'dataset-001'
    node._active_episode_id = 'episode-001'
    node._phase = 'COUNTDOWN'
    node._finish_requested = threading.Event()

    wrong = CollectionNode.request_finalize(
        node,
        FinalizeEpisode.Request(
            dataset_id='dataset-001', episode_id='episode-002'
        ),
        FinalizeEpisode.Response(),
    )
    assert wrong.error.code == CapabilityError.NOT_FOUND

    late_other_dataset = CollectionNode.request_finalize(
        node,
        FinalizeEpisode.Request(
            dataset_id='dataset-old', episode_id='episode-001'
        ),
        FinalizeEpisode.Response(),
    )
    assert late_other_dataset.error.code == CapabilityError.NOT_FOUND
    assert not node._finish_requested.is_set()

    early = CollectionNode.request_finalize(
        node,
        FinalizeEpisode.Request(
            dataset_id='dataset-001', episode_id='episode-001'
        ),
        FinalizeEpisode.Response(),
    )
    assert early.error.code == CapabilityError.INVALID_GOAL
    assert not node._finish_requested.is_set()

    node._phase = 'RECORDING'
    accepted = CollectionNode.request_finalize(
        node,
        FinalizeEpisode.Request(
            dataset_id='dataset-001', episode_id='episode-001'
        ),
        FinalizeEpisode.Response(),
    )
    repeated = CollectionNode.request_finalize(
        node,
        FinalizeEpisode.Request(
            dataset_id='dataset-001', episode_id='episode-001'
        ),
        FinalizeEpisode.Response(),
    )
    assert accepted.error.code == CapabilityError.NONE
    assert repeated.error.code == CapabilityError.NONE
    assert node._finish_requested.is_set()

    node._finish_requested.clear()
    node._phase = 'FINALIZING'
    in_progress = CollectionNode.request_finalize(
        node,
        FinalizeEpisode.Request(
            dataset_id='dataset-001', episode_id='episode-001'
        ),
        FinalizeEpisode.Response(),
    )
    assert in_progress.error.code == CapabilityError.NONE
    assert CollectionNode.cancel(node, object()) == CancelResponse.REJECT


def test_recorder_cancel_drains_terminal_result_when_ack_is_lost():
    result = RecordEpisode.Result()
    result.error.code = CapabilityError.CANCELED
    terminal = CompletedFuture(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED,
        result=result,
    ))

    class LostAcknowledgment:
        @staticmethod
        def done():
            return True

        @staticmethod
        def result():
            raise RuntimeError('cancel response lost')

    handle = SimpleNamespace(
        cancel_goal_async=lambda: LostAcknowledgment()
    )
    CollectionNode._cancel_recorder(handle, terminal)


def test_alignment_heartbeat_failure_cancels_and_drains_child(monkeypatch):
    events = []

    class DelayedGoalFuture:
        def __init__(self, value):
            self.value = value
            self.polls = 0

        def done(self):
            self.polls += 1
            return self.polls >= 3

        def result(self):
            return self.value

    class ChildHandle:
        accepted = True

        def __init__(self):
            self.result_future = PendingFuture()

        def get_result_async(self):
            return self.result_future

        def cancel_goal_async(self):
            events.append('child_cancel')
            self.result_future.set_result(SimpleNamespace(
                status=GoalStatus.STATUS_CANCELED,
                result=SimpleNamespace(),
            ))
            return CompletedFuture(SimpleNamespace())

    class Clock:
        now = 0.0

        def monotonic(self):
            self.now += 0.1
            return self.now

    child = ChildHandle()
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: timeout_sec > 0.0,
        send_goal_async=lambda *_args, **_kwargs: DelayedGoalFuture(child),
    )
    node = object.__new__(CollectionNode)
    node._control_heartbeat_period_s = 0.1
    node._session_lock = threading.Lock()
    node._cleanup_blocked = False
    parent = SimpleNamespace(is_cancel_requested=False)
    monkeypatch.setattr(collection_node.time, 'monotonic', Clock().monotonic)
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)

    def fail_refresh():
        events.append('torque_refresh')
        raise RuntimeError('refresh response lost')

    with pytest.raises(RuntimeError, match='control lease refresh failed'):
        CollectionNode._action(
            node,
            client,
            object(),
            parent,
            label='Leader alignment',
            heartbeat_callback=fail_refresh,
        )

    assert events == ['torque_refresh', 'child_cancel']
    assert child.result_future.done()
    assert node._cleanup_blocked is False


def test_countdown_torque_refresh_failure_releases_and_cancels_recorder(
    monkeypatch,
):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    enable_count = 0

    def set_torque(enabled):
        nonlocal enable_count
        events.append(f'torque_{"on" if enabled else "off"}')
        if enabled:
            enable_count += 1
            if enable_count == 3:
                raise RuntimeError('torque heartbeat failed')

    node._set_torque = set_torque
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert 'torque heartbeat failed' in result.error.message
    assert events.count('torque_on') == 3
    assert events.index('torque_off') < events.index('recorder_cancel')
    assert 'teleop_on' not in events


def test_recording_teleop_refresh_failure_stops_and_cancels_recorder(
    monkeypatch,
):
    class Clock:
        now = 1000.0

        def monotonic(self):
            self.now += 0.05
            return self.now

    clock = Clock()
    monkeypatch.setattr(collection_node.time, 'monotonic', clock.monotonic)
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    node._joint_state_timeout_s = 100.0
    enable_count = 0

    def set_teleop(enabled):
        nonlocal enable_count
        events.append(f'teleop_{"on" if enabled else "off"}')
        if enabled:
            enable_count += 1
            if enable_count == 2:
                raise RuntimeError('teleop heartbeat failed')

    node._set_teleop = set_teleop
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='continue')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert 'teleop heartbeat failed' in result.error.message
    assert events.count('teleop_on') == 2
    assert events.index('teleop_off') < events.index('recorder_cancel')


def test_cancel_after_torque_off_prevents_recorder_mark(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    handle = CollectionHandle(node, events, ending='finish')

    def set_torque(enabled):
        events.append(f'torque_{"on" if enabled else "off"}')
        if not enabled:
            handle.is_cancel_requested = True

    node._set_torque = set_torque
    result = CollectionNode.execute(node, handle)

    assert result.error.code == CapabilityError.CANCELED
    assert 'recorder_start' not in events
    assert 'teleop_on' not in events
    assert 'recorder_cancel' in events


def test_cancel_returning_from_mark_prevents_teleop(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    handle = CollectionHandle(node, events, ending='finish')
    service = node._service

    def cancel_after_mark(client, request, timeout=5.0):
        response = service(client, request, timeout)
        if client is node.episode_event:
            handle.is_cancel_requested = True
        return response

    node._service = cancel_after_mark
    result = CollectionNode.execute(node, handle)

    assert result.error.code == CapabilityError.CANCELED
    assert 'recorder_start' in events
    assert 'teleop_on' not in events
    assert 'recorder_cancel' in events


def test_cancel_with_first_sample_prevents_teleop(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    handle = CollectionHandle(node, events, ending='finish')
    service = node._service

    def mark_then_defer_first_sample(client, request, timeout=5.0):
        if client is not node.episode_event:
            return service(client, request, timeout)
        events.append('recorder_start')
        response = MarkEpisodeEvent.Response()
        response.error.code = CapabilityError.NONE

        def cancel_and_sample():
            handle.is_cancel_requested = True
            node.recorder.emit_first_sample()

        threading.Timer(0.01, cancel_and_sample).start()
        return response

    node._service = mark_then_defer_first_sample
    result = CollectionNode.execute(node, handle)

    assert result.error.code == CapabilityError.CANCELED
    assert events.index('recorder_start') < events.index(
        'recorder_first_sample'
    )
    assert 'teleop_on' not in events
    assert 'recorder_cancel' in events


def test_missing_first_sample_never_enables_teleop(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    node._recorder_first_sample_timeout_s = 0.03
    node.recorder.emit_first_sample = lambda: None

    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert 'first baseline sample' in result.error.message
    assert 'teleop_on' not in events
    assert 'recorder_cancel' in events


def test_stale_leader_after_alignment_never_enables_teleop(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    emit_first_sample = node.recorder.emit_first_sample

    def stale_then_sample():
        node._state_received_at['leader'] = time.monotonic() - 2.0
        emit_first_sample()

    node.recorder.emit_first_sample = stale_then_sample
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert 'fresh complete Leader and Follower' in result.error.message
    assert 'teleop_on' not in events
    assert 'recorder_cancel' in events


def test_leader_deviation_after_alignment_never_enables_teleop(monkeypatch):
    monkeypatch.setattr(collection_node.time, 'sleep', lambda _seconds: None)
    events = []
    node = coordinator(events)
    emit_first_sample = node.recorder.emit_first_sample

    def deviate_then_sample():
        node._states['leader'][LEADER_JOINTS[0]] += 0.2
        emit_first_sample()

    node.recorder.emit_first_sample = deviate_then_sample
    result = CollectionNode.execute(
        node, CollectionHandle(node, events, ending='finish')
    )

    assert result.error.code == CapabilityError.BACKEND_FAILURE
    assert 'live alignment exceeds 0.15 rad' in result.error.message
    assert 'teleop_on' not in events
    assert 'recorder_cancel' in events


def test_alignment_result_rejects_stale_leader_state():
    node = object.__new__(CollectionNode)
    node.align = object()
    node._joint_state_timeout_s = 0.5
    node._state_lock = threading.Lock()
    node._states = {
        'leader': {
            name: float(index) / 10.0
            for index, name in enumerate(LEADER_JOINTS)
        },
    }
    node._state_received_at = {
        'leader': time.monotonic() - 2.0,
    }
    node._action = lambda *_args, **_kwargs: SimpleNamespace(
        error_code=0,
        error_string='',
    )

    with pytest.raises(RuntimeError, match='alignment state is stale'):
        CollectionNode._align_leader(
            node,
            [float(index) / 10.0 for index in range(len(LEADER_JOINTS))],
            SimpleNamespace(is_cancel_requested=False),
        )
