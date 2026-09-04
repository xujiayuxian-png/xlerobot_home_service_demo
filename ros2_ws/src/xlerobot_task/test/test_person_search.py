from concurrent.futures import Future
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
import pytest
import rclpy

from xlerobot_interfaces.msg import CapabilityError
from xlerobot_task.person_search_node import (
    PersonSearchNode,
    SearchFailure,
)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def search_fixture():
    search = object.__new__(PersonSearchNode)
    search.retreat_distance_m = 0.4
    search.retreat_speed_mps = 0.08
    search.manual_retreat_extra_timeout_s = 2.0
    search.child_cancel_timeout_s = 0.25
    search._lock = threading.Lock()
    search._odom_lock = threading.Lock()
    search._base_xy = None
    search._goal_active = True
    return search


class Parent:
    is_cancel_requested = False


class FakeClient:
    def __init__(self, send_future):
        self.send_future = send_future

    @staticmethod
    def server_is_ready():
        return True

    def send_goal_async(self, _goal):
        return self.send_future


class FakeHandle:
    accepted = True

    def __init__(self):
        self.cancel_count = 0
        self.cancel_future = Future()
        self.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
        self.result_future = Future()

    def cancel_goal_async(self):
        self.cancel_count += 1
        return self.cancel_future

    def get_result_async(self):
        return self.result_future


def successful_result():
    result = SimpleNamespace(error_code=0)
    return SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=result)


class RecordingPublisher:
    def __init__(self, on_publish=None):
        self.commands = []
        self.on_publish = on_publish

    def publish(self, command):
        self.commands.append(command)
        if self.on_publish is not None:
            self.on_publish(command)


def test_remaining_retreat_uses_measured_odom_distance():
    search = search_fixture()
    search._base_xy = (0.3, 0.4)

    assert search._remaining_retreat((0.0, 0.0)) == pytest.approx(0.0)


def test_remaining_retreat_falls_back_to_full_distance_without_odom():
    search = search_fixture()

    assert search._remaining_retreat((0.0, 0.0)) == pytest.approx(0.4)
    assert search._remaining_retreat(None) == pytest.approx(0.4)


def test_manual_retreat_publishes_direct_reverse_and_always_stops():
    search = search_fixture()

    def update_odom(command):
        if command.linear.x < 0.0:
            with search._odom_lock:
                search._base_xy = (0.4, 0.0)

    publisher = RecordingPublisher(update_odom)
    search.cmd_vel_pub = publisher
    search._base_xy = (0.0, 0.0)

    search._manual_retreat(Parent(), (0.0, 0.0), 0.4)

    assert publisher.commands[0].linear.x == pytest.approx(-0.08)
    assert publisher.commands[-1].linear.x == pytest.approx(0.0)
    assert len(publisher.commands) == 2


def test_manual_retreat_cancellation_publishes_stop():
    search = search_fixture()
    publisher = RecordingPublisher()
    search.cmd_vel_pub = publisher
    parent = Parent()
    parent.is_cancel_requested = True

    with pytest.raises(SearchFailure) as caught:
        search._manual_retreat(parent, None, 0.4)

    assert caught.value.code == CapabilityError.CANCELED
    assert len(publisher.commands) == 1
    assert publisher.commands[0].linear.x == pytest.approx(0.0)


def test_pending_goal_does_not_finish_parent_before_late_child_is_terminal():
    search = search_fixture()
    send_future = Future()
    client = FakeClient(send_future)
    parent = Parent()
    outcome = {}

    def run_call():
        try:
            search._call(client, object(), 1.0, parent, 'pending child')
        except SearchFailure as exc:
            outcome['failure'] = exc

    thread = threading.Thread(target=run_call)
    thread.start()
    time.sleep(0.03)
    parent.is_cancel_requested = True
    time.sleep(0.03)
    assert thread.is_alive()

    handle = FakeHandle()
    send_future.set_result(handle)
    deadline = time.monotonic() + 1.0
    while handle.cancel_count == 0 and time.monotonic() < deadline:
        time.sleep(0.005)
    assert handle.cancel_count == 1
    assert thread.is_alive()

    handle.result_future.set_result(successful_result())
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert outcome['failure'].code == CapabilityError.CANCELED


def test_goal_response_timeout_holds_parent_until_late_terminal_result():
    search = search_fixture()
    search.child_cancel_timeout_s = 0.05
    send_future = Future()
    client = FakeClient(send_future)
    outcome = {}
    errors = []
    search.get_logger = lambda: SimpleNamespace(error=errors.append)

    def run_call():
        try:
            search._call(client, object(), 0.03, Parent(), 'pending child')
        except SearchFailure as exc:
            outcome['failure'] = exc

    thread = threading.Thread(target=run_call)
    thread.start()
    time.sleep(0.12)
    assert thread.is_alive()
    assert any('holding the parent action' in message for message in errors)

    handle = FakeHandle()
    send_future.set_result(handle)
    deadline = time.monotonic() + 1.0
    while handle.cancel_count == 0 and time.monotonic() < deadline:
        time.sleep(0.005)
    assert handle.cancel_count == 1
    assert thread.is_alive()
    handle.result_future.set_result(successful_result())
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert outcome['failure'].code == CapabilityError.TIMEOUT


def test_active_cancel_waits_for_child_terminal_result():
    search = search_fixture()
    handle = FakeHandle()
    send_future = Future()
    send_future.set_result(handle)
    client = FakeClient(send_future)
    parent = Parent()

    def complete_after_cancel():
        time.sleep(0.03)
        parent.is_cancel_requested = True
        while handle.cancel_count == 0:
            time.sleep(0.005)
        time.sleep(0.03)
        handle.result_future.set_result(
            SimpleNamespace(
                status=GoalStatus.STATUS_CANCELED,
                result=SimpleNamespace(error_code=0),
            )
        )

    thread = threading.Thread(target=complete_after_cancel)
    thread.start()
    with pytest.raises(SearchFailure) as caught:
        search._call(client, object(), 1.0, parent, 'active child')
    thread.join(timeout=1.0)

    assert caught.value.code == CapabilityError.CANCELED
    assert handle.cancel_count == 1
    assert handle.result_future.done()


def test_child_timeout_is_one_end_to_end_deadline():
    search = search_fixture()
    send_future = Future()
    handle = FakeHandle()
    client = FakeClient(send_future)
    goal_response = threading.Timer(0.03, lambda: send_future.set_result(handle))
    terminal = threading.Timer(
        0.08, lambda: handle.result_future.set_result(successful_result())
    )
    goal_response.start()
    terminal.start()
    started = time.monotonic()
    try:
        with pytest.raises(SearchFailure) as caught:
            search._call(client, object(), 0.05, Parent(), 'bounded child')
    finally:
        goal_response.cancel()
        terminal.cancel()

    assert caught.value.code == CapabilityError.TIMEOUT
    assert handle.cancel_count == 1
    assert time.monotonic() - started >= 0.07


def test_dry_run_rechecks_cancel_before_reporting_success():
    search = search_fixture()

    class DryRunGoal:
        def __init__(self):
            self.cancel_checks = 0
            self.request = SimpleNamespace(dry_run=True)
            self.succeeded = False
            self.was_canceled = False

        @property
        def is_cancel_requested(self):
            self.cancel_checks += 1
            return self.cancel_checks >= 2

        def succeed(self):
            self.succeeded = True

        def canceled(self):
            self.was_canceled = True

        @staticmethod
        def abort():
            pytest.fail('dry-run cancel must not abort')

    goal = DryRunGoal()
    result = search.execute(goal)

    assert result.error.code == CapabilityError.CANCELED
    assert goal.was_canceled
    assert not goal.succeeded


@pytest.mark.parametrize(
    ('status', 'payload_code', 'expected_code'),
    [
        (
            GoalStatus.STATUS_SUCCEEDED,
            CapabilityError.BACKEND_FAILURE,
            CapabilityError.BACKEND_FAILURE,
        ),
        (
            GoalStatus.STATUS_ABORTED,
            CapabilityError.NONE,
            CapabilityError.BACKEND_FAILURE,
        ),
        (
            GoalStatus.STATUS_ABORTED,
            CapabilityError.CANCELED,
            CapabilityError.BACKEND_FAILURE,
        ),
        (
            GoalStatus.STATUS_SUCCEEDED,
            CapabilityError.CANCELED,
            CapabilityError.BACKEND_FAILURE,
        ),
    ],
)
def test_view_status_and_payload_must_agree(
    status, payload_code, expected_code
):
    search = search_fixture()
    search.retreat_distance_m = 0.0
    search.body_turns_rad = [0.0]
    search.scan_pan_rad = [0.0]
    search.spin_client = object()
    search.view_client = object()
    search._require_success = lambda *_args: successful_result()
    search._move_head = lambda *_args: None
    search._wait_settle = lambda *_args: None
    search._feedback = lambda *_args: None
    search.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    view_result = SimpleNamespace(
        error=SimpleNamespace(code=payload_code, message='view mismatch')
    )
    search._call = lambda *_args: SimpleNamespace(
        status=status,
        result=view_result,
    )

    class Goal:
        is_cancel_requested = False
        request = SimpleNamespace(
            dry_run=False,
            recipient_id='nearest_person',
        )
        aborted = False

        @staticmethod
        def publish_feedback(_feedback):
            pass

        @staticmethod
        def succeed():
            pytest.fail('inconsistent child result must not succeed')

        @staticmethod
        def canceled():
            pytest.fail('inconsistent child result must not cancel the parent')

        def abort(self):
            self.aborted = True

    goal = Goal()
    result = search.execute(goal)

    assert goal.aborted
    assert result.error.code == expected_code


def test_unexpected_search_exception_returns_structured_error():
    search = search_fixture()
    search.retreat_distance_m = 0.0
    search.body_turns_rad = [0.0]
    search.scan_pan_rad = [0.0]
    search.spin_client = object()
    search.view_client = object()
    search._require_success = lambda *_args: successful_result()
    search._move_head = lambda *_args: None
    search._wait_settle = lambda *_args: None
    search._feedback = lambda *_args: None

    def fail_transport(*_args):
        raise RuntimeError('transport lost')

    search._call = fail_transport

    class Goal:
        is_cancel_requested = False
        request = SimpleNamespace(
            dry_run=False,
            recipient_id='nearest_person',
        )
        aborted = False

        @staticmethod
        def publish_feedback(_feedback):
            pass

        @staticmethod
        def succeed():
            pytest.fail('unexpected exception must not succeed')

        @staticmethod
        def canceled():
            pytest.fail('unexpected exception must not cancel the parent')

        def abort(self):
            self.aborted = True

    goal = Goal()
    result = search.execute(goal)

    assert goal.aborted
    assert result.error.code == CapabilityError.INTERNAL_ERROR
    assert 'transport lost' in result.error.message
