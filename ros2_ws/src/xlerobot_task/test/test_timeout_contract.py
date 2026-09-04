from concurrent.futures import Future
from pathlib import Path
import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
import pytest
import yaml

from xlerobot_task.fetch_deliver_task_node import (
    ChildOperation,
    FetchDeliverTaskNode,
    TaskFailure,
)
import xlerobot_task.fetch_deliver_task_node as task_node_module
from xlerobot_interfaces.msg import CapabilityError


def test_task_timeouts_cover_the_owned_navigation_budgets():
    task_config = Path(__file__).parents[1] / 'config' / 'fetch_deliver_task.yaml'
    navigation_root = Path(__file__).parents[2] / 'xlerobot_navigation' / 'config'
    task = yaml.safe_load(task_config.read_text(encoding='utf-8'))[
        'fetch_deliver_task'
    ]['ros__parameters']
    named = yaml.safe_load(
        (navigation_root / 'named_navigation.yaml').read_text(encoding='utf-8')
    )['named_navigation_server']['ros__parameters']
    approach = yaml.safe_load(
        (navigation_root / 'approach_target.yaml').read_text(encoding='utf-8')
    )['approach_target_server']['ros__parameters']

    named_budget = (
        named['nav_timeout_s'] + named['spin_timeout_s'] + named['dock_timeout_s']
    )
    assert task['navigation_timeout_s'] > named_budget
    assert task['approach_timeout_s'] > approach['nav_timeout_s']
    assert task['task_timeout_s'] >= task['navigation_timeout_s']
    assert task['standoff_m'] == 0.5
    assert task['detection_head_stable_duration_s'] >= 0.5
    assert task['detection_head_post_settle_s'] > 0.0
    candidate_count = 1 + len(approach['fallback_standoff_m'])
    assert (
        approach['nav_attempt_timeout_s'] * (candidate_count + 1)
        + approach['backup_timeout_s']
        <= approach['nav_timeout_s']
    )


def test_child_cancel_waits_for_action_server_terminal_result():
    request_count = 0
    terminal = Future()
    terminal.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))

    class GoalHandle:
        accepted = True

        @staticmethod
        def cancel_goal_async():
            nonlocal request_count
            request_count += 1
            future = Future()
            future.set_result(object())
            return future

        @staticmethod
        def get_result_async():
            return terminal

    handle = GoalHandle()
    operation = ChildOperation(
        'navigation',
        Future(),
        handle=handle,
        result_future=handle.get_result_async(),
    )
    node = object.__new__(FetchDeliverTaskNode)
    assert node._cancel_operation_and_wait(operation, 'navigation', 0.1)
    assert request_count == 1


def test_pending_goal_response_is_canceled_after_acceptance():
    send = Future()
    terminal = Future()
    cancel_count = 0

    class GoalHandle:
        accepted = True

        @staticmethod
        def cancel_goal_async():
            nonlocal cancel_count
            cancel_count += 1
            future = Future()
            future.set_result(object())
            terminal.set_result(
                SimpleNamespace(status=GoalStatus.STATUS_CANCELED)
            )
            return future

        @staticmethod
        def get_result_async():
            return terminal

    operation = ChildOperation('navigation', send)
    node = object.__new__(FetchDeliverTaskNode)
    node.child_cancel_timeout_s = 0.2
    timer = threading.Timer(0.01, lambda: send.set_result(GoalHandle()))
    timer.start()
    try:
        assert node._cancel_operation_and_wait(operation, 'navigation')
    finally:
        timer.cancel()
    assert cancel_count == 1
    assert operation.handle is not None
    assert operation.result_future is terminal


def test_uncertain_goal_response_failure_is_not_treated_as_terminal():
    send = Future()
    send.set_exception(RuntimeError('transport lost'))
    warnings = []

    node = object.__new__(FetchDeliverTaskNode)
    node.child_cancel_timeout_s = 0.1
    node.get_logger = lambda: SimpleNamespace(warning=warnings.append)

    assert not node._cancel_operation_and_wait(
        ChildOperation('navigation', send), 'navigation'
    )
    assert any('goal response failed' in message for message in warnings)


def test_exceptional_result_future_is_not_treated_as_terminal():
    terminal = Future()
    terminal.set_exception(RuntimeError('result transport lost'))

    class GoalHandle:
        accepted = True

        @staticmethod
        def cancel_goal_async():
            future = Future()
            future.set_result(object())
            return future

    warnings = []
    node = object.__new__(FetchDeliverTaskNode)
    node.child_cancel_timeout_s = 0.1
    node.get_logger = lambda: SimpleNamespace(warning=warnings.append)
    operation = ChildOperation(
        'navigation', Future(), handle=GoalHandle(), result_future=terminal
    )

    assert not node._cancel_operation_and_wait(operation, 'navigation')
    assert any('terminal result could not be observed' in item for item in warnings)


def test_unknown_action_status_is_not_treated_as_terminal():
    terminal = Future()
    terminal.set_result(SimpleNamespace(status=GoalStatus.STATUS_UNKNOWN))

    class GoalHandle:
        accepted = True

        @staticmethod
        def cancel_goal_async():
            future = Future()
            future.set_result(object())
            return future

    warnings = []
    node = object.__new__(FetchDeliverTaskNode)
    node.child_cancel_timeout_s = 0.1
    node.get_logger = lambda: SimpleNamespace(warning=warnings.append)
    operation = ChildOperation(
        'navigation', Future(), handle=GoalHandle(), result_future=terminal
    )

    assert not node._cancel_operation_and_wait(operation, 'navigation')
    assert any('non-terminal result status' in item for item in warnings)


def test_aborted_child_with_default_success_payload_cannot_advance_task():
    node = object.__new__(FetchDeliverTaskNode)
    node._check_parent = lambda _parent, _deadline: None
    node._feedback = lambda *_args: None
    node._wait_for_action_server = lambda *_args: True
    node._run_child_action = lambda *_args, **_kwargs: SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(
            error=SimpleNamespace(code=CapabilityError.NONE, message='')
        ),
    )
    parent = SimpleNamespace(is_cancel_requested=False)

    with pytest.raises(TaskFailure) as caught:
        node._call(
            object(), object(), 'detect_object', parent,
            10_000.0, 1, 1.0,
        )

    assert caught.value.code == CapabilityError.BACKEND_FAILURE


@pytest.mark.parametrize(
    'status',
    [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED],
)
def test_child_cancel_payload_requires_parent_cancel(status):
    node = object.__new__(FetchDeliverTaskNode)
    node._check_parent = lambda _parent, _deadline: None
    node._feedback = lambda *_args: None
    node._wait_for_action_server = lambda *_args: True
    node._run_child_action = lambda *_args, **_kwargs: SimpleNamespace(
        status=status,
        result=SimpleNamespace(
            error=SimpleNamespace(
                code=CapabilityError.CANCELED,
                message='child canceled itself',
            )
        ),
    )

    with pytest.raises(TaskFailure) as caught:
        node._call(
            object(),
            object(),
            'detect_object',
            SimpleNamespace(is_cancel_requested=False),
            10_000.0,
            1,
            1.0,
        )

    assert caught.value.code == CapabilityError.BACKEND_FAILURE


def test_future_completing_after_deadline_is_still_timed_out(monkeypatch):
    terminal = Future()
    terminal.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    cancel_count = 0

    class GoalHandle:
        accepted = True

        @staticmethod
        def cancel_goal_async():
            nonlocal cancel_count
            cancel_count += 1
            future = Future()
            future.set_result(object())
            return future

    node = object.__new__(FetchDeliverTaskNode)
    node.child_cancel_timeout_s = 0.1
    operation = ChildOperation(
        'late child',
        Future(),
        handle=GoalHandle(),
        result_future=terminal,
    )
    clock_calls = 0

    def monotonic():
        nonlocal clock_calls
        clock_calls += 1
        return 100.0 if clock_calls == 1 else 100.06

    monkeypatch.setattr(task_node_module.time, 'monotonic', monotonic)
    monkeypatch.setattr(task_node_module.rclpy, 'ok', lambda: True)

    with pytest.raises(TaskFailure) as caught:
        node._wait_future(
            terminal,
            SimpleNamespace(is_cancel_requested=False),
            0.05,
            'late child',
            operation,
        )

    assert caught.value.code == CapabilityError.TIMEOUT
    assert cancel_count == 1
