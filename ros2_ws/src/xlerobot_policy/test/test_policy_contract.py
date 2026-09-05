import math
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from action_msgs.msg import GoalStatus
from xlerobot_interfaces.action import ExecutePolicyStream
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_policy.act_policy_node import (
    ExecutorFeedback, PolicyFailure, duration_message, duration_seconds,
)


@pytest.mark.parametrize('seconds', [1.0 / 30.0, 1.0, 19.999999999])
def test_duration_conversion_is_normalized(seconds):
    message = duration_message(seconds)
    assert 0 <= message.nanosec < 1_000_000_000
    assert duration_seconds(message) == pytest.approx(seconds, abs=1.0e-9)


def test_duration_conversion_preserves_nonfinite_for_boundary_rejection():
    message = duration_message(1.0)
    assert math.isfinite(duration_seconds(message))


def executor_result(*, code=0, status=GoalStatus.STATUS_SUCCEEDED, session='stream'):
    result = ExecutePolicyStream.Result()
    result.session_id = session
    result.error.code = code
    result.error.message = 'policy input watchdog expired'
    result.accepted_chunks = 3
    result.executed_samples = 30
    future = Future()
    future.set_result(SimpleNamespace(status=status, result=result))
    return future


def test_executor_failure_preserves_cause_instead_of_chunk_timeout():
    state = ExecutorFeedback('stream')
    state.finish(executor_result(
        code=CapabilityError.SAFETY_REJECTED, status=GoalStatus.STATUS_ABORTED,
    ))
    with pytest.raises(PolicyFailure, match='policy input watchdog expired') as caught:
        state.raise_if_failed()
    assert caught.value.code == CapabilityError.SAFETY_REJECTED


def test_final_result_acknowledges_chunks_even_if_last_feedback_is_late():
    state = ExecutorFeedback('stream')
    state.finish(executor_result())
    state.raise_if_failed()
    assert state.accepted_chunks == 3
    assert state.executed_samples == 30


def test_terminal_result_cannot_change_session():
    state = ExecutorFeedback('stream')
    state.finish(executor_result(session='another-stream'))
    with pytest.raises(PolicyFailure, match='session ID'):
        state.raise_if_failed()


def test_aborted_result_without_error_is_not_success():
    state = ExecutorFeedback('stream')
    state.finish(executor_result(status=GoalStatus.STATUS_ABORTED))
    with pytest.raises(PolicyFailure, match='without a successful result'):
        state.raise_if_failed()


def test_executor_result_transport_failure_is_reported():
    state = ExecutorFeedback('stream')
    future = Future()
    future.set_exception(RuntimeError('transport lost'))
    state.finish(future)
    with pytest.raises(PolicyFailure, match='transport lost'):
        state.raise_if_failed()
