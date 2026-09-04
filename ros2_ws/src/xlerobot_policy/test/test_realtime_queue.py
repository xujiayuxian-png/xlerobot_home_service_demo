import pytest

from xlerobot_policy.postprocess import (
    GripperGovernorConfig,
    PhaseAwareGripperGovernor,
)
from xlerobot_policy.realtime_queue import (
    RealtimeActionQueue,
    RealtimeQueueConfig,
)


def make_queue(*, close_hold_steps=3, blend_steps=2):
    governor = PhaseAwareGripperGovernor(
        GripperGovernorConfig(
            min_close_delay_s=0.0,
            close_ramp_s=0.2,
            control_hz=10.0,
            close_lookahead_steps=10,
            bottom_lookahead_steps=10,
        )
    )
    return RealtimeActionQueue(
        governor,
        RealtimeQueueConfig(
            queued_action_steps=6,
            refill_threshold_steps=2,
            blend_steps=blend_steps,
            close_hold_steps=close_hold_steps,
            control_hz=10.0,
        ),
    )


def rows(count, *, base=0.0, close_from=None):
    output = []
    for index in range(count):
        gripper = 1.0 if close_from is None or index < close_from else 0.0
        descent_step = index if close_from is None else min(index, close_from)
        output.append([base + index, -float(descent_step), 0.2, 0.3, 0.4, gripper])
    return output


def test_timestamp_alignment_and_tail_blending_preserve_future_order():
    queue = make_queue(close_hold_steps=0)
    first = queue.merge_prediction(
        rows(10), prediction_id=0, observation_step=0, future_step=0
    )
    assert first.accepted
    assert queue.raw_steps == 6
    assert queue.emit(4)[-1][0] == 3.0
    assert queue.needs_refill

    second = queue.merge_prediction(
        rows(10, base=100.0),
        prediction_id=1,
        observation_step=2,
        future_step=6,
    )
    assert second.skipped_steps == 4
    assert second.blended_steps == 2
    assert second.appended_steps == 4
    emitted = queue.emit(6)
    assert emitted[0][0] == pytest.approx((2.0 / 3.0) * 4.0 + (1.0 / 3.0) * 104.0)
    assert emitted[1][0] == pytest.approx((1.0 / 3.0) * 5.0 + (2.0 / 3.0) * 105.0)
    assert [row[0] for row in emitted[2:]] == [106.0, 107.0, 108.0, 109.0]


def test_close_trigger_locks_refill_holds_arm_and_drains_tail_closed():
    queue = make_queue(close_hold_steps=3, blend_steps=0)
    queue.merge_prediction(
        rows(6, close_from=2), prediction_id=0, observation_step=0, future_step=0
    )
    emitted = queue.emit(20)
    assert queue.locked_after_close
    assert not queue.needs_refill
    assert queue.complete
    assert len(emitted) == 9
    trigger_arm = emitted[2][:5]
    assert all(row[:5] == trigger_arm for row in emitted[3:6])
    assert all(row[-1] == 0.0 for row in emitted[-3:])
    rejected = queue.merge_prediction(
        rows(6, base=200.0),
        prediction_id=1,
        observation_step=2,
        future_step=6,
    )
    assert not rejected.accepted


def test_stale_and_duplicate_predictions_never_change_the_queue():
    queue = make_queue()
    queue.merge_prediction(
        rows(6), prediction_id=3, observation_step=0, future_step=0
    )
    before = queue.raw_steps
    duplicate = queue.merge_prediction(
        rows(6, base=100.0),
        prediction_id=3,
        observation_step=0,
        future_step=0,
    )
    stale = queue.merge_prediction(
        rows(3),
        prediction_id=4,
        observation_step=0,
        future_step=10,
    )
    assert not duplicate.accepted
    assert not stale.accepted
    assert queue.raw_steps == before


@pytest.mark.parametrize(
    'config',
    [
        RealtimeQueueConfig(queued_action_steps=1, refill_threshold_steps=1),
        RealtimeQueueConfig(blend_steps=-1),
        RealtimeQueueConfig(control_hz=0.0),
    ],
)
def test_invalid_queue_geometry_is_rejected(config):
    with pytest.raises(ValueError):
        config.validate()


def test_queue_requires_a_latched_closed_completion_state():
    governor = PhaseAwareGripperGovernor(
        GripperGovernorConfig(lock_closed=False)
    )
    with pytest.raises(ValueError, match='lock_closed'):
        RealtimeActionQueue(governor)
