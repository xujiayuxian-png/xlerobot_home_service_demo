import math

import pytest

from xlerobot_policy.postprocess import (
    freeze_joint,
    GRIPPER_INDEX,
    GripperGovernorConfig,
    PhaseAwareGripperGovernor,
    split_chunk,
    validate_action_chunk,
)


def action(lift, gripper, pan=0.11):
    return [pan, lift, 0.22, 0.33, 0.44, gripper]


def test_chunk_schema_is_exact_finite_and_bounded():
    assert validate_action_chunk(action(0.4, 1.0)) == [action(0.4, 1.0)]
    with pytest.raises(ValueError, match='exactly 6'):
        validate_action_chunk([[0.0] * 7])
    with pytest.raises(ValueError, match='nonfinite'):
        validate_action_chunk([action(0.4, math.nan)])
    with pytest.raises(ValueError, match='exceeds'):
        validate_action_chunk([action(0.4, 1.0)] * 3, max_steps=2)


def test_freeze_pan_copies_and_split_preserves_every_row():
    source = [action(0.4, 1.0, 0.2), action(0.3, 0.5, 0.4), action(0.2, 0.0, 0.6)]
    frozen = freeze_joint(source, 0, -0.33)
    assert [row[0] for row in frozen] == [-0.33, -0.33, -0.33]
    assert [row[0] for row in source] == [0.2, 0.4, 0.6]
    assert split_chunk(frozen, 2) == [frozen[:2], frozen[2:]]


def test_governor_keeps_open_until_bottom_then_ramps_and_locks():
    governor = PhaseAwareGripperGovernor(GripperGovernorConfig(
        close_confirm_steps=1,
        close_lookahead_steps=4,
        min_close_delay_s=0.0,
        close_ramp_s=0.3,
        control_hz=10.0,
    ))
    high, info = governor.process(
        [action(-0.6, 0.1)],
        future_actions=[action(-1.0, 0.0)],
        elapsed_s=1.0,
    )
    assert high[0][GRIPPER_INDEX] == 1.0
    assert not info['near_bottom']
    first, first_info = governor.process([action(-1.0, 0.0)], elapsed_s=1.1)
    second, _ = governor.process([action(-1.0, 0.0)], elapsed_s=1.2)
    third, third_info = governor.process([action(-1.0, 0.0)], elapsed_s=1.3)
    assert first[0][GRIPPER_INDEX] == pytest.approx(2.0 / 3.0)
    assert first_info['state'] == PhaseAwareGripperGovernor.CLOSING
    assert second[0][GRIPPER_INDEX] == pytest.approx(1.0 / 3.0)
    assert third[0][GRIPPER_INDEX] == 0.0
    assert third_info['state'] == PhaseAwareGripperGovernor.CLOSED
    held, _ = governor.process([action(-0.5, 1.5)], elapsed_s=2.0)
    assert held[0][GRIPPER_INDEX] == 0.0


def test_verified_defaults_do_not_close_before_delay():
    governor = PhaseAwareGripperGovernor()
    rows, info = governor.process([action(-1.0, 0.0)], elapsed_s=0.5)
    assert rows[0][GRIPPER_INDEX] == 1.0
    assert info['state'] == PhaseAwareGripperGovernor.OPENING


def test_invalid_governor_config_is_rejected_not_coerced():
    with pytest.raises(ValueError):
        PhaseAwareGripperGovernor(GripperGovernorConfig(control_hz=0.0))
