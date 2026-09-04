"""Validated ACT chunk transformations preserved from the verified demo."""

from __future__ import annotations

from dataclasses import dataclass
import math


SHOULDER_LIFT_INDEX = 1
GRIPPER_INDEX = 5


def validate_action_chunk(actions, *, joint_count=6, max_steps=500):
    """Return finite float rows with one exact, bounded joint layout."""
    if not isinstance(actions, (list, tuple)) or not actions:
        raise ValueError('action chunk must be a nonempty list')
    if len(actions) > max_steps:
        raise ValueError(f'action chunk exceeds {max_steps} steps')
    if isinstance(actions[0], (int, float)):
        actions = [actions]
    validated = []
    for index, raw in enumerate(actions):
        if not isinstance(raw, (list, tuple)) or len(raw) != joint_count:
            raise ValueError(
                f'action step {index} must contain exactly {joint_count} values'
            )
        try:
            row = [float(value) for value in raw]
        except (TypeError, ValueError) as exc:
            raise ValueError(f'action step {index} contains a nonnumeric value') from exc
        if not all(math.isfinite(value) for value in row):
            raise ValueError(f'action step {index} contains a nonfinite value')
        validated.append(row)
    return validated


def freeze_joint(actions, index, value):
    """Replace one policy dimension without mutating the remote response."""
    rows = [list(action) for action in actions]
    if value is None:
        return rows
    if not rows or index < 0 or any(index >= len(row) for row in rows):
        raise ValueError('frozen joint index is outside the action layout')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('frozen joint value must be finite')
    for row in rows:
        row[index] = value
    return rows


def split_chunk(actions, max_samples):
    """Split validated rows so a producer can respect executor queue bounds."""
    if max_samples <= 0:
        raise ValueError('max_samples must be positive')
    return [
        [list(row) for row in actions[start:start + max_samples]]
        for start in range(0, len(actions), max_samples)
    ]


@dataclass(frozen=True)
class GripperGovernorConfig:
    """Phase and timing parameters with the verified reference defaults."""

    open_floor: float = 1.0
    closed_value: float = 0.0
    raw_close_threshold: float = 0.30
    close_confirm_steps: int = 1
    close_lookahead_steps: int = 100
    near_bottom_margin_rad: float = 0.08
    bottom_lookahead_steps: int = 30
    min_close_delay_s: float = 0.8
    close_ramp_s: float = 0.25
    lock_closed: bool = True
    control_hz: float = 30.0

    def validate(self):
        """Reject nonsensical thresholds instead of coercing them silently."""
        finite = (
            self.open_floor,
            self.closed_value,
            self.raw_close_threshold,
            self.near_bottom_margin_rad,
            self.min_close_delay_s,
            self.close_ramp_s,
            self.control_hz,
        )
        if not all(math.isfinite(value) for value in finite):
            raise ValueError('governor parameters must be finite')
        if self.close_confirm_steps <= 0 or self.bottom_lookahead_steps <= 0:
            raise ValueError('governor step counts must be positive')
        if self.close_lookahead_steps < 0:
            raise ValueError('close_lookahead_steps must not be negative')
        if self.near_bottom_margin_rad < 0.0 or self.min_close_delay_s < 0.0:
            raise ValueError('governor margins and delays must not be negative')
        if self.close_ramp_s < 0.0 or self.control_hz <= 0.0:
            raise ValueError('governor rate must be positive and ramp nonnegative')


class PhaseAwareGripperGovernor:
    """Keep open during descent, close near the bottom, then hold closed."""

    OPENING = 'OPENING'
    CLOSING = 'CLOSING'
    CLOSED = 'CLOSED'

    def __init__(self, config=GripperGovernorConfig()):
        config.validate()
        self.config = config
        self.close_ramp_steps = max(1, round(config.close_ramp_s * config.control_hz))
        self.reset()

    def reset(self):
        """Begin a new grasp with an explicitly open gripper phase."""
        self.state = self.OPENING
        self.low_close_count = 0
        self.ramp_start_value = self.config.open_floor
        self.ramp_step = 0
        self.last_info = {}

    def process(self, actions, *, future_actions=None, elapsed_s=0.0):
        """Process sequential rows while retaining phase across remote chunks."""
        actions = validate_action_chunk(actions)
        future = [] if not future_actions else validate_action_chunk(future_actions)
        combined = actions + future
        processed = []
        for index, action in enumerate(actions):
            current = list(action)
            raw_gripper = current[GRIPPER_INDEX]
            raw_lift = current[SHOULDER_LIFT_INDEX]
            step_elapsed = float(elapsed_s) + index / self.config.control_hz
            near_bottom = self._near_bottom(combined, index, raw_lift)
            close_intent = self._close_intent(combined, index, raw_gripper)
            if self.state == self.OPENING:
                published = max(raw_gripper, self.config.open_floor)
                if (
                    step_elapsed >= self.config.min_close_delay_s
                    and close_intent
                    and near_bottom
                ):
                    self.low_close_count += 1
                else:
                    self.low_close_count = 0
                if self.low_close_count >= self.config.close_confirm_steps:
                    self.state = self.CLOSING
                    self.ramp_start_value = published
                    self.ramp_step = 0
                    published = self._next_ramp_value()
            elif self.state == self.CLOSING:
                published = self._next_ramp_value()
            else:
                published = self.config.closed_value
            current[GRIPPER_INDEX] = published
            processed.append(current)
            self.last_info = {
                'state': self.state,
                'raw_gripper': raw_gripper,
                'published_gripper': published,
                'raw_shoulder_lift': raw_lift,
                'near_bottom': near_bottom,
                'close_intent': close_intent,
                'close_confirm_count': self.low_close_count,
            }
        return processed, dict(self.last_info)

    def _near_bottom(self, actions, index, raw_lift):
        end = min(len(actions), index + self.config.bottom_lookahead_steps)
        bottom = min(row[SHOULDER_LIFT_INDEX] for row in actions[index:end])
        return raw_lift <= bottom + self.config.near_bottom_margin_rad

    def _close_intent(self, actions, index, raw_gripper):
        if raw_gripper <= self.config.raw_close_threshold:
            return True
        if self.config.close_lookahead_steps == 0:
            return False
        end = min(len(actions), index + self.config.close_lookahead_steps)
        return any(
            row[GRIPPER_INDEX] <= self.config.raw_close_threshold
            for row in actions[index:end]
        )

    def _next_ramp_value(self):
        self.ramp_step += 1
        alpha = min(1.0, self.ramp_step / self.close_ramp_steps)
        value = (
            (1.0 - alpha) * self.ramp_start_value
            + alpha * self.config.closed_value
        )
        if alpha >= 1.0 and self.config.lock_closed:
            self.state = self.CLOSED
        elif alpha >= 1.0:
            self.state = self.OPENING
            self.low_close_count = 0
        return value
