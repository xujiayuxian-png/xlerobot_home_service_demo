"""Deterministic rolling ACT queue without ROS or controller ownership."""

from __future__ import annotations

from dataclasses import dataclass
import math

from xlerobot_policy.postprocess import PhaseAwareGripperGovernor
from xlerobot_policy.postprocess import validate_action_chunk


@dataclass(frozen=True)
class RealtimeQueueConfig:
    """Bounds preserved from the verified realtime chunk queue."""

    queued_action_steps: int = 60
    refill_threshold_steps: int = 15
    blend_steps: int = 10
    close_hold_steps: int = 30
    control_hz: float = 30.0

    def validate(self):
        """Reject invalid queue geometry instead of coercing it."""
        if self.queued_action_steps <= 0 or self.refill_threshold_steps <= 0:
            raise ValueError('queue and refill step counts must be positive')
        if self.refill_threshold_steps >= self.queued_action_steps:
            raise ValueError('refill threshold must be smaller than queue size')
        if self.blend_steps < 0 or self.close_hold_steps < 0:
            raise ValueError('blend and hold step counts must not be negative')
        if not math.isfinite(self.control_hz) or self.control_hz <= 0.0:
            raise ValueError('control_hz must be finite and positive')


@dataclass(frozen=True)
class MergeInfo:
    """Explain how one timestamped proposal changed the local queue."""

    accepted: bool
    skipped_steps: int
    appended_steps: int
    blended_steps: int


class RealtimeActionQueue:
    """Align rolling proposals, govern the gripper, and stop refilling safely."""

    def __init__(self, governor, config=RealtimeQueueConfig()):
        config.validate()
        if not isinstance(governor, PhaseAwareGripperGovernor):
            raise TypeError('governor must be a PhaseAwareGripperGovernor')
        if not governor.config.lock_closed:
            raise ValueError('realtime completion requires lock_closed=true')
        self.governor = governor
        self.config = config
        self.reset()

    def reset(self):
        """Start a new empty inference session."""
        self.governor.reset()
        self._raw = []
        self._hold_action = None
        self._hold_remaining = 0
        self._emitted_samples = 0
        self._last_prediction_id = -1
        self._locked_after_close = False

    @property
    def raw_steps(self):
        """Return the number of proposal rows not yet committed."""
        return len(self._raw)

    @property
    def hold_steps(self):
        """Return remaining post-trigger close dwell samples."""
        return self._hold_remaining

    @property
    def emitted_samples(self):
        """Return the number of governed rows produced this session."""
        return self._emitted_samples

    @property
    def locked_after_close(self):
        """Report whether new inference proposals are permanently ignored."""
        return self._locked_after_close

    @property
    def needs_refill(self):
        """Request a fresh observation before the local reserve drains."""
        return (
            not self._locked_after_close
            and len(self._raw) <= self.config.refill_threshold_steps
        )

    @property
    def complete(self):
        """Complete only after close, dwell, and the committed tail drain."""
        return (
            self._locked_after_close
            and self.governor.state == self.governor.CLOSED
            and self._hold_remaining == 0
            and not self._raw
        )

    def merge_prediction(
        self,
        actions,
        *,
        prediction_id,
        observation_step,
        future_step,
    ):
        """Time-align and blend a fresh proposal into the uncommitted tail."""
        rows = validate_action_chunk(actions)
        if self._locked_after_close or prediction_id <= self._last_prediction_id:
            return MergeInfo(False, 0, 0, 0)
        observation_step = int(observation_step)
        future_step = int(future_step)
        if observation_step < 0 or future_step < observation_step:
            raise ValueError('proposal timeline steps are invalid')
        offset = future_step - observation_step
        self._last_prediction_id = int(prediction_id)
        if offset >= len(rows):
            return MergeInfo(False, offset, 0, 0)
        segment = [
            list(row)
            for row in rows[offset:offset + self.config.queued_action_steps]
        ]
        if not self._raw:
            self._raw = segment
            return MergeInfo(True, offset, len(segment), 0)

        blend_count = min(
            self.config.blend_steps,
            len(self._raw),
            len(segment),
        )
        if blend_count:
            tail_start = len(self._raw) - blend_count
            for index in range(blend_count):
                alpha = float(index + 1) / float(blend_count + 1)
                previous = self._raw[tail_start + index]
                target = segment[index]
                self._raw[tail_start + index] = [
                    (1.0 - alpha) * old + alpha * new
                    for old, new in zip(previous, target)
                ]
        self._raw.extend(segment[blend_count:])
        return MergeInfo(
            True,
            offset,
            len(segment) - blend_count,
            blend_count,
        )

    def emit(self, max_samples):
        """Commit at most max_samples governed rows for executor validation."""
        if max_samples <= 0:
            raise ValueError('max_samples must be positive')
        output = []
        while len(output) < max_samples:
            if self._hold_remaining > 0:
                raw = list(self._hold_action)
                self._hold_remaining -= 1
            elif self._raw:
                raw = self._raw.pop(0)
            else:
                break
            before = self.governor.state
            processed, _info = self.governor.process(
                [raw],
                future_actions=self._raw,
                elapsed_s=self._emitted_samples / self.config.control_hz,
            )
            row = processed[0]
            output.append(row)
            self._emitted_samples += 1
            if (
                before == self.governor.OPENING
                and self.governor.state
                in (self.governor.CLOSING, self.governor.CLOSED)
            ):
                self._locked_after_close = True
                self._hold_action = list(row)
                self._hold_remaining = self.config.close_hold_steps
        return output
