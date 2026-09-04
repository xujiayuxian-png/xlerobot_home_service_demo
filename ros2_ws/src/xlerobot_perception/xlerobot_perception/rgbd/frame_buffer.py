"""Thread-safe synchronized RGBD snapshot storage."""

from __future__ import annotations

import threading
import time


def stamp_ns(message) -> int:
    """Return a ROS message header stamp in nanoseconds."""
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def stamps_synchronized(messages, tolerance_s: float) -> bool:
    """Require nonzero RGBD stamps inside one explicit tolerance."""
    stamps = [stamp_ns(message) for message in messages]
    return min(stamps) > 0 and max(stamps) - min(stamps) <= tolerance_s * 1.0e9


class RgbdFrameBuffer:
    """Keep the latest color, aligned depth, and camera-info messages."""

    KEYS = ('color', 'depth', 'info')

    def __init__(self):
        self._condition = threading.Condition()
        self._latest = {key: None for key in self.KEYS}

    def store(self, key, message) -> None:
        """Replace one stream value and wake snapshot waiters."""
        if key not in self._latest:
            raise ValueError(f'unknown RGBD stream: {key}')
        with self._condition:
            self._latest[key] = message
            self._condition.notify_all()

    def wait_snapshot(
        self,
        *,
        timeout_s: float,
        sync_tolerance_s: float,
        max_age_s: float,
        min_stamp_ns: int = 0,
        now_ns,
        check_interrupt,
    ):
        """Return a synchronized current triple newer than an optional barrier."""
        if timeout_s <= 0.0 or sync_tolerance_s <= 0.0 or max_age_s <= 0.0:
            raise ValueError('RGBD snapshot bounds must be positive')
        if min_stamp_ns < 0:
            raise ValueError('min_stamp_ns must be nonnegative')
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while time.monotonic() < deadline:
                check_interrupt()
                messages = tuple(self._latest[key] for key in self.KEYS)
                if all(message is not None for message in messages):
                    if stamps_synchronized(messages, sync_tolerance_s):
                        oldest_stamp = min(stamp_ns(message) for message in messages)
                        newest_stamp = max(stamp_ns(message) for message in messages)
                        age_s = (int(now_ns()) - newest_stamp) * 1.0e-9
                        if (
                            oldest_stamp >= min_stamp_ns
                            and -sync_tolerance_s <= age_s <= max_age_s
                        ):
                            return messages
                self._condition.wait(
                    timeout=min(0.05, max(0.0, deadline - time.monotonic()))
                )
        raise TimeoutError('timed out waiting for synchronized, current RGBD frames')
