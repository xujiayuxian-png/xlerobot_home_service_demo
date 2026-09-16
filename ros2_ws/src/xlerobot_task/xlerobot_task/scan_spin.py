"""Odometry-controlled person-search turns through the protected Nav2 chain."""

import math
import time


def run_scan_spin(target, read_state, publish_speed, canceled, running,
                  *, clock=time.monotonic, sleep=time.sleep):
    """State is (yaw, angular speed, monotonic receipt time), or None.

    The caller must publish upstream of velocity_smoother/collision_monitor.
    This routine never consumes or modifies the navigation costmap.
    """
    if not math.isfinite(target):
        raise ValueError('invalid body turn')
    started = clock()
    previous = None
    traveled = 0.0
    try:
        while running():
            if canceled():
                raise InterruptedError('person search canceled')
            now = clock()
            if now - started >= 21.0:
                raise TimeoutError('body turn timed out; check laser stop or drive feedback')
            state = read_state()
            if state is None:
                if previous is not None or now - started > 2.0:
                    raise RuntimeError('body turn has no fresh odometry')
                sleep(0.05)
                continue
            yaw, speed, received = state
            if (not all(math.isfinite(v) for v in state)
                    or not 0.0 <= now - received <= 0.3):
                raise RuntimeError('body turn odometry is stale or invalid')
            if previous is not None:
                traveled += math.atan2(math.sin(yaw - previous), math.cos(yaw - previous))
            previous = yaw
            remaining = target - traveled
            if abs(remaining) <= 0.03:
                return
            # Same maximum speed as Nav2 Spin; slow down near the target.
            publish_speed(math.copysign(min(0.35, max(0.04, abs(remaining))), remaining))
            sleep(0.05)
        raise RuntimeError('body turn interrupted by shutdown')
    finally:
        # Keep ownership while the smoother decelerates. Do not start a head
        # view until fresh feedback confirms that rotation has stopped.
        stop_started = clock()
        while True:
            publish_speed(0.0)
            state = read_state()
            now = clock()
            if (now - stop_started >= 0.2 and state is not None
                    and all(math.isfinite(v) for v in state)
                    and 0.0 <= now - state[2] <= 0.3 and abs(state[1]) <= 0.03):
                break
            if now - stop_started >= 2.0:
                raise RuntimeError('body turn stop could not be confirmed from odometry')
            sleep(0.05)
