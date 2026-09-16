import math

import pytest

from xlerobot_task.scan_spin import run_scan_spin


class Simulation:
    def __init__(self):
        self.now = 0.0
        self.yaw = 3.0
        self.speed = 0.0
        self.commands = []
        self.blocked = False
        self.stale = False

    def publish(self, speed):
        self.commands.append(speed)
        self.speed = 0.0 if self.blocked else speed

    def sleep(self, seconds):
        self.now += seconds
        self.yaw += self.speed * seconds

    def state(self):
        return (math.atan2(math.sin(self.yaw), math.cos(self.yaw)),
                self.speed, 0.0 if self.stale else self.now)

    def run(self, target, canceled=lambda: False):
        return run_scan_spin(target, self.state, self.publish, canceled, lambda: True,
                             clock=lambda: self.now, sleep=self.sleep)


@pytest.mark.parametrize('target', [math.pi / 2, -math.pi])
def test_turn_tracks_signed_angle_across_wrap_and_stops(target):
    sim = Simulation()
    sim.run(target)
    assert sim.yaw - 3.0 == pytest.approx(target, abs=.03)
    assert max(abs(v) for v in sim.commands) <= .35
    assert sim.commands[-1] == 0.0


def test_laser_stop_cannot_be_mistaken_for_completed_turn():
    sim = Simulation()
    sim.blocked = True
    with pytest.raises(TimeoutError):
        sim.run(math.pi / 2)
    assert sim.commands[-1] == 0.0


def test_cancel_stops_before_returning():
    sim = Simulation()
    with pytest.raises(InterruptedError):
        sim.run(math.pi, canceled=lambda: sim.now > .25)
    assert sim.commands[-1] == 0.0
    assert sim.speed == 0.0


def test_stale_odom_stops_and_fails():
    sim = Simulation()
    sim.stale = True
    with pytest.raises(RuntimeError):
        sim.run(math.pi)
    assert sim.commands[-1] == 0.0
    assert sum(v != 0 for v in sim.commands) <= 7
