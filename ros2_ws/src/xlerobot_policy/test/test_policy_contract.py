import math

import pytest

from xlerobot_policy.act_policy_node import duration_message, duration_seconds


@pytest.mark.parametrize('seconds', [1.0 / 30.0, 1.0, 19.999999999])
def test_duration_conversion_is_normalized(seconds):
    message = duration_message(seconds)
    assert 0 <= message.nanosec < 1_000_000_000
    assert duration_seconds(message) == pytest.approx(seconds, abs=1.0e-9)


def test_duration_conversion_preserves_nonfinite_for_boundary_rejection():
    message = duration_message(1.0)
    assert math.isfinite(duration_seconds(message))
