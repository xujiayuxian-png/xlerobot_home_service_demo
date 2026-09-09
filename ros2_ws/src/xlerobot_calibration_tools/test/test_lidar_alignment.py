import numpy as np
import pytest
from xlerobot_calibration_tools.lidar_alignment import estimate_yaw


def test_inverted_straight_wall_produces_correction_without_apply(tmp_path):
    path = tmp_path / 'wall.csv'
    np.savetxt(path, np.column_stack((-np.ones(30), np.linspace(-.5, .5, 30))),
               header='x,y', comments='', delimiter=',')
    result = estimate_yaw(path, 186.442, 180, 0)
    assert result['candidate_lidar_yaw_deg'] == pytest.approx(180)
    assert result['correction_deg'] == pytest.approx(-6.442)
    assert not result['applied']
    with pytest.raises(ValueError):
        estimate_yaw(path, 90, 180, 0)
    with pytest.raises(ValueError):
        estimate_yaw(path, 180, 45, 0)


def test_short_or_invalid_wall_is_rejected(tmp_path):
    path = tmp_path / 'wall.csv'
    np.savetxt(path, np.column_stack((-np.ones(30), np.linspace(-.05, .05, 30))),
               header='x,y', comments='', delimiter=',')
    with pytest.raises(ValueError, match='wall'):
        estimate_yaw(path, 180, 180, 0)
