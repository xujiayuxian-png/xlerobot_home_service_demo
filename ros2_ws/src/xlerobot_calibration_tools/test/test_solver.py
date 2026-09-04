import numpy as np
from scipy.spatial.transform import Rotation

from xlerobot_calibration_tools.solver import (
    ARM_MODEL,
    CalibrationSample,
    HEAD_MODEL,
    solve,
)
from xlerobot_calibration_tools.transforms import invert


def transform(rotvec, translation):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    value[:3, 3] = translation
    return value


def moving_transforms():
    values = []
    for index in range(12):
        scale = index + 1
        values.append(
            transform(
                [0.09 * scale, -0.04 * (index % 5), 0.07 * (index % 4)],
                [0.20 + 0.015 * scale, -0.10 + 0.02 * (index % 3), 0.6 + 0.01 * scale],
            )
        )
    return values


def test_solves_moving_head_camera_with_fixed_target():
    expected_x = transform([0.04, -0.02, 0.03], [0.031, 0.048, 0.034])
    expected_y = transform([0.1, -0.05, 0.2], [0.8, 0.1, 0.75])
    samples = [
        CalibrationSample(index, moving, invert(expected_x) @ invert(moving) @ expected_y)
        for index, moving in enumerate(moving_transforms())
    ]
    solution = solve(samples, HEAD_MODEL)
    assert np.allclose(solution.x, expected_x, atol=1.0e-6)
    assert solution.metrics['translation_rmse_mm'] < 1.0e-6
    assert solution.metrics['rotation_rmse_deg'] < 1.0e-6


def test_solves_fixed_camera_with_moving_gripper_target():
    expected_x = transform([-0.2, 0.1, 0.4], [-0.27, -0.016, 1.18])
    expected_y = transform([0.08, 0.01, 0.04], [0.002, -0.047, 0.025])
    samples = [
        CalibrationSample(index, moving, invert(expected_x) @ moving @ expected_y)
        for index, moving in enumerate(moving_transforms())
    ]
    solution = solve(samples, ARM_MODEL)
    assert np.allclose(solution.x, expected_x, atol=1.0e-6)
    assert np.allclose(solution.y, expected_y, atol=1.0e-6)
    assert solution.metrics['translation_rmse_mm'] < 1.0e-6


def test_rejects_invalid_sample_transform():
    invalid = np.eye(4)
    invalid[3, 3] = 2.0
    samples = [CalibrationSample(index, invalid, np.eye(4)) for index in range(4)]
    try:
        solve(samples, HEAD_MODEL)
    except ValueError as error:
        assert 'homogeneous row' in str(error)
    else:
        raise AssertionError('invalid transform was accepted')
