import cv2
import numpy as np

from xlerobot_calibration_tools.fiducial import FixedTargetDetector


CAMERA = np.asarray([
    [600.0, 0.0, 320.0],
    [0.0, 600.0, 240.0],
    [0.0, 0.0, 1.0],
])
DISTORTION = np.zeros(5)
RVEC = np.asarray([[0.08], [-0.05], [0.03]])
TVEC = np.asarray([[0.02], [-0.01], [0.7]])


def projected(points):
    image, _ = cv2.projectPoints(
        np.asarray(points), RVEC, TVEC, CAMERA, DISTORTION
    )
    return image.reshape(1, -1, 2).astype(np.float32)


def test_verified_head_board_requires_all_tags_and_recovers_pose():
    detector = FixedTargetDetector('head_camera')
    corners = [projected(detector._board[index]) for index in range(16)]
    ids = np.arange(16, dtype=np.int32).reshape(-1, 1)

    estimate = detector.estimate(corners, ids, CAMERA, DISTORTION)

    assert estimate is not None
    assert estimate.tag_count == 16
    assert estimate.reprojection_rmse_px < 1.0e-3
    assert np.allclose(estimate.target_in_camera[:3, 3], TVEC.ravel(), atol=1e-5)
    assert detector.estimate(corners[:-1], ids[:-1], CAMERA, DISTORTION) is None


def test_right_handeye_accepts_only_tag23_under_quality_limit():
    detector = FixedTargetDetector('right_handeye')
    half = 0.030
    points = np.asarray([
        [-half, half, 0.0], [half, half, 0.0],
        [half, -half, 0.0], [-half, -half, 0.0],
    ], dtype=np.float32)
    corners = [projected(points)]

    estimate = detector.estimate(
        corners, np.asarray([[23]], dtype=np.int32), CAMERA, DISTORTION
    )

    assert estimate is not None
    assert estimate.tag_ids == (23,)
    assert estimate.reprojection_rmse_px < 1.0e-3
    assert detector.estimate(
        corners, np.asarray([[10]], dtype=np.int32), CAMERA, DISTORTION
    ) is None
