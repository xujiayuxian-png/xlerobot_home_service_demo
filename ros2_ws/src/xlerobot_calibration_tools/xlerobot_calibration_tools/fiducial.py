"""Fixed, delivery-profile fiducial geometry and pose estimation."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
from cv2 import aruco
import numpy as np
from scipy.spatial.transform import Rotation


HEAD_WORKFLOW = 'head_camera'
ARM_WORKFLOW = 'right_handeye'


@dataclass(frozen=True)
class TargetEstimate:
    """One accepted target pose in the camera optical frame."""

    target_in_camera: np.ndarray
    tag_count: int
    reprojection_rmse_px: float
    tag_ids: tuple[int, ...]


def _rmse(object_points, image_points, rvec, tvec, camera_matrix, distortion):
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion
    )
    errors = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(errors**2, axis=1))))


def _matrix(rvec, tvec):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(np.asarray(rvec).reshape(3)).as_matrix()
    result[:3, 3] = np.asarray(tvec).reshape(3)
    return result


class FixedTargetDetector:
    """Estimate only the two fiducials shipped with the reference profile."""

    def __init__(self, workflow: str):
        if workflow not in {HEAD_WORKFLOW, ARM_WORKFLOW}:
            raise ValueError('target detector supports only visual calibration')
        self.workflow = workflow
        self.dictionary = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)
        self.parameters = aruco.DetectorParameters_create()
        if hasattr(aruco, 'CORNER_REFINE_APRILTAG'):
            self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 30
        self._board = self._head_board_points()

    @staticmethod
    def _head_board_points() -> dict[int, np.ndarray]:
        tag_size = 0.040
        pitch = tag_size + 0.012
        width = 4 * tag_size + 3 * 0.012
        origin = -width * 0.5
        half = tag_size * 0.5
        points = {}
        for row in range(4):
            for column in range(4):
                marker_id = row * 4 + column
                center_x = origin + column * pitch + half
                center_y = origin + row * pitch + half
                # This physical board was verified with OpenCV detection order
                # mapped to BR, BL, TL, TR in board coordinates.
                points[marker_id] = np.asarray([
                    [center_x + half, center_y + half, 0.0],
                    [center_x - half, center_y + half, 0.0],
                    [center_x - half, center_y - half, 0.0],
                    [center_x + half, center_y - half, 0.0],
                ], dtype=np.float32)
        return points

    def detect(self, image_bgr, camera_matrix, distortion):
        """Detect marker corners then estimate the configured target."""
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )
        return self.estimate(corners, ids, camera_matrix, distortion)

    def estimate(self, corners, ids, camera_matrix, distortion):
        """Estimate from already detected corners; separated for replay tests."""
        if ids is None or not len(ids):
            return None
        if self.workflow == HEAD_WORKFLOW:
            return self._estimate_head(corners, ids, camera_matrix, distortion)
        return self._estimate_arm(corners, ids, camera_matrix, distortion)

    def _estimate_head(self, corners, ids, camera_matrix, distortion):
        detected = {int(value): index for index, value in enumerate(ids.flatten())}
        if set(detected).intersection(self._board) != set(self._board):
            return None
        object_points = np.concatenate([self._board[index] for index in range(16)])
        image_points = np.concatenate([
            np.asarray(corners[detected[index]][0], dtype=np.float32)
            for index in range(16)
        ])
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, camera_matrix, distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        if hasattr(cv2, 'solvePnPRefineLM'):
            rvec, tvec = cv2.solvePnPRefineLM(
                object_points, image_points, camera_matrix, distortion,
                rvec, tvec,
            )
        error = _rmse(
            object_points, image_points, rvec, tvec, camera_matrix, distortion
        )
        if error >= 1.0:
            return None
        return TargetEstimate(_matrix(rvec, tvec), 16, error, tuple(range(16)))

    def _estimate_arm(self, corners, ids, camera_matrix, distortion):
        matches = [
            index for index, marker_id in enumerate(ids.flatten())
            if int(marker_id) == 23
        ]
        if len(matches) != 1:
            return None
        marker = np.asarray(corners[matches[0]], dtype=np.float32)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            marker, 0.060, camera_matrix, distortion
        )
        rvec = rvecs[0].reshape(3, 1)
        tvec = tvecs[0].reshape(3, 1)
        half = 0.030
        object_points = np.asarray([
            [-half, half, 0.0], [half, half, 0.0],
            [half, -half, 0.0], [-half, -half, 0.0],
        ], dtype=np.float32)
        error = _rmse(
            object_points, marker[0], rvec, tvec, camera_matrix, distortion
        )
        if error >= 1.2:
            return None
        return TargetEstimate(_matrix(rvec, tvec), 1, error, (23,))
