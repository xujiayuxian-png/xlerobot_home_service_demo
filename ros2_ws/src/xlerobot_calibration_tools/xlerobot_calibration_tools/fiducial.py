"""Fixed demo-profile fiducial geometry and pose estimation."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
from cv2 import aruco
import numpy as np
from scipy.spatial.transform import Rotation

from .profiles import quality_profile, target_profile


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
        self.target = target_profile(workflow)
        self.maximum_reprojection_rmse_px = float(
            quality_profile(workflow)['maximum_reprojection_rmse_px']
        )
        if self.target.get('dictionary') != 'apriltag_36h11':
            raise ValueError('only the shipped AprilTag 36h11 targets are supported')
        if hasattr(aruco, 'getPredefinedDictionary'):
            self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        else:
            self.dictionary = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)
        # OpenCV 4.6 exposes both APIs, but assigning APRILTAG refinement on
        # the constructor-created object can crash in that release.
        if hasattr(aruco, 'DetectorParameters_create'):
            self.parameters = aruco.DetectorParameters_create()
        else:
            self.parameters = aruco.DetectorParameters()
        if hasattr(aruco, 'CORNER_REFINE_APRILTAG'):
            self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 30
        self._board = self._head_board_points(self.target) if workflow == HEAD_WORKFLOW else {}
        self.diagnostic = {'tag_count': 0, 'reprojection_rmse_px': 0.0,
                           'detail': 'waiting for an image'}

    @staticmethod
    def _head_board_points(profile: dict | None = None) -> dict[int, np.ndarray]:
        profile = profile or target_profile(HEAD_WORKFLOW)
        rows = int(profile['rows'])
        columns = int(profile['columns'])
        first_id = int(profile['first_id'])
        tag_size = float(profile['tag_size_m'])
        separation = float(profile['tag_separation_m'])
        pitch = tag_size + separation
        width = columns * tag_size + (columns - 1) * separation
        height = rows * tag_size + (rows - 1) * separation
        origin = -width * 0.5
        origin_y = -height * 0.5
        half = tag_size * 0.5
        points = {}
        for row in range(rows):
            for column in range(columns):
                marker_id = first_id + row * columns + column
                center_x = origin + column * pitch + half
                center_y = origin_y + row * pitch + half
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

    def detect_with_debug(self, image_bgr, camera_matrix, distortion):
        """Detect once and annotate the same image, including rejected targets."""
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        estimate = self.estimate(corners, ids, camera_matrix, distortion)
        debug = image_bgr.copy()
        if ids is not None and len(ids):
            aruco.drawDetectedMarkers(debug, corners, ids)
        color = (80, 210, 80) if estimate is not None else (40, 170, 255)
        cv2.rectangle(debug, (0, 0), (debug.shape[1], 42), (25, 25, 25), -1)
        cv2.putText(debug, self.diagnostic['detail'], (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
        return estimate, debug

    def estimate(self, corners, ids, camera_matrix, distortion):
        """Estimate from already detected corners; separated for replay tests."""
        required = set(self._board) if self.workflow == HEAD_WORKFLOW else {int(self.target['marker_id'])}
        seen = set(int(value) for value in ids.flatten()) if ids is not None else set()
        count = len(seen & required)
        self.diagnostic = {'tag_count': count, 'reprojection_rmse_px': 0.0,
                           'detail': f'tags {count}/{len(required)}: keep the full target visible'}
        if ids is None or not len(ids):
            return None
        if self.workflow == HEAD_WORKFLOW:
            result = self._estimate_head(corners, ids, camera_matrix, distortion)
        else:
            result = self._estimate_arm(corners, ids, camera_matrix, distortion)
        if result is not None:
            self.diagnostic['detail'] = f'tags {count}/{len(required)} accepted | reprojection {result.reprojection_rmse_px:.2f} px'
        return result

    def _estimate_head(self, corners, ids, camera_matrix, distortion):
        detected = {int(value): index for index, value in enumerate(ids.flatten())}
        if set(detected).intersection(self._board) != set(self._board):
            return None
        marker_ids = sorted(self._board)
        object_points = np.concatenate([self._board[index] for index in marker_ids])
        image_points = np.concatenate([
            np.asarray(corners[detected[index]][0], dtype=np.float32)
            for index in marker_ids
        ])
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, camera_matrix, distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            self.diagnostic['detail'] = 'full board detected, but pose estimation failed'
            return None
        if hasattr(cv2, 'solvePnPRefineLM'):
            rvec, tvec = cv2.solvePnPRefineLM(
                object_points, image_points, camera_matrix, distortion,
                rvec, tvec,
            )
        error = _rmse(
            object_points, image_points, rvec, tvec, camera_matrix, distortion
        )
        self._record_reprojection(error)
        if not np.isfinite(error) or error >= self.maximum_reprojection_rmse_px:
            return None
        return TargetEstimate(
            _matrix(rvec, tvec), len(marker_ids), error, tuple(marker_ids)
        )

    def _estimate_arm(self, corners, ids, camera_matrix, distortion):
        matches = [
            index for index, marker_id in enumerate(ids.flatten())
            if int(marker_id) == int(self.target['marker_id'])
        ]
        if len(matches) != 1:
            return None
        marker = np.asarray(corners[matches[0]], dtype=np.float32)
        tag_size = float(self.target['tag_size_m'])
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            marker, tag_size, camera_matrix, distortion
        )
        rvec = rvecs[0].reshape(3, 1)
        tvec = tvecs[0].reshape(3, 1)
        half = tag_size * 0.5
        object_points = np.asarray([
            [-half, half, 0.0], [half, half, 0.0],
            [half, -half, 0.0], [-half, -half, 0.0],
        ], dtype=np.float32)
        error = _rmse(
            object_points, marker[0], rvec, tvec, camera_matrix, distortion
        )
        self._record_reprojection(error)
        if not np.isfinite(error) or error >= self.maximum_reprojection_rmse_px:
            return None
        return TargetEstimate(
            _matrix(rvec, tvec), 1, error, (int(self.target['marker_id']),)
        )

    def _record_reprojection(self, error):
        if not np.isfinite(error):
            self.diagnostic['detail'] = 'target pose has a non-finite reprojection error'
            return
        self.diagnostic['reprojection_rmse_px'] = float(error)
        if error >= self.maximum_reprojection_rmse_px:
            self.diagnostic['detail'] = (
                f'tags {self.diagnostic["tag_count"]}: reprojection {error:.2f} px '
                f'>= {self.maximum_reprojection_rmse_px:g}; check print size, blur and board flatness')
