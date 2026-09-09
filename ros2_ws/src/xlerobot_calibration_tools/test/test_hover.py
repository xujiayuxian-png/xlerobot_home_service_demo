from pathlib import Path
import subprocess

import cv2
import numpy as np
import pytest

from xlerobot_calibration_tools.hover import ArmGeometry, HoverDetector, JOINTS, measurement, same_board_pose
from scipy.spatial.transform import Rotation


CAMERA = np.array([[600., 0, 320], [0, 600, 240], [0, 0, 1]])
DISTORTION = np.zeros(5)


def test_board_motion_gate_compares_metres_and_angles_separately():
    actual = np.eye(4)
    actual[:3, :3] = Rotation.from_euler('x', .85, degrees=True).as_matrix()
    actual[0, 3] = .001
    assert same_board_pose(actual, np.eye(4))
    actual[:3, :3] = Rotation.from_euler('x', 1.1, degrees=True).as_matrix()
    assert not same_board_pose(actual, np.eye(4))
    actual[:3, :3] = np.eye(3)
    actual[:3, 3] = [.004, .004, .004]  # Each coefficient <5 mm, but norm >5 mm.
    assert not same_board_pose(actual, np.eye(4))


def detections(visible):
    detector = HoverDetector()
    points = [detector.board._board[i] for i in visible]
    points += [np.array([[-.03, .03, 0], [.03, .03, 0], [.03, -.03, 0], [-.03, -.03, 0]], dtype=np.float32)]
    corners = [cv2.projectPoints(p, np.array([.08, -.05, .03]), np.array([.02, -.01, .7]),
                                CAMERA, DISTORTION)[0].reshape(1, -1, 2) for p in points]
    return detector, corners, np.array([*visible, 23], dtype=np.int32).reshape(-1, 1)


def test_partial_board_hover_does_not_relax_calibration_gate():
    detector, corners, ids = detections([0, 12, 13])
    board, tag = detector.estimate(corners, ids, CAMERA, DISTORTION)
    assert board.tag_ids == (0, 12, 13) and tag.tag_ids == (23,)
    assert board.reprojection_rmse_px < .001
    assert detector.board.estimate(corners, ids, CAMERA, DISTORTION) is None
    assert len(detector.board._board) == 16


@pytest.mark.parametrize('visible', [[0, 1], [0, 1, 2], [0, 4, 8]])
def test_occluded_or_collinear_board_refused(visible):
    detector, corners, ids = detections(visible)
    with pytest.raises(ValueError):
        detector.estimate(corners, ids, CAMERA, DISTORTION)


def test_tag23_must_be_present_in_same_image():
    detector, corners, ids = detections([0, 12, 13])
    with pytest.raises(ValueError):
        detector.estimate(corners[:-1], ids[:-1], CAMERA, DISTORTION)


def test_measurement_separates_height_planar_tracking_and_closure():
    board = np.eye(4)
    tag = np.eye(4)
    tag[:3, 3] = [.003, .004, -.16]
    row = {'camera_from_board': board, 'camera_from_tag': tag,
           'joints': dict.fromkeys(JOINTS, .01), 'board_ids': [0, 12, 13],
           'board_reprojection_px': .4, 'tag_reprojection_px': .1}
    class Geometry:
        def fk(self, q):
            result = tag.copy()
            result[2, 3] += .02
            return result
    result = measurement(row, {'target_board_xyz': [0, 0, -.2], 'goal': [0]*5},
                         Geometry(), np.eye(4), np.eye(4))
    assert result['planar_error_mm'] == pytest.approx(5)
    assert result['height_shortfall_mm'] == pytest.approx(40)
    assert result['height_mm'] == pytest.approx(160)
    assert result['visual_fk_closure_mm'] == pytest.approx(20)
    assert result['joint_tracking_error_deg'] == pytest.approx([.572957795]*5)


@pytest.fixture(scope='module')
def geometry():
    repo = Path(__file__).resolve().parents[4]
    urdf = subprocess.check_output(['xacro', str(repo / 'ros2_ws/src/xlerobot_description/urdf/two_wheel_reference.urdf.xacro')])
    return ArmGeometry(urdf)


def test_reference_geometry_and_nonfinite_inputs(geometry):
    assert geometry.limits.shape == (5, 2)
    assert np.isfinite(geometry.fk([0]*5)).all()
    with pytest.raises(ValueError):
        geometry.fk([float('nan')]*5)
    with pytest.raises(ValueError, match='center/right/left'):
        geometry.plan([0]*5, np.eye(4), np.eye(4), np.eye(4), 'arbitrary')
    with pytest.raises(ValueError, match='不水平'):
        geometry.plan([0]*5, np.eye(4), np.eye(4), np.eye(4), 'center')


def test_bounded_preview_solves_a_synthetic_reachable_goal_and_checks_clearance(geometry, monkeypatch):
    q = np.array([-.44, -.8, -.65, -.29, -.02])
    jaw = geometry.fk(q)
    y = np.eye(4)
    y[:3, :3] = jaw[:3, :3].T
    board = np.diag([1., -1., -1., 1.])
    board[:3, 3] = jaw[:3, 3] - [0, 0, .2]
    plan = geometry.plan(q, board, np.eye(4), y, 'center')
    assert np.allclose(plan['goal'], q, atol=.001)
    assert not plan['hardware_executed']
    assert plan['minimum_table_clearance_mm'] >= 50
    monkeypatch.setattr(geometry, 'clearance', lambda q, board: .049)
    with pytest.raises(ValueError, match='50 mm'):
        geometry.plan(q, board, np.eye(4), y, 'center')


def test_scene_uses_head_calibration_without_changing_handeye_target(geometry, monkeypatch):
    q = np.array([-.44, -.8, -.65, -.29, -.02])
    jaw = geometry.fk(q)
    y = np.eye(4)
    y[:3, :3] = jaw[:3, :3].T
    board = np.diag([1., -1., -1., 1.])
    board[:3, 3] = jaw[:3, 3] - [0, 0, .2]
    scene = board.copy()
    scene[2, 3] -= .01
    checked = []
    original = geometry.clearance

    def clearance(joints, physical_board):
        checked.append(physical_board.copy())
        return original(joints, physical_board)

    monkeypatch.setattr(geometry, 'clearance', clearance)
    plan = geometry.plan(q, board, np.eye(4), y, 'center', scene_base_from_board=scene)
    assert len(checked) == 81 and all(np.allclose(p, scene) for p in checked)
    assert np.allclose(plan['goal'], q, atol=.001)
    assert np.allclose(plan['base_from_board'], board)
    assert np.allclose(plan['scene_base_from_board'], scene)
    assert plan['scene_source'] == 'image-time calibrated head TF'
    # A physical obstacle still blocks the path even with a reachable hand-eye target.
    scene[2, 3] += .3
    with pytest.raises(ValueError, match='50 mm'):
        geometry.plan(q, board, np.eye(4), y, 'center', scene_base_from_board=scene)


def test_hover_is_exclusive_launch_and_no_raw_joint_http_input():
    repo = Path(__file__).resolve().parents[4]
    source = (repo / 'ros2_ws/src/xlerobot_bringup/xlerobot_bringup/calibration_launch.py').read_text()
    assert source.count("condition=UnlessCondition(LaunchConfiguration('hover_mode'))") == 3
    node = (repo / 'ros2_ws/src/xlerobot_calibration_tools/xlerobot_calibration_tools/hover_node.py').read_text()
    assert "'/right_arm_controller/follow_joint_trajectory'" in node
    assert "('execution_enabled', False)" in node
    assert 'send_goal_async' in node and 'cancel_goal_async' in node
    assert "body.get('goal')" not in node and "body.get('joints')" not in node
