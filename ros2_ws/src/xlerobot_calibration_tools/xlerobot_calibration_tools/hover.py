"""Same-image board/Tag23 hover metrology and bounded reference-arm planning.

The full-board calibration detector is unchanged. Partial-board estimation is
only for validation; a hidden point is inferred from three non-collinear tags.
"""
from __future__ import annotations

import copy
import itertools
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .fiducial import FixedTargetDetector
from .transforms import validate_transform


JOINTS = ['right_arm_' + name for name in (
    'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')]


def same_board_pose(actual, expected):
    """Stationary-scene gate in physical units: 5 mm translation and 1 degree.

    Full-to-partial board PnP changes orientation uncertainty. Element-wise
    allclose mixed metres and rotation coefficients and mislabeled this as
    movement even at sub-millimetre translation. Never restamp or refit evidence.
    """
    actual, expected = validate_transform(actual), validate_transform(expected)
    translation = np.linalg.norm(actual[:3, 3] - expected[:3, 3])
    angle = Rotation.from_matrix(actual[:3, :3] @ expected[:3, :3].T).magnitude()
    return translation <= .005 and angle <= np.deg2rad(1.)


class HoverDetector:
    def __init__(self):
        self.board = FixedTargetDetector('head_camera')
        self.tag = FixedTargetDetector('right_handeye')

    def estimate(self, corners, ids, camera, distortion):
        visible = set(map(int, ids.flatten())) if ids is not None else set()
        partial = copy.copy(self.board)
        partial._board = {i: p for i, p in self.board._board.items() if i in visible}
        if len(partial._board) < 3:
            raise ValueError('桌面至少需要露出 3 个分散的 Tag，同时露出夹爪 Tag 23')
        centers = np.array([p.mean(axis=0)[:2] for p in partial._board.values()])
        if np.linalg.matrix_rank(centers - centers.mean(axis=0), tol=.005) < 2:
            raise ValueError('可见桌面 Tag 近共线，请露出另一行的 Tag')
        board = partial.estimate(corners, ids, camera, distortion)
        tag = self.tag.estimate(corners, ids, camera, distortion)
        if board is None or tag is None:
            raise ValueError('同一图像中的桌面板 / Tag 23 未通过识别与重投影检查')
        for value in (board, tag):
            validate_transform(value.target_in_camera)
            if value.target_in_camera[2, 3] <= 0:
                raise ValueError('invalid target depth')
        return board, tag

    def detect(self, image, camera, distortion):
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), self.board.dictionary,
            parameters=self.board.parameters)
        return self.estimate(corners, ids, camera, distortion)


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        transform[:3, :3] = Rotation.from_euler(
            'xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return transform


class ArmGeometry:
    def __init__(self, urdf):
        root = ET.fromstring(urdf)
        self.links = {link.get('name'): link for link in root.findall('link')}
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.chain = []
        link = 'right_arm_fixed_jaw_link'
        while link != 'base_link':
            joint = by_child[link]
            self.chain.insert(0, joint)
            link = joint.find('parent').get('link')
        by_name = {j.get('name'): j for j in self.chain}
        self.limits = np.array([[float(by_name[n].find('limit').get(k))
                                for k in ('lower', 'upper')] for n in JOINTS])

    def fk(self, q, all_links=False):
        q = np.asarray(q, dtype=float)
        if q.shape != (5,) or not np.all(np.isfinite(q)):
            raise ValueError('five finite right-arm joints required')
        transform = np.eye(4)
        links = {}
        for joint in self.chain:
            motion = np.eye(4)
            if joint.get('name') in JOINTS:
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                motion[:3, :3] = Rotation.from_rotvec(axis * q[JOINTS.index(joint.get('name'))]).as_matrix()
            transform = transform @ origin(joint.find('origin')) @ motion
            links[joint.find('child').get('link')] = transform.copy()
        return links if all_links else transform

    def clearance(self, q, board):
        links = self.fk(q, True)
        up = -board[:3, 2]
        envelopes = [(links['right_arm_fixed_jaw_link'],
                      np.array(list(itertools.product((-.06, .06), (-.14, .03), (-.07, .07)))))]
        for name in ('right_arm_shoulder_lift_link', 'right_arm_elbow_flex_link',
                     'right_arm_wrist_flex_link'):
            for collision in self.links[name].findall('collision'):
                box = collision.find('geometry/box')
                if box is None:
                    raise ValueError('reference hover requires box collision geometry')
                half = np.fromstring(box.get('size'), sep=' ') / 2
                points = np.array(list(itertools.product(*[(-v, v) for v in half])))
                envelopes.append((links[name] @ origin(collision.find('origin')), points))
        return min(float(np.min(((t[:3, :3] @ p.T).T + t[:3, 3] - board[:3, 3]) @ up))
                   for t, p in envelopes)

    def plan(self, q0, camera_from_board, x, y, target, *, scene_base_from_board=None):
        """Only three high reference points; no arbitrary joint/pose HTTP input."""
        targets = {'center': [0, 0, -.20], 'right': [.03, 0, -.20], 'left': [-.03, 0, -.20]}
        if target not in targets:
            raise ValueError('choose center/right/left at 200 mm above the board')
        for value in (camera_from_board, x, y):
            validate_transform(value)
        q0 = np.asarray(q0, dtype=float)
        self.fk(q0)
        if np.any(q0 <= self.limits[:, 0]) or np.any(q0 >= self.limits[:, 1]):
            raise ValueError('current joints outside URDF bounds')
        board = x @ camera_from_board
        # Hand-eye X is fitted jointly with the arm FK and can absorb arm model
        # bias. Use the independently calibrated head TF for physical obstacles;
        # keep hand-eye X/Y for the target whose accuracy we are measuring.
        scene_board = board if scene_base_from_board is None else validate_transform(scene_base_from_board)
        up = -board[:3, 2]
        if up[2] < .95 or -scene_board[2, 2] < .95:
            raise ValueError('桌面板不水平或板姿态歧义，请检查识别')
        goal = (board @ np.r_[targets[target], 1])[:3]

        def residual(q):
            t = self.fk(q) @ y
            return np.r_[(t[:3, 3] - goal) * 10, np.cross(t[:3, 2], up), (q - q0) * .001]

        fit = least_squares(residual, q0, bounds=(self.limits[:, 0] + .001, self.limits[:, 1] - .001),
                            max_nfev=300)
        t = self.fk(fit.x) @ y
        if np.linalg.norm(t[:3, 3] - goal) >= .002 or t[:3, 2] @ up <= .98:
            raise ValueError('该点不可达，未生成运动命令；换一个点或调整桌面板位置')
        if np.max(np.abs(fit.x - q0)) >= 2.1:
            raise ValueError('关节变化过大，请先回到合适的手眼观测姿态')
        clearance = min(self.clearance(q0 + u * (fit.x - q0), scene_board) for u in np.linspace(0, 1, 81))
        if clearance < .05:
            raise ValueError('路径上机械臂 / 腕相机距离桌面不足 50 mm，未生成运动命令')
        return {'joint_names': JOINTS, 'start': q0.tolist(), 'goal': fit.x.tolist(),
                'target': target, 'target_board_xyz': targets[target],
                'base_from_board': board.tolist(), 'minimum_table_clearance_mm': clearance * 1000,
                'scene_base_from_board': scene_board.tolist(),
                'scene_source': 'image-time calibrated head TF' if scene_base_from_board is not None else 'hand-eye estimate',
                'duration_sec': 12., 'hardware_executed': False,
                'scope': 'reference arm table-plane envelope only; operator must check other obstacles'}


def measurement(row, plan, geometry, x, y):
    board = np.asarray(row['camera_from_board'])
    tag = np.asarray(row['camera_from_tag'])
    visual = (np.linalg.inv(board) @ tag)[:3, 3]
    target = np.asarray(plan['target_board_xyz'])
    error = (visual - target) * 1000
    q = np.array([row['joints'][n] for n in JOINTS])
    fk = geometry.fk(q) @ y
    closure = np.linalg.norm(fk[:3, 3] - (x @ tag)[:3, 3]) * 1000
    return {'planar_error_mm': float(np.linalg.norm(error[:2])),
            'height_shortfall_mm': float(error[2]), 'height_mm': float(-visual[2] * 1000),
            'error_3d_mm': float(np.linalg.norm(error)), 'visual_fk_closure_mm': float(closure),
            'joint_tracking_error_deg': np.rad2deg(q - np.array(plan['goal'])).tolist(),
            'board_ids': row['board_ids'], 'board_reprojection_px': row['board_reprojection_px'],
            'tag_reprojection_px': row['tag_reprojection_px']}
