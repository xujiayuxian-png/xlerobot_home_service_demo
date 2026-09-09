"""Evaluate reference URDF FK only. This does not certify collision clearance."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def test_fit_and_heldout_poses_are_in_limits_and_kinematically_distinct():
    source = Path(__file__).resolve().parents[2]
    config = yaml.safe_load((source / 'xlerobot_calibration_tools/config/right_handeye_poses.yaml').read_text())
    xml = subprocess.run(['xacro', str(source / 'xlerobot_description/urdf/two_wheel_reference.urdf.xacro')],
                         check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(xml)
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    by_child = {j.find('child').attrib['link']: j for j in joints.values()}
    chain = []
    link = 'right_arm_fixed_jaw_link'
    while link != 'base_link':
        joint = by_child[link]
        chain.insert(0, joint)
        link = joint.find('parent').attrib['link']
    transforms = []
    legacy_flex = [.4510, .4525, .4525, .4403, .4495, .4495,
                   -.4771, -.4648, -.4633, -.7762, -.7747, -.7747,
                   .4456, .4464, .4464, .4449, -.0184, -.0123, -.0115,
                   .4449, .4517, .4525, .4490, -.4710, -.7750, .4460]
    legacy_poses = [row[:3] + [flex, row[4]]
                    for row, flex in zip(config['poses'], legacy_flex)]
    adjustments = np.array(legacy_flex) - np.array(config['poses'])[:, 3]
    assert np.all(adjustments >= .12 - 1e-8)
    assert np.all(adjustments <= .24 + 1e-8)
    assert len(set(np.round(adjustments, 4))) >= 5
    for row in config['poses'] + legacy_poses:
        positions = dict(zip(config['joint_order'], row))
        positions['right_arm_wrist_roll'] += config.get('wrist_roll_offset_rad', 0.0)
        for name, angle in positions.items():
            limits = joints[name].find('limit')
            assert float(limits.attrib['lower']) <= angle <= float(limits.attrib['upper'])
        pose = np.eye(4)
        for joint in chain:
            origin = joint.find('origin')
            fixed = np.eye(4)
            if origin is not None:
                fixed[:3, 3] = np.fromstring(origin.attrib.get('xyz', '0 0 0'), sep=' ')
                fixed[:3, :3] = Rotation.from_euler('xyz', np.fromstring(
                    origin.attrib.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            rotation = np.eye(4)
            if joint.attrib['name'] in positions:
                axis = np.fromstring(joint.find('axis').attrib['xyz'], sep=' ')
                rotation[:3, :3] = Rotation.from_rotvec(axis * positions[joint.attrib['name']]).as_matrix()
            pose = pose @ fixed @ rotation
        transforms.append(pose)
    # Check the actual mounted orientation, not the pre-offset source angles.
    # A negative flex adjustment raises the fingertip; visibility/collision
    # clearance still require a supervised hardware check.
    tip = np.array([0, -.097, 0, 1])
    for i in range(26):
        rise = ((transforms[i] - transforms[i + 26]) @ tip)[2]
        assert .015 < rise < .040, f'pose {i + 1} must lift the fingertip'
    transforms = transforms[:26]
    # Retain multiple rotation axes in the fitting subset, not just positions.
    rotations = np.array([Rotation.from_matrix(transforms[0][:3, :3].T @ t[:3, :3]).as_rotvec()
                          for t in transforms[1:20]])
    assert np.linalg.matrix_rank(rotations, tol=.01) == 3
    for i, pose in enumerate(transforms):
        for previous in transforms[:i]:
            distance = np.linalg.norm(pose[:3, 3] - previous[:3, 3])
            angle = np.degrees(Rotation.from_matrix(previous[:3, :3].T @ pose[:3, :3]).magnitude())
            assert distance >= .002 or angle >= 2, f'pose {i + 1} is not independent'
