import math
from pathlib import Path
import subprocess

import pytest

from xlerobot_bringup.initial_pose import command_limits, outside_limits


def test_leader_limits_match_verified_raw_boundaries():
    model = Path(__file__).parents[2] / 'xlerobot_description/urdf/leader_attachment.urdf.xacro'
    xml = subprocess.check_output(['xacro', str(model)], text=True)
    limits = command_limits(xml)
    assert limits['leader_elbow_flex'][1] == pytest.approx((3130 - 1987) * 2 * math.pi / 4096)
    assert limits['leader_gripper'][0] == 0.0
    assert outside_limits(limits, dict(zip(limits, [0.0, 1.56, 1.531, 0.78, 0.03, 0.025]))) == []
    assert outside_limits(limits, {name: 10.0 for name in limits}) == list(limits)


def test_position_admission_requires_all_finite_in_range_joints():
    limits = {'elbow': (-1.65, 1.4), 'gripper': (0.05, 1.65)}
    assert outside_limits(limits, {}) == ['elbow', 'gripper']
    assert outside_limits(limits, {'elbow': 1.52, 'gripper': 0.5}) == ['elbow']
    assert outside_limits(limits, {'elbow': math.nan, 'gripper': 0.5}) == ['elbow']
    assert outside_limits(limits, {'elbow': 1.4, 'gripper': 0.05}) == []


def test_limits_come_from_control_contract_not_fixed_or_wheel_joints():
    xml = '''<robot><ros2_control>
      <joint name="elbow"><command_interface name="position">
        <param name="min">-1.65</param><param name="max">1.4</param>
      </command_interface></joint>
      <joint name="wheel"><command_interface name="velocity"/></joint>
    </ros2_control></robot>'''
    assert command_limits(xml) == {'elbow': (-1.65, 1.4)}
    with pytest.raises(ValueError):
        command_limits(xml.replace('1.4', 'nan'))
    with pytest.raises(ValueError):
        command_limits('<robot/>')
