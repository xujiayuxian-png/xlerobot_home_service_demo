import math
import threading
import time
from types import SimpleNamespace

import pytest

from xlerobot_commissioning.collection_readiness import position_limits, pose_issues
from xlerobot_commissioning.collection_node import CollectionNode
from rclpy.serialization import serialize_message, deserialize_message
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from builtin_interfaces.msg import Time


def test_pose_explanation_includes_joint_measured_and_allowed_range():
    limits = {'leader_gripper': (0., 1.65)}
    assert pose_issues(limits, {'leader_gripper': .1}) == []
    issue = pose_issues(limits, {'leader_gripper': -.01})[0]
    assert 'leader_gripper' in issue and '-0.010' in issue and '[0.000, 1.650]' in issue
    assert '无有效反馈' in pose_issues(limits, {})[0]
    assert '无有效反馈' in pose_issues(limits, {'leader_gripper': math.nan})[0]


def test_command_limits_ignore_velocity_wheels():
    xml = '''<robot><ros2_control><joint name="wheel"><command_interface name="velocity"/></joint>
      <joint name="arm"><command_interface name="position"><param name="min">-1</param>
      <param name="max">2</param></command_interface></joint></ros2_control></robot>'''
    assert position_limits(xml) == {'arm': (-1., 2.)}
    with pytest.raises(ValueError):
        position_limits(xml.replace('>2<', '>nan<'))


@pytest.mark.parametrize('position,stale,level', [(0., False, DiagnosticStatus.OK),
                                               (2., False, DiagnosticStatus.ERROR),
                                               (0., True, DiagnosticStatus.WARN)])
def test_pose_diagnostic_serializes_as_real_ros_message(position, stale, level):
    published = []
    node = SimpleNamespace(
        _state_lock=threading.Lock(), _joint_state_timeout_s=.5,
        _leader_limits={'leader': (-1., 1.)}, _follower_limits={'follower': (-1., 1.)},
        _states={'leader': {'leader': position}, 'follower': {'follower': position}},
        _state_received_at={key: 0. if stale else time.monotonic() for key in ('leader', 'follower')},
        _pose_diagnostics=SimpleNamespace(publish=published.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
    )
    CollectionNode._publish_pose_status(node)
    restored = deserialize_message(serialize_message(published[0]), DiagnosticArray)
    assert restored.status[0].level == level
    assert restored.status[0].name == 'xlerobot/collection_initial_pose'
