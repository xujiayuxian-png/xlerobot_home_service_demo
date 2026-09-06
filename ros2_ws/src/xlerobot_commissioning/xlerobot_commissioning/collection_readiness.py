"""Read-only startup-pose explanation; never changes motion admission."""

import math
import xml.etree.ElementTree as ET


def position_limits(description):
    limits = {}
    for joint in ET.fromstring(description).findall('./ros2_control/joint'):
        interface = joint.find("command_interface[@name='position']")
        if interface is None:
            continue
        values = {p.attrib['name']: float(p.text) for p in interface.findall('param')}
        low, high = values['min'], values['max']
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            raise ValueError('invalid command limits')
        limits[joint.attrib['name']] = (low, high)
    return limits


def pose_issues(limits, state):
    issues = []
    for name, (low, high) in limits.items():
        value = state.get(name)
        if value is None or not math.isfinite(value):
            issues.append(f'{name}：无有效反馈')
        elif not low <= value <= high:
            issues.append(f'{name}：当前 {value:.3f} rad，允许 [{low:.3f}, {high:.3f}] rad')
    return issues
