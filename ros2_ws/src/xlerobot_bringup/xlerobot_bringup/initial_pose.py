"""Read-only admission check for motion-controller startup."""
import math
import xml.etree.ElementTree as ET


def command_limits(description):
    root = ET.fromstring(description)
    limits = {}
    for joint in root.findall('./ros2_control/joint'):
        interface = joint.find("command_interface[@name='position']")
        if interface is None:
            continue
        values = {p.attrib['name']: float(p.text) for p in interface.findall('param')}
        low, high = values['min'], values['max']
        if not math.isfinite(low) or not math.isfinite(high) or low > high:
            raise ValueError('invalid position limits')
        limits[joint.attrib['name']] = (low, high)
    if not limits:
        raise ValueError('no position command limits found')
    return limits


def outside_limits(limits, positions):
    return [name for name, (low, high) in limits.items()
            if name not in positions or not math.isfinite(positions[name])
            or not low <= positions[name] <= high]
