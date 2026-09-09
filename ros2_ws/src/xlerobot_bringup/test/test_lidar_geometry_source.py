"""Only expand xacro; no ROS graph, serial devices or motion."""
from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET
import yaml


def test_lidar_yaw_comes_from_selected_geometry_unless_explicitly_overridden(tmp_path):
    source = Path(__file__).resolve().parents[2] / 'xlerobot_description'
    geometry = yaml.safe_load((source / 'config/two_wheel_reference_geometry.yaml').read_text())
    geometry['sensors']['lidar_yaw_deg'] = 180
    path = tmp_path / 'geometry.yaml'
    path.write_text(yaml.safe_dump(geometry))
    for extra, expected in [([], math.pi), (['lidar_yaw_deg:=90'], math.pi / 2)]:
        xml = subprocess.check_output(['xacro', str(source / 'urdf/two_wheel_reference.urdf.xacro'),
                                       f'geometry_file:={path}', *extra])
        root = ET.fromstring(xml)
        joint = next(j for j in root.findall('joint') if j.get('name') == 'laser_joint')
        assert math.isclose(float(joint.find('origin').get('rpy').split()[2]), expected)
