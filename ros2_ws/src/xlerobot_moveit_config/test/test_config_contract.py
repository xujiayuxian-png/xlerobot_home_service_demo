import ast
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / 'config'
DESCRIPTION = PACKAGE.parent / 'xlerobot_description' / 'urdf' / 'two_wheel_reference.urdf.xacro'


def test_scene_monitor_does_not_share_topics_between_message_types():
    tree = ast.parse((PACKAGE / 'launch' / 'move_group.launch.py').read_text())
    options = next(
        ast.literal_eval(node.value)
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == 'move_group_options'
                for target in node.targets)
    )['planning_scene_monitor_options']
    assert options['attached_collision_object_topic'] not in {
        options['publish_planning_scene_topic'],
        options['monitored_planning_scene_topic'],
        options['joint_state_topic'],
    }


def test_humble_pipeline_uses_available_adapter_plugins():
    import os
    import pytest
    from ament_index_python.packages import get_package_share_directory

    if os.environ.get('ROS_DISTRO') != 'humble':
        pytest.skip('Humble plugin inventory check')
    ompl = yaml.safe_load((CONFIG / 'ompl_planning_humble.yaml').read_text())['ompl']
    plugins = ET.parse(Path(get_package_share_directory('moveit_ros_planning')) /
                       'planning_request_adapters_plugin_description.xml').getroot()
    available = {node.attrib['name'] for node in plugins.findall('.//class')}
    assert isinstance(ompl['request_adapters'], str)
    assert set(ompl['request_adapters'].split()) <= available
    assert ompl['planning_plugin'] == 'ompl_interface/OMPLPlanner'


def urdf_root():
    result = subprocess.run(
        ['xacro', str(DESCRIPTION)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def test_srdf_name_links_and_joints_exist_in_reference_urdf():
    urdf = urdf_root()
    srdf = ET.parse(CONFIG / 'two_wheel_reference.srdf').getroot()
    assert srdf.attrib['name'] == urdf.attrib['name']
    links = {element.attrib['name'] for element in urdf.findall('link')}
    joints = {element.attrib['name'] for element in urdf.findall('joint')}
    for chain in srdf.findall('./group/chain'):
        assert chain.attrib['base_link'] in links
        assert chain.attrib['tip_link'] in links
    for joint in srdf.findall('./group/joint') + srdf.findall('./group_state/joint'):
        assert joint.attrib['name'] in joints
    for collision in srdf.findall('disable_collisions'):
        assert collision.attrib['link1'] in links
        assert collision.attrib['link2'] in links


def test_planning_groups_and_controllers_match_canonical_joint_owners():
    kinematics = yaml.safe_load((CONFIG / 'kinematics.yaml').read_text())
    assert set(kinematics) == {'right_arm', 'left_arm', 'head'}
    assert kinematics['right_arm']['position_only_ik']
    assert kinematics['left_arm']['position_only_ik']
    controllers = yaml.safe_load((CONFIG / 'moveit_controllers.yaml').read_text())
    manager = controllers['moveit_simple_controller_manager']
    assert manager['right_arm_controller']['joints'] == [
        'right_arm_shoulder_pan',
        'right_arm_shoulder_lift',
        'right_arm_elbow_flex',
        'right_arm_wrist_flex',
        'right_arm_wrist_roll',
    ]
    assert manager['right_gripper_controller']['joints'] == ['right_arm_gripper']
    assert manager['head_controller']['joints'] == ['head_pan_joint', 'head_tilt_joint']


def test_launch_defaults_cannot_execute_or_open_hardware():
    source = (PACKAGE / 'launch' / 'move_group.launch.py').read_text()
    assert "'allow_trajectory_execution': False" in source
    assert "LaunchConfiguration('allow_trajectory_execution')" not in source
    assert "' mock_hardware:=true'" in source
    assert "' hardware_enabled:=false'" in source
    assert "' torque_enabled:=false'" in source
