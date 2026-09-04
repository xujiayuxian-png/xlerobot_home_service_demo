from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


def render(**arguments):
    model = (
        Path(__file__).resolve().parents[1]
        / "urdf"
        / "two_wheel_reference.urdf.xacro"
    )
    command = ["xacro", str(model)]
    command.extend(f"{key}:={value}" for key, value in arguments.items())
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout


def test_base_xacro_defaults_to_mock_and_locked_hardware_keys():
    urdf = render()
    assert "<plugin>xlerobot_hardware/RightBusSystem</plugin>" in urdf
    assert "<plugin>xlerobot_hardware/LeftBusSystem</plugin>" in urdf
    assert '<param name="mock_hardware">true</param>' in urdf
    assert '<param name="hardware_enabled">false</param>' in urdf
    assert '<param name="torque_enabled">false</param>' in urdf
    assert '<param name="read_only">false</param>' in urdf
    # The exact tag without a type attribute is the ros2_control declaration.
    assert urdf.count('<joint name="left_wheel_joint">') == 1
    assert urdf.count('<joint name="right_wheel_joint">') == 1


def test_hardware_keys_are_independently_rendered():
    urdf = render(
        mock_hardware="false",
        hardware_enabled="true",
        torque_enabled="false",
        read_only="true",
    )
    assert '<param name="mock_hardware">false</param>' in urdf
    assert '<param name="hardware_enabled">true</param>' in urdf
    assert '<param name="torque_enabled">false</param>' in urdf
    assert '<param name="read_only">true</param>' in urdf


def test_runtime_geometry_and_servo_files_override_reference_defaults(tmp_path):
    config = Path(__file__).resolve().parents[1] / 'config'
    geometry = yaml.safe_load(
        (config / 'two_wheel_reference_geometry.yaml').read_text()
    )
    servos = yaml.safe_load(
        (config / 'two_wheel_reference_servos.yaml').read_text()
    )
    geometry['base']['wheel_radius'] = 0.071
    servos['right_arm']['joints']['shoulder_pan']['offset'] = 2222
    geometry_path = tmp_path / 'geometry.yaml'
    servo_path = tmp_path / 'servos.yaml'
    geometry_path.write_text(yaml.safe_dump(geometry), encoding='utf-8')
    servo_path.write_text(yaml.safe_dump(servos), encoding='utf-8')
    root = ET.fromstring(render(
        geometry_file=geometry_path, servo_calibration_file=servo_path,
    ))
    shoulder = root.find(
        "ros2_control[@name='right_bus_system']/joint[@name='right_arm_shoulder_pan']"
    )
    assert shoulder.find("param[@name='offset']").text == '2222'
    wheel = root.find("link[@name='left_wheel']/visual/geometry/cylinder")
    assert float(wheel.attrib['radius']) == 0.071


def test_reference_model_has_one_root_and_verified_sensor_chains():
    root = ET.fromstring(render())
    links = {element.attrib["name"] for element in root.findall("link")}
    children = {
        element.find("child").attrib["link"] for element in root.findall("joint")
    }
    assert links - children == {"base_link"}
    assert "base_footprint" not in links

    parents = {
        element.find("child").attrib["link"]: element.find("parent").attrib["link"]
        for element in root.findall("joint")
    }
    assert parents["laser"] == "base_link"
    assert parents["top_base_link"] == "base_link"
    assert parents["head_pan_link"] == "top_base_link"
    assert parents["head_tilt_link"] == "head_pan_link"
    assert parents["head_camera_link"] == "head_tilt_link"
    assert parents["d455_head_camera_link"] == "head_camera_link"
    assert "imu_link" not in links
    assert parents["right_arm_base_link"] == "base_link"
    assert parents["left_arm_base_link"] == "base_link"


def test_reference_model_has_exactly_one_owner_per_physical_bus():
    root = ET.fromstring(render())
    control_systems = root.findall("ros2_control")
    assert [system.attrib["name"] for system in control_systems] == [
        "right_bus_system",
        "left_bus_system",
    ]


def test_observation_profile_can_exclude_the_unneeded_left_bus_owner():
    root = ET.fromstring(render(include_left_bus_control="false"))
    assert [
        system.attrib["name"] for system in root.findall("ros2_control")
    ] == ["right_bus_system"]


def test_bus_owners_match_verified_wiring():
    root = ET.fromstring(render())
    control_systems = root.findall("ros2_control")
    right_joints = {
        joint.attrib["name"]
        for joint in control_systems[0].findall("joint")
    }
    left_joints = {
        joint.attrib["name"]
        for joint in control_systems[1].findall("joint")
    }
    assert right_joints == {
        "left_wheel_joint",
        "right_wheel_joint",
        "right_arm_shoulder_pan",
        "right_arm_shoulder_lift",
        "right_arm_elbow_flex",
        "right_arm_wrist_flex",
        "right_arm_wrist_roll",
        "right_arm_gripper",
    }
    assert left_joints == {
        "head_pan_joint",
        "head_tilt_joint",
        "left_arm_shoulder_pan",
        "left_arm_shoulder_lift",
        "left_arm_elbow_flex",
        "left_arm_wrist_flex",
        "left_arm_wrist_roll",
        "left_arm_gripper",
    }


def test_verified_act_closed_value_is_valid_at_every_runtime_boundary():
    src = Path(__file__).resolve().parents[2]
    geometry = yaml.safe_load(
        (src / "xlerobot_description/config/two_wheel_reference_geometry.yaml")
        .read_text()
    )
    servos = yaml.safe_load(
        (src / "xlerobot_description/config/two_wheel_reference_servos.yaml")
        .read_text()
    )
    policy = yaml.safe_load(
        (src / "xlerobot_policy/config/act_box_wrist_only.yaml").read_text()
    )
    grasp = yaml.safe_load(
        (src / "xlerobot_manipulation/config/grasp_object.yaml").read_text()
    )["grasp_object_server"]["ros__parameters"]
    executor = yaml.safe_load(
        (src / "xlerobot_manipulation/config/streaming_executor.yaml").read_text()
    )["streaming_joint_executor"]["ros__parameters"]
    closed = policy["gripper_governor"]["closed_value"]

    assert closed == 0.0
    assert geometry["right_arm"]["limits"]["gripper"][0] == closed
    assert servos["right_arm"]["joints"]["gripper"]["limit_min"] == closed
    assert executor["lower_positions"][-1] == closed
    assert executor["position_setpoint_joints"] == ["right_arm_gripper"]
    assert grasp["gripper_joint"] == "right_arm_gripper"
    assert grasp["pregrasp_gripper_position"] == 1.64
    assert grasp["pregrasp_gripper_position"] <= servos["right_arm"]["joints"][
        "gripper"
    ]["limit_max"]

    root = ET.fromstring(render())
    right_system = root.find("ros2_control[@name='right_bus_system']")
    gripper = right_system.find("joint[@name='right_arm_gripper']")
    command = gripper.find("command_interface[@name='position']")
    state = gripper.find("state_interface[@name='position']")
    assert float(command.find("param[@name='min']").text) == closed
    assert float(state.find("param[@name='initial_value']").text) == closed
