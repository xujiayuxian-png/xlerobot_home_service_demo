"""Real ros2_control lifecycle with the in-memory bus, never motor devices."""
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml


@pytest.mark.parametrize('passive_outside_limits', [False, True])
def test_leader_position_ownership_is_scoped_to_preparation(tmp_path, monkeypatch, passive_outside_limits):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(30 + (os.getpid() + time.monotonic_ns()) % 170))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    description = subprocess.check_output([
        'xacro', str(Path(get_package_share_directory('xlerobot_description')) /
                     'urdf/leader_attachment.urdf.xacro'),
        'mock_hardware:=true', 'hardware_enabled:=false',
        'port:=/nonexistent/collection-lifecycle-test'], text=True)
    if passive_outside_limits:
        root = ET.fromstring(description)
        # Inject a passive observation with ROS's stock in-memory hardware;
        # do not weaken the real bus's calibration/initial-value validation.
        root.find('./ros2_control/hardware/plugin').text = 'mock_components/GenericSystem'
        root.find("./ros2_control/joint[@name='leader_shoulder_lift']/state_interface[@name='position']/param").text = '1.8607'
        description = ET.tostring(root, encoding='unicode')
    config = yaml.safe_load((Path(get_package_share_directory('xlerobot_bringup')) /
                             'config/leader_controllers.yaml').read_text())
    config['/leader/controller_manager']['ros__parameters']['robot_description'] = description
    config_path = tmp_path / 'controllers.yaml'
    config_path.write_text(yaml.safe_dump(config))
    log = (tmp_path / 'controller.log').open('w')
    process = None
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node('collection_lifecycle_test')
        publisher = node.create_publisher(String, '/leader/robot_description',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher.publish(String(data=description))
        process = subprocess.Popen([
            '/opt/ros/jazzy/lib/controller_manager/ros2_control_node', '--ros-args',
            '-r', '__ns:=/leader', '--params-file', str(config_path)], stdout=log, stderr=log)

        def spin_until(predicate, timeout=5.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                assert process.poll() is None, (tmp_path / 'controller.log').read_text()
                rclpy.spin_once(node, timeout_sec=0.02)
            assert predicate(), (tmp_path / 'controller.log').read_text()

        def call(client, request):
            assert client.wait_for_service(timeout_sec=5)
            future = client.call_async(request)
            spin_until(future.done)
            return future.result()

        listing = node.create_client(ListControllers, '/leader/controller_manager/list_controllers')
        assert listing.wait_for_service(timeout_sec=8), (tmp_path / 'controller.log').read_text()
        spawner = '/opt/ros/jazzy/lib/controller_manager/spawner'
        for names, inactive in [(['leader_joint_state_broadcaster', 'leader_torque_controller'], False),
                                (['leader_arm_controller'], True)]:
            subprocess.run([spawner, *names, '-c', '/leader/controller_manager',
                            *(['--inactive'] if inactive else [])], timeout=15, check=True,
                           stdout=log, stderr=log)
        states = []
        node.create_subscription(JointState, '/leader/joint_states', states.append, 10)
        spin_until(lambda: len(states) >= 3)

        def controller_state():
            return {c.name: c.state for c in call(listing, ListControllers.Request()).controller}

        assert controller_state()['leader_arm_controller'] == 'inactive'
        if passive_outside_limits:
            # This exact pose killed the previously always-active controller.
            measured = dict(zip(states[-1].name, states[-1].position))
            assert measured['leader_shoulder_lift'] == pytest.approx(1.8607)
            before = len(states)
            spin_until(lambda: len(states) > before + 30)
            assert controller_state()['leader_arm_controller'] == 'inactive'
            return

        torque = node.create_client(SetBool, '/leader_bus/set_torque_enabled')
        switch = node.create_client(SwitchController, '/leader/controller_manager/switch_controller')
        action = ActionClient(node, FollowJointTrajectory,
                              '/leader/leader_arm_controller/follow_joint_trajectory')

        def activate(value):
            response = call(switch, SwitchController.Request(
                activate_controllers=['leader_arm_controller'] if value else [],
                deactivate_controllers=[] if value else ['leader_arm_controller'],
                strictness=SwitchController.Request.STRICT, timeout=Duration(sec=2)))
            assert response.ok, response.message

        for target in (0.2, -0.2):
            assert call(torque, SetBool.Request(data=False)).success
            activate(True)
            assert call(torque, SetBool.Request(data=True)).success
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(states[-1].name)
            positions = [target if name == 'leader_shoulder_pan' else
                         0.05 if name == 'leader_gripper' else 0.0 for name in goal.trajectory.joint_names]
            goal.trajectory.points = [JointTrajectoryPoint(
                positions=positions, time_from_start=Duration(nanosec=400_000_000))]
            assert action.wait_for_server(timeout_sec=3)
            future = action.send_goal_async(goal)
            spin_until(future.done)
            handle = future.result()
            assert handle.accepted
            result = handle.get_result_async()
            spin_until(result.done)
            assert result.result().result.error_code == 0
            assert call(torque, SetBool.Request(data=True)).success
            activate(False)
            assert call(torque, SetBool.Request(data=False)).success
            assert controller_state()['leader_arm_controller'] == 'inactive'
            spin_until(lambda: abs(dict(zip(states[-1].name, states[-1].position))['leader_shoulder_pan'] - target) < .03)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        log.close()
