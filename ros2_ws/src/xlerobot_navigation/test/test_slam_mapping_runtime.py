import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from lifecycle_msgs.srv import GetState
import rclpy


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '71'
    package_share = Path(get_package_share_directory('xlerobot_navigation'))
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / 'slam_mapping.launch.py')
        )
    )
    return launch.LaunchDescription([mapping, launch_testing.actions.ReadyToTest()])


class TestSlamMappingRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_slam_mapping_runtime')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_slam_toolbox_reaches_active_lifecycle_state(self):
        client = self.node.create_client(GetState, '/slam_toolbox/get_state')
        self.assertTrue(client.wait_for_service(timeout_sec=15.0))
        # The service appears before the lifecycle configure transition finishes.
        time.sleep(1.0)
        state = ''
        deadline = time.monotonic() + 15.0
        while state != 'active' and time.monotonic() < deadline:
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            if future.done():
                state = future.result().current_state.label
            time.sleep(0.1)
        self.assertEqual(state, 'active')
        self.node.destroy_client(client)
