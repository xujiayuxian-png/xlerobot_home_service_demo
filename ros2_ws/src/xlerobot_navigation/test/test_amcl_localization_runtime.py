import os
from pathlib import Path
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


_MAP_DIRECTORY = tempfile.TemporaryDirectory(prefix='xlerobot-amcl-test-')


def _test_map() -> Path:
    directory = Path(_MAP_DIRECTORY.name)
    image = directory / 'map.pgm'
    image.write_text(
        '\n'.join([
            'P2',
            '# Tiny generated map used only for a hardware-free lifecycle test.',
            '10 10',
            '255',
            '0 0 0 0 0 0 0 0 0 0',
            *(['0 255 255 255 255 255 255 255 255 0'] * 8),
            '0 0 0 0 0 0 0 0 0 0',
            '',
        ]),
        encoding='ascii',
    )
    metadata = directory / 'map.yaml'
    metadata.write_text(
        '\n'.join([
            'image: map.pgm',
            'mode: trinary',
            'resolution: 0.1',
            'origin: [-0.5, -0.5, 0.0]',
            'negate: 0',
            'occupied_thresh: 0.65',
            'free_thresh: 0.25',
            '',
        ]),
        encoding='ascii',
    )
    return metadata


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '71'
    package_share = Path(get_package_share_directory('xlerobot_navigation'))
    map_file = _test_map()
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / 'amcl_localization.launch.py')
        ),
        launch_arguments={'map': str(map_file)}.items(),
    )
    return launch.LaunchDescription(
        [localization, launch_testing.actions.ReadyToTest()]
    )


class TestAmclLocalizationRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_amcl_localization_runtime')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def lifecycle_state(self, node_name):
        client = self.node.create_client(GetState, f'/{node_name}/get_state')
        self.assertTrue(client.wait_for_service(timeout_sec=15.0))
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        state = future.result().current_state.label
        self.node.destroy_client(client)
        return state

    def test_map_server_and_amcl_are_active_and_publish_map(self):
        deadline = time.monotonic() + 20.0
        states = ('', '')
        while time.monotonic() < deadline:
            states = (
                self.lifecycle_state('map_server'),
                self.lifecycle_state('amcl'),
            )
            if states == ('active', 'active'):
                break
            time.sleep(0.1)
        self.assertEqual(states, ('active', 'active'))

        maps = []
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        subscription = self.node.create_subscription(
            OccupancyGrid, 'map', maps.append, qos
        )
        deadline = time.monotonic() + 5.0
        while not maps and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(maps)
        self.assertEqual(maps[-1].header.frame_id, 'map')
        self.assertEqual((maps[-1].info.width, maps[-1].info.height), (10, 10))
        self.node.destroy_subscription(subscription)
