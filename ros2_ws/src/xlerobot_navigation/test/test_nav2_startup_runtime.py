"""Configure the real Nav2 plugins with synthetic sensors and no motor nodes."""
import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '174'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    share = Path(get_package_share_directory('xlerobot_navigation'))
    navigation = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(share / 'launch' / 'nav2_navigation.launch.py')))
    return launch.LaunchDescription([navigation, launch_testing.actions.ReadyToTest()])


class TestNav2Startup(unittest.TestCase):
    def test_plugins_and_behavior_tree_activate_without_hardware(self):
        rclpy.init()
        node = rclpy.create_node('nav2_synthetic_sensors')
        try:
            tf = StaticTransformBroadcaster(node)
            transforms = []
            for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
                transform = TransformStamped()
                transform.header.stamp = node.get_clock().now().to_msg()
                transform.header.frame_id = parent
                transform.child_frame_id = child
                transform.transform.rotation.w = 1.0
                transforms.append(transform)
            tf.sendTransform(transforms)
            map_publisher = node.create_publisher(
                OccupancyGrid, '/map', QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.resolution = 0.05
            grid.info.width = grid.info.height = 200
            grid.info.origin.position.x = grid.info.origin.position.y = -5.0
            grid.info.origin.orientation.w = 1.0
            grid.data = [0] * 40000
            map_publisher.publish(grid)
            scan_publisher = node.create_publisher(LaserScan, '/scan', 10)

            def publish_scan():
                scan = LaserScan()
                scan.header.stamp = node.get_clock().now().to_msg()
                scan.header.frame_id = 'base_link'
                scan.angle_min, scan.angle_max = -3.14, 3.14
                scan.angle_increment = 6.28 / 359
                scan.range_min, scan.range_max = 0.05, 8.0
                scan.ranges = [2.0] * 360
                scan_publisher.publish(scan)

            node.create_timer(0.1, publish_scan)

            def spin_until(predicate, seconds=20.0):
                deadline = time.monotonic() + seconds
                while not predicate() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                self.assertTrue(predicate())

            for name in ['controller_server', 'smoother_server', 'behavior_server',
                         'velocity_smoother', 'collision_monitor']:
                client = node.create_client(GetState, f'/{name}/get_state')
                spin_until(client.service_is_ready)
                deadline = time.monotonic() + 20.0
                state = ''
                while state != 'active' and time.monotonic() < deadline:
                    future = client.call_async(GetState.Request())
                    spin_until(future.done, 2.0)
                    state = future.result().current_state.label
                self.assertEqual(state, 'active', name)
                node.destroy_client(client)

            manager = node.create_client(
                ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
            spin_until(manager.service_is_ready)
            future = manager.call_async(ManageLifecycleNodes.Request(
                command=ManageLifecycleNodes.Request.STARTUP))
            spin_until(future.done)
            self.assertTrue(future.result().success)
        finally:
            node.destroy_node()
            rclpy.shutdown()
