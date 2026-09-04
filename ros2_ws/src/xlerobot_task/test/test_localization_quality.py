import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import pytest
import rclpy

from xlerobot_interfaces.msg import ScanMapConsistency
from xlerobot_task.fetch_deliver_task_node import (
    FetchDeliverTaskNode,
    should_request_reached_place_skip,
)
import xlerobot_task.fetch_deliver_task_node as task_node_module


@pytest.fixture
def task_node():
    rclpy.init()
    node = FetchDeliverTaskNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def amcl_pose(*, position_stddev=0.05, yaw_stddev=0.05):
    message = PoseWithCovarianceStamped()
    message.pose.covariance[0] = position_stddev ** 2
    message.pose.covariance[7] = position_stddev ** 2
    message.pose.covariance[35] = yaw_stddev ** 2
    return message


def scan_map_result(*, valid=True, consistent=True, score=0.8):
    message = ScanMapConsistency()
    message.valid = valid
    message.consistent = consistent
    message.score = score
    message.message = 'test result'
    return message


def test_good_covariance_and_current_scan_map_match_skip_relocalization(task_node):
    task_node._on_amcl_pose(amcl_pose())
    task_node._on_scan_map_consistency(scan_map_result())
    assert task_node._localization_ready()


@pytest.mark.parametrize(
    ('position_stddev', 'yaw_stddev'),
    [(0.21, 0.05), (0.05, 0.21)],
)
def test_excessive_covariance_still_requires_relocalization(
    task_node, position_stddev, yaw_stddev
):
    task_node._on_amcl_pose(amcl_pose(
        position_stddev=position_stddev,
        yaw_stddev=yaw_stddev,
    ))
    task_node._on_scan_map_consistency(scan_map_result())
    assert not task_node._localization_ready()


def test_scan_map_mismatch_requires_relocalization_even_with_good_covariance(task_node):
    task_node._on_amcl_pose(amcl_pose())
    task_node._on_scan_map_consistency(scan_map_result(consistent=False, score=0.1))
    assert not task_node._localization_ready()


def test_stale_scan_map_result_requires_relocalization(task_node, monkeypatch):
    task_node._on_amcl_pose(amcl_pose())
    task_node._on_scan_map_consistency(scan_map_result())
    future = time.monotonic() + task_node.scan_map_max_age_s + 0.1
    monkeypatch.setattr(task_node_module.time, 'monotonic', lambda: future)
    assert not task_node._localization_ready()


def test_only_trusted_real_table_task_requests_already_reached_skip():
    assert should_request_reached_place_skip(
        localization_was_ready=True, source_place='table', dry_run=False
    )
    assert not should_request_reached_place_skip(
        localization_was_ready=False, source_place='table', dry_run=False
    )
    assert not should_request_reached_place_skip(
        localization_was_ready=True, source_place='table', dry_run=True
    )
    assert not should_request_reached_place_skip(
        localization_was_ready=True, source_place='charging_station', dry_run=False
    )
