from builtin_interfaces.msg import Time
import pytest
from sensor_msgs.msg import Image

from xlerobot_perception.detect_object_node import validate_goal_fields
from xlerobot_perception.execution_modes import validate_dry_run_mode
from xlerobot_perception.rgbd.frame_buffer import RgbdFrameBuffer
from xlerobot_perception.rgbd.frame_buffer import stamps_synchronized


def stamped_message(seconds, nanoseconds=0):
    message = Image()
    message.header.stamp = Time(sec=seconds, nanosec=nanoseconds)
    return message


def test_goal_accepts_unicode_object_and_relative_frame():
    validate_goal_fields('露营灯', 'map')
    validate_goal_fields('camping_lamp', 'base_link/camera')


@pytest.mark.parametrize(
    ('object_id', 'target_frame'),
    [('', 'map'), ('lamp\n', 'map'), ('lamp', '/map'), ('lamp', 'bad frame')],
)
def test_goal_rejects_ambiguous_identifiers(object_id, target_frame):
    with pytest.raises(ValueError):
        validate_goal_fields(object_id, target_frame)


def test_sync_requires_nonzero_close_timestamps():
    assert stamps_synchronized(
        [stamped_message(10), stamped_message(10, 50_000_000)], 0.1
    )
    assert not stamps_synchronized(
        [stamped_message(10), stamped_message(10, 200_000_000)], 0.1
    )
    assert not stamps_synchronized([stamped_message(0), stamped_message(0)], 0.1)


def test_dry_run_mode_is_explicit():
    validate_dry_run_mode('contract_only')
    validate_dry_run_mode('observe')
    with pytest.raises(ValueError, match='dry_run_mode'):
        validate_dry_run_mode('sometimes')


def test_rgbd_buffer_rejects_unknown_stream():
    buffer = RgbdFrameBuffer()
    with pytest.raises(ValueError, match='unknown RGBD stream'):
        buffer.store('infrared', stamped_message(1))


def test_rgbd_buffer_requires_every_stream_after_capture_barrier():
    buffer = RgbdFrameBuffer()
    for key in buffer.KEYS:
        buffer.store(key, stamped_message(10))

    with pytest.raises(TimeoutError):
        buffer.wait_snapshot(
            timeout_s=0.01,
            sync_tolerance_s=0.1,
            max_age_s=1.0,
            min_stamp_ns=10_500_000_000,
            now_ns=lambda: 10_600_000_000,
            check_interrupt=lambda: None,
        )

    for key in buffer.KEYS:
        buffer.store(key, stamped_message(11))
    snapshot = buffer.wait_snapshot(
        timeout_s=0.01,
        sync_tolerance_s=0.1,
        max_age_s=1.0,
        min_stamp_ns=10_500_000_000,
        now_ns=lambda: 11_100_000_000,
        check_interrupt=lambda: None,
    )
    assert all(message.header.stamp.sec == 11 for message in snapshot)
