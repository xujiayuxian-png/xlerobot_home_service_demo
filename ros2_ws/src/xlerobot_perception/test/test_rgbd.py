import math

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image

from xlerobot_perception.geometry.pinhole import project_pixel_to_point
from xlerobot_perception.observations import make_observation
from xlerobot_perception.rgbd.depth import depth_median
from xlerobot_perception.rgbd.images import image_to_array


def make_image(values, encoding, *, padding=0):
    array = np.asarray(values)
    message = Image()
    message.height, message.width = array.shape[:2]
    message.encoding = encoding
    if encoding == '16UC1':
        array = array.astype(np.uint16)
    elif encoding == '32FC1':
        array = array.astype(np.float32)
    else:
        raise ValueError(encoding)
    packed = array.view(np.uint8).reshape((message.height, -1))
    message.step = packed.shape[1] + padding
    rows = [row.tobytes() + bytes(padding) for row in packed]
    message.data = b''.join(rows)
    return message


def test_depth_median_16uc1_handles_stride_and_returns_metres():
    message = make_image(
        [[0, 1000, 1200], [0, 1100, 0], [900, 0, 0]],
        '16UC1',
        padding=4,
    )
    assert depth_median(message, 1, 1, [1]) == 1.05
    assert image_to_array(message).shape == (3, 3)


def test_depth_median_32fc1_ignores_invalid_values():
    message = make_image([[math.nan, 1.5], [0.0, 2.5]], '32FC1')
    assert depth_median(message, 0, 0, [1]) == 2.0


def test_perception_observation_preserves_bbox_image_and_target_identity():
    image = Image(height=12, width=20, encoding='bgr8')
    image.header.stamp.sec = 12
    image.header.stamp.nanosec = 34
    info = CameraInfo()
    info.k = [100.0, 0.0, 10.0, 0.0, 100.0, 6.0, 0.0, 0.0, 1.0]
    target = project_pixel_to_point(
        10, 6, 1.0, info, frame_id='camera',
        stamp=image.header.stamp,
    )
    observation = make_observation(
        kind='object', label='羽毛球', camera_id='head', image=image,
        bbox=(2, 3, 15, 10), confidence=0.9, target=target,
    )
    assert observation.observation_id == 'object-12-34'
    assert (observation.image_width, observation.image_height) == (20, 12)
    assert observation.bbox_x2 == 15
    assert observation.target.point.z == pytest.approx(1.0)


def test_image_rejects_short_rows():
    message = make_image([[1, 2]], '16UC1')
    message.step = 2
    with pytest.raises(ValueError, match='smaller than packed row'):
        image_to_array(message)


def test_pinhole_projection_uses_intrinsics_and_stamp():
    info = CameraInfo()
    info.header.frame_id = 'camera_optical_frame'
    info.header.stamp.sec = 42
    info.k = [100.0, 0.0, 10.0, 0.0, 100.0, 20.0, 0.0, 0.0, 1.0]
    point = project_pixel_to_point(20.0, 40.0, 2.0, info)
    assert point.header.frame_id == 'camera_optical_frame'
    assert point.header.stamp.sec == 42
    assert point.point.x == 0.2
    assert point.point.y == 0.4
    assert point.point.z == 2.0
