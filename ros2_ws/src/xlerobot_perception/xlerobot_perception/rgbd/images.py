"""Convert supported ROS Image messages without hiding layout assumptions."""

from __future__ import annotations

import cv2
import numpy as np


_ENCODINGS = {
    'rgb8': (np.uint8, 3),
    'bgr8': (np.uint8, 3),
    'rgba8': (np.uint8, 4),
    'bgra8': (np.uint8, 4),
    'mono8': (np.uint8, 1),
    '16UC1': (np.uint16, 1),
    '32FC1': (np.float32, 1),
}


def image_to_array(msg):
    """Return a contiguous array while respecting ROS row stride and endianness."""
    if msg.encoding not in _ENCODINGS:
        raise ValueError(f'unsupported image encoding: {msg.encoding}')
    dtype, channels = _ENCODINGS[msg.encoding]
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    packed_row_bytes = int(msg.width) * channels * dtype.itemsize
    step = int(msg.step) or packed_row_bytes
    if step < packed_row_bytes:
        raise ValueError(f'image step {step} is smaller than packed row {packed_row_bytes}')
    required = step * int(msg.height)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < required:
        raise ValueError(f'image data too short: got {raw.size}, expected {required}')
    packed = raw[:required].reshape((int(msg.height), step))[:, :packed_row_bytes]
    values = np.ascontiguousarray(packed).view(dtype)
    if channels == 1:
        return values.reshape((int(msg.height), int(msg.width)))
    return values.reshape((int(msg.height), int(msg.width), channels))


def color_to_bgr(msg) -> np.ndarray:
    """Convert one supported color encoding to BGR8."""
    image = image_to_array(msg)
    if msg.encoding == 'bgr8':
        return image.copy()
    conversions = {
        'rgb8': cv2.COLOR_RGB2BGR,
        'rgba8': cv2.COLOR_RGBA2BGR,
        'bgra8': cv2.COLOR_BGRA2BGR,
        'mono8': cv2.COLOR_GRAY2BGR,
    }
    if msg.encoding not in conversions:
        raise ValueError(f'unsupported color encoding: {msg.encoding}')
    return cv2.cvtColor(image, conversions[msg.encoding])
