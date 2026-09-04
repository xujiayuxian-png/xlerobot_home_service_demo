"""Typed, read-only observations for operator visualization."""

from xlerobot_interfaces.msg import PerceptionObservation


def make_observation(
    *, kind, label, camera_id, image, bbox, confidence, target
) -> PerceptionObservation:
    """Describe the exact image-space detection used for a 3-D target."""
    observation = PerceptionObservation()
    observation.stamp = image.header.stamp
    observation.observation_id = (
        f'{kind}-{image.header.stamp.sec}-{image.header.stamp.nanosec}'
    )
    observation.kind = str(kind)
    observation.label = str(label)
    observation.camera_id = str(camera_id)
    observation.image_width = int(image.width)
    observation.image_height = int(image.height)
    observation.bbox_x1 = float(bbox[0])
    observation.bbox_y1 = float(bbox[1])
    observation.bbox_x2 = float(bbox[2])
    observation.bbox_y2 = float(bbox[3])
    observation.confidence = float(confidence)
    observation.target = target
    return observation
