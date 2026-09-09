import threading
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.clock import ClockType
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import TransformException
import yaml

from xlerobot_calibration_tools.collector_node import TransformSampleCollector
from xlerobot_calibration_tools.sample_set import transform_to_matrix, TransformSampleSet
from xlerobot_calibration_tools.solver import ARM_MODEL, HEAD_MODEL
from xlerobot_interfaces.msg import CalibrationTargetObservation, CapabilityError
from xlerobot_interfaces.srv import CaptureCalibrationSample


def pose(yaw=0.0, x=0.0, y=0.0, z=0.0):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    value[:3, 3] = [x, y, z]
    return value


def sample_set():
    return TransformSampleSet(
        calibration_id='head_run_001',
        model=HEAD_MODEL,
        base_frame='base_link',
        moving_frame='head_tilt_link',
        camera_frame='camera_optical',
        target_frame='board',
    )


def test_rejects_duplicate_pose_and_writes_atomic_schema(tmp_path):
    samples = sample_set()
    samples.append(
        pose(), pose(x=0.5), 1.0,
        {'tag_count': 16, 'reprojection_rmse_px': 0.4},
    )
    with pytest.raises(ValueError, match='too close'):
        samples.append(pose(yaw=0.01), pose(x=0.5), 1.1)
    samples.append(pose(yaw=0.1), pose(x=0.5), 1.2)

    output = tmp_path / 'samples.yaml'
    samples.write_atomic(output)
    text = output.read_text(encoding='utf-8')
    assert 'schema: xlerobot_transform_samples/v1' in text
    assert 'stamp_sec: 1.2' in text
    assert 'reprojection_rmse_px: 0.4' in text
    assert not (tmp_path / '.samples.yaml.tmp').exists()


def test_rejects_a_b_a_against_all_historical_poses():
    samples = sample_set()
    samples.append(pose(x=0.0), pose(x=0.5), 1.0)
    samples.append(pose(x=0.01), pose(x=0.5), 1.1)
    with pytest.raises(ValueError, match=r'existing.*\(0\)'):
        samples.append(pose(x=0.001), pose(x=0.5), 1.2)


def test_append_atomic_write_failure_rolls_back_memory_and_temporary_file(
    tmp_path, monkeypatch
):
    samples = sample_set()
    samples.append_and_write_atomic(
        tmp_path / 'samples.yaml',
        pose(),
        pose(x=0.5),
        1.0,
        {'reprojection_rmse_px': 0.4},
    )
    before = samples.to_dict()

    def fail_replace(_source, _destination):
        raise OSError('injected atomic replace failure')

    monkeypatch.setattr('pathlib.Path.replace', fail_replace)
    with pytest.raises(OSError, match='injected atomic replace failure'):
        samples.append_and_write_atomic(
            tmp_path / 'samples.yaml',
            pose(x=0.01),
            pose(x=0.5),
            2.0,
            {'reprojection_rmse_px': 0.5},
        )

    assert samples.to_dict() == before
    assert len(samples.samples) == len(samples.stamps_sec) == len(
        samples.qualities
    ) == 1
    assert not (tmp_path / '.samples.yaml.tmp').exists()
    restored = TransformSampleSet.read(
        tmp_path / 'samples.yaml',
        expected_model=HEAD_MODEL,
        expected_calibration_id='head_run_001',
    )
    assert len(restored.samples) == 1


def test_restores_atomic_samples_and_reports_factual_pose_coverage(tmp_path):
    samples = TransformSampleSet(
        calibration_id='arm_run_001',
        model=ARM_MODEL,
        base_frame='base_link',
        moving_frame='right_arm_fixed_jaw_link',
        camera_frame='d455_color_optical_frame',
        target_frame='calibration_target',
    )
    samples.append(
        pose(yaw=0.0, x=-0.1, y=0.2, z=0.3),
        pose(x=0.5),
        1.0,
        {'reprojection_rmse_px': 0.4},
    )
    samples.append(
        pose(yaw=0.5, x=0.2, y=-0.2, z=0.5),
        pose(x=0.5),
        2.0,
        {'reprojection_rmse_px': 0.5},
    )
    output = tmp_path / 'samples.yaml'
    samples.write_atomic(output)

    restored = TransformSampleSet.read(
        output,
        expected_model=ARM_MODEL,
        expected_calibration_id='arm_run_001',
        expected_frames={
            'base': 'base_link',
            'moving': 'right_arm_fixed_jaw_link',
            'camera': 'd455_color_optical_frame',
            'target': 'calibration_target',
        },
    )
    assert len(restored.samples) == 2
    assert restored.stamps_sec == [1.0, 2.0]
    coverage = restored.coverage()
    assert coverage['sample_count'] == 2
    assert coverage['spans_m'] == pytest.approx({
        'x': 0.3, 'y': 0.4, 'z': 0.2,
    })
    assert coverage['max_pairwise_pose_angle_deg'] == pytest.approx(
        np.degrees(0.5)
    )
    restored.append(
        pose(yaw=0.9, x=0.4, y=0.1, z=0.4),
        pose(x=0.5),
        3.0,
        {'reprojection_rmse_px': 0.6},
    )
    restored.write_atomic(output)
    resumed = TransformSampleSet.read(output, expected_model=ARM_MODEL)
    assert [sample.index for sample in resumed.samples] == [0, 1, 2]
    assert resumed.stamps_sec == [1.0, 2.0, 3.0]


@pytest.mark.parametrize(
    'mutation',
    [
        lambda document: document.update(schema='wrong/v1'),
        lambda document: document['samples'][0]['moving_in_base'][0].__setitem__(
            3, float('nan')
        ),
        lambda document: document['samples'][0].update(index=4),
        lambda document: document['samples'][0].update(stamp_sec='1.0'),
        lambda document: document['samples'][0]['quality'].update(
            reprojection_rmse_px=float('inf')
        ),
    ],
)
def test_restore_fails_closed_on_bad_schema_or_nonfinite_data(
    tmp_path, mutation
):
    samples = sample_set()
    samples.append(
        pose(), pose(x=0.5), 1.0,
        {'reprojection_rmse_px': 0.4},
    )
    document = samples.to_dict()
    mutation(document)
    output = tmp_path / 'samples.yaml'
    output.write_text(
        yaml.safe_dump(document, sort_keys=False), encoding='utf-8'
    )
    with pytest.raises(ValueError):
        TransformSampleSet.read(output, expected_model=HEAD_MODEL)


def test_transform_message_conversion_rejects_zero_quaternion():
    class Value:
        pass

    transform = Value()
    transform.translation = Value()
    transform.translation.x = 0.1
    transform.translation.y = 0.2
    transform.translation.z = 0.3
    transform.rotation = Value()
    transform.rotation.x = 0.0
    transform.rotation.y = 0.0
    transform.rotation.z = 0.0
    transform.rotation.w = 0.0
    with pytest.raises(ValueError, match='quaternion'):
        transform_to_matrix(transform)

    transform.rotation.w = 1.0
    matrix = transform_to_matrix(transform)
    assert np.allclose(matrix[:3, 3], [0.1, 0.2, 0.3])


def test_collector_never_falls_back_to_latest_moving_transform(tmp_path):
    observation = CalibrationTargetObservation()
    observation.header.stamp.sec = 10
    observation.accepted = True

    class Buffer:
        def __init__(self):
            self.calls = []

        def lookup_transform(self, target, source, stamp):
            self.calls.append((target, source, stamp.nanoseconds))
            if source == 'moving':
                raise TransformException('historical moving TF unavailable')
            return SimpleNamespace(transform=object())

    buffer = Buffer()
    collector = SimpleNamespace(
        _lock=threading.Lock(),
        _observation_condition=threading.Condition(),
        _observation=observation,
        _max_target_age_sec=0.25,
        _buffer=buffer,
        _output=tmp_path / 'samples.yaml',
        _samples=TransformSampleSet('test', HEAD_MODEL, 'base', 'moving', 'camera', 'target'),
        get_clock=lambda: SimpleNamespace(
            now=lambda: Time(seconds=10.1, clock_type=ClockType.ROS_TIME)
        ),
    )
    collector._fresh_snapshot = lambda: TransformSampleCollector._fresh_snapshot(collector)
    response = TransformSampleCollector._capture(
        collector,
        CaptureCalibrationSample.Request(job_id='job-001'),
        CaptureCalibrationSample.Response(),
    )

    assert response.error.code == CapabilityError.INVALID_GOAL
    assert 'historical moving TF unavailable' in response.error.message
    # Exact-time TF may be retried while waiting for the next synchronized
    # observation, but a zero/latest timestamp must never be requested.
    assert len(buffer.calls) >= 2
    assert buffer.calls[::2] == [('camera', 'target', 10_000_000_000)] * (len(buffer.calls) // 2)
    assert buffer.calls[1::2] == [('base', 'moving', 10_000_000_000)] * (len(buffer.calls) // 2)
