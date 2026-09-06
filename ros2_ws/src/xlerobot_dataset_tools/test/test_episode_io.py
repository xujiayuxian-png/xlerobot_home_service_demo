import json

import cv2
import numpy as np
import pytest

from xlerobot_dataset_tools.episode_io import EpisodeWriter, validate_episode


def frame(value: int) -> np.ndarray:
    return np.full((480, 640, 3), value, dtype=np.uint8)


@pytest.mark.parametrize('stop_event', ['teleop_disabled', 'recording_stopped'])
def test_writer_preserves_follower_next_state_and_atomically_publishes(tmp_path, stop_event):
    writer = EpisodeWriter(
        tmp_path, 'episode-001', instruction='抓住羽毛球', object_label='羽毛球',
        profile_id='two_wheel_pick', revision='test', unit_id='robot-1',
        calibration_version='cal-1', fps=30,
    )
    writer.begin()
    writer.mark_teleop_enabled()
    writer.append(np.arange(6), {'head': frame(10), 'wrist': frame(20)}, 0.0)
    writer.append(np.arange(6) + 1, {'head': frame(11), 'wrist': frame(21)}, 1 / 30)
    writer.append(np.arange(6) + 2, {'head': frame(12), 'wrist': frame(22)}, 2 / 30)
    getattr(writer, 'mark_' + stop_event)()
    output = writer.finish('user')
    assert output == tmp_path / 'episode-001'
    assert not list(tmp_path.glob('.incomplete-*'))
    with np.load(output / 'data.npz') as data:
        np.testing.assert_array_equal(data['observation_state'][0], np.arange(6))
        np.testing.assert_array_equal(data['action'][0], np.arange(6) + 1)
        np.testing.assert_array_equal(data['observation_state'][1], np.arange(6) + 1)
        np.testing.assert_array_equal(data['action'][1], np.arange(6) + 2)
    manifest = validate_episode(output)
    assert manifest['frame_count'] == 2
    assert manifest['action_semantics'] == 'follower_next_state'
    assert manifest['teleop_enabled_at']
    assert manifest[stop_event + '_at']
    if stop_event == 'recording_stopped':
        assert not manifest['teleop_disabled_at']
    for name in ('head', 'wrist'):
        capture = cv2.VideoCapture(str(output / 'videos' / f'{name}.mp4'))
        assert round(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == 2
        assert round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) == 640
        assert round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)) == 480
        capture.release()


def test_cancel_keeps_incomplete_episode_out_of_complete_namespace(tmp_path):
    writer = EpisodeWriter(
        tmp_path, 'episode-002', instruction='test', object_label='object',
        profile_id='two_wheel_pick', revision='test', unit_id='robot-1',
        calibration_version='', fps=30,
    )
    writer.begin()
    incomplete = writer.abort('canceled')
    assert incomplete.name == '.incomplete-episode-002'
    assert not (tmp_path / 'episode-002').exists()
    manifest = json.loads((incomplete / 'manifest.json').read_text())
    assert manifest['status'] == 'failed'


def test_same_episode_name_is_never_overwritten(tmp_path):
    (tmp_path / 'episode-003').mkdir()
    writer = EpisodeWriter(
        tmp_path, 'episode-003', instruction='test', object_label='object',
        profile_id='two_wheel_pick', revision='test', unit_id='robot-1',
        calibration_version='', fps=30,
    )
    try:
        writer.begin()
    except FileExistsError:
        pass
    else:
        raise AssertionError('existing episode should be rejected')


def test_incomplete_episode_id_must_be_reviewed_before_retry(tmp_path):
    (tmp_path / '.incomplete-episode-004-deadbeef').mkdir()
    writer = EpisodeWriter(
        tmp_path, 'episode-004', instruction='test', object_label='object',
        profile_id='two_wheel_pick', revision='test', unit_id='robot-1',
        calibration_version='', fps=30,
    )

    try:
        writer.begin()
    except FileExistsError:
        pass
    else:
        raise AssertionError('incomplete episode ID should be rejected')


def test_episode_id_prefix_does_not_collide_with_another_incomplete(tmp_path):
    (tmp_path / '.incomplete-episode-010').mkdir()
    writer = EpisodeWriter(
        tmp_path, 'episode-01', instruction='test', object_label='object',
        profile_id='two_wheel_pick', revision='test', unit_id='robot-1',
        calibration_version='', fps=30,
    )

    writer.begin()

    assert writer.work == tmp_path / '.incomplete-episode-01'


def test_deterministic_incomplete_path_is_an_atomic_writer_claim(tmp_path):
    first = EpisodeWriter(
        tmp_path, 'episode-005', instruction='test', object_label='object',
        profile_id='two_wheel_pick', revision='test', unit_id='robot-1',
        calibration_version='', fps=30,
    )
    second = EpisodeWriter(
        tmp_path, 'episode-005', instruction='test', object_label='object',
        profile_id='two_wheel_pick', revision='test', unit_id='robot-1',
        calibration_version='', fps=30,
    )
    first.begin()

    try:
        second.begin()
    except FileExistsError:
        pass
    else:
        raise AssertionError('a second writer must not share an episode ID')

    first_manifest = (first.work / 'manifest.json').read_bytes()
    second.abort('must not mutate another writer')
    assert (first.work / 'manifest.json').read_bytes() == first_manifest
