import json

import numpy as np
import pytest

from xlerobot_dataset_tools.default_selection import keep_new_episode
from xlerobot_dataset_tools.episode_io import EpisodeWriter
from xlerobot_dataset_tools.convert_accepted import selection_plan


def complete_episode(tmp_path):
    raw = tmp_path / 'dataset' / 'raw'
    writer = EpisodeWriter(raw, 'episode-001', instruction='pick', object_label='object',
                           profile_id='two_wheel_pick', revision='test', unit_id='robot-1',
                           calibration_version='cal-1', fps=30)
    writer.begin()
    writer.mark_teleop_enabled()
    for i in range(4):
        images = {name: np.full((480, 640, 3), i, dtype=np.uint8) for name in ('head', 'wrist')}
        writer.append(np.full(6, i * .01), images, i / 30)
    writer.mark_recording_stopped()
    return writer.finish('user')


def test_default_keep_is_persistent_and_selected_for_conversion(tmp_path):
    episode = complete_episode(tmp_path)
    before = {p: p.read_bytes() for p in episode.rglob('*') if p.is_file()}
    selection = keep_new_episode(episode)
    document = json.loads(selection.read_text())
    assert document['status'] == 'accepted'
    assert document['selection_source'] == 'automatic_on_save'
    assert document['operator'] == 'recorder'
    assert selection_plan(episode.parent.parent)[1]['episodes'] == [episode.name]
    assert all(p.read_bytes() == content for p, content in before.items())
    original = selection.read_bytes()
    keep_new_episode(episode)
    assert selection.read_bytes() == original


def test_automatic_keep_never_overwrites_rejection(tmp_path):
    episode = complete_episode(tmp_path)
    selection = keep_new_episode(episode)
    document = json.loads(selection.read_text())
    document.update(status='rejected', operator='operator')
    selection.write_text(json.dumps(document))
    before = selection.read_bytes()
    keep_new_episode(episode)
    assert selection.read_bytes() == before
    assert selection_plan(episode.parent.parent)[1]['episode_count'] == 0


@pytest.mark.parametrize('damage', ['checksum', 'incomplete'])
def test_invalid_episode_is_never_automatically_kept(tmp_path, damage):
    episode = complete_episode(tmp_path)
    if damage == 'checksum':
        (episode / 'data.npz').write_bytes(b'corrupt')
    else:
        path = episode / 'manifest.json'
        document = json.loads(path.read_text())
        document['status'] = 'failed'
        path.write_text(json.dumps(document))
    with pytest.raises(ValueError):
        keep_new_episode(episode)
    assert not (episode.parent.parent / 'reviews').exists()
