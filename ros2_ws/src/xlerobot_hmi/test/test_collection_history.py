import json

import pytest

from xlerobot_hmi.collection_history import recent_episodes


def test_history_survives_restart_and_never_auto_accepts_old_data(tmp_path):
    dataset = tmp_path / 'datasets/trial'
    for i in range(23):
        episode_id = f'episode-{i:03d}'
        episode = dataset / 'raw' / episode_id
        episode.mkdir(parents=True)
        (episode / 'manifest.json').write_text(json.dumps({
            'schema': 'xlerobot_raw_episode/v1', 'episode_id': episode_id,
            'status': 'complete', 'frame_count': 30, 'duration_s': 1.,
            'created_at': f'{i:03d}',
        }))
    reviews = dataset / 'reviews'
    reviews.mkdir()
    path = reviews / 'episode-022.json'
    path.write_text(json.dumps({'episode_id': 'episode-022', 'status': 'rejected'}))
    first = recent_episodes(tmp_path, 'trial')
    assert len(first) == 20
    assert first[0]['episode_id'] == 'episode-022'
    assert first[0]['review_status'] == 'rejected'
    assert first[1]['review_status'] is None
    assert recent_episodes(tmp_path, 'trial') == first
    assert list(reviews.iterdir()) == [path]


def test_history_rejects_path_traversal_and_symlinks(tmp_path):
    with pytest.raises(ValueError):
        recent_episodes(tmp_path, '../other')
    (tmp_path / 'datasets').symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError):
        recent_episodes(tmp_path, 'trial')
