import json
from pathlib import Path

import xlerobot_dataset_tools.convert_accepted as convert_accepted
from xlerobot_dataset_tools.convert_accepted import accepted_episode_paths
from xlerobot_dataset_tools.record_episode_node import load_profile


def test_repository_profile_uses_follower_next_state_at_30_hz():
    root = Path(__file__).parents[1] / 'config'
    profile = load_profile('two_wheel_pick', root)
    assert profile['fps'] == 30
    assert profile['observation']['topic'] == '/joint_states'
    assert profile['action']['kind'] == 'follower_next_state'
    assert set(profile['cameras']) == {'head', 'wrist'}
    assert all(camera['width'] == 640 for camera in profile['cameras'].values())
    assert all(camera['height'] == 480 for camera in profile['cameras'].values())


def test_batch_conversion_selects_only_accepted_reviews(tmp_path: Path):
    dataset = tmp_path / 'dataset'
    (dataset / 'reviews').mkdir(parents=True)
    for episode_id, status in [('good', 'accepted'), ('bad', 'rejected')]:
        (dataset / 'raw' / episode_id).mkdir(parents=True)
        (dataset / 'reviews' / f'{episode_id}.json').write_text(json.dumps({
            'schema': 'xlerobot_episode_review/v1',
            'episode_id': episode_id, 'status': status,
        }))
    assert accepted_episode_paths(dataset) == [dataset / 'raw/good']


def test_batch_conversion_atomically_publishes_lerobot_and_card(
    tmp_path: Path, monkeypatch
):
    dataset = tmp_path / 'dataset'
    (dataset / 'reviews').mkdir(parents=True)
    episode = dataset / 'raw/episode-001'
    episode.mkdir(parents=True)
    (dataset / 'reviews/episode-001.json').write_text(json.dumps({
        'schema': 'xlerobot_episode_review/v1',
        'episode_id': 'episode-001', 'status': 'accepted',
    }))

    monkeypatch.setattr(
        convert_accepted, 'load_episode',
        lambda path, **_kwargs: type('Episode', (), {'name': path.name})(),
    )

    def fake_convert(_episodes, *, output_root, repo_id):
        assert repo_id == 'local/test'
        output_root.mkdir(parents=True)
        return output_root

    monkeypatch.setattr(convert_accepted, 'convert', fake_convert)
    convert_accepted.main([
        '--dataset', str(dataset), '--version', 'v1', '--repo-id', 'local/test',
    ])
    output = dataset / 'derived/v1'
    manifest = json.loads((output / 'conversion_manifest.json').read_text())
    assert manifest['accepted_only'] is True
    assert manifest['outputs'] == {'lerobot_v3': 'lerobot'}
    assert 'Follower next-state' in (output / 'DATASET_CARD.md').read_text()
    assert not list((dataset / 'derived').glob('.v1.incomplete-*'))
