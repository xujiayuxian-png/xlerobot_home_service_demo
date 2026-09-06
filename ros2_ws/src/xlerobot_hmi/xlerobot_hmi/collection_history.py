"""Small on-disk collection inventory; raw episodes are never changed here."""

import json
from pathlib import Path
import re


IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$')


def recent_episodes(root: Path, dataset_id: str) -> list[dict]:
    if not IDENTIFIER.fullmatch(dataset_id):
        raise ValueError('invalid dataset_id')
    dataset = root / 'datasets' / dataset_id
    raw = dataset / 'raw'
    if any(p.is_symlink() for p in (root / 'datasets', dataset, raw)):
        raise ValueError('dataset must not be a symlink')
    if not raw.is_dir():
        return []
    episodes = []
    for episode in raw.iterdir():
        if not IDENTIFIER.fullmatch(episode.name) or episode.is_symlink() or not episode.is_dir():
            continue
        try:
            path = episode / 'manifest.json'
            if path.is_symlink():
                continue
            manifest = json.loads(path.read_text(encoding='utf-8'))
            if (manifest.get('schema') != 'xlerobot_raw_episode/v1'
                    or manifest.get('status') != 'complete'
                    or manifest.get('episode_id') != episode.name):
                continue
            review_path = dataset / 'reviews' / f'{episode.name}.json'
            review = {}
            if not review_path.parent.is_symlink() and not review_path.is_symlink() and review_path.is_file():
                review = json.loads(review_path.read_text(encoding='utf-8'))
            status = review.get('status') if review.get('episode_id') == episode.name else None
            episodes.append({
                'dataset_id': dataset_id, 'episode_id': episode.name,
                'created_at': str(manifest.get('created_at', '')),
                'frame_count': int(manifest['frame_count']),
                'duration_s': float(manifest['duration_s']),
                'review_status': status if status in {'accepted', 'rejected'} else None,
            })
        except (OSError, ValueError, TypeError, KeyError, AttributeError):
            continue
    return sorted(episodes, key=lambda e: (e['created_at'], e['episode_id']), reverse=True)[:20]
