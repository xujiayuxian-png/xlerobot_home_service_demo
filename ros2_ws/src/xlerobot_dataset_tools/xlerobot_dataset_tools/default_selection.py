"""Keep newly finalized, validated episodes without requiring a UI click."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import uuid

from .gpu_cli import validate_raw_episode


def keep_new_episode(episode: Path) -> Path:
    """Called only by the recorder after commit, never as a historical backfill.

    Publish without replacement so a concurrent human rejection always wins.
    A crash before publication leaves an unselected raw episode, not lost data.
    """
    validate_raw_episode(episode, episode.name)
    reviews = episode.parent.parent / 'reviews'
    if reviews.is_symlink():
        raise ValueError('review directory must not be a symlink')
    reviews.mkdir(parents=True, exist_ok=True)
    target = reviews / f'{episode.name}.json'
    temporary = reviews / f'.{episode.name}.{uuid.uuid4().hex}.tmp'
    document = {
        'schema': 'xlerobot_episode_review/v1',
        'episode_id': episode.name, 'status': 'accepted',
        'operator': 'recorder', 'selection_source': 'automatic_on_save',
        'failure_reason': '', 'notes': '',
        'reviewed_at': datetime.now(timezone.utc).isoformat(),
    }
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(document, ensure_ascii=False, indent=2) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, target)
        except FileExistsError:
            if target.is_symlink() or not target.is_file():
                raise ValueError('existing selection is not a regular file')
            existing = json.loads(target.read_text(encoding='utf-8'))
            if (existing.get('schema') != document['schema']
                    or existing.get('episode_id') != episode.name
                    or existing.get('status') not in {'accepted', 'rejected'}):
                raise ValueError('invalid existing selection')
        directory = os.open(reviews, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        temporary.unlink(missing_ok=True)
    return target
