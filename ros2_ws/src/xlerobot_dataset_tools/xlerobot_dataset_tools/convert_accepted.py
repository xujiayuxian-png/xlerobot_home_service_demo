"""Convert only accepted immutable episodes to the LeRobot interchange."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import uuid

from .legacy_lerobot import convert, load_episode


def accepted_episode_paths(dataset: Path) -> list[Path]:
    result = []
    for review_path in sorted((dataset / 'reviews').glob('*.json')):
        review = json.loads(review_path.read_text(encoding='utf-8'))
        if review.get('schema') != 'xlerobot_episode_review/v1':
            raise ValueError(f'invalid review schema: {review_path}')
        if review.get('status') == 'accepted':
            episode = dataset / 'raw' / review['episode_id']
            if not episode.is_dir():
                raise FileNotFoundError(f'accepted raw episode is missing: {episode}')
            result.append(episode)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--version', required=True)
    parser.add_argument('--repo-id', required=True)
    parser.add_argument('--fps', type=int, default=30)
    args = parser.parse_args(argv)
    dataset = args.dataset.expanduser().resolve()
    derived = dataset / 'derived'
    output = derived / args.version
    if output.exists():
        raise SystemExit(f'derived version already exists: {output}')
    episodes = accepted_episode_paths(args.dataset)
    if not episodes:
        raise SystemExit('dataset has no accepted episodes')
    derived.mkdir(parents=True, exist_ok=True)
    work = derived / f'.{args.version}.incomplete-{uuid.uuid4().hex[:8]}'
    try:
        contracts = [load_episode(episode, action_mode='recorded') for episode in episodes]
        converted = [episode.name for episode in contracts]
        convert(contracts, output_root=work / 'lerobot', repo_id=args.repo_id)
        review_hashes = {}
        for episode_id in converted:
            content = (dataset / 'reviews' / f'{episode_id}.json').read_bytes()
            review_hashes[episode_id] = hashlib.sha256(content).hexdigest()
        (work / 'conversion_manifest.json').write_text(json.dumps({
            'schema': 'xlerobot_conversion_manifest/v1',
            'created_at': datetime.now(timezone.utc).isoformat(),
            'accepted_only': True,
            'action_mode': 'recorded',
            'repo_id': args.repo_id,
            'episodes': converted,
            'review_sha256': review_hashes,
            'outputs': {'lerobot_v3': 'lerobot'},
        }, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        (work / 'DATASET_CARD.md').write_text(
            '# XLeRobot ACT dataset\n\n'
            f'- Repository ID: `{args.repo_id}`\n'
            f'- Episodes: {len(converted)} accepted episodes\n'
            '- Robot: `xlerobot_two_wheel_reference`\n'
            '- Action provenance: Follower next-state target\n'
            '- Selection: accepted review sidecars only\n\n'
            'Raw NPZ and MP4 episodes remain immutable and are not duplicated. '
            'Review camera content and consent before publication.\n',
            encoding='utf-8',
        )
        work.rename(output)
    except Exception as error:
        raise SystemExit(
            f'accepted conversion failed ({type(error).__name__}: {error}); '
            f'incomplete output kept at {work}'
        ) from error
    print(output)
