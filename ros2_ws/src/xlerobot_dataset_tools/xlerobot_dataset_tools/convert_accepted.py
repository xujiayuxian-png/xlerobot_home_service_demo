"""Convert only reviewed, accepted, immutable episodes to LeRobot v3."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import uuid

from .gpu_cli import validate_raw_episode
from .legacy_lerobot import conversion_plan, convert, load_episode


IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$')


def selection_plan(dataset: Path) -> tuple[list[Path], dict]:
    """Inventory raw/review state without silently treating raw as accepted."""
    raw_root = dataset / 'raw'
    review_root = dataset / 'reviews'
    if (
        raw_root.is_symlink()
        or review_root.is_symlink()
        or not raw_root.is_dir()
        or not review_root.is_dir()
    ):
        raise ValueError('dataset must contain real raw/ and reviews/ directories')

    raw_ids = set()
    for path in raw_root.iterdir():
        if path.name.startswith('.'):
            continue
        if path.is_symlink() or not path.is_dir():
            raise ValueError(f'raw episode must be a real directory: {path}')
        if not IDENTIFIER.fullmatch(path.name):
            raise ValueError(f'invalid raw episode ID: {path.name}')
        raw_ids.add(path.name)

    reviews = {}
    for path in sorted(review_root.iterdir()):
        if path.name.startswith('.'):
            continue
        if path.is_symlink() or not path.is_file() or path.suffix != '.json':
            raise ValueError(f'review must be a regular JSON file: {path}')
        try:
            review = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f'invalid review JSON: {path}') from error
        if not isinstance(review, dict):
            raise ValueError(f'invalid immutable review contract: {path}')
        episode_id = str(review.get('episode_id', ''))
        status = review.get('status')
        if (
            review.get('schema') != 'xlerobot_episode_review/v1'
            or not IDENTIFIER.fullmatch(episode_id)
            or path.stem != episode_id
            or status not in {'accepted', 'rejected'}
        ):
            raise ValueError(f'invalid immutable review contract: {path}')
        reviews[episode_id] = status

    accepted_reviews = {key for key, value in reviews.items() if value == 'accepted'}
    rejected_reviews = {key for key, value in reviews.items() if value == 'rejected'}
    accepted = sorted(accepted_reviews & raw_ids)
    missing_raw = sorted(set(reviews) - raw_ids)
    missing_review = sorted(raw_ids - set(reviews))
    plan = {
        'schema': 'xlerobot_accepted_conversion_plan/v1',
        'dataset': dataset.name,
        'accepted_only': True,
        'episode_count': len(accepted),
        'accepted_review_count': len(accepted_reviews),
        'rejected_count': len(rejected_reviews),
        'missing_raw_count': len(missing_raw),
        'missing_review_count': len(missing_review),
        'episodes': accepted,
        'rejected': sorted(rejected_reviews),
        'missing_raw': missing_raw,
        'missing_review': missing_review,
    }
    return [raw_root / episode_id for episode_id in accepted], plan


def accepted_episode_paths(dataset: Path) -> list[Path]:
    """Return present raw episodes whose immutable review says accepted."""
    return selection_plan(dataset)[0]


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--version', required=True)
    parser.add_argument('--repo-id', required=True)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    dataset = args.dataset.expanduser().resolve()
    if not IDENTIFIER.fullmatch(args.version):
        raise SystemExit('conversion version must be a simple identifier')
    try:
        episodes, plan = selection_plan(dataset)
        missing_accepted = sorted(
            episode_id
            for episode_id in plan['missing_raw']
            if episode_id not in plan['rejected']
        )
        plan['missing_accepted_raw'] = missing_accepted
        for episode in episodes:
            validate_raw_episode(episode, episode.name)
        contracts = [load_episode(episode, action_mode='recorded') for episode in episodes]
        plan['conversion'] = conversion_plan(contracts) if contracts else None
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        raise SystemExit(f'accepted dataset validation failed: {error}') from error

    if args.dry_run:
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        return 2 if missing_accepted or not episodes else 0

    if not episodes:
        raise SystemExit('dataset has no accepted episodes with raw data')
    if missing_accepted:
        raise SystemExit(
            'accepted review has no raw episode: ' + ', '.join(missing_accepted)
        )

    derived = dataset / 'derived'
    output = derived / args.version
    if output.exists():
        raise SystemExit(f'derived version already exists: {output}')
    derived.mkdir(parents=True, exist_ok=True)
    work = derived / f'.{args.version}.incomplete-{uuid.uuid4().hex[:8]}'
    try:
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
            'episode_count': len(converted),
            'episodes': converted,
            'rejected': plan['rejected'],
            'missing_review': plan['missing_review'],
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
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
