"""GPU-side dataset inspection, legacy import, conversion, and reporting CLI."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
from pathlib import PurePosixPath
import re
import shutil
import stat
import uuid

import cv2
import numpy as np

from .legacy_lerobot import (
    JOINT_NAMES, conversion_plan, convert, discover_episodes, load_episode,
)

IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$')
SHA256 = re.compile(r'^[0-9a-f]{64}$')
TRANSFER_SCHEMA = 'xlerobot_dataset_transfer/v1'


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def content_checksum(files: dict[str, dict]) -> str:
    """Hash sorted POSIX paths and their verified content hashes."""
    value = hashlib.sha256()
    for relative in sorted(files):
        value.update(relative.encode('utf-8'))
        value.update(b'\0')
        value.update(files[relative]['sha256'].encode('ascii'))
        value.update(b'\n')
    return value.hexdigest()


def relative_path(value: object) -> PurePosixPath:
    if not isinstance(value, str) or not value:
        raise ValueError('transfer manifest contains an invalid file path')
    path = PurePosixPath(value)
    if (
        path.is_absolute()
        or path.as_posix() != value
        or any(part in {'', '.', '..'} for part in path.parts)
    ):
        raise ValueError(f'transfer manifest contains an unsafe file path: {value}')
    return path


def regular_files(root: Path, *, prefix: str = '') -> dict[str, dict]:
    """Return an exact, symlink-free content inventory below one directory."""
    files = {}
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise ValueError(f'dataset content must not contain symlinks: {path}')
        mode = path.stat().st_mode
        if stat.S_ISDIR(mode):
            continue
        if not stat.S_ISREG(mode):
            raise ValueError(f'dataset content contains a special file: {path}')
        relative = path.relative_to(root).as_posix()
        key = f'{prefix}/{relative}' if prefix else relative
        files[key] = {'size': path.stat().st_size, 'sha256': digest(path)}
    return files


def validate_raw_episode(
    path: Path, episode_id: str, expected_unit_id: str | None = None,
) -> None:
    manifest_path = path / 'manifest.json'
    try:
        if manifest_path.is_symlink() or not manifest_path.is_file():
            raise ValueError(f'{episode_id}: raw manifest is missing or is a symlink')
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f'{episode_id}: invalid raw manifest') from error
    if not isinstance(manifest, dict):
        raise ValueError(f'{episode_id}: raw manifest is not an object')
    if (
        manifest.get('schema') != 'xlerobot_raw_episode/v1'
        or manifest.get('status') != 'complete'
        or manifest.get('action_semantics') != 'follower_next_state'
        or str(manifest.get('episode_id', '')) != episode_id
    ):
        raise ValueError(f'{episode_id}: invalid complete raw episode contract')
    raw_unit_id = manifest.get('unit_id')
    if expected_unit_id is not None:
        if raw_unit_id != expected_unit_id:
            raise ValueError(
                f'{episode_id}: raw unit_id does not match transfer unit_id'
            )
    elif raw_unit_id is not None and not IDENTIFIER.fullmatch(str(raw_unit_id)):
        raise ValueError(f'{episode_id}: raw manifest has invalid unit_id')
    declared = manifest.get('files')
    if not isinstance(declared, dict) or not declared:
        raise ValueError(f'{episode_id}: raw manifest has no file checksums')

    declared_paths = set()
    for value, expected in declared.items():
        relative = relative_path(value)
        if not isinstance(expected, dict):
            raise ValueError(f'{episode_id}: invalid raw file record: {value}')
        expected_hash = expected.get('sha256')
        expected_size = expected.get('size')
        if (
            not isinstance(expected_hash, str)
            or not SHA256.fullmatch(expected_hash)
            or isinstance(expected_size, bool)
            or not isinstance(expected_size, int)
            or expected_size < 0
        ):
            raise ValueError(f'{episode_id}: invalid raw hash or size: {value}')
        file_path = path.joinpath(*relative.parts)
        if file_path.is_symlink() or not file_path.is_file():
            raise ValueError(f'{episode_id}: raw payload is missing: {value}')
        if file_path.stat().st_size != expected_size or digest(file_path) != expected_hash:
            raise ValueError(f'{episode_id}: raw manifest mismatch: {value}')
        declared_paths.add(relative.as_posix())

    actual = set()
    for file_path in path.rglob('*'):
        if file_path.is_symlink():
            raise ValueError(f'{episode_id}: raw episode contains a symlink')
        mode = file_path.stat().st_mode
        if stat.S_ISREG(mode):
            relative = file_path.relative_to(path).as_posix()
            if relative != 'manifest.json':
                actual.add(relative)
        elif not stat.S_ISDIR(mode):
            raise ValueError(f'{episode_id}: raw episode contains a special file')
    if actual != declared_paths:
        raise ValueError(f'{episode_id}: raw payload differs from its manifest')
    try:
        episode = load_episode(path, action_mode='recorded')
        conversion_plan([episode])
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        raise ValueError(f'{episode_id}: raw episode content is invalid') from error


def validate_transfer(staging: Path) -> dict:
    if staging.is_symlink() or not staging.is_dir():
        raise ValueError('incoming staging must be a real directory')
    expected_top = {'raw', 'reviews', 'transfer-manifest.json'}
    if {path.name for path in staging.iterdir()} != expected_top:
        raise ValueError('incoming dataset contains unexpected top-level entries')
    raw_root = staging / 'raw'
    review_root = staging / 'reviews'
    manifest_path = staging / 'transfer-manifest.json'
    if (
        raw_root.is_symlink()
        or review_root.is_symlink()
        or not raw_root.is_dir()
        or not review_root.is_dir()
        or manifest_path.is_symlink()
        or not manifest_path.is_file()
    ):
        raise ValueError('incoming transfer layout contains a symlink or wrong type')

    try:
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError('incoming transfer manifest is invalid JSON') from error
    if not isinstance(manifest, dict):
        raise ValueError('incoming transfer manifest is not an object')
    if manifest.get('schema') != TRANSFER_SCHEMA:
        raise ValueError('incoming transfer manifest schema is invalid')
    for field in ('unit_id', 'dataset'):
        if not IDENTIFIER.fullmatch(str(manifest.get(field, ''))):
            raise ValueError(f'incoming transfer manifest has invalid {field}')

    declared = manifest.get('files')
    if not isinstance(declared, dict) or not declared:
        raise ValueError('incoming transfer manifest has no files')
    files = {}
    for value, expected in declared.items():
        relative = relative_path(value)
        relative_string = relative.as_posix()
        if not (
            (
                len(relative.parts) >= 3
                and relative.parts[0] == 'raw'
                and IDENTIFIER.fullmatch(relative.parts[1])
            )
            or (
                len(relative.parts) == 2
                and relative.parts[0] == 'reviews'
                and relative.parts[1].endswith('.json')
                and IDENTIFIER.fullmatch(relative.parts[1][:-5])
            )
        ):
            raise ValueError(f'invalid transfer payload path: {relative_string}')
        if not isinstance(expected, dict):
            raise ValueError(f'invalid transfer file record: {relative_string}')
        expected_hash = expected.get('sha256')
        expected_size = expected.get('size')
        if (
            not isinstance(expected_hash, str)
            or not SHA256.fullmatch(expected_hash)
            or isinstance(expected_size, bool)
            or not isinstance(expected_size, int)
            or expected_size < 0
        ):
            raise ValueError(f'invalid transfer hash or size: {relative_string}')
        files[relative_string] = {
            'size': expected_size,
            'sha256': expected_hash,
        }

    actual = regular_files(staging)
    actual.pop('transfer-manifest.json', None)
    if set(actual) != set(files):
        raise ValueError('incoming payload differs from its transfer manifest')
    for relative, expected in files.items():
        if actual[relative] != expected:
            raise ValueError(f'incoming transfer checksum mismatch: {relative}')
    bundle_hash = manifest.get('bundle_sha256')
    if (
        not isinstance(bundle_hash, str)
        or not SHA256.fullmatch(bundle_hash)
        or content_checksum(files) != bundle_hash
    ):
        raise ValueError('incoming bundle checksum is invalid')

    episode_entries = manifest.get('episodes')
    if not isinstance(episode_entries, list) or not episode_entries:
        raise ValueError('incoming transfer manifest has no episodes')
    episodes = {}
    for entry in episode_entries:
        if not isinstance(entry, dict):
            raise ValueError('incoming transfer manifest has an invalid episode')
        episode_id = str(entry.get('episode_id', ''))
        episode_hash = entry.get('sha256')
        if (
            not IDENTIFIER.fullmatch(episode_id)
            or episode_id in episodes
            or not isinstance(episode_hash, str)
            or not SHA256.fullmatch(episode_hash)
        ):
            raise ValueError('incoming transfer manifest has an invalid episode')
        episode_files = {
            relative: record for relative, record in files.items()
            if relative.startswith(f'raw/{episode_id}/')
            or relative == f'reviews/{episode_id}.json'
        }
        if (
            f'raw/{episode_id}/manifest.json' not in episode_files
            or f'reviews/{episode_id}.json' not in episode_files
            or content_checksum(episode_files) != episode_hash
        ):
            raise ValueError(f'incoming episode checksum is invalid: {episode_id}')
        episodes[episode_id] = {'sha256': episode_hash, 'files': episode_files}

    raw_ids = {
        path.name for path in raw_root.iterdir()
        if path.is_dir() and not path.is_symlink()
    }
    review_ids = {path.stem for path in review_root.iterdir()}
    if raw_ids != set(episodes) or review_ids != set(episodes):
        raise ValueError('incoming raw episodes and accepted reviews differ')
    for episode_id in sorted(episodes):
        review_path = review_root / f'{episode_id}.json'
        try:
            review = json.loads(review_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f'invalid incoming review: {episode_id}') from error
        if not isinstance(review, dict):
            raise ValueError(f'incoming review is not an object: {episode_id}')
        if (
            review.get('schema') != 'xlerobot_episode_review/v1'
            or review.get('status') != 'accepted'
            or str(review.get('episode_id', '')) != episode_id
        ):
            raise ValueError(f'incoming review is not accepted: {episode_id}')
        validate_raw_episode(
            raw_root / episode_id, episode_id, manifest['unit_id'],
        )
    manifest['_validated_episodes'] = episodes
    return manifest


def inspect(source: Path, action_mode: str, limit: int = 0) -> tuple[list, dict]:
    episodes = discover_episodes(source, action_mode=action_mode, limit=limit)
    return episodes, conversion_plan(episodes)


def copy_video(source: Path, target: Path, frames: int, fps: int) -> None:
    capture = cv2.VideoCapture(str(source))
    writer = cv2.VideoWriter(
        str(target), cv2.VideoWriter_fourcc(*'mp4v'), float(fps), (640, 480)
    )
    if not capture.isOpened() or not writer.isOpened():
        capture.release()
        writer.release()
        raise RuntimeError(f'could not open legacy video: {source}')
    try:
        for index in range(frames):
            ok, image = capture.read()
            if not ok or image.shape[:2] != (480, 640):
                raise ValueError(f'{source}: invalid frame {index}')
            writer.write(image)
    finally:
        capture.release()
        writer.release()


def import_legacy(source: Path, destination: Path) -> list[Path]:
    episodes = discover_episodes(source, action_mode='legacy_next_state')
    destination.mkdir(parents=True, exist_ok=True)
    outputs = []
    for episode in episodes:
        final = destination / episode.name
        if final.exists():
            raise FileExistsError(f'episode already exists: {final}')
        work = destination / f'.incomplete-{episode.name}-{uuid.uuid4().hex[:8]}'
        (work / 'videos').mkdir(parents=True)
        try:
            np.savez(
                work / 'data.npz', observation_state=episode.states,
                action=episode.actions, timestamp=episode.timestamps,
                frame_index=np.arange(episode.frame_count, dtype=np.int64),
            )
            copy_video(
                episode.head_video, work / 'videos/head.mp4',
                episode.frame_count, episode.fps,
            )
            copy_video(
                episode.wrist_video, work / 'videos/wrist.mp4',
                episode.frame_count, episode.fps,
            )
            files = [work / 'data.npz', work / 'videos/head.mp4', work / 'videos/wrist.mp4']
            manifest = {
                'schema': 'xlerobot_raw_episode/v1', 'status': 'complete',
                'episode_id': episode.name, 'fps': episode.fps,
                'joint_names': JOINT_NAMES, 'action_semantics': 'follower_next_state',
                'language_instruction': episode.instruction,
                'created_at': datetime.now(timezone.utc).isoformat(),
                'source_format': episode.source_format,
                'frame_count': episode.frame_count,
                'files': {
                    str(path.relative_to(work)): {
                        'size': path.stat().st_size, 'sha256': digest(path),
                    } for path in files
                },
            }
            (work / 'manifest.json').write_text(
                json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
                encoding='utf-8',
            )
            os.rename(work, final)
            outputs.append(final)
        except Exception:
            # A failed attempt remains explicitly incomplete for diagnosis.
            raise
    return outputs


def existing_episode_files(
    destination: Path, episode_id: str,
) -> tuple[dict[str, dict], bool, bool]:
    raw_path = destination / 'raw' / episode_id
    review_path = destination / 'reviews' / f'{episode_id}.json'
    if raw_path.is_symlink() or review_path.is_symlink():
        raise ValueError(f'existing episode contains a symlink: {episode_id}')
    has_raw = raw_path.exists()
    has_review = review_path.exists()
    if has_raw and not raw_path.is_dir():
        raise ValueError(f'existing raw episode is not a directory: {episode_id}')
    if has_review and not review_path.is_file():
        raise ValueError(f'existing review is not a regular file: {episode_id}')
    files = {}
    if has_raw:
        files.update(regular_files(raw_path, prefix=f'raw/{episode_id}'))
    if has_review:
        files[f'reviews/{episode_id}.json'] = {
            'size': review_path.stat().st_size,
            'sha256': digest(review_path),
        }
    return files, has_raw, has_review


def receive(staging: Path, destination: Path) -> dict:
    """Validate and incrementally publish one transfer under a dataset lock."""
    if staging.is_symlink():
        raise ValueError('incoming staging must not be a symlink')
    if destination.is_symlink():
        raise ValueError('dataset destination must not be a symlink')
    staging = staging.absolute()
    destination = destination.absolute()
    destination.parent.mkdir(parents=True, exist_ok=True)
    lock_path = destination.parent / f'.{destination.name}.receive.lock'
    flags = os.O_CREAT | os.O_RDONLY
    if hasattr(os, 'O_CLOEXEC'):
        flags |= os.O_CLOEXEC
    if hasattr(os, 'O_NOFOLLOW'):
        flags |= os.O_NOFOLLOW
    try:
        lock_fd = os.open(lock_path, flags, 0o660)
    except OSError as error:
        raise ValueError(f'could not open dataset receive lock: {lock_path}') from error
    try:
        if not stat.S_ISREG(os.fstat(lock_fd).st_mode):
            raise ValueError(f'dataset receive lock is not a regular file: {lock_path}')
        try:
            os.fchmod(lock_fd, 0o660)
        except PermissionError:
            # A different xlerobot group member may own an existing readable lock.
            pass
        fcntl.flock(lock_fd, fcntl.LOCK_EX)
        manifest = validate_transfer(staging)
        if (
            destination.name != manifest['dataset']
            or destination.parent.name != manifest['unit_id']
        ):
            raise ValueError('transfer identity does not match dataset destination')
        if staging.stat().st_dev != destination.parent.stat().st_dev:
            raise ValueError('incoming and dataset roots must share one filesystem')
        if destination.exists():
            if destination.is_symlink() or not destination.is_dir():
                raise ValueError('dataset destination must be a real directory')
            for name in ('raw', 'reviews'):
                path = destination / name
                if path.is_symlink() or (path.exists() and not path.is_dir()):
                    raise ValueError(f'dataset {name} must be a real directory')

        episodes = manifest['_validated_episodes']
        added = []
        unchanged = []
        conflicts = []
        raw_to_publish = []
        for episode_id in sorted(episodes):
            incoming = episodes[episode_id]['files']
            incoming_raw = {
                relative: record for relative, record in incoming.items()
                if relative.startswith(f'raw/{episode_id}/')
            }
            existing, has_raw, has_review = existing_episode_files(
                destination, episode_id,
            )
            if not has_raw and not has_review:
                added.append(episode_id)
                raw_to_publish.append(episode_id)
            elif has_raw and not has_review and existing == incoming_raw:
                # Recover a crash after raw publication but before review publication.
                added.append(episode_id)
            elif (
                has_raw
                and has_review
                and existing == incoming
                and content_checksum(existing) == episodes[episode_id]['sha256']
            ):
                unchanged.append(episode_id)
            else:
                conflicts.append(episode_id)

        result = {
            'status': 'conflict' if conflicts else 'complete',
            'destination': str(destination),
            'bundle_sha256': manifest['bundle_sha256'],
            'added': [] if conflicts else added,
            'unchanged': unchanged,
            'conflicts': conflicts,
        }
        if conflicts:
            return result

        if added:
            raw_root = destination / 'raw'
            review_root = destination / 'reviews'
            raw_root.mkdir(parents=True, exist_ok=True)
            review_root.mkdir(parents=True, exist_ok=True)
            for episode_id in raw_to_publish:
                os.rename(
                    staging / 'raw' / episode_id,
                    raw_root / episode_id,
                )
            # An accepted review is the publication marker. Publish every raw
            # directory first, then atomically expose each review.
            for episode_id in added:
                os.rename(
                    staging / 'reviews' / f'{episode_id}.json',
                    review_root / f'{episode_id}.json',
                )
        shutil.rmtree(staging)
        return result
    finally:
        fcntl.flock(lock_fd, fcntl.LOCK_UN)
        os.close(lock_fd)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog='xlerobot-data')
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('inspect', 'validate', 'report'):
        command = commands.add_parser(name)
        command.add_argument('source', type=Path)
        command.add_argument(
            '--action-mode', choices=('recorded', 'legacy_next_state'), default='recorded'
        )
        command.add_argument('--limit-episodes', type=int, default=0)
        if name == 'report':
            command.add_argument('--output', type=Path)
    legacy = commands.add_parser('import-legacy')
    legacy.add_argument('source', type=Path)
    legacy.add_argument('destination', type=Path)
    conversion = commands.add_parser('convert')
    conversion.add_argument('source', type=Path)
    conversion.add_argument('output', type=Path)
    conversion.add_argument('--repo-id', required=True)
    conversion.add_argument(
        '--action-mode', choices=('recorded', 'legacy_next_state'), default='recorded'
    )
    conversion.add_argument('--limit-episodes', type=int, default=0)
    conversion.add_argument('--dry-run', action='store_true')
    incoming = commands.add_parser('receive')
    incoming.add_argument('staging', type=Path)
    incoming.add_argument('destination', type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == 'receive':
            result = receive(args.staging, args.destination)
            print(json.dumps(result, ensure_ascii=False, sort_keys=True))
            return 3 if result['conflicts'] else 0
        if args.command == 'import-legacy':
            outputs = import_legacy(args.source.resolve(), args.destination.resolve())
            print(json.dumps({'imported': [str(path) for path in outputs]}, indent=2))
            return 0
        episodes, plan = inspect(
            args.source.resolve(), args.action_mode, args.limit_episodes
        )
        if args.command in {'inspect', 'validate'}:
            plan['status'] = 'valid'
            print(json.dumps(plan, ensure_ascii=False, indent=2))
            return 0
        if args.command == 'report':
            report = (
                '# XLeRobot dataset report\n\n'
                f'- Episodes: {plan["episode_count"]}\n'
                f'- Frames: {plan["frame_count"]}\n'
                f'- FPS: {plan["fps"]}\n'
                f'- Image: {plan["image_width"]}x{plan["image_height"]}\n'
                '- Action: `follower_state[t+1]`\n'
            )
            if args.output:
                args.output.write_text(report, encoding='utf-8')
                print(args.output)
            else:
                print(report, end='')
            return 0
        if args.dry_run:
            print(json.dumps(plan, ensure_ascii=False, indent=2))
            return 0
        output = convert(episodes, output_root=args.output, repo_id=args.repo_id)
        print(output)
        return 0
    except (FileExistsError, OSError, RuntimeError, ValueError) as error:
        raise SystemExit(str(error)) from error


if __name__ == '__main__':
    raise SystemExit(main())
