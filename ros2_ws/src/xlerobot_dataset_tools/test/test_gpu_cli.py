import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import pytest

from xlerobot_dataset_tools.gpu_cli import (
    content_checksum,
    import_legacy,
    inspect,
    main,
    receive,
    regular_files,
)
from xlerobot_dataset_tools.legacy_lerobot import JOINT_NAMES


def video(path: Path, values: list[int]) -> None:
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter_fourcc(*'mp4v'), 30.0, (640, 480)
    )
    assert writer.isOpened()
    for value in values:
        writer.write(np.full((480, 640, 3), value, dtype=np.uint8))
    writer.release()


def legacy_dataset(
    root: Path, episode_id: str = 'ep_001', offset: int = 0,
) -> tuple[Path, np.ndarray]:
    episode = root / episode_id
    (episode / 'videos').mkdir(parents=True)
    states = np.arange(18, dtype=np.float64).reshape(3, 6) / 10.0 + offset
    np.savez(
        episode / 'data.npz', observation_state=states, action=states,
        timestamp=np.arange(3) / 30.0, frame_index=np.arange(3),
    )
    (episode / 'meta.json').write_text(json.dumps({
        'language_instruction': '抓住羽毛球', 'fps': 30,
        'joint_names': JOINT_NAMES,
    }), encoding='utf-8')
    video(episode / 'videos/head.mp4', [10 + offset, 20 + offset, 30 + offset])
    video(episode / 'videos/wrist.mp4', [40 + offset, 50 + offset, 60 + offset])
    return episode, states


def write_transfer_manifest(
    staging: Path, *, unit_id: str = 'robot-1', dataset: str = 'pick',
) -> dict:
    manifest_path = staging / 'transfer-manifest.json'
    manifest_path.unlink(missing_ok=True)
    files = regular_files(staging)
    episodes = []
    for raw in sorted((staging / 'raw').iterdir()):
        episode_id = raw.name
        selected = {
            relative: record for relative, record in files.items()
            if relative.startswith(f'raw/{episode_id}/')
            or relative == f'reviews/{episode_id}.json'
        }
        episodes.append({
            'episode_id': episode_id,
            'sha256': content_checksum(selected),
        })
    manifest = {
        'schema': 'xlerobot_dataset_transfer/v1',
        'unit_id': unit_id,
        'dataset': dataset,
        'bundle_sha256': content_checksum(files),
        'episodes': episodes,
        'files': files,
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    return manifest


def transfer_staging(
    root: Path, episode_ids: tuple[str, ...] = ('ep_001',),
) -> Path:
    legacy = root / 'legacy'
    for index, episode_id in enumerate(episode_ids):
        legacy_dataset(legacy, episode_id, index)
    staging = root / 'incoming'
    imported = import_legacy(legacy, staging / 'raw')
    for raw in imported:
        manifest_path = raw / 'manifest.json'
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        manifest['unit_id'] = 'robot-1'
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
    (staging / 'reviews').mkdir()
    for episode_id in episode_ids:
        (staging / 'reviews' / f'{episode_id}.json').write_text(json.dumps({
            'schema': 'xlerobot_episode_review/v1',
            'episode_id': episode_id,
            'status': 'accepted',
        }), encoding='utf-8')
    write_transfer_manifest(staging)
    return staging


def test_legacy_inspect_and_import_preserve_verified_next_state(tmp_path: Path):
    _, states = legacy_dataset(tmp_path / 'legacy')
    episodes, plan = inspect(tmp_path / 'legacy', 'legacy_next_state')
    assert plan['episode_count'] == 1
    assert plan['frame_count'] == 2
    output = tmp_path / 'product'
    paths = import_legacy(tmp_path / 'legacy', output)
    assert paths == [output / 'ep_001']
    with np.load(paths[0] / 'data.npz') as data:
        np.testing.assert_allclose(data['observation_state'], states[:-1])
        np.testing.assert_allclose(data['action'], states[1:])
    manifest = json.loads((paths[0] / 'manifest.json').read_text())
    assert manifest['action_semantics'] == 'follower_next_state'
    for camera in ('head', 'wrist'):
        capture = cv2.VideoCapture(str(paths[0] / f'videos/{camera}.mp4'))
        assert round(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == 2
        capture.release()
    assert episodes[0].instruction == '抓住羽毛球'


def test_receive_publishes_only_matching_accepted_product_episodes(tmp_path: Path):
    staging = transfer_staging(tmp_path)
    destination = tmp_path / 'datasets/robot-1/pick'
    result = receive(staging, destination)
    assert result['status'] == 'complete'
    assert result['added'] == ['ep_001']
    assert result['unchanged'] == []
    assert result['conflicts'] == []
    assert not staging.exists()
    assert (destination / 'raw/ep_001/data.npz').is_file()


def test_receive_is_incremental_idempotent_and_preserves_derived(tmp_path: Path):
    first = transfer_staging(tmp_path / 'first')
    rerun = tmp_path / 'rerun/incoming'
    rerun.parent.mkdir()
    shutil.copytree(first, rerun)
    destination = tmp_path / 'datasets/robot-1/pick'
    (destination / 'derived/v1').mkdir(parents=True)
    marker = destination / 'derived/v1/model-info.json'
    marker.write_text('{"preserved": true}\n', encoding='utf-8')

    assert receive(first, destination)['added'] == ['ep_001']
    same = receive(rerun, destination)
    assert same['added'] == []
    assert same['unchanged'] == ['ep_001']
    assert same['conflicts'] == []

    incremental = transfer_staging(
        tmp_path / 'incremental', episode_ids=('ep_002',),
    )
    appended = receive(incremental, destination)
    assert appended['added'] == ['ep_002']
    assert appended['unchanged'] == []
    assert marker.read_text(encoding='utf-8') == '{"preserved": true}\n'
    assert (destination.parent / '.pick.receive.lock').is_file()


def test_receive_recovers_matching_raw_without_published_review(tmp_path: Path):
    staging = transfer_staging(tmp_path / 'transfer')
    destination = tmp_path / 'datasets/robot-1/pick'
    (destination / 'raw').mkdir(parents=True)
    shutil.copytree(
        staging / 'raw/ep_001',
        destination / 'raw/ep_001',
    )

    result = receive(staging, destination)

    assert result['added'] == ['ep_001']
    assert result['conflicts'] == []
    assert (destination / 'reviews/ep_001.json').is_file()
    assert not staging.exists()


def test_receive_conflict_rejects_the_entire_batch_without_writes(
    tmp_path: Path, capsys,
):
    first = transfer_staging(tmp_path / 'first')
    conflict = tmp_path / 'conflict/incoming'
    conflict.parent.mkdir()
    shutil.copytree(first, conflict)
    destination = tmp_path / 'datasets/robot-1/pick'
    receive(first, destination)
    existing_review = destination / 'reviews/ep_001.json'
    original_review = existing_review.read_bytes()

    second = transfer_staging(
        tmp_path / 'second', episode_ids=('ep_002',),
    )
    shutil.copytree(second / 'raw/ep_002', conflict / 'raw/ep_002')
    shutil.copy2(
        second / 'reviews/ep_002.json',
        conflict / 'reviews/ep_002.json',
    )
    review_path = conflict / 'reviews/ep_001.json'
    review = json.loads(review_path.read_text(encoding='utf-8'))
    review['note'] = 'same ID but different immutable content'
    review_path.write_text(json.dumps(review), encoding='utf-8')
    write_transfer_manifest(conflict)

    result = receive(conflict, destination)
    assert result['status'] == 'conflict'
    assert result['added'] == []
    assert result['conflicts'] == ['ep_001']
    assert existing_review.read_bytes() == original_review
    assert not (destination / 'raw/ep_002').exists()
    assert not (destination / 'reviews/ep_002.json').exists()
    assert conflict.exists()

    assert main(['receive', str(conflict), str(destination)]) == 3
    wire_result = json.loads(capsys.readouterr().out)
    assert wire_result['added'] == []
    assert wire_result['unchanged'] == []
    assert wire_result['conflicts'] == ['ep_001']


def test_receive_rejects_illegal_hash_and_symlink(tmp_path: Path):
    invalid_hash = transfer_staging(tmp_path / 'hash')
    manifest_path = invalid_hash / 'transfer-manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    first_file = next(iter(manifest['files']))
    manifest['files'][first_file]['sha256'] = 'not-a-sha256'
    manifest_path.write_text(json.dumps(manifest), encoding='utf-8')
    destination = tmp_path / 'datasets/robot-1/pick'
    with pytest.raises(ValueError, match='invalid transfer hash'):
        receive(invalid_hash, destination)
    assert not destination.exists()

    invalid_raw_hash = transfer_staging(tmp_path / 'raw-hash')
    raw_manifest_path = invalid_raw_hash / 'raw/ep_001/manifest.json'
    raw_manifest = json.loads(raw_manifest_path.read_text(encoding='utf-8'))
    raw_manifest['files']['data.npz']['sha256'] = 'invalid'
    raw_manifest_path.write_text(json.dumps(raw_manifest), encoding='utf-8')
    write_transfer_manifest(invalid_raw_hash)
    with pytest.raises(ValueError, match='invalid raw hash'):
        receive(invalid_raw_hash, destination)
    assert not destination.exists()

    symlink = transfer_staging(tmp_path / 'symlink')
    payload = symlink / 'raw/ep_001/data.npz'
    external = tmp_path / 'external.npz'
    external.write_bytes(payload.read_bytes())
    payload.unlink()
    payload.symlink_to(external)
    with pytest.raises(ValueError, match='symlink'):
        receive(symlink, destination)
    assert not destination.exists()


def test_receive_rejects_raw_episode_from_a_different_unit(tmp_path: Path):
    staging = transfer_staging(tmp_path / 'unit-mismatch')
    raw_manifest_path = staging / 'raw/ep_001/manifest.json'
    raw_manifest = json.loads(raw_manifest_path.read_text(encoding='utf-8'))
    raw_manifest['unit_id'] = 'robot-2'
    raw_manifest_path.write_text(
        json.dumps(raw_manifest, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    write_transfer_manifest(staging, unit_id='robot-1')

    with pytest.raises(ValueError, match='raw unit_id does not match'):
        receive(staging, tmp_path / 'datasets/robot-1/pick')
