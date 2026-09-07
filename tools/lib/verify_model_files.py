#!/usr/bin/env python3
"""Verify an ACT checkpoint against a public or locally qualified manifest."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import re


MODEL_ID = 'xlerobot-act-local-grasp-v1'
MODEL_REPO_ID = 'lissajous/xlerobot-act-local-grasp-v1'
DATASET_REPO_ID = 'lissajous/xlerobot-glue-stick-grasp-30'
SHA256 = re.compile(r'^[0-9a-f]{64}$')
REVISION = re.compile(r'^[0-9a-f]{40}$')
EXPECTED_STRUCTURE = {
    'policy_type': 'ACT',
    'input_features': {
        'observation.state': [6],
        'observation.images.wrist': [3, 480, 640],
    },
    'output_features': {'action': [6]},
    'chunk_size': 100,
    'n_action_steps': 100,
    'dim_model': 512,
    'dim_feedforward': 3200,
    'n_encoder_layers': 4,
    'n_decoder_layers': 1,
    'n_heads': 8,
    'latent_dim': 32,
    'vision_backbone': 'resnet18',
    'pretrained_backbone_weights': 'ResNet18_Weights.IMAGENET1K_V1',
    'use_vae': True,
    'use_amp': False,
}


def digest(path: Path) -> str:
    state = sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            state.update(block)
    return state.hexdigest()


def _validate_manifest(document: object) -> tuple[str, dict[str, str]]:
    if not isinstance(document, dict):
        raise ValueError('model manifest must be a JSON object')
    schema = document.get('schema')
    if document.get('model_id') != MODEL_ID:
        raise ValueError(f'model manifest must identify {MODEL_ID}')
    if schema == 'xlerobot_model_download/v1':
        if document.get('repo_id') != MODEL_REPO_ID:
            raise ValueError('public model manifest has the wrong Hub repository')
        if not isinstance(document.get('revision'), str) or not REVISION.fullmatch(
            document['revision']
        ):
            raise ValueError('public model manifest has no immutable Hub revision')
        if document.get('availability') != 'published':
            raise ValueError('public model manifest is not marked published')
        if document.get('license') != 'Apache-2.0':
            raise ValueError('public model manifest has the wrong license')
    elif schema == 'xlerobot_act_qualification/v1':
        if document.get('status') != 'passed':
            raise ValueError('local ACT qualification did not pass')
        source = document.get('source')
        if not isinstance(source, dict) or source.get('kind') != 'local-training':
            raise ValueError('local qualification has no training provenance')
        if source.get('dataset_repo_id') != DATASET_REPO_ID:
            raise ValueError('local qualification has the wrong dataset identity')
        if source.get('tool') != 'tools/act evaluate':
            raise ValueError('local qualification was not produced by the public tool')
        if source.get('lerobot_version') != '0.5.1':
            raise ValueError('local qualification has the wrong LeRobot version')
        basename = source.get('checkpoint_basename')
        if (
            not isinstance(basename, str)
            or not basename
            or Path(basename).name != basename
        ):
            raise ValueError('local qualification checkpoint basename is invalid')
        if document.get('structure') != EXPECTED_STRUCTURE:
            raise ValueError('local qualification model structure is not supported')
    else:
        raise ValueError(f'unsupported model manifest schema: {schema!r}')

    files = document.get('files')
    if not isinstance(files, dict) or not files:
        raise ValueError('model manifest has no file hashes')
    required = {'config.json', 'model.safetensors'}
    if not required.issubset(files):
        raise ValueError('model manifest omits a required ACT checkpoint file')
    for name, expected in files.items():
        relative = Path(name) if isinstance(name, str) else Path('.')
        if (
            not isinstance(name, str)
            or not name
            or '\\' in name
            or relative.is_absolute()
            or '..' in relative.parts
            or relative.as_posix() != name
            or not isinstance(expected, str)
            or not SHA256.fullmatch(expected)
        ):
            raise ValueError('model manifest has an invalid path or SHA-256')
    return str(schema), files


def _local_inventory(root: Path, manifest_path: Path) -> set[str]:
    inventory: set[str] = set()
    resolved_manifest = manifest_path.resolve()
    for directory, names, filenames in os.walk(root, followlinks=False):
        directory_path = Path(directory)
        for name in names:
            path = directory_path / name
            if path.is_symlink():
                raise ValueError(f'checkpoint contains a symlink directory: {path}')
        for name in filenames:
            path = directory_path / name
            if path.resolve() == resolved_manifest:
                continue
            if path.is_symlink() or not path.is_file():
                raise ValueError(f'checkpoint contains a non-regular file: {path}')
            inventory.add(path.relative_to(root).as_posix())
    return inventory


def verify(manifest_path: Path, root: Path) -> int:
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise FileNotFoundError(f'model manifest missing or is a symlink: {manifest_path}')
    if root.is_symlink() or not root.is_dir():
        raise FileNotFoundError(f'checkpoint root missing or is a symlink: {root}')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    schema, files = _validate_manifest(manifest)
    for name, expected in files.items():
        relative = Path(name)
        path = root / relative
        if path.is_symlink() or not path.is_file():
            raise FileNotFoundError(f'model file missing or is a symlink: {path}')
        actual = digest(path)
        if actual != expected:
            raise ValueError(
                f'model file SHA-256 mismatch: {name}: {actual} != {expected}'
            )
    if schema == 'xlerobot_act_qualification/v1':
        inventory = _local_inventory(root, manifest_path)
        if inventory != set(files):
            missing = sorted(set(files) - inventory)
            unlisted = sorted(inventory - set(files))
            raise ValueError(
                f'local checkpoint inventory differs from qualification; '
                f'missing={missing}, unlisted={unlisted}'
            )
    return len(files)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--root', type=Path, required=True)
    arguments = parser.parse_args()
    try:
        count = verify(arguments.manifest, arguments.root)
    except (FileNotFoundError, json.JSONDecodeError, OSError, TypeError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
    print(f'model files verified: {count}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
