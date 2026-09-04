#!/usr/bin/env python3
"""Fetch or verify the explicit AGPL Ultralytics person-detector extra."""

from __future__ import annotations

import argparse
from hashlib import sha256
import os
from pathlib import Path
import shutil
import tempfile
from urllib.request import urlopen

import yaml


def digest(path: Path) -> str:
    state = sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            state.update(block)
    return state.hexdigest()


def load_contract(manifest: Path) -> tuple[str, str, int]:
    document = yaml.safe_load(manifest.read_text(encoding='utf-8'))
    record = document['models']['person_detector']
    url = record['public_download']
    expected = record['files']['yolov8n.pt']
    size = record['file_sizes']['yolov8n.pt']
    if (
        not isinstance(url, str)
        or not url.startswith('https://github.com/ultralytics/assets/')
        or not isinstance(expected, str)
        or len(expected) != 64
        or set(expected) - set('0123456789abcdef')
        or isinstance(size, bool)
        or not isinstance(size, int)
        or size <= 0
    ):
        raise ValueError('invalid person-detector manifest contract')
    return url, expected, size


def valid(path: Path, expected: str, size: int) -> bool:
    return (
        path.is_file()
        and not path.is_symlink()
        and path.stat().st_size == size
        and digest(path) == expected
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        url, expected, size = load_contract(arguments.manifest)
        if valid(arguments.output, expected, size):
            print(f'person detector verified: {arguments.output}')
            return 0
        if arguments.check:
            raise RuntimeError(f'person detector missing or invalid: {arguments.output}')
        if arguments.output.exists() or arguments.output.is_symlink():
            raise RuntimeError(
                f'{arguments.output} exists but does not match the manifest; '
                'move it aside and rerun setup'
            )
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix='.yolov8n.', suffix='.download', dir=arguments.output.parent
        )
        try:
            with os.fdopen(descriptor, 'wb') as output, urlopen(
                url, timeout=60
            ) as response:
                shutil.copyfileobj(response, output)
            temporary = Path(temporary_name)
            if not valid(temporary, expected, size):
                raise RuntimeError('downloaded person detector failed size or SHA-256')
            os.replace(temporary, arguments.output)
        finally:
            Path(temporary_name).unlink(missing_ok=True)
    except (KeyError, OSError, RuntimeError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise SystemExit(str(exc)) from exc
    print(f'person detector downloaded and verified: {arguments.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
