"""Internal fail-closed runtime gate for repository launch scripts."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
from typing import Any

import yaml

from .bundle import UnitCalibrationStore
from .workflow import load_yaml


def verify(
    config: Path,
    *,
    repo_root: Path,
) -> tuple[str, dict[str, Any]]:
    """Verify the configured unit runtime and return its unit plus manifest."""
    repo_root = repo_root.expanduser().resolve()
    config = config.expanduser()
    if not config.is_absolute():
        config = repo_root / config
    document = load_yaml(config.resolve())
    if document.get('schema') != 'xlerobot_demo/v1':
        raise ValueError('expected xlerobot_demo/v1 config')
    calibration = document.get('calibration')
    if not isinstance(calibration, dict):
        raise ValueError('config.calibration must be a mapping')
    state_value = calibration.get('state_root', '.xlerobot')
    if not isinstance(state_value, str) or not state_value:
        raise ValueError('config.calibration.state_root must be a path string')
    state_root = Path(state_value).expanduser()
    if not state_root.is_absolute():
        state_root = repo_root / state_root
    unit = calibration.get('unit', 'demo-01')
    if not isinstance(unit, str):
        raise ValueError('config.calibration.unit must be a string')
    manifest = UnitCalibrationStore(state_root, repo_root).verify_runtime(unit)
    return unit, manifest


def run(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Internal launch gate for rendered calibration runtime.'
    )
    default_repo = Path(
        os.environ.get('XLEROBOT_DEMO_ROOT', Path.cwd())
    )
    parser.add_argument('--repo-root', type=Path, default=default_repo)
    parser.add_argument(
        '--config', type=Path,
        default=Path(os.environ.get('XLEROBOT_CONFIG', 'config/local.yaml')),
    )
    args = parser.parse_args(argv)
    try:
        unit, manifest = verify(args.config, repo_root=args.repo_root)
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
        print(f'calibration runtime invalid: {error}', file=sys.stderr)
        return 2
    print(
        'CALIBRATION RUNTIME OK '
        f'unit={unit} version={manifest["active_version"]}'
    )
    return 0


def main() -> None:
    raise SystemExit(run())


if __name__ == '__main__':
    main()
