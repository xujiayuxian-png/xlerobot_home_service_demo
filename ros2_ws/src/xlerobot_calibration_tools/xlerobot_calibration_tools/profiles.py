"""Single-source calibration target, pose, and quality profiles."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .paths import default_config


def _mapping(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError(f'expected YAML mapping: {path}')
    return document


def target_profile(workflow: str, path: Path | None = None) -> dict[str, Any]:
    document = _mapping(path or default_config('targets.yaml'))
    if document.get('schema') != 'xlerobot_calibration_targets/v1':
        raise ValueError('unsupported calibration target profile schema')
    targets = document.get('targets')
    if not isinstance(targets, dict) or workflow not in targets:
        raise ValueError(f'calibration target is not defined for {workflow}')
    profile = targets[workflow]
    if not isinstance(profile, dict):
        raise ValueError(f'invalid calibration target profile: {workflow}')
    return profile


def quality_profile(workflow: str, path: Path | None = None) -> dict[str, Any]:
    document = _mapping(path or default_config('quality.yaml'))
    if document.get('schema') != 'xlerobot_calibration_quality/v1':
        raise ValueError('unsupported calibration quality profile schema')
    workflows = document.get('workflows')
    if not isinstance(workflows, dict) or workflow not in workflows:
        raise ValueError(f'calibration quality is not defined for {workflow}')
    profile = workflows[workflow]
    if not isinstance(profile, dict):
        raise ValueError(f'invalid calibration quality profile: {workflow}')
    return profile
