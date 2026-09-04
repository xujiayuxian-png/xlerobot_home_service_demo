"""Command-line interfaces for offline calibration solving."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys

import numpy as np
import yaml

from .solver import ARM_MODEL, CalibrationSample, HEAD_MODEL, solve
from .transforms import to_dict


SAMPLE_SCHEMA = 'xlerobot_transform_samples/v1'
RESULT_SCHEMA = 'xlerobot_transform_calibration/v1'


def load_samples(path: Path, expected_model: str) -> tuple[dict, list[CalibrationSample]]:
    """Load a strict, versioned offline sample file."""
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('schema') != SAMPLE_SCHEMA:
        raise ValueError(f'expected schema {SAMPLE_SCHEMA}')
    if document.get('model') != expected_model:
        raise ValueError(f'expected model {expected_model}')
    rows = document.get('samples')
    if not isinstance(rows, list):
        raise ValueError('samples must be a list')
    samples = []
    for position, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f'sample {position} must be a mapping')
        samples.append(
            CalibrationSample(
                index=int(row.get('index', position)),
                moving_in_base=np.asarray(row['moving_in_base'], dtype=np.float64),
                target_in_camera=np.asarray(
                    row['target_in_camera'], dtype=np.float64
                ),
            )
        )
    return document, samples


def _parser(model: str) -> argparse.ArgumentParser:
    description = (
        'Solve head mount-to-camera with a fixed target.'
        if model == HEAD_MODEL
        else 'Solve fixed base-to-camera and moving gripper-to-target hand-eye.'
    )
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    translation_default = 3.0 if model == HEAD_MODEL else 10.0
    rotation_default = 0.5 if model == HEAD_MODEL else 5.0
    parser.add_argument(
        '--max-translation-rmse-mm', type=float, default=translation_default
    )
    parser.add_argument(
        '--max-rotation-rmse-deg', type=float, default=rotation_default
    )
    parser.add_argument('--allow-low-quality', action='store_true')
    return parser


def run(model: str, argv: list[str] | None = None) -> int:
    """Solve one sample file and enforce explicit quality gates."""
    args = _parser(model).parse_args(argv)
    if args.max_translation_rmse_mm <= 0 or args.max_rotation_rmse_deg <= 0:
        print('quality limits must be positive', file=sys.stderr)
        return 1
    try:
        source, samples = load_samples(args.input, model)
        solution = solve(samples, model)
    except (OSError, KeyError, TypeError, ValueError, RuntimeError, yaml.YAMLError) as error:
        print(str(error), file=sys.stderr)
        return 1
    metrics = solution.metrics
    quality_ok = (
        metrics['translation_rmse_mm'] <= args.max_translation_rmse_mm
        and metrics['rotation_rmse_deg'] <= args.max_rotation_rmse_deg
    )
    result = {
        'schema': RESULT_SCHEMA,
        'model': model,
        'calibration_id': source.get('calibration_id', args.input.stem),
        'created_at': datetime.now(timezone.utc).isoformat(),
        'sample_count': len(samples),
        'method': solution.method,
        'quality_ok': quality_ok,
        'quality_limits': {
            'max_translation_rmse_mm': args.max_translation_rmse_mm,
            'max_rotation_rmse_deg': args.max_rotation_rmse_deg,
        },
        'metrics': metrics,
        'x_semantics': (
            'mount_from_camera' if model == HEAD_MODEL else 'base_from_camera'
        ),
        'y_semantics': (
            'base_from_target' if model == HEAD_MODEL else 'gripper_from_target'
        ),
        'x': to_dict(solution.x),
        'y': to_dict(solution.y),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(result, sort_keys=False), encoding='utf-8'
    )
    print(yaml.safe_dump(result, sort_keys=False))
    if not quality_ok and not args.allow_low_quality:
        return 2
    return 0


def main_head(argv: list[str] | None = None) -> int:
    """Solve the moving-head-camera/fixed-target model."""
    return run(HEAD_MODEL, argv)


def main_arm(argv: list[str] | None = None) -> int:
    """Solve the fixed-camera/moving-gripper-target model."""
    return run(ARM_MODEL, argv)
