"""One small public CLI for calibration drafts, replay, and activation."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
from typing import Any

import yaml

from .bundle import UnitCalibrationStore
from .quality import validate_component
from .workflow import (
    fit_base_geometry,
    fit_grasp_alignment,
    load_yaml,
    solve_transform_samples,
)


def _repo_root() -> Path:
    configured = os.environ.get('XLEROBOT_DEMO_ROOT')
    if configured:
        return Path(configured).expanduser().resolve()
    source = Path(__file__).resolve()
    candidate = source.parents[4]
    if (candidate / 'ros2_ws/src').is_dir():
        return candidate
    return Path.cwd().resolve()


def _common() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--config', type=Path, help='demo local YAML')
    parser.add_argument(
        '--state-root', type=Path, help='override calibration local-state root'
    )
    parser.add_argument('--unit', help='override physical unit identifier')
    return parser


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        prog='tools/calibrate',
        description='Validate, replay, version, and render XLeRobot calibration.',
    )
    commands = result.add_subparsers(dest='command', required=True)
    common = _common()
    commands.add_parser('status', parents=[common])

    capture = commands.add_parser(
        'capture', help='start one live capture workspace (requires --hardware)'
    )
    capture.add_argument(
        'workflow', choices=('servo', 'base', 'head-camera', 'right-handeye')
    )
    capture.add_argument('--hardware', action='store_true')
    capture_mode = capture.add_mutually_exclusive_group()
    capture_mode.add_argument('--fresh', action='store_true')
    capture_mode.add_argument('--resume', action='store_true')
    capture.add_argument('--config', type=Path)
    capture.add_argument('--web-port', type=int, default=8080)

    servo = commands.add_parser('servo', parents=[common])
    servo.add_argument('--input', type=Path, required=True)

    base = commands.add_parser('base', parents=[common])
    base_input = base.add_mutually_exclusive_group(required=True)
    base_input.add_argument('--measurements', type=Path)
    base_input.add_argument(
        '--input', type=Path,
        help='validated xlerobot_base_geometry/v1 result from the capture UI',
    )

    for command in ('head-camera', 'right-handeye'):
        visual = commands.add_parser(command, parents=[common])
        visual.add_argument('--samples', type=Path, required=True)

    alignment = commands.add_parser('grasp-alignment', parents=[common])
    group = alignment.add_mutually_exclusive_group(required=True)
    group.add_argument('--measurements', type=Path)
    group.add_argument('--input', type=Path)

    activate = commands.add_parser('activate', parents=[common])
    activate.add_argument('--version', default='')

    rollback = commands.add_parser('rollback', parents=[common])
    rollback.add_argument('--version', required=True)

    render = commands.add_parser('render', parents=[common])
    render.add_argument(
        '--for', dest='stage_for',
        choices=('head-camera', 'right-handeye', 'grasp-alignment'),
        help='render passing prerequisite drafts for this next capture step',
    )

    replay = commands.add_parser('replay')
    replay.add_argument('workflow', choices=('head-camera', 'right-handeye'))
    replay.add_argument('--samples', type=Path)
    replay.add_argument('--expected', type=Path)
    return result


def _defaults(args: argparse.Namespace, repo: Path) -> tuple[Path, str]:
    config = args.config or Path(
        os.environ.get('XLEROBOT_CONFIG', 'config/local.yaml')
    )
    config = Path(config).expanduser()
    if not config.is_absolute():
        config = repo / config
    config = config.resolve()
    if not config.is_file():
        raise FileNotFoundError(
            f'demo config is required for calibration: {config}'
        )
    document: dict[str, Any] = load_yaml(config)
    if document.get('schema') != 'xlerobot_demo/v1':
        raise ValueError(f'unsupported demo config schema: {config}')
    robot = document.get('robot', {})
    calibration = document.get('calibration', {})
    if not isinstance(robot, dict) or not isinstance(calibration, dict):
        raise ValueError('config robot/calibration must be mappings')
    robot_unit = str(robot.get('unit_id', ''))
    calibration_unit = str(calibration.get('unit', ''))
    if not robot_unit or robot_unit != calibration_unit:
        raise ValueError(
            'robot.unit_id and calibration.unit must be the same identity'
        )
    state_value = (
        args.state_root
        or os.environ.get('XLEROBOT_STATE_ROOT')
        or calibration.get('state_root')
        or '.xlerobot'
    )
    state = Path(state_value).expanduser()
    if not state.is_absolute():
        state = repo / state
    unit = args.unit or os.environ.get('XLEROBOT_UNIT') or calibration_unit
    return state.resolve(), str(unit)


def _dump(document: dict[str, Any]) -> None:
    print(yaml.safe_dump(document, sort_keys=False, allow_unicode=True).rstrip())


def _save(
    store: UnitCalibrationStore,
    unit: str,
    name: str,
    document: dict[str, Any],
) -> dict[str, Any]:
    metrics = store.save_component(unit, name, document)
    return {
        'status': 'draft_saved',
        'unit': unit,
        'component': name,
        'path': str(store.draft_components(unit) / f'{name}.yaml'),
        'metrics': metrics,
    }


def _nested(document: dict[str, Any], dotted: str) -> Any:
    value: Any = document
    for part in dotted.split('.'):
        if isinstance(value, list) and part.isdigit() and int(part) < len(value):
            value = value[int(part)]
        elif isinstance(value, dict) and part in value:
            value = value[part]
        else:
            raise ValueError(f'replay result does not contain {dotted}')
    return value


def verify_expected(result: dict[str, Any], expected: dict[str, Any]) -> None:
    if expected.get('schema') != 'xlerobot_calibration_replay_expected/v1':
        raise ValueError('unsupported replay expected-result schema')
    checks = expected.get('checks')
    if not isinstance(checks, dict) or not checks:
        raise ValueError('replay expected result has no checks')
    for dotted, check in checks.items():
        actual = _nested(result, dotted)
        if isinstance(check, dict) and set(check) == {'value', 'absolute_tolerance'}:
            target = float(check['value'])
            tolerance = float(check['absolute_tolerance'])
            if abs(float(actual) - target) > tolerance:
                raise ValueError(
                    f'replay check {dotted}={actual} differs from {target} '
                    f'by more than {tolerance}'
                )
        elif actual != check:
            raise ValueError(
                f'replay check {dotted}: expected {check!r}, got {actual!r}'
            )


def run(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    repo = _repo_root()
    try:
        if args.command == 'replay':
            workflow = args.workflow.replace('-', '_')
            fixture = repo / 'examples/calibration' / args.workflow
            samples = (args.samples or fixture / 'samples.yaml').expanduser().resolve()
            expected_path = (args.expected or fixture / 'expected.yaml').expanduser().resolve()
            result = solve_transform_samples(samples, workflow)
            expected = load_yaml(expected_path)
            verify_expected(result, expected)
            _dump({
                'status': 'replay_passed',
                'workflow': workflow,
                'samples': str(samples),
                'expected': str(expected_path),
                'result': result,
            })
            return 0

        if args.command == 'capture':
            raise RuntimeError(
                'live capture must be invoked through the repository tools/calibrate wrapper'
            )

        state_root, unit = _defaults(args, repo)
        store = UnitCalibrationStore(state_root, repo)
        if args.command == 'status':
            _dump(store.status(unit))
        elif args.command == 'servo':
            document = load_yaml(args.input.expanduser().resolve())
            _dump(_save(store, unit, 'servo', document))
        elif args.command == 'base':
            document = (
                fit_base_geometry(args.measurements)
                if args.measurements
                else load_yaml(args.input.expanduser().resolve())
            )
            _dump(_save(store, unit, 'base_geometry', document))
        elif args.command in ('head-camera', 'right-handeye'):
            name = args.command.replace('-', '_')
            _dump(_save(store, unit, name, solve_transform_samples(args.samples, name)))
        elif args.command == 'grasp-alignment':
            if args.measurements:
                document = fit_grasp_alignment(args.measurements)
            else:
                document = load_yaml(args.input.expanduser().resolve())
                validate_component('grasp_alignment', document)
            _dump(_save(store, unit, 'grasp_alignment', document))
        elif args.command == 'activate':
            version = store.activate(unit, args.version)
            _dump({
                'status': 'activated', 'unit': unit, 'version': version,
                'active': str(store.unit_root(unit) / 'active'),
            })
        elif args.command == 'rollback':
            version = store.rollback(unit, args.version)
            _dump({'status': 'rolled_back', 'unit': unit, 'version': version})
        elif args.command == 'render':
            if args.stage_for:
                workflow = {
                    'head-camera': 'head_camera',
                    'right-handeye': 'right_handeye',
                    'grasp-alignment': 'grasp_alignment',
                }[args.stage_for]
                runtime = store.render_draft_for(unit, workflow)
                _dump({
                    'status': 'draft_stage_rendered',
                    'unit': unit,
                    'workflow': workflow,
                    'runtime': str(runtime),
                    'final_demo_runtime': False,
                })
            else:
                runtime = store.render_active(unit)
                _dump({
                    'status': 'rendered', 'unit': unit,
                    'runtime': str(runtime),
                })
        return 0
    except (KeyError, OSError, TypeError, ValueError, RuntimeError, yaml.YAMLError) as error:
        print(f'calibration error: {error}', file=sys.stderr)
        return 2


def main() -> None:
    raise SystemExit(run())


if __name__ == '__main__':
    main()
