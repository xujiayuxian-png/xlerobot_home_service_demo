from argparse import Namespace
from copy import deepcopy
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from xlerobot_calibration_tools.bundle import COMPONENTS, UnitCalibrationStore
from xlerobot_calibration_tools.public_cli import _defaults, verify_expected
from xlerobot_calibration_tools.quality import (
    observability,
    validate_component,
    validate_servo,
)
from xlerobot_calibration_tools.runtime_gate import run as run_runtime_gate
from xlerobot_calibration_tools.solver import CalibrationSample, HEAD_MODEL
from xlerobot_calibration_tools.workflow import (
    fit_base_geometry,
    fit_grasp_alignment,
    load_yaml,
    solve_transform_samples,
)
import yaml


REPO = Path(__file__).resolve().parents[4]
EXAMPLES = REPO / 'examples/calibration'


def test_non_replay_config_is_required_repo_relative_and_identity_checked(
    tmp_path, monkeypatch,
):
    config = tmp_path / 'config/local.yaml'
    config.parent.mkdir()
    config.write_text(yaml.safe_dump({
        'schema': 'xlerobot_demo/v1',
        'robot': {'unit_id': 'demo-unit'},
        'calibration': {'unit': 'demo-unit', 'state_root': '.state'},
    }))
    elsewhere = tmp_path / 'caller'
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    state, unit = _defaults(Namespace(
        config=Path('config/local.yaml'), state_root=None, unit=None,
    ), tmp_path)
    assert state == (tmp_path / '.state').resolve()
    assert unit == 'demo-unit'

    with pytest.raises(FileNotFoundError, match='config is required'):
        _defaults(Namespace(
            config=Path('missing.yaml'), state_root=None, unit=None,
        ), tmp_path)

    document = yaml.safe_load(config.read_text())
    document['calibration']['unit'] = 'different-unit'
    config.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='same identity'):
        _defaults(Namespace(
            config=Path('config/local.yaml'), state_root=None, unit=None,
        ), tmp_path)


@pytest.fixture(scope='module')
def components():
    return {
        'servo': load_yaml(EXAMPLES / 'servo.yaml'),
        'base_geometry': fit_base_geometry(EXAMPLES / 'base_measurements.yaml'),
        'head_camera': solve_transform_samples(
            EXAMPLES / 'head-camera/samples.yaml', 'head_camera'
        ),
        'right_handeye': solve_transform_samples(
            EXAMPLES / 'right-handeye/samples.yaml', 'right_handeye'
        ),
        'grasp_alignment': fit_grasp_alignment(
            EXAMPLES / 'grasp_alignment_measurements.yaml'
        ),
    }


def test_selective_replace_preserves_base_and_restores_original(tmp_path, components):
    store = UnitCalibrationStore(tmp_path, REPO)
    for name, doc in components.items():
        store.save_component('unit', name, deepcopy(doc))
    store.activate('unit', 'original')
    runtime = store.render_active('unit')
    before = {p.name: p.read_bytes() for p in runtime.glob('*.yaml') if p.name != 'manifest.yaml'}
    preview = store.replace_from_draft('unit', 'new', ['head_camera', 'right_handeye'], dry_run=True)
    assert preview['retained'] == ['servo', 'base_geometry', 'grasp_alignment']
    assert store.active_version('unit').name == 'original'
    store.replace_from_draft('unit', 'new', ['head_camera', 'right_handeye'])
    assert store.verify_runtime('unit')['active_version'] == 'new'
    assert (runtime / 'controllers.yaml').read_bytes() == before['controllers.yaml']
    assert (runtime / 'servos.yaml').read_bytes() == before['servos.yaml']
    with pytest.raises(ValueError):
        store.replace_from_draft('unit', 'new', ['head_camera'])
    store.switch('unit', 'original')
    assert store.verify_runtime('unit')['active_version'] == 'original'
    assert all((runtime / name).read_bytes() == value for name, value in before.items())
    with pytest.raises(ValueError):
        store.switch('unit', 'missing')
    assert store.verify_runtime('unit')['active_version'] == 'original'


def test_public_replays_pass_strict_and_robust_quality_gates(components):
    head = components['head_camera']
    handeye = components['right_handeye']
    assert head['metrics']['sample_count'] == 12
    assert head['metrics']['observability']['rotation_rank'] >= 2
    assert handeye['metrics']['sample_count'] == 36
    assert handeye['metrics']['translation_p95_mm'] < 15.0
    # The maximum stays visible, but one outlier is not allowed to replace the
    # documented robust p95 publication rule.
    assert handeye['metrics']['translation_max_mm'] > 15.0
    verify_expected(head, load_yaml(EXAMPLES / 'head-camera/expected.yaml'))
    verify_expected(
        handeye, load_yaml(EXAMPLES / 'right-handeye/expected.yaml')
    )
    missing_evidence = deepcopy(handeye)
    del missing_evidence['metrics']['observability']
    with pytest.raises(ValueError, match='observability is required'):
        validate_component('right_handeye', missing_evidence)


def test_servo_schema_rejects_non_integer_and_underobserved_values():
    document = load_yaml(EXAMPLES / 'servo.yaml')
    metrics = validate_servo(document)
    assert metrics['joint_count'] == 14

    wrong_type = deepcopy(document)
    wrong_type['head']['joints']['pan']['offset'] = 2015.0
    with pytest.raises(ValueError, match='must be an integer'):
        validate_servo(wrong_type)

    underobserved = deepcopy(document)
    row = underobserved['right_arm']['joints']['shoulder_pan']
    row['raw_min'], row['raw_max'] = row['offset'] - 10, row['offset'] + 10
    with pytest.raises(ValueError, match='captured range covers'):
        validate_servo(underobserved)


def test_visual_observability_rejects_one_axis_motion():
    samples = []
    for index in range(12):
        moving = np.eye(4)
        angle = 0.08 * index
        moving[:3, :3] = [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(angle), -np.sin(angle)],
            [0.0, np.sin(angle), np.cos(angle)],
        ]
        samples.append(CalibrationSample(index, moving, np.eye(4)))
    with pytest.raises(ValueError, match='rotationally observable'):
        observability(samples, HEAD_MODEL)


def test_bundle_requires_every_component_and_renders_rollback(tmp_path, components):
    store = UnitCalibrationStore(tmp_path / '.xlerobot', REPO)
    unit = 'test-unit'
    for name in COMPONENTS[:-1]:
        store.save_component(unit, name, deepcopy(components[name]))
    with pytest.raises(ValueError, match='incomplete'):
        store.activate(unit, 'partial')

    store.save_component(unit, 'grasp_alignment', deepcopy(components['grasp_alignment']))
    assert store.activate(unit, 'v1') == 'v1'
    first_runtime = store.render_active(unit)
    assert first_runtime.is_dir()

    changed = deepcopy(components['base_geometry'])
    changed['wheel_radius_m'] = 0.064
    store.save_component(unit, 'base_geometry', changed)
    assert store.activate(unit, 'v2') == 'v2'
    assert not first_runtime.exists()
    assert store.rollback(unit, 'v1') == 'v1'
    runtime = store.render_active(unit)
    geometry = yaml.safe_load((runtime / 'geometry.yaml').read_text())
    manifest = yaml.safe_load((runtime / 'manifest.yaml').read_text())
    assert geometry['base']['wheel_radius'] == (
        components['base_geometry']['wheel_radius_m']
    )
    assert manifest['active_version'] == 'v1'
    assert store.verify_runtime(unit)['active_version'] == 'v1'
    assert {path.name for path in runtime.iterdir()} == {
        'geometry.yaml', 'servos.yaml', 'controllers.yaml', 'transforms.yaml',
        'grasp_alignment.yaml', 'manifest.yaml',
    }


def test_existing_runtime_import_preserves_values_and_rejects_tampering(tmp_path, components):
    store = UnitCalibrationStore(tmp_path / '.xlerobot', REPO)
    for name, document in components.items():
        store.save_component('source-unit', name, deepcopy(document))
    store.activate('source-unit', 'measured')
    source = store.render_active('source-unit')
    alignment = load_yaml(source / 'grasp_alignment.yaml')
    del alignment['metrics']
    alignment.update(validation='existing_unit_runtime', provenance={'source': 'existing-unit'})
    (source / 'grasp_alignment.yaml').write_text(yaml.safe_dump(alignment))
    transforms = load_yaml(source / 'transforms.yaml')
    transforms['provenance'] = {'source': 'existing-unit'}
    (source / 'transforms.yaml').write_text(yaml.safe_dump(transforms))
    store.import_runtime('existing-unit', source, 'adopted')
    rendered = store.render_active('existing-unit')
    assert load_yaml(rendered / 'geometry.yaml') == load_yaml(source / 'geometry.yaml')
    assert store.verify_runtime('existing-unit')['source'] == 'existing_unit_runtime'
    with pytest.raises(FileExistsError):
        store.import_runtime('existing-unit', source, 'adopted')
    bad = load_yaml(source / 'geometry.yaml')
    bad['base']['wheel_radius'] = float('nan')
    (source / 'geometry.yaml').write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError, match='finite'):
        store.import_runtime('existing-unit', source, 'invalid')
    version = store.active_version('existing-unit')
    (version / 'servos.yaml').write_text('{}')
    with pytest.raises(ValueError, match='checksum'):
        store.verify_runtime('existing-unit')


def test_staged_render_uses_only_passing_preceding_drafts(tmp_path, components):
    store = UnitCalibrationStore(tmp_path / '.xlerobot', REPO)
    unit = 'test-unit'
    store.save_component(unit, 'servo', deepcopy(components['servo']))
    head_stage = store.render_draft_for(unit, 'head_camera')
    head_manifest = yaml.safe_load((head_stage / 'manifest.yaml').read_text())
    assert head_manifest['source'] == 'draft'
    assert head_manifest['final_demo_runtime'] is False
    assert set(head_manifest['draft_components']) == {'servo'}
    assert not (store.unit_root(unit) / 'runtime').exists()

    with pytest.raises(ValueError, match='requires a passing head_camera'):
        store.render_draft_for(unit, 'right_handeye')
    store.save_component(
        unit, 'head_camera', deepcopy(components['head_camera'])
    )
    handeye_stage = store.render_draft_for(unit, 'right_handeye')
    handeye_geometry = yaml.safe_load(
        (handeye_stage / 'geometry.yaml').read_text()
    )
    assert handeye_geometry['sensors']['head_camera_xyz'].startswith('0.031 ')

    # Base fitting is independent and may run in parallel with the visual
    # chain; it becomes mandatory only for final grasp alignment/publication.
    store.save_component(
        unit, 'base_geometry', deepcopy(components['base_geometry'])
    )
    store.save_component(
        unit, 'right_handeye', deepcopy(components['right_handeye'])
    )
    alignment_stage = store.render_draft_for(unit, 'grasp_alignment')
    alignment_manifest = yaml.safe_load(
        (alignment_stage / 'manifest.yaml').read_text()
    )
    assert set(alignment_manifest['draft_components']) == {
        'servo', 'base_geometry', 'head_camera', 'right_handeye',
    }
    assert not (store.unit_root(unit) / 'runtime').exists()


def test_render_fails_closed_after_bundle_tampering(tmp_path, components):
    store = UnitCalibrationStore(tmp_path / '.xlerobot', REPO)
    for name, document in components.items():
        store.save_component('test-unit', name, deepcopy(document))
    version = store.activate('test-unit')
    assert 'T' in version and 'Z-' in version
    active = store.active_version('test-unit')
    assert active is not None
    with (active / 'components/base_geometry.yaml').open('a') as stream:
        stream.write('\n# changed after activation\n')
    with pytest.raises(ValueError, match='checksum failed'):
        store.render_active('test-unit')


def test_runtime_verification_detects_rendered_file_changes(tmp_path, components):
    store = UnitCalibrationStore(tmp_path / '.xlerobot', REPO)
    for name, document in components.items():
        store.save_component('test-unit', name, deepcopy(document))
    store.activate('test-unit', 'v1')
    runtime = store.render_active('test-unit')
    with (runtime / 'geometry.yaml').open('a') as stream:
        stream.write('\n# accidental local edit\n')
    with pytest.raises(ValueError, match='runtime checksum failed'):
        store.verify_runtime('test-unit')


def test_internal_runtime_gate_uses_local_config(
    tmp_path, components, capsys
):
    state_root = tmp_path / '.xlerobot'
    store = UnitCalibrationStore(state_root, REPO)
    for name, document in components.items():
        store.save_component('test-unit', name, deepcopy(document))
    store.activate('test-unit', 'v1')
    runtime = store.render_active('test-unit')
    config = tmp_path / 'local.yaml'
    config.write_text(yaml.safe_dump({
        'schema': 'xlerobot_demo/v1',
        'calibration': {
            'state_root': str(state_root),
            'unit': 'test-unit',
        },
    }))
    assert run_runtime_gate([
        '--repo-root', str(REPO), '--config', str(config),
    ]) == 0
    assert 'CALIBRATION RUNTIME OK' in capsys.readouterr().out

    with (runtime / 'geometry.yaml').open('a') as stream:
        stream.write('\n# changed\n')
    assert run_runtime_gate([
        '--repo-root', str(REPO), '--config', str(config),
    ]) == 2
    assert 'runtime checksum failed' in capsys.readouterr().err


def test_print_targets_are_committed_deterministic_pdf_and_svg(tmp_path):
    generator = (
        REPO
        / 'ros2_ws/src/xlerobot_calibration_tools/scripts/'
        'generate_calibration_targets.py'
    )
    completed = subprocess.run(
        [sys.executable, str(generator), '--output-dir', str(tmp_path)],
        text=True,
        capture_output=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    assets = REPO / 'assets/calibration_boards'
    names = (
        'head_4x4_ids_0-15_40mm.svg',
        'head_4x4_ids_0-15_40mm.pdf',
        'handeye_tag23_60mm.svg',
        'handeye_tag23_60mm.pdf',
    )
    for name in names:
        assert (tmp_path / name).read_bytes() == (assets / name).read_bytes()
    head_pdf = (assets / names[1]).read_bytes()
    handeye_pdf = (assets / names[3]).read_bytes()
    assert head_pdf.startswith(b'%PDF-1.4\n')
    assert b'/MediaBox [0 0 841.889764 595.275591]' in head_pdf
    assert b'/MediaBox [0 0 595.275591 841.889764]' in handeye_pdf
