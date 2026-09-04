from pathlib import Path

import pytest
import yaml

from xlerobot_commissioning.calibration_assets import UnitCalibrationStore


def write(path: Path, document: dict) -> str:
    path.write_text(yaml.safe_dump(document), encoding='utf-8')
    return path.as_uri()


def transform(model: str, rmse: float = 2.0, maximum: float = 4.0) -> dict:
    return {
        'schema': 'xlerobot_transform_calibration/v1', 'model': model,
        'sample_count': 20,
        'metrics': {
            'reprojection_rmse_px': 0.4,
            'translation_rmse_mm': rmse, 'translation_max_mm': maximum,
            'rotation_rmse_deg': 0.5,
        },
        'x': {'translation': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]},
        'y': {'translation': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]},
    }


def populate(store: UnitCalibrationStore, tmp_path: Path) -> None:
    joint = {
        'servo_id': 1, 'direction': 1, 'offset': 2048,
        'raw_min': 100, 'raw_max': 3900,
        'limit_min': -1.0, 'limit_max': 1.0,
    }
    documents = {
        'servo': {
            'schema': 'xlerobot_servo_calibration/v1',
            'right_arm': {'joints': {
                name: {**joint, 'servo_id': index + 1}
                for index, name in enumerate((
                    'shoulder_pan', 'shoulder_lift', 'elbow_flex',
                    'wrist_flex', 'wrist_roll', 'gripper'))
            }},
            'left_arm': {'joints': {
                name: {**joint, 'servo_id': index + 1}
                for index, name in enumerate((
                    'shoulder_pan', 'shoulder_lift', 'elbow_flex',
                    'wrist_flex', 'wrist_roll', 'gripper'))
            }},
            'head': {'joints': {
                'pan': {**joint, 'servo_id': 7},
                'tilt': {**joint, 'servo_id': 8},
            }},
        },
        'base_geometry': {
            'schema': 'xlerobot_base_geometry/v1',
            'wheel_radius_m': 0.05, 'wheel_separation_m': 0.32,
            'straight_error_percent': 2.0, 'rotation_error_percent': -3.0,
        },
        'head_camera': transform('moving_camera_fixed_target'),
        'right_handeye': transform('fixed_camera_moving_target'),
    }
    for name, document in documents.items():
        _, quality, _ = store.import_result(
            'unit-001', name, write(tmp_path / f'{name}.yaml', document)
        )
        assert quality


def test_complete_quality_checked_bundle_can_activate_and_rollback(tmp_path: Path):
    store = UnitCalibrationStore(tmp_path / 'artifacts')
    populate(store, tmp_path)
    first = store.finalize_and_activate('unit-001', 'v1')
    second = store.finalize_and_activate('unit-001', 'v2')
    assert first.path.joinpath('components/servo.yaml').is_file()
    assert store.catalog.active('units', 'unit-001').version == 'v2'
    store.catalog.activate('units', 'unit-001', 'v1')
    assert store.catalog.active('units', 'unit-001').version == 'v1'


def test_failed_component_blocks_unit_activation(tmp_path: Path):
    store = UnitCalibrationStore(tmp_path / 'artifacts')
    populate(store, tmp_path)
    failed = transform('fixed_camera_moving_target', rmse=12.0, maximum=20.0)
    _, quality, _ = store.import_result(
        'unit-001', 'right_handeye', write(tmp_path / 'failed.yaml', failed)
    )
    assert not quality
    with pytest.raises(ValueError, match='failed quality'):
        store.finalize_and_activate('unit-001')


def test_base_geometry_measurements_fit_radius_and_separation(tmp_path: Path):
    store = UnitCalibrationStore(tmp_path / 'artifacts')
    draft, quality, metrics = store.save_base_geometry_measurements(
        'unit-001',
        nominal_radius=0.05,
        nominal_separation=0.32,
        straight_commanded=[1.0, 2.0],
        straight_actual=[0.9, 1.8],
        rotation_commanded=[6.0, 3.0],
        rotation_actual=[5.0, 2.5],
    )

    document = yaml.safe_load(
        draft.joinpath('components/base_geometry.yaml').read_text()
    )
    assert quality
    assert document['wheel_radius_m'] == pytest.approx(0.045)
    assert document['wheel_separation_m'] == pytest.approx(0.384)
    assert metrics['straight_error_percent'] == pytest.approx(0.0)
    assert metrics['rotation_error_percent'] == pytest.approx(0.0)
    assert document['fit']['straight_trials'][0] == {
        'commanded_m': 1.0,
        'actual_m': 0.9,
    }


@pytest.mark.parametrize(
    ('changes', 'message'),
    (
        ({'straight_actual': [1.0]}, 'two straight and two rotation'),
        ({'straight_actual': [1.0, 1.0, 1.0]}, 'lengths do not match'),
        ({'rotation_actual': [6.0, 0.0]}, 'finite and positive'),
        ({'nominal_radius': float('nan')}, 'finite and positive'),
    ),
)
def test_base_geometry_measurements_reject_invalid_input(
    tmp_path: Path, changes: dict, message: str
):
    store = UnitCalibrationStore(tmp_path / 'artifacts')
    values = {
        'nominal_radius': 0.05,
        'nominal_separation': 0.32,
        'straight_commanded': [1.0, 1.0],
        'straight_actual': [1.0, 1.0],
        'rotation_commanded': [6.0, 6.0],
        'rotation_actual': [6.0, 6.0],
    }
    values.update(changes)
    with pytest.raises(ValueError, match=message):
        store.save_base_geometry_measurements('unit-001', **values)
