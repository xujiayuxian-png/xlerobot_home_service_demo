import json
from pathlib import Path

import pytest
import yaml

from xlerobot_commissioning.site_assets import SiteDraftStore


def make_map(tmp_path: Path) -> Path:
    (tmp_path / 'source.pgm').write_bytes(b'P5\n2 2\n255\n\0\0\0\0')
    path = tmp_path / 'source.yaml'
    path.write_text(yaml.safe_dump({
        'image': 'source.pgm', 'resolution': 0.05,
        'origin': [0.0, 0.0, 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }))
    return path


def test_validated_draft_becomes_active_immutable_site_bundle(tmp_path: Path):
    store = SiteDraftStore(tmp_path / 'artifacts', software_revision='abc123')
    store.save_map('home', 'ground-floor', make_map(tmp_path))
    store.set_place(
        'home', 'table', frame_id='map', x=1.0, y=2.0, yaw=0.3,
        nav_offset_m=0.25, dock=True,
    )
    assert not store.ready('home')
    assert store.record_validation(
        'home', 'table', localization_passed=True, navigation_passed=True,
        dock_passed=True, position_error_m=0.02, yaw_error_rad=0.03,
        duration_s=12.0, message='passed',
    )
    reference = store.finalize_and_activate('home', version='v1')
    assert reference.path.joinpath('map.yaml').is_file()
    assert reference.path.joinpath('places.yaml').is_file()
    assert store.catalog.active('sites', 'home').version == 'v1'


def test_activation_rejects_unvalidated_or_incomplete_draft(tmp_path: Path):
    store = SiteDraftStore(tmp_path / 'artifacts')
    store.save_map('home', 'map', make_map(tmp_path))
    with pytest.raises(ValueError, match='incomplete'):
        store.finalize_and_activate('home')


def test_map_yaml_must_reference_an_existing_supported_image(tmp_path: Path):
    path = make_map(tmp_path)
    document = yaml.safe_load(path.read_text())
    document['image'] = '../missing.pgm'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='next to'):
        SiteDraftStore(tmp_path / 'artifacts').save_map('home', 'map', path)


def test_named_place_can_be_removed_without_leaving_validation(tmp_path: Path):
    store = SiteDraftStore(tmp_path / 'artifacts')
    store.save_map('home', 'ground', make_map(tmp_path))
    for place in ('table', 'person'):
        store.set_place(
            'home', place, frame_id='map', x=1.0, y=2.0, yaw=0.0,
            nav_offset_m=0.0, dock=False,
        )
        store.record_validation(
            'home', place, localization_passed=True,
            navigation_passed=True, dock_passed=False,
            position_error_m=0.0, yaw_error_rad=0.0,
            duration_s=1.0, message='ok',
        )
    store.remove_place('home', 'person')
    assert 'person' not in store._places(store.draft('home'))
    validation = json.loads(
        (store.draft('home') / 'validation.json').read_text(encoding='utf-8')
    )
    assert 'person' not in validation['places']
    assert store.ready('home')
