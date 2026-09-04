"""Mutable site drafts that atomically become immutable site bundles."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
from typing import Any

from xlerobot_assets import ArtifactCatalog, ArtifactRef
import yaml


IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$')


def _id(value: str, label: str) -> str:
    value = str(value).strip()
    if not IDENTIFIER.fullmatch(value):
        raise ValueError(f'invalid {label}: {value!r}')
    return value


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _atomic_yaml(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding='utf-8',
    )
    os.replace(temporary, path)


def _atomic_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary, path)


class SiteDraftStore:
    """Own site staging data separately from immutable catalog versions."""

    def __init__(
        self,
        artifact_root: Path | str,
        *,
        software_revision: str = 'development',
        operator: str = 'technician',
    ) -> None:
        self.catalog = ArtifactCatalog(artifact_root)
        self.root = self.catalog.root / 'sites' / '.drafts'
        self.root.mkdir(parents=True, exist_ok=True)
        self.software_revision = software_revision
        self.operator = operator

    def draft(self, site_id: str) -> Path:
        return self.root / _id(site_id, 'site_id')

    def _ensure_session(self, site_id: str) -> Path:
        draft = self.draft(site_id)
        draft.mkdir(parents=True, exist_ok=True)
        session = draft / 'mapping_session.json'
        if not session.exists():
            _atomic_json(session, {
                'schema': 'xlerobot_mapping_session/v1',
                'site_id': site_id,
                'started_at': _utc_now(),
                'operator': self.operator,
                'software_revision': self.software_revision,
                'status': 'staging',
            })
        return draft

    def save_map(self, site_id: str, map_name: str, source_yaml: Path | str) -> Path:
        """Validate and copy a map YAML and its image into a site draft."""
        site_id = _id(site_id, 'site_id')
        map_name = _id(map_name, 'map_name')
        source_yaml = Path(source_yaml).expanduser().resolve()
        if not source_yaml.is_file():
            raise FileNotFoundError(f'map YAML does not exist: {source_yaml}')
        document = yaml.safe_load(source_yaml.read_text(encoding='utf-8')) or {}
        if not isinstance(document, dict) or not document.get('image'):
            raise ValueError('map YAML must contain an image field')
        for key in ('resolution', 'origin'):
            if key not in document:
                raise ValueError(f'map YAML is missing {key}')
        image = (source_yaml.parent / str(document['image'])).resolve()
        if not image.is_relative_to(source_yaml.parent):
            raise ValueError('map image must be next to the map YAML')
        if not image.is_file():
            raise FileNotFoundError(f'map image does not exist: {image}')
        if image.suffix.lower() not in {'.pgm', '.png'}:
            raise ValueError('site maps must use PGM or PNG images')

        draft = self._ensure_session(site_id)
        image_name = f'map{image.suffix.lower()}'
        staging = Path(tempfile.mkdtemp(prefix='.map.', dir=draft))
        try:
            shutil.copy2(image, staging / image_name)
            document['image'] = image_name
            (staging / 'map.yaml').write_text(
                yaml.safe_dump(document, sort_keys=False), encoding='utf-8'
            )
            for old in draft.glob('map.*'):
                old.unlink()
            os.replace(staging / image_name, draft / image_name)
            os.replace(staging / 'map.yaml', draft / 'map.yaml')
        finally:
            shutil.rmtree(staging, ignore_errors=True)
        session = json.loads((draft / 'mapping_session.json').read_text())
        session.update({'map_name': map_name, 'map_saved_at': _utc_now()})
        _atomic_json(draft / 'mapping_session.json', session)
        return draft

    def set_place(
        self,
        site_id: str,
        place_id: str,
        *,
        frame_id: str,
        x: float,
        y: float,
        yaw: float,
        nav_offset_m: float,
        dock: bool,
    ) -> Path:
        """Create or replace one named place in the staging bundle."""
        site_id = _id(site_id, 'site_id')
        place_id = _id(place_id, 'place_id')
        if not frame_id.strip() or nav_offset_m < 0.0:
            raise ValueError('place frame must be set and nav offset must be nonnegative')
        values = [float(x), float(y), float(yaw), float(nav_offset_m)]
        if not all(value == value and abs(value) != float('inf') for value in values):
            raise ValueError('place values must be finite')
        draft = self._ensure_session(site_id)
        path = draft / 'places.yaml'
        document = yaml.safe_load(path.read_text()) if path.exists() else None
        if not isinstance(document, dict):
            document = {'named_places': {'frame_id': 'map', 'places': {}}}
        root = document.setdefault('named_places', {})
        places = root.setdefault('places', {})
        places[place_id] = {
            'frame_id': frame_id,
            'x': values[0], 'y': values[1], 'yaw': values[2],
            'nav_offset_m': values[3], 'dock': bool(dock),
        }
        _atomic_yaml(path, document)
        validation = draft / 'validation.json'
        if validation.exists():
            state = json.loads(validation.read_text())
            state.setdefault('places', {}).pop(place_id, None)
            state['passed'] = False
            _atomic_json(validation, state)
        return draft

    def remove_place(self, site_id: str, place_id: str) -> Path:
        """Remove one named place and any validation tied to that pose."""
        site_id = _id(site_id, 'site_id')
        place_id = _id(place_id, 'place_id')
        draft = self.draft(site_id)
        path = draft / 'places.yaml'
        if not path.is_file():
            raise KeyError(f'unknown named place: {place_id}')
        document = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        places = document.get('named_places', {}).get('places', {})
        if place_id not in places:
            raise KeyError(f'unknown named place: {place_id}')
        del places[place_id]
        _atomic_yaml(path, document)
        validation = draft / 'validation.json'
        if validation.exists():
            state = json.loads(validation.read_text(encoding='utf-8'))
            state.setdefault('places', {}).pop(place_id, None)
            state['passed'] = self._validation_complete(places, state)
            state['updated_at'] = _utc_now()
            _atomic_json(validation, state)
        return draft

    def record_validation(
        self,
        site_id: str,
        place_id: str,
        *,
        localization_passed: bool,
        navigation_passed: bool,
        dock_passed: bool,
        position_error_m: float,
        yaw_error_rad: float,
        duration_s: float,
        message: str,
    ) -> bool:
        draft = self.draft(site_id)
        places = self._places(draft)
        if place_id not in places:
            raise KeyError(f'unknown named place: {place_id}')
        path = draft / 'validation.json'
        document = json.loads(path.read_text()) if path.exists() else {
            'schema': 'xlerobot_site_validation/v1', 'site_id': site_id, 'places': {}
        }
        passed = bool(localization_passed and navigation_passed)
        if places[place_id].get('dock', False):
            passed = passed and bool(dock_passed)
        document['places'][place_id] = {
            'localization_passed': bool(localization_passed),
            'navigation_passed': bool(navigation_passed),
            'dock_passed': bool(dock_passed),
            'position_error_m': float(position_error_m),
            'yaw_error_rad': float(yaw_error_rad),
            'duration_s': float(duration_s),
            'message': str(message),
            'validated_at': _utc_now(),
            'passed': passed,
        }
        document['passed'] = self._validation_complete(places, document)
        document['updated_at'] = _utc_now()
        _atomic_json(path, document)
        return bool(document['passed'])

    @staticmethod
    def _places(draft: Path) -> dict[str, Any]:
        path = draft / 'places.yaml'
        if not path.is_file():
            return {}
        document = yaml.safe_load(path.read_text()) or {}
        return document.get('named_places', {}).get('places', {})

    @staticmethod
    def _validation_complete(places: dict[str, Any], validation: dict[str, Any]) -> bool:
        results = validation.get('places', {})
        return bool(places) and all(
            place_id in results and results[place_id].get('passed', False)
            for place_id in places
        )

    def ready(self, site_id: str) -> bool:
        draft = self.draft(site_id)
        if not (draft / 'map.yaml').is_file():
            return False
        places = self._places(draft)
        validation_path = draft / 'validation.json'
        if not validation_path.is_file():
            return False
        return self._validation_complete(
            places, json.loads(validation_path.read_text())
        )

    def finalize_and_activate(self, site_id: str, version: str = '') -> ArtifactRef:
        """Publish one validated draft and atomically make it active."""
        site_id = _id(site_id, 'site_id')
        draft = self.draft(site_id)
        if not self.ready(site_id):
            raise ValueError('site draft is incomplete or has unvalidated places')
        map_images = list(draft.glob('map.pgm')) + list(draft.glob('map.png'))
        if len(map_images) != 1:
            raise ValueError('site draft must contain exactly one map image')
        session = json.loads((draft / 'mapping_session.json').read_text())
        session.update({'status': 'finalized', 'finalized_at': _utc_now()})
        _atomic_json(draft / 'mapping_session.json', session)
        reference = self.catalog.create(
            kind='sites', artifact_id=site_id, schema='xlerobot_site/v1',
            version=version,
            files={
                'map.yaml': draft / 'map.yaml',
                map_images[0].name: map_images[0],
                'places.yaml': draft / 'places.yaml',
                'mapping_session.json': draft / 'mapping_session.json',
                'validation.json': draft / 'validation.json',
            },
            metadata={
                'software_revision': self.software_revision,
                'operator': self.operator,
            },
        )
        return self.catalog.activate('sites', site_id, reference.version)
