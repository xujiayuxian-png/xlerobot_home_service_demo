"""Small filesystem-backed unit calibration bundles for source installs."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import re
import shutil
import tempfile
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .quality import validate_component
from .transforms import validate_transform
from .workflow import load_yaml, utc_now
from .runtime_import import FILES as IMPORT_FILES, SCHEMA as IMPORT_SCHEMA, validate_documents


COMPONENTS = (
    'servo',
    'base_geometry',
    'head_camera',
    'right_handeye',
    'grasp_alignment',
)
STRUCTURAL_COMPONENTS = COMPONENTS[:-1]
DRAFT_PREREQUISITES = {
    'head_camera': ('servo',),
    'right_handeye': ('servo', 'head_camera'),
    'grasp_alignment': STRUCTURAL_COMPONENTS,
}
_IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$')


def _id(value: str, label: str) -> str:
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise ValueError(f'{label} must match {_IDENTIFIER.pattern}')
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _atomic_yaml(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.tmp')
    try:
        temporary.write_text(
            yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
            encoding='utf-8',
        )
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _inside(child: Path, parent: Path) -> bool:
    try:
        child.relative_to(parent)
    except ValueError:
        return False
    return True


class UnitCalibrationStore:
    """Manage draft, immutable versions, active pointer, and rendered runtime."""

    def __init__(self, state_root: Path | str, repo_root: Path | str):
        self.state_root = Path(state_root).expanduser().resolve()
        self.repo_root = Path(repo_root).expanduser().resolve()

    def unit_root(self, unit: str) -> Path:
        return self.state_root / 'units' / _id(unit, 'unit')

    def draft_components(self, unit: str) -> Path:
        return self.unit_root(unit) / 'draft' / 'components'

    def save_component(
        self, unit: str, component: str, document: dict[str, Any]
    ) -> dict[str, float]:
        if component not in COMPONENTS:
            raise ValueError(f'unknown calibration component: {component}')
        metrics = validate_component(component, document)
        target = self.draft_components(unit) / f'{component}.yaml'
        _atomic_yaml(target, document)
        _atomic_yaml(
            self.unit_root(unit) / 'draft' / 'status.yaml',
            self.status(unit, include_active=False),
        )
        return metrics

    def status(self, unit: str, *, include_active: bool = True) -> dict[str, Any]:
        rows: dict[str, Any] = {}
        for component in COMPONENTS:
            path = self.draft_components(unit) / f'{component}.yaml'
            if not path.is_file():
                rows[component] = {'present': False, 'quality_passed': False}
                continue
            try:
                metrics = validate_component(component, load_yaml(path))
            except (KeyError, TypeError, ValueError, OSError) as error:
                rows[component] = {
                    'present': True,
                    'quality_passed': False,
                    'error': str(error),
                }
            else:
                rows[component] = {
                    'present': True,
                    'quality_passed': True,
                    'sha256': _sha256(path),
                    'metrics': metrics,
                }
        result: dict[str, Any] = {
            'schema': 'xlerobot_unit_calibration_status/v1',
            'unit': _id(unit, 'unit'),
            'draft_complete': all(row['quality_passed'] for row in rows.values()),
            'components': rows,
        }
        if include_active:
            try:
                active = self.active_version(unit)
            except (OSError, ValueError) as error:
                result['active'] = None
                result['active_error'] = str(error)
            else:
                result['active'] = active.name if active is not None else None
                runtime_manifest = self.unit_root(unit) / 'runtime/manifest.yaml'
                result['runtime_present'] = runtime_manifest.is_file()
                if runtime_manifest.is_file() and active is not None:
                    try:
                        self.verify_runtime(unit)
                    except (KeyError, OSError, TypeError, ValueError):
                        result['runtime_matches_active'] = False
                    else:
                        result['runtime_matches_active'] = True
                else:
                    result['runtime_matches_active'] = False
        return result

    def activate(self, unit: str, version: str = '') -> str:
        """Publish a complete passing draft, then atomically select it."""
        unit = _id(unit, 'unit')
        status = self.status(unit, include_active=False)
        if not status['draft_complete']:
            missing = [
                name for name, row in status['components'].items()
                if not row['quality_passed']
            ]
            raise ValueError(
                'unit calibration draft is incomplete or failed quality: '
                + ', '.join(missing)
            )
        digest = hashlib.sha256(''.join(
            status['components'][name]['sha256'] for name in COMPONENTS
        ).encode()).hexdigest()[:8]
        if not version:
            timestamp = utc_now().replace('+00:00', 'Z').replace('-', '').replace(':', '')
            version = timestamp + '-' + digest
        version = _id(version, 'version')
        unit_root = self.unit_root(unit)
        versions = unit_root / 'versions'
        versions.mkdir(parents=True, exist_ok=True)
        destination = versions / version
        if destination.exists() or destination.is_symlink():
            raise FileExistsError(f'calibration version already exists: {version}')
        staging = Path(tempfile.mkdtemp(prefix=f'.{version}-', dir=versions))
        try:
            components = staging / 'components'
            components.mkdir()
            manifest_components = {}
            for name in COMPONENTS:
                source = self.draft_components(unit) / f'{name}.yaml'
                target = components / f'{name}.yaml'
                shutil.copyfile(source, target)
                metrics = validate_component(name, load_yaml(target))
                manifest_components[name] = {
                    'path': f'components/{name}.yaml',
                    'sha256': _sha256(target),
                    'metrics': metrics,
                }
            manifest = {
                'schema': 'xlerobot_unit_calibration_bundle/v1',
                'unit': unit,
                'version': version,
                'created_at': utc_now(),
                'required_components': list(COMPONENTS),
                'components': manifest_components,
            }
            _atomic_yaml(staging / 'manifest.yaml', manifest)
            os.replace(staging, destination)
        finally:
            if staging.exists():
                shutil.rmtree(staging)
        self._select(unit, version)
        return version

    def rollback(self, unit: str, version: str) -> str:
        unit = _id(unit, 'unit')
        version = _id(version, 'version')
        self._verify_version(unit, version)
        self._select(unit, version)
        return version

    def import_runtime(self, unit: str, source: Path, version: str) -> str:
        """Activate a complete existing-unit snapshot, with no new quality claim."""
        unit, version = _id(unit, 'unit'), _id(version, 'version')
        documents = {name: load_yaml(source / name) for name in IMPORT_FILES}
        validate_documents(documents, self.repo_root)
        versions = self.unit_root(unit) / 'versions'
        versions.mkdir(parents=True, exist_ok=True)
        destination = versions / version
        if destination.exists() or destination.is_symlink():
            raise FileExistsError(f'calibration version already exists: {version}')
        staging = Path(tempfile.mkdtemp(prefix='.import-', dir=versions))
        try:
            for name, document in documents.items():
                _atomic_yaml(staging / name, document)
            manifest = {
                'schema': IMPORT_SCHEMA, 'unit': unit, 'version': version,
                'created_at': utc_now(), 'source': 'existing_unit_runtime',
                'quality_revalidated': False,
                'sha256': {name: _sha256(staging / name) for name in IMPORT_FILES},
            }
            _atomic_yaml(staging / 'manifest.yaml', manifest)
            os.replace(staging, destination)
        finally:
            if staging.exists():
                shutil.rmtree(staging)
        self._select(unit, version)
        return version

    def active_version(self, unit: str) -> Path | None:
        active = self.unit_root(unit) / 'active'
        if not active.exists() and not active.is_symlink():
            return None
        if not active.is_symlink():
            raise ValueError('active calibration pointer must be a symlink')
        resolved = active.resolve(strict=True)
        versions = (self.unit_root(unit) / 'versions').resolve()
        if not _inside(resolved, versions) or resolved.parent != versions:
            raise ValueError('active calibration pointer escapes the unit versions directory')
        return resolved

    def render_active(self, unit: str) -> Path:
        """Render only a complete, checksum-valid active bundle."""
        active = self.active_version(unit)
        if active is None:
            raise ValueError('unit has no active calibration bundle')
        manifest = self._verify_version(unit, active.name)
        if manifest['schema'] == IMPORT_SCHEMA:
            documents = {name: load_yaml(active / name) for name in IMPORT_FILES}
        else:
            component_documents = {
                name: load_yaml(active / manifest['components'][name]['path'])
                for name in COMPONENTS
            }
            documents = self._structural_runtime_documents(component_documents)
            documents['grasp_alignment.yaml'] = component_documents['grasp_alignment']
        runtime_manifest = {
            'schema': 'xlerobot_calibration_runtime/v1',
            'unit': unit,
            'active_version': active.name,
            'active_manifest_sha256': _sha256(active / 'manifest.yaml'),
            'rendered_at': utc_now(),
            'files': list(documents),
            'source': manifest.get('source', 'measured_bundle'),
        }
        documents['manifest.yaml'] = runtime_manifest
        runtime = self.unit_root(unit) / 'runtime'
        self._replace_runtime(runtime, documents)
        return runtime

    def render_draft_for(self, unit: str, workflow: str) -> Path:
        """Compose only preceding draft components for the next capture step."""
        unit = _id(unit, 'unit')
        if workflow not in DRAFT_PREREQUISITES:
            raise ValueError(f'unsupported staged calibration workflow: {workflow}')
        component_documents = {}
        component_hashes = {}
        for name in DRAFT_PREREQUISITES[workflow]:
            path = self.draft_components(unit) / f'{name}.yaml'
            if not path.is_file() or path.is_symlink():
                raise ValueError(
                    f'{workflow} requires a passing {name} draft first'
                )
            document = load_yaml(path)
            validate_component(name, document)
            component_documents[name] = document
            component_hashes[name] = _sha256(path)
        documents = self._structural_runtime_documents(component_documents)
        stage_manifest = {
            'schema': 'xlerobot_calibration_stage/v1',
            'unit': unit,
            'purpose': workflow,
            'source': 'draft',
            'draft_components': component_hashes,
            'rendered_at': utc_now(),
            'files': list(documents),
            'final_demo_runtime': False,
        }
        documents['manifest.yaml'] = stage_manifest
        runtime = self.unit_root(unit) / 'draft/runtime' / workflow
        self._replace_runtime(runtime, documents)
        return runtime

    def _structural_runtime_documents(
        self, component_documents: dict[str, dict[str, Any]]
    ) -> dict[str, dict[str, Any]]:
        """Merge a validated structural prefix with the reference profile."""
        geometry = load_yaml(
            self.repo_root
            / 'ros2_ws/src/xlerobot_description/config/'
            'two_wheel_reference_geometry.yaml'
        )
        controllers = load_yaml(
            self.repo_root / 'ros2_ws/src/xlerobot_bringup/config/platform_controllers.yaml'
        )
        servo = component_documents.get('servo')
        if servo is None:
            raise ValueError('runtime composition requires servo calibration')
        servos = {
            name: servo[name] for name in ('right_arm', 'left_arm', 'head')
        }
        base = component_documents.get('base_geometry')
        if base is not None:
            radius = float(base['wheel_radius_m'])
            separation = float(base['wheel_separation_m'])
            geometry['base']['wheel_radius'] = radius
            geometry['base']['wheel_separation'] = separation
            geometry['base']['left_wheel_xyz'] = (
                f'0 {separation / 2.0:.12g} {radius:.12g}'
            )
            geometry['base']['right_wheel_xyz'] = (
                f'0 {-separation / 2.0:.12g} {radius:.12g}'
            )
            controllers['base_controller']['ros__parameters'][
                'wheel_radius'
            ] = radius
            controllers['base_controller']['ros__parameters'][
                'wheel_separation'
            ] = separation
        head = component_documents.get('head_camera')
        if head is not None:
            (
                geometry['sensors']['head_camera_xyz'],
                geometry['sensors']['head_camera_rpy'],
            ) = self._pose_strings(head['x'])
        handeye = component_documents.get('right_handeye')
        if handeye is not None:
            (
                geometry['right_arm']['tag23_xyz'],
                geometry['right_arm']['tag23_rpy'],
            ) = self._pose_strings(handeye['y'])
        transforms = {
            'schema': 'xlerobot_runtime_transforms/v1',
            **(
                {'head_camera': head}
                if head is not None else {}
            ),
            **(
                {'right_handeye': handeye}
                if handeye is not None else {}
            ),
        }
        return {
            'geometry.yaml': geometry,
            'servos.yaml': servos,
            'controllers.yaml': controllers,
            'transforms.yaml': transforms,
        }

    def verify_runtime(self, unit: str) -> dict[str, Any]:
        """Verify the rendered files still match the selected active bundle."""
        unit = _id(unit, 'unit')
        active = self.active_version(unit)
        if active is None:
            raise ValueError('unit has no active calibration bundle')
        self._verify_version(unit, active.name)
        runtime = self.unit_root(unit) / 'runtime'
        if not runtime.is_dir() or runtime.is_symlink():
            raise ValueError('calibration runtime is missing or invalid')
        manifest_path = runtime / 'manifest.yaml'
        if not manifest_path.is_file() or manifest_path.is_symlink():
            raise ValueError('calibration runtime manifest is missing or invalid')
        manifest = load_yaml(manifest_path)
        if manifest.get('schema') != 'xlerobot_calibration_runtime/v1':
            raise ValueError('calibration runtime manifest has an unsupported schema')
        if manifest.get('unit') != unit or manifest.get('active_version') != active.name:
            raise ValueError('calibration runtime does not match the active version')
        if manifest.get('active_manifest_sha256') != _sha256(active / 'manifest.yaml'):
            raise ValueError('calibration runtime active-manifest checksum failed')
        expected = [
            'geometry.yaml', 'servos.yaml', 'controllers.yaml',
            'transforms.yaml', 'grasp_alignment.yaml',
        ]
        if manifest.get('files') != expected:
            raise ValueError('calibration runtime file list is incomplete')
        digests = manifest.get('sha256')
        if not isinstance(digests, dict) or set(digests) != set(expected):
            raise ValueError('calibration runtime checksums are incomplete')
        for name in expected:
            path = runtime / name
            if not path.is_file() or path.is_symlink():
                raise ValueError(f'calibration runtime file is missing: {name}')
            if digests[name] != _sha256(path):
                raise ValueError(f'calibration runtime checksum failed: {name}')
        return manifest

    def _verify_version(self, unit: str, version: str) -> dict[str, Any]:
        version_root = self.unit_root(unit) / 'versions' / _id(version, 'version')
        if not version_root.is_dir() or version_root.is_symlink():
            raise ValueError(f'calibration version does not exist: {version}')
        manifest = load_yaml(version_root / 'manifest.yaml')
        if manifest.get('schema') == IMPORT_SCHEMA:
            if manifest.get('unit') != unit or manifest.get('version') != version:
                raise ValueError('imported runtime identity does not match its path')
            digests = manifest.get('sha256', {})
            if set(digests) != set(IMPORT_FILES):
                raise ValueError('imported runtime checksums are incomplete')
            documents = {}
            for name in IMPORT_FILES:
                path = version_root / name
                if not path.is_file() or path.is_symlink() or _sha256(path) != digests[name]:
                    raise ValueError(f'imported runtime checksum failed: {name}')
                documents[name] = load_yaml(path)
            validate_documents(documents, self.repo_root)
            return manifest
        if manifest.get('schema') != 'xlerobot_unit_calibration_bundle/v1':
            raise ValueError('calibration bundle manifest has an unsupported schema')
        if manifest.get('unit') != unit or manifest.get('version') != version:
            raise ValueError('calibration bundle identity does not match its path')
        if manifest.get('required_components') != list(COMPONENTS):
            raise ValueError('calibration bundle required-components list is incomplete')
        entries = manifest.get('components')
        if not isinstance(entries, dict) or set(entries) != set(COMPONENTS):
            raise ValueError('calibration bundle component set is incomplete')
        for name in COMPONENTS:
            entry = entries[name]
            expected_path = f'components/{name}.yaml'
            if not isinstance(entry, dict) or entry.get('path') != expected_path:
                raise ValueError(f'invalid component path for {name}')
            path = version_root / expected_path
            if not path.is_file() or path.is_symlink():
                raise ValueError(f'calibration component is missing: {name}')
            if entry.get('sha256') != _sha256(path):
                raise ValueError(f'calibration component checksum failed: {name}')
            validate_component(name, load_yaml(path))
        return manifest

    def _select(self, unit: str, version: str) -> None:
        self._verify_version(unit, version)
        unit_root = self.unit_root(unit)
        runtime = unit_root / 'runtime'
        stale = unit_root / '.runtime.previous'
        if runtime.is_symlink() or stale.is_symlink():
            raise ValueError('runtime paths must not be symbolic links')
        temporary = unit_root / '.active.tmp'
        temporary.unlink(missing_ok=True)
        temporary.symlink_to(Path('versions') / version)
        os.replace(temporary, unit_root / 'active')
        # A consumer must never combine a newly selected bundle with files
        # rendered from the old one.  Preserve the previous generated runtime
        # off-path until the next successful render replaces it.
        if runtime.exists():
            if stale.exists():
                shutil.rmtree(stale)
            os.replace(runtime, stale)

    @staticmethod
    def _pose_strings(transform: dict[str, Any]) -> tuple[str, str]:
        if not isinstance(transform, dict) or 'matrix' not in transform:
            raise ValueError('runtime transform must contain a matrix')
        matrix = validate_transform(
            np.asarray(transform['matrix'], dtype=np.float64),
            'runtime transform',
        )
        xyz = matrix[:3, 3]
        rpy = Rotation.from_matrix(matrix[:3, :3]).as_euler('xyz')
        return (
            ' '.join(f'{float(value):.12g}' for value in xyz),
            ' '.join(f'{float(value):.12g}' for value in rpy),
        )

    @staticmethod
    def _replace_runtime(runtime: Path, documents: dict[str, dict[str, Any]]) -> None:
        runtime.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(tempfile.mkdtemp(prefix='.runtime-', dir=runtime.parent))
        backup = runtime.with_name(f'.{runtime.name}.previous')
        try:
            values = dict(documents)
            manifest = dict(values.pop('manifest.yaml'))
            for name, document in values.items():
                _atomic_yaml(staging / name, document)
            manifest['sha256'] = {
                name: _sha256(staging / name) for name in values
            }
            _atomic_yaml(staging / 'manifest.yaml', manifest)
            if runtime.is_symlink() or backup.is_symlink():
                raise ValueError('runtime paths must not be symbolic links')
            if backup.exists():
                shutil.rmtree(backup)
            moved_previous = False
            if runtime.exists():
                os.replace(runtime, backup)
                moved_previous = True
            try:
                os.replace(staging, runtime)
            except Exception:
                if moved_previous and backup.exists():
                    os.replace(backup, runtime)
                raise
            if backup.exists():
                shutil.rmtree(backup)
        finally:
            if staging.exists():
                shutil.rmtree(staging)
