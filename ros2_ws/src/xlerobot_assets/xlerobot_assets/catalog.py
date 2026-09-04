"""Immutable artifact directories with a rebuildable SQLite index."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sqlite3
import tarfile
import tempfile
from typing import Mapping


IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$')
KINDS = {'units', 'sites', 'datasets'}


def _identifier(value: str, label: str) -> str:
    value = str(value).strip()
    if not IDENTIFIER.fullmatch(value):
        raise ValueError(f'invalid {label}: {value!r}')
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


@dataclass(frozen=True)
class ArtifactRef:
    """One immutable artifact version."""

    kind: str
    artifact_id: str
    version: str
    path: Path

    @property
    def uri(self) -> str:
        return self.path.as_uri()


class ArtifactCatalog:
    """Create, activate, list, export, and reindex local artifacts."""

    def __init__(self, root: Path | str) -> None:
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / 'exports').mkdir(exist_ok=True)
        self.database = self.root / 'catalog.sqlite3'
        self._initialize_database()

    def _initialize_database(self) -> None:
        with sqlite3.connect(self.database) as connection:
            connection.execute(
                'CREATE TABLE IF NOT EXISTS artifacts ('
                'kind TEXT NOT NULL, artifact_id TEXT NOT NULL, version TEXT NOT NULL, '
                'uri TEXT NOT NULL, created_at TEXT NOT NULL, schema TEXT NOT NULL, '
                'PRIMARY KEY(kind, artifact_id, version))'
            )

    def _artifact_root(self, kind: str, artifact_id: str) -> Path:
        if kind not in KINDS:
            raise ValueError(f'unsupported artifact kind: {kind!r}')
        return self.root / kind / _identifier(artifact_id, 'artifact_id')

    def create(
        self,
        *,
        kind: str,
        artifact_id: str,
        schema: str,
        files: Mapping[str, Path | str | bytes],
        version: str = '',
        metadata: Mapping[str, object] | None = None,
    ) -> ArtifactRef:
        """Atomically publish a new immutable version and index it."""
        artifact_root = self._artifact_root(kind, artifact_id)
        artifact_root.mkdir(parents=True, exist_ok=True)
        if not version:
            version = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        version = _identifier(version, 'version')
        destination = artifact_root / version
        if destination.exists():
            raise FileExistsError(f'artifact version already exists: {destination}')
        staging = Path(tempfile.mkdtemp(prefix=f'.{version}.', dir=artifact_root))
        try:
            for relative, source in files.items():
                relative_path = Path(relative)
                if relative_path.is_absolute() or '..' in relative_path.parts:
                    raise ValueError(f'unsafe artifact relative path: {relative!r}')
                target = staging / relative_path
                target.parent.mkdir(parents=True, exist_ok=True)
                if isinstance(source, bytes):
                    target.write_bytes(source)
                elif isinstance(source, Path):
                    if source.is_dir():
                        shutil.copytree(source, target)
                    else:
                        shutil.copy2(source, target)
                else:
                    target.write_text(str(source), encoding='utf-8')
            checksums = {
                str(path.relative_to(staging)): _sha256(path)
                for path in sorted(staging.rglob('*')) if path.is_file()
            }
            created_at = datetime.now(timezone.utc).isoformat()
            manifest = {
                'schema': schema,
                'kind': kind,
                'artifact_id': artifact_id,
                'version': version,
                'created_at': created_at,
                'metadata': dict(metadata or {}),
                'checksums': checksums,
            }
            (staging / 'manifest.json').write_text(
                json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
                encoding='utf-8',
            )
            os.replace(staging, destination)
        except Exception:
            shutil.rmtree(staging, ignore_errors=True)
            raise
        reference = ArtifactRef(kind, artifact_id, version, destination)
        self._index(reference, created_at, schema)
        return reference

    def _index(self, reference: ArtifactRef, created_at: str, schema: str) -> None:
        with sqlite3.connect(self.database) as connection:
            connection.execute(
                'INSERT OR REPLACE INTO artifacts VALUES (?, ?, ?, ?, ?, ?)',
                (
                    reference.kind,
                    reference.artifact_id,
                    reference.version,
                    reference.uri,
                    created_at,
                    schema,
                ),
            )

    def activate(self, kind: str, artifact_id: str, version: str) -> ArtifactRef:
        reference = self.get(kind, artifact_id, version)
        artifact_root = self._artifact_root(kind, artifact_id)
        current = artifact_root / 'current'
        if current.exists() and not current.is_symlink():
            raise ValueError(f'artifact current pointer is not a symlink: {current}')
        active = artifact_root / 'active.json'
        temporary = active.with_suffix('.json.tmp')
        temporary.write_text(
            json.dumps({'version': version, 'uri': reference.uri}, indent=2) + '\n',
            encoding='utf-8',
        )
        current_temporary = artifact_root / f'.current.{os.getpid()}.tmp'
        current_temporary.symlink_to(version, target_is_directory=True)
        os.replace(current_temporary, current)
        os.replace(temporary, active)
        return reference

    def active(self, kind: str, artifact_id: str) -> ArtifactRef | None:
        active = self._artifact_root(kind, artifact_id) / 'active.json'
        if not active.exists():
            return None
        document = json.loads(active.read_text(encoding='utf-8'))
        return self.get(kind, artifact_id, document['version'])

    def get(self, kind: str, artifact_id: str, version: str) -> ArtifactRef:
        version = _identifier(version, 'version')
        path = self._artifact_root(kind, artifact_id) / version
        if not path.is_dir() or not (path / 'manifest.json').is_file():
            raise FileNotFoundError(f'artifact version not found: {path}')
        return ArtifactRef(kind, artifact_id, version, path)

    def versions(self, kind: str, artifact_id: str) -> list[ArtifactRef]:
        root = self._artifact_root(kind, artifact_id)
        if not root.exists():
            return []
        return [
            ArtifactRef(kind, artifact_id, path.name, path)
            for path in sorted(root.iterdir())
            if not path.is_symlink()
            and path.is_dir() and (path / 'manifest.json').is_file()
        ]

    def export(self, reference: ArtifactRef) -> Path:
        output = self.root / 'exports' / (
            f'{reference.kind}-{reference.artifact_id}-{reference.version}.tar.gz'
        )
        with tarfile.open(output, 'w:gz') as archive:
            archive.add(reference.path, arcname=reference.path.name)
        return output

    def rebuild_index(self) -> int:
        with sqlite3.connect(self.database) as connection:
            connection.execute('DELETE FROM artifacts')
        count = 0
        for kind in sorted(KINDS):
            kind_root = self.root / kind
            if not kind_root.exists():
                continue
            for manifest_path in kind_root.glob('*/*/manifest.json'):
                if manifest_path.parent.is_symlink():
                    continue
                document = json.loads(manifest_path.read_text(encoding='utf-8'))
                reference = ArtifactRef(
                    kind,
                    document['artifact_id'],
                    document['version'],
                    manifest_path.parent,
                )
                self._index(
                    reference, document['created_at'], document['schema']
                )
                count += 1
        return count
