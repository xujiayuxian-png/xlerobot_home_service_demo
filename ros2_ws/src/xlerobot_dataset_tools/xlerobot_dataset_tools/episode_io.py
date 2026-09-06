"""Atomic NPZ and dual-MP4 storage for ACT demonstration episodes."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path

import cv2
import numpy as np


SCHEMA = 'xlerobot_raw_episode/v1'
JOINT_NAMES = [
    'right_arm_shoulder_pan',
    'right_arm_shoulder_lift',
    'right_arm_elbow_flex',
    'right_arm_wrist_flex',
    'right_arm_wrist_roll',
    'right_arm_gripper',
]


def fsync_file(path: Path) -> None:
    with path.open('rb') as stream:
        os.fsync(stream.fileno())


def fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


class EpisodeWriter:
    """Write one immutable episode and publish it with one atomic rename."""

    def __init__(
        self, raw_root: Path, episode_id: str, *, instruction: str,
        object_label: str, profile_id: str, revision: str, unit_id: str,
        calibration_version: str, fps: int = 30,
    ):
        self.raw_root = raw_root
        self.episode_id = episode_id
        self.final = raw_root / episode_id
        self.work = raw_root / f'.incomplete-{episode_id}'
        self.manifest = {
            'schema': SCHEMA,
            'episode_id': episode_id,
            'collection_profile_id': profile_id,
            'language_instruction': instruction,
            'object_label': object_label,
            'fps': fps,
            'joint_names': JOINT_NAMES,
            'action_semantics': 'follower_next_state',
            'software_revision': revision,
            'unit_id': unit_id,
            'calibration_version': calibration_version,
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': 'recording',
            'teleop_enabled_at': '',
            'teleop_disabled_at': '',
        }
        self.fps = fps
        self.claimed = False
        self.states: list[np.ndarray] = []
        self.actions: list[np.ndarray] = []
        self.timestamps: list[float] = []
        self.previous: tuple[np.ndarray, dict[str, np.ndarray], float] | None = None
        self.videos: dict[str, cv2.VideoWriter] = {}

    def begin(self) -> None:
        legacy_incomplete = any(
            self.raw_root.glob(f'.incomplete-{self.episode_id}-*')
        )
        if self.final.exists() or self.work.exists() or legacy_incomplete:
            raise FileExistsError(f'episode already exists: {self.final}')
        self.raw_root.mkdir(parents=True, exist_ok=True)
        self.work.mkdir()
        self.claimed = True
        (self.work / 'videos').mkdir()
        self._write_manifest()
        fsync_directory(self.work / 'videos')
        fsync_directory(self.work)
        fsync_directory(self.raw_root)

    def mark_teleop_enabled(self) -> None:
        self.manifest['teleop_enabled_at'] = datetime.now(timezone.utc).isoformat()
        self._write_manifest()

    def mark_teleop_disabled(self) -> None:
        self.manifest['teleop_disabled_at'] = datetime.now(timezone.utc).isoformat()
        self._write_manifest()

    def mark_recording_stopped(self) -> None:
        self.manifest['recording_stopped_at'] = datetime.now(timezone.utc).isoformat()
        self._write_manifest()

    def append(self, state, images: dict[str, np.ndarray], timestamp: float) -> None:
        state = np.asarray(state, dtype=np.float64)
        if state.shape != (len(JOINT_NAMES),) or not np.isfinite(state).all():
            raise ValueError('Follower state must contain six finite joint positions')
        copied = {}
        for name in ('head', 'wrist'):
            image = np.asarray(images[name])
            if image.shape != (480, 640, 3):
                raise ValueError(f'{name} image must be 640x480 RGB')
            copied[name] = image.copy()
        if self.previous is None:
            self.previous = (state.copy(), copied, float(timestamp))
            return
        previous_state, previous_images, previous_timestamp = self.previous
        self.states.append(previous_state)
        self.actions.append(state.copy())
        self.timestamps.append(previous_timestamp)
        for name, image in previous_images.items():
            self._video(name).write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        self.previous = (state.copy(), copied, float(timestamp))

    def _video(self, name: str) -> cv2.VideoWriter:
        writer = self.videos.get(name)
        if writer is None:
            writer = cv2.VideoWriter(
                str(self.work / 'videos' / f'{name}.mp4'),
                cv2.VideoWriter_fourcc(*'mp4v'), float(self.fps), (640, 480),
            )
            if not writer.isOpened():
                raise RuntimeError(f'could not open {name} MP4 writer')
            self.videos[name] = writer
        return writer

    def finish(self, stop_reason: str) -> Path:
        for writer in self.videos.values():
            writer.release()
        self.videos.clear()
        if not self.manifest.get('teleop_enabled_at'):
            self.abort('teleop enable boundary is missing')
            raise RuntimeError('teleop enable boundary is missing')
        if not (self.manifest.get('recording_stopped_at') or self.manifest.get('teleop_disabled_at')):
            self.abort('teleop disable boundary is missing')
            raise RuntimeError('teleop disable boundary is missing')
        if not self.states:
            self.abort('no synchronized frame pairs')
            raise RuntimeError('episode contains no synchronized frame pairs')
        frame_count = len(self.states)
        np.savez(
            self.work / 'data.npz',
            observation_state=np.stack(self.states),
            action=np.stack(self.actions),
            timestamp=np.asarray(self.timestamps, dtype=np.float64),
            frame_index=np.arange(frame_count, dtype=np.int64),
        )
        required = [
            self.work / 'data.npz', self.work / 'videos/head.mp4',
            self.work / 'videos/wrist.mp4',
        ]
        if any(not path.is_file() or path.stat().st_size == 0 for path in required):
            self.abort('required output is missing or empty')
            raise RuntimeError('required episode output is missing or empty')
        for path in required:
            fsync_file(path)
        fsync_directory(self.work / 'videos')
        self.manifest.update({
            'status': 'complete', 'stop_reason': stop_reason,
            'frame_count': frame_count,
            'duration_s': float(self.timestamps[-1]),
            'files': {
                str(path.relative_to(self.work)): {
                    'size': path.stat().st_size, 'sha256': sha256(path),
                } for path in required
            },
        })
        self._write_manifest()
        fsync_directory(self.work)
        os.rename(self.work, self.final)
        fsync_directory(self.raw_root)
        return self.final

    def abort(self, reason: str) -> Path:
        for writer in self.videos.values():
            writer.release()
        self.videos.clear()
        if not self.claimed:
            return self.work
        self.manifest.update({'status': 'failed', 'stop_reason': reason})
        if self.work.is_dir():
            for path in self.work.rglob('*'):
                if path.is_file():
                    fsync_file(path)
            if (self.work / 'videos').is_dir():
                fsync_directory(self.work / 'videos')
            self._write_manifest()
            fsync_directory(self.work)
            fsync_directory(self.raw_root)
        return self.work

    def _write_manifest(self) -> None:
        temporary = self.work / 'manifest.json.tmp'
        with temporary.open('w', encoding='utf-8') as stream:
            stream.write(
                json.dumps(self.manifest, ensure_ascii=False, indent=2) + '\n'
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.work / 'manifest.json')
        fsync_directory(self.work)


def validate_episode(path: Path) -> dict:
    manifest = json.loads((path / 'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('schema') != SCHEMA or manifest.get('status') != 'complete':
        raise ValueError('episode is not a complete xlerobot_raw_episode/v1')
    if manifest.get('action_semantics') != 'follower_next_state':
        raise ValueError('episode action semantics do not match the ACT baseline')
    stop_boundary = ('recording_stopped_at' if manifest.get('recording_stopped_at')
                     else 'teleop_disabled_at')
    for boundary in ('teleop_enabled_at', stop_boundary):
        if not isinstance(manifest.get(boundary), str) or not manifest[boundary]:
            raise ValueError(f'episode is missing {boundary}')
    with np.load(path / 'data.npz') as data:
        state = data['observation_state']
        action = data['action']
        if state.shape != action.shape or state.ndim != 2 or state.shape[1] != 6:
            raise ValueError('episode state/action arrays must have shape (T, 6)')
    for relative, expected in manifest['files'].items():
        file_path = path / relative
        if sha256(file_path) != expected['sha256']:
            raise ValueError(f'checksum mismatch: {relative}')
    return manifest
