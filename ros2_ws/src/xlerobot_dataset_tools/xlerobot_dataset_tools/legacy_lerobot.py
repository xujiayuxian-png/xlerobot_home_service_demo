"""Audited conversion of product or legacy NPZ/MP4 episodes to LeRobot v3."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import uuid

import numpy as np


JOINT_NAMES = [
    'right_arm_shoulder_pan',
    'right_arm_shoulder_lift',
    'right_arm_elbow_flex',
    'right_arm_wrist_flex',
    'right_arm_wrist_roll',
    'right_arm_gripper',
]
LEGACY_FORMAT = 'xlerobot_legacy_npz_mp4/v0'
INTERCHANGE_FORMAT = 'xlerobot_episode_interchange/v1'
RAW_FORMAT = 'xlerobot_raw_episode/v1'
CONVERSION_FORMAT = 'xlerobot_lerobot_conversion/v1'


@dataclass(frozen=True)
class EpisodeContract:
    """Validated arrays and metadata for one source episode."""

    name: str
    instruction: str
    fps: int
    states: np.ndarray
    actions: np.ndarray
    timestamps: np.ndarray
    head_video: Path
    wrist_video: Path
    action_provenance: str
    source_format: str

    @property
    def frame_count(self) -> int:
        return int(self.states.shape[0])


def supervised_arrays(
    states: np.ndarray,
    recorded_actions: np.ndarray,
    timestamps: np.ndarray,
    *,
    action_mode: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    """Select stored targets or reconstruct the verified legacy next-state target."""
    states = np.asarray(states)
    recorded_actions = np.asarray(recorded_actions)
    timestamps = np.asarray(timestamps)
    if states.ndim != 2 or states.shape[1] != len(JOINT_NAMES):
        raise ValueError('observation_state must have shape (frames, 6)')
    if recorded_actions.shape != states.shape:
        raise ValueError('action must have the same shape as observation_state')
    if timestamps.shape != (states.shape[0],):
        raise ValueError('timestamp must have one value per state frame')
    if states.shape[0] < 2:
        raise ValueError('episode needs at least two frames')
    if not (
        np.isfinite(states).all()
        and np.isfinite(recorded_actions).all()
        and np.isfinite(timestamps).all()
    ):
        raise ValueError('episode arrays contain nonfinite values')
    if not np.all(np.diff(timestamps) > 0.0):
        raise ValueError('timestamps must be strictly increasing')

    if action_mode == 'recorded':
        return (
            states.astype(np.float32),
            recorded_actions.astype(np.float32),
            timestamps.astype(np.float64),
            'recorded_action',
        )
    if action_mode == 'legacy_next_state':
        return (
            states[:-1].astype(np.float32),
            states[1:].astype(np.float32),
            timestamps[:-1].astype(np.float64),
            'follower_next_state_reconstructed',
        )
    raise ValueError(f'unsupported action mode: {action_mode}')


def load_episode(path: Path, *, action_mode: str) -> EpisodeContract:
    """Load and validate one immutable source episode directory."""
    try:
        metadata_path = path / 'manifest.json'
        if not metadata_path.is_file():
            metadata_path = path / 'meta.json'
        metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        with np.load(path / 'data.npz') as data:
            required = {'observation_state', 'action', 'timestamp', 'frame_index'}
            missing = sorted(required - set(data.files))
            if missing:
                raise ValueError(f'missing NPZ field(s): {", ".join(missing)}')
            states, actions, timestamps, provenance = supervised_arrays(
                data['observation_state'],
                data['action'],
                data['timestamp'],
                action_mode=action_mode,
            )
            frame_index = np.asarray(data['frame_index'])
    except (OSError, json.JSONDecodeError, KeyError) as exc:
        raise ValueError(f'invalid source episode: {path}') from exc
    expected_index = np.arange(frame_index.shape[0])
    if frame_index.shape != expected_index.shape or not np.array_equal(
        frame_index, expected_index
    ):
        raise ValueError(f'{path.name}: frame_index is not contiguous from zero')
    if list(metadata.get('joint_names', [])) != JOINT_NAMES:
        raise ValueError(f'{path.name}: joint order does not match the reference profile')
    fps_value = float(metadata.get('fps', 0.0))
    fps = round(fps_value)
    if fps <= 0 or not math.isclose(fps_value, fps, abs_tol=1.0e-3):
        raise ValueError(f'{path.name}: fps must be a positive integer rate')
    instruction = str(metadata.get('language_instruction', '')).strip()
    if not instruction:
        raise ValueError(f'{path.name}: language instruction is empty')
    head_video = path / 'videos' / 'head.mp4'
    wrist_video = path / 'videos' / 'wrist.mp4'
    if not head_video.is_file() or not wrist_video.is_file():
        raise ValueError(f'{path.name}: both legacy MP4 files are required')
    source_format = str(metadata.get('schema', metadata.get('format', LEGACY_FORMAT)))
    if source_format not in {LEGACY_FORMAT, INTERCHANGE_FORMAT, RAW_FORMAT}:
        raise ValueError(f'{path.name}: unsupported source format {source_format!r}')
    if action_mode == 'legacy_next_state' and source_format != LEGACY_FORMAT:
        raise ValueError('legacy_next_state is allowed only for legacy NPZ/MP4 data')
    if source_format == INTERCHANGE_FORMAT:
        provenance = str(metadata.get('action_provenance', {}).get('kind', ''))
        if not provenance.startswith('recorded_'):
            raise ValueError(f'{path.name}: interchange action provenance is invalid')
    if source_format == RAW_FORMAT:
        if metadata.get('status') != 'complete':
            raise ValueError(f'{path.name}: raw episode is not complete')
        if metadata.get('action_semantics') != 'follower_next_state':
            raise ValueError(f'{path.name}: raw action semantics are invalid')
        if states.shape[0] > 1 and not np.allclose(
            actions[:-1], states[1:], rtol=0.0, atol=1.0e-6
        ):
            raise ValueError(
                f'{path.name}: action[t] does not match follower_state[t+1]'
            )
        provenance = 'follower_next_state'
    return EpisodeContract(
        name=path.name,
        instruction=instruction,
        fps=fps,
        states=states,
        actions=actions,
        timestamps=timestamps,
        head_video=head_video,
        wrist_video=wrist_video,
        action_provenance=provenance,
        source_format=source_format,
    )


def discover_episodes(
    source_root: Path,
    *,
    action_mode: str,
    limit: int = 0,
) -> list[EpisodeContract]:
    """Load a stable sorted set of episode directories."""
    paths = sorted(
        path for path in source_root.iterdir()
        if path.is_dir() and not path.name.startswith('.')
        and (path / 'data.npz').is_file()
    )
    if limit > 0:
        paths = paths[:limit]
    if not paths:
        raise ValueError(f'no complete episode directories found under {source_root}')
    episodes = [load_episode(path, action_mode=action_mode) for path in paths]
    fps_values = {episode.fps for episode in episodes}
    if len(fps_values) != 1:
        raise ValueError(f'episodes have mixed FPS values: {sorted(fps_values)}')
    return episodes


def video_properties(path: Path) -> tuple[int, int, int]:
    """Return frame count, width, and height through OpenCV."""
    import cv2

    capture = cv2.VideoCapture(str(path))
    try:
        if not capture.isOpened():
            raise ValueError(f'could not open video: {path}')
        frames = round(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        width = round(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    finally:
        capture.release()
    if frames <= 0 or width <= 0 or height <= 0:
        raise ValueError(f'invalid video properties: {path}')
    return frames, width, height


def conversion_plan(episodes: list[EpisodeContract]) -> dict:
    """Validate video coverage and return a JSON-compatible conversion plan."""
    entries = []
    image_size = None
    for episode in episodes:
        head = video_properties(episode.head_video)
        wrist = video_properties(episode.wrist_video)
        if head[1:] != wrist[1:]:
            raise ValueError(f'{episode.name}: head and wrist image sizes differ')
        if min(head[0], wrist[0]) < episode.frame_count:
            raise ValueError(
                f'{episode.name}: video is shorter than supervised arrays '
                f'({head[0]}, {wrist[0]} < {episode.frame_count})'
            )
        if image_size is None:
            image_size = head[1:]
        elif head[1:] != image_size:
            raise ValueError(f'{episode.name}: image size differs across episodes')
        entries.append(
            {
                'episode': episode.name,
                'frames': episode.frame_count,
                'raw_head_frames': head[0],
                'raw_wrist_frames': wrist[0],
                'action_provenance': episode.action_provenance,
            }
        )
    source_formats = sorted({episode.source_format for episode in episodes})
    return {
        'format': CONVERSION_FORMAT,
        'source_format': source_formats[0] if len(source_formats) == 1 else 'mixed',
        'episode_count': len(episodes),
        'frame_count': sum(episode.frame_count for episode in episodes),
        'fps': episodes[0].fps,
        'image_width': image_size[0],
        'image_height': image_size[1],
        'joint_names': JOINT_NAMES,
        'episodes': entries,
    }


def _frames(path: Path, count: int):
    import cv2

    capture = cv2.VideoCapture(str(path))
    try:
        for index in range(count):
            ok, image = capture.read()
            if not ok:
                raise ValueError(f'{path}: failed to decode frame {index}')
            yield cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    finally:
        capture.release()


def convert(
    episodes: list[EpisodeContract],
    *,
    output_root: Path,
    repo_id: str,
) -> Path:
    """Atomically build and finalize one LeRobot v3 dataset."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    plan = conversion_plan(episodes)
    output_root = output_root.expanduser().resolve()
    if output_root.exists():
        raise FileExistsError(f'output already exists: {output_root}')
    output_root.parent.mkdir(parents=True, exist_ok=True)
    work_root = output_root.parent / f'.{output_root.name}.incomplete-{uuid.uuid4().hex[:8]}'
    features = {
        'observation.state': {
            'dtype': 'float32',
            'shape': (len(JOINT_NAMES),),
            'names': JOINT_NAMES,
        },
        'action': {
            'dtype': 'float32',
            'shape': (len(JOINT_NAMES),),
            'names': JOINT_NAMES,
        },
        'observation.images.head': {
            'dtype': 'video',
            'shape': (3, plan['image_height'], plan['image_width']),
            'names': ['channels', 'height', 'width'],
        },
        'observation.images.wrist': {
            'dtype': 'video',
            'shape': (3, plan['image_height'], plan['image_width']),
            'names': ['channels', 'height', 'width'],
        },
    }
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=plan['fps'],
        root=work_root,
        robot_type='xlerobot_two_wheel_reference',
        features=features,
        use_videos=True,
    )
    try:
        for episode in episodes:
            head_frames = _frames(episode.head_video, episode.frame_count)
            wrist_frames = _frames(episode.wrist_video, episode.frame_count)
            for index, (head, wrist) in enumerate(zip(head_frames, wrist_frames)):
                dataset.add_frame(
                    {
                        'observation.state': episode.states[index],
                        'action': episode.actions[index],
                        'observation.images.head': head,
                        'observation.images.wrist': wrist,
                        'task': episode.instruction,
                    }
                )
            dataset.save_episode()
        dataset.finalize()
        (work_root / 'xlerobot_conversion.json').write_text(
            json.dumps(
                {
                    **plan,
                    'repo_id': repo_id,
                    'action_semantics': 'follower_next_state',
                },
                indent=2,
                ensure_ascii=False,
            )
            + '\n',
            encoding='utf-8',
        )
        work_root.rename(output_root)
    except Exception as exc:
        raise RuntimeError(
            f'conversion failed ({type(exc).__name__}: {exc}); '
            f'incomplete output kept at {work_root}'
        ) from exc
    return output_root


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Convert audited product or legacy NPZ/MP4 data to LeRobot v3.'
    )
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--repo-id', required=True)
    parser.add_argument(
        '--action-mode',
        choices=('recorded', 'legacy_next_state'),
        default='recorded',
    )
    parser.add_argument('--limit-episodes', type=int, default=0)
    parser.add_argument('--dry-run', action='store_true')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.limit_episodes < 0:
        raise SystemExit('--limit-episodes cannot be negative')
    try:
        episodes = discover_episodes(
            args.source_root.expanduser().resolve(),
            action_mode=args.action_mode,
            limit=args.limit_episodes,
        )
        plan = conversion_plan(episodes)
        if args.dry_run:
            print(json.dumps(plan, indent=2, ensure_ascii=False))
            return 0
        path = convert(episodes, output_root=args.output_root, repo_id=args.repo_id)
    except (FileExistsError, RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
    print(path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
