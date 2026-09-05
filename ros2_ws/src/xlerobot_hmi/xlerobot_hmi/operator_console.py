"""
Asynchronous web gateway for the XLeRobot Operator Console.

The gateway deliberately translates web protocols to stable ROS interfaces.  It
does not implement task, mapping, calibration, or collection state machines.
"""

from __future__ import annotations

import asyncio
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import secrets
import shutil
import sqlite3
import threading
import time
from typing import Any

from action_msgs.msg import GoalInfo, GoalStatus
from action_msgs.srv import CancelGoal
from aiohttp import web
from ament_index_python.packages import get_package_share_directory
import cv2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as NavigationPath
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from slam_toolbox.srv import Reset, SaveMap
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from xlerobot_assets import ArtifactCatalog
from xlerobot_hmi.manual_control import ManualControlCoordinator
from xlerobot_interfaces.action import (
    AutoLocalize,
    CalibrationJob,
    CollectEpisode,
    ExecuteTask,
    MoveCalibrationPose,
    NavigateToNamedPlace,
    SetMaintenancePreset,
)
from xlerobot_interfaces.msg import (
    CapabilityError,
    PerceptionObservation,
    TaskEvent,
)
from xlerobot_interfaces.srv import (
    ActivateArtifact,
    CaptureCalibrationSample,
    FinalizeEpisode,
    ImportCalibrationResult,
    SaveBaseGeometryCalibration,
    ServoCalibrationStep,
    SolveCalibrationSamples,
)
from xlerobot_interfaces.srv import RecordSiteValidation, ReviewEpisode
from xlerobot_interfaces.srv import RemoveNamedPlace, SaveSiteMap, SetNamedPlace
import yaml


def utc_now() -> str:
    """Return a stable UTC timestamp for persisted records."""
    return datetime.now(timezone.utc).isoformat()


def stamp_utc(stamp) -> str:
    """Convert a ROS timestamp to the persisted UTC representation."""
    seconds = int(stamp.sec) + int(stamp.nanosec) * 1.0e-9
    if seconds <= 0.0:
        return utc_now()
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat()


def observation_dict(message: PerceptionObservation) -> dict[str, Any] | None:
    """Convert a typed perception result to the bounded browser contract."""
    if not message.kind or not message.observation_id:
        return None
    return {
        'observation_id': message.observation_id,
        'kind': message.kind,
        'label': message.label,
        'camera_id': message.camera_id,
        'image_width': int(message.image_width),
        'image_height': int(message.image_height),
        'bbox': [
            float(message.bbox_x1), float(message.bbox_y1),
            float(message.bbox_x2), float(message.bbox_y2),
        ],
        'confidence': float(message.confidence),
        'target': {
            'frame_id': message.target.header.frame_id,
            'x': float(message.target.point.x),
            'y': float(message.target.point.y),
            'z': float(message.target.point.z),
        },
    }


IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$')


def _uint8(value: Any) -> int:
    """Normalize ROS Python uint8 fields across bytes/int generators."""
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError('uint8 byte field must have length one')
        return value[0]
    return int(value)


def _diagnostic_level_for_readiness(name: str, item: dict[str, Any]) -> int:
    """Map non-fatal ros2_control timing alarms to degraded readiness."""
    level = _uint8(item.get('level', DiagnosticStatus.OK))
    message = str(item.get('message', ''))
    timing_status = name in {
        'controller_manager: Controllers Activity',
        'controller_manager: Hardware Components Activity',
    }
    if (
        level >= _uint8(DiagnosticStatus.ERROR)
        and timing_status
        and 'High execution jitter or mean error' in message
    ):
        return _uint8(DiagnosticStatus.WARN)
    return level


def parse_task_request(payload: Any) -> ExecuteTask.Goal:
    """Validate an API request and map it to the stable task contract."""
    if not isinstance(payload, dict):
        raise ValueError('request body must be a JSON object')
    fields = {
        'object_id': str(payload.get('object_id', '')).strip(),
        'source_place': str(payload.get('source_place', 'table')).strip(),
        'recipient_id': str(payload.get('recipient_id', 'nearest_person')).strip(),
        'grasp_backend': str(payload.get('grasp_backend', '')).strip(),
    }
    empty = [name for name, value in fields.items() if not value]
    if empty:
        raise ValueError(f'task fields must not be empty: {empty}')
    goal = ExecuteTask.Goal()
    goal.object_id = fields['object_id']
    goal.source_place = fields['source_place']
    goal.recipient_id = fields['recipient_id']
    if fields['grasp_backend'] not in {'act', 'centroid', 'gpd'}:
        raise ValueError('grasp_backend must be one of: act, centroid, gpd')
    goal.grasp_backend = fields['grasp_backend']
    goal.dry_run = bool(payload.get('dry_run', False))
    return goal


def available_workspaces(configured_workspace: str) -> list[str]:
    """Expose only the workspace owned by the active launch profile."""
    workspace = str(configured_workspace).strip()
    if workspace in {'operator', 'mapping', 'calibration', 'collection'}:
        return [workspace]
    return []


def map_visualization_enabled(configured_workspace: str) -> bool:
    """Allow demo observation without exposing mapping teleoperation."""
    return str(configured_workspace).strip() in {'operator', 'mapping'}


def map_qos_profile() -> QoSProfile:
    """Match the latched QoS used by Nav2 map_server and SLAM Toolbox."""
    profile = QoSProfile(depth=1)
    profile.reliability = ReliabilityPolicy.RELIABLE
    profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return profile


def transient_state_qos_profile(depth: int = 1) -> QoSProfile:
    """Receive the latest latched product state after an HMI restart."""
    profile = QoSProfile(depth=depth)
    profile.reliability = ReliabilityPolicy.RELIABLE
    profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return profile


def goal_info_from_task_id(task_id: str) -> GoalInfo:
    """Map the public TaskEvent UUID into the standard action cancel contract."""
    normalized = str(task_id).strip().lower()
    if not re.fullmatch(r'[0-9a-f]{32}', normalized):
        raise ValueError('task_id is not a ROS action UUID')
    goal = GoalInfo()
    goal.goal_id.uuid = list(bytes.fromhex(normalized))
    return goal


def load_named_places(path: str) -> list[dict[str, Any]]:
    """Load the active named-place asset for bounded operator navigation."""
    configured = Path(os.path.expanduser(str(path).strip()))
    if not str(path).strip() or not configured.is_file():
        return []
    document = yaml.safe_load(configured.read_text(encoding='utf-8')) or {}
    places = document.get('named_places', {}).get('places', {})
    if not isinstance(places, dict):
        raise ValueError('named_places.places must be a mapping')
    result = []
    for place_id, value in places.items():
        if not isinstance(value, dict):
            continue
        coordinates = [
            float(value.get('x', 0.0)), float(value.get('y', 0.0)),
            float(value.get('yaw', 0.0)), float(value.get('nav_offset_m', 0.0)),
        ]
        if not all(math.isfinite(item) for item in coordinates):
            continue
        result.append({
            'id': str(place_id), 'x': coordinates[0], 'y': coordinates[1],
            'yaw': coordinates[2], 'nav_offset_m': coordinates[3],
        })
    return result


@dataclass
class TaskRecord:
    """Web-facing snapshot of one ExecuteTask goal."""

    task_id: str
    object_id: str
    source_place: str
    recipient_id: str
    grasp_backend: str
    dry_run: bool
    grasp_backend_used: str = ''
    cancelable: bool = True
    status: str = 'ACCEPTING'
    current_capability: str = ''
    phase: str = ''
    progress: float = 0.0
    message: str = ''
    error_code: int = 0
    created_at: str = ''
    updated_at: str = ''
    completed_at: str = ''
    stage_elapsed_s: float = 0.0
    capability_durations: dict[str, float] = field(default_factory=dict)


class TaskHistory:
    """Small persistent task index; task semantics remain in ExecuteTask."""

    def __init__(self, database: Path):
        database.parent.mkdir(parents=True, exist_ok=True)
        self._database = database
        self._trace_root = database.parent
        self._lock = threading.Lock()
        with self._connect() as connection:
            connection.execute(
                'CREATE TABLE IF NOT EXISTS tasks ('
                'task_id TEXT PRIMARY KEY, created_at TEXT NOT NULL, '
                'record_json TEXT NOT NULL)'
            )
            connection.execute(
                'CREATE TABLE IF NOT EXISTS engineering_audit ('
                'id INTEGER PRIMARY KEY AUTOINCREMENT, at TEXT NOT NULL, '
                'operator TEXT NOT NULL, operation TEXT NOT NULL, '
                'result TEXT NOT NULL, detail_json TEXT NOT NULL)'
            )

    def _connect(self):
        return sqlite3.connect(self._database, timeout=5.0)

    def save(self, record: TaskRecord) -> None:
        payload = json.dumps(asdict(record), ensure_ascii=False, sort_keys=True)
        with self._lock, self._connect() as connection:
            connection.execute(
                'INSERT OR REPLACE INTO tasks(task_id, created_at, record_json) '
                'VALUES (?, ?, ?)',
                (record.task_id, record.created_at, payload),
            )
            connection.execute(
                'DELETE FROM tasks WHERE task_id NOT IN ('
                'SELECT task_id FROM tasks ORDER BY created_at DESC LIMIT 100)'
            )
            trace = {
                'schema': 'xlerobot_task_trace/v1',
                'at': utc_now(),
                'task': json.loads(payload),
            }
            with (self._trace_root / f'{record.task_id}.jsonl').open(
                'a', encoding='utf-8'
            ) as stream:
                stream.write(json.dumps(trace, ensure_ascii=False, sort_keys=True) + '\n')

    def recent(self, limit: int = 20) -> list[dict[str, Any]]:
        limit = max(1, min(int(limit), 100))
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                'SELECT record_json FROM tasks ORDER BY created_at DESC LIMIT ?',
                (limit,),
            ).fetchall()
        return [json.loads(row[0]) for row in rows]

    def audit(
        self, operator: str, operation: str, result: str,
        detail: dict[str, Any] | None = None,
    ) -> None:
        with self._lock, self._connect() as connection:
            connection.execute(
                'INSERT INTO engineering_audit('
                'at, operator, operation, result, detail_json) VALUES (?, ?, ?, ?, ?)',
                (utc_now(), operator, operation, result,
                 json.dumps(detail or {}, ensure_ascii=False, sort_keys=True)),
            )


class EventHub:
    """Fan ROS-thread state changes out to asyncio SSE clients."""

    def __init__(self):
        self._clients: set[tuple[asyncio.AbstractEventLoop, asyncio.Queue]] = set()
        self._lock = threading.Lock()

    def subscribe(self) -> tuple[asyncio.AbstractEventLoop, asyncio.Queue]:
        client = (asyncio.get_running_loop(), asyncio.Queue(maxsize=64))
        with self._lock:
            self._clients.add(client)
        return client

    def unsubscribe(self, client) -> None:
        with self._lock:
            self._clients.discard(client)

    def publish(self, event_type: str, payload: dict[str, Any]) -> None:
        event = {'type': event_type, 'at': utc_now(), 'data': payload}
        with self._lock:
            clients = list(self._clients)
        for loop, queue in clients:
            def enqueue(target=queue, value=event):
                if target.full():
                    target.get_nowait()
                target.put_nowait(value)

            loop.call_soon_threadsafe(enqueue)


class OperatorConsoleNode(Node):
    """ROS side of the protocol adapter."""

    def __init__(self):
        super().__init__('xlerobot_operator_console')
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('artifact_root', '.xlerobot/artifacts')
        self.declare_parameter('release_id', 'development')
        self.declare_parameter('unit_id', 'reference-two-wheel')
        self.declare_parameter(
            'default_dataset_id', 'xlerobot-glue-stick-grasp-30'
        )
        self.declare_parameter('site_id', '')
        self.declare_parameter('places_file', '')
        self.declare_parameter('workspace', 'operator')
        self.declare_parameter('mapping_phase', '')
        self.declare_parameter('calibration_workflow', '')
        self.declare_parameter('calibration_capture_only', False)
        self.declare_parameter('enable_engineering_tools', False)
        self.declare_parameter('task_history_root', '.xlerobot/logs/tasks')
        self.declare_parameter(
            'head_camera_topic', '/xlerobot/d455/color/image_raw'
        )
        self.declare_parameter(
            'wrist_camera_topic', '/right_wrist_camera/image_raw'
        )
        self.declare_parameter('camera_max_fps', 10.0)

        root = Path(os.path.expanduser(str(self.get_parameter('artifact_root').value)))
        self.artifact_root = root
        self.catalog = ArtifactCatalog(root)
        history_root = Path(os.path.expanduser(str(
            self.get_parameter('task_history_root').value
        )))
        self.history = TaskHistory(history_root / 'task_history.sqlite3')
        self.events = EventHub()
        self.task_client = ActionClient(self, ExecuteTask, '/execute_task')
        self.voice_state = 'UNKNOWN'
        self.voice_transcript = ''
        self.diagnostics: dict[str, dict[str, Any]] = {}
        self.active_task: TaskRecord | None = None
        self.external_task: TaskRecord | None = None
        self.active_goal_handle = None
        self.active_server_task_id: str | None = None
        self._task_cancel_requested = threading.Event()
        self._owned_task_ids: set[str] = set()
        self.manual_control = ManualControlCoordinator()
        self.active_collection: dict[str, Any] | None = None
        self.collection_goal_handle = None
        self._collection_lock = threading.Lock()
        self._stage_started_monotonic = 0.0
        self._external_stage_started_monotonic = 0.0
        self._task_lock = threading.Lock()
        self._camera_lock = threading.Lock()
        self._camera_frames: dict[str, tuple[int, bytes]] = {}
        self._camera_last_encoded: dict[str, float] = {}
        self._camera_received_at: dict[str, float] = {}
        self._observation_lock = threading.Lock()
        self._observation_frames: dict[str, bytes] = {}
        self._perception_state: dict[str, Any] | None = None
        try:
            self.named_places = load_named_places(str(self.parameter('places_file')))
        except (OSError, ValueError, TypeError) as error:
            self.get_logger().warning(f'Unable to load named places: {error}')
            self.named_places = []
        self._mapping_lock = threading.Lock()
        self._mapping_state: dict[str, Any] = {
            'map': None, 'pose': None, 'scan': None, 'path': None,
            'slam': 'WAITING',
        }
        self._last_scan_event = 0.0
        self._teleop_publisher = None
        self._teleop_last_command = 0.0
        self._teleop_was_active = False
        self._mapping_localized = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(String, '/voice/state', self._on_voice_state, 10)
        self.create_subscription(
            String, '/voice/transcript', self._on_voice_transcript, 10
        )
        self.create_subscription(
            TaskEvent,
            '/task/events',
            self._on_task_event,
            transient_state_qos_profile(20),
        )
        self.create_subscription(
            PerceptionObservation,
            '/perception/observations',
            self._on_perception_observation,
            10,
        )
        self.create_subscription(
            DiagnosticArray, '/diagnostics', self._on_diagnostics, 20
        )
        self.create_subscription(
            Image, str(self.parameter('head_camera_topic')),
            lambda message: self._on_image('head', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, str(self.parameter('wrist_camera_topic')),
            lambda message: self._on_image('wrist', message),
            qos_profile_sensor_data,
        )
        if map_visualization_enabled(str(self.parameter('workspace'))):
            self.create_subscription(
                OccupancyGrid, '/map', self._on_map, map_qos_profile()
            )
            self.create_subscription(
                LaserScan, '/scan', self._on_scan, qos_profile_sensor_data
            )
            self.create_subscription(
                NavigationPath, '/plan', self._on_navigation_path, 10
            )
            self.create_timer(0.05, self._mapping_timer)
        if self._mapping_workspace():
            self.site_save_client = self.create_client(
                SaveSiteMap, '/site_manager/save_map'
            )
            self.site_place_client = self.create_client(
                SetNamedPlace, '/site_manager/set_place'
            )
            self.site_remove_place_client = self.create_client(
                RemoveNamedPlace, '/site_manager/remove_place'
            )
            self.site_validation_client = self.create_client(
                RecordSiteValidation, '/site_manager/record_validation'
            )
            self.site_activate_client = self.create_client(
                ActivateArtifact, '/site_manager/activate'
            )
        if self._mapping_enabled() or str(self.parameter('workspace')) == 'operator':
            self.declare_parameter('teleop_timeout_s', 0.35)
            self.declare_parameter('teleop_max_linear_mps', 0.12)
            self.declare_parameter('teleop_max_angular_radps', 0.50)
            self._teleop_publisher = self.create_publisher(
                Twist, '/cmd_vel_teleop', 10
            )
        if self._mapping_enabled():
            self.map_saver = self.create_client(SaveMap, '/slam_toolbox/save_map')
            self.map_resetter = self.create_client(Reset, '/slam_toolbox/reset')
        if self._mapping_validation_enabled():
            self.localize_client = ActionClient(
                self, AutoLocalize, '/auto_localize'
            )
            self.named_navigation_client = ActionClient(
                self, NavigateToNamedPlace, '/navigate_to_named_place'
            )
        if str(self.parameter('workspace')) == 'operator':
            self.localize_client = ActionClient(
                self, AutoLocalize, '/auto_localize'
            )
            self.named_navigation_client = ActionClient(
                self, NavigateToNamedPlace, '/navigate_to_named_place'
            )
            self.maintenance_preset_client = ActionClient(
                self, SetMaintenancePreset, '/maintenance_preset'
            )
            self.manual_reservation_client = self.create_client(
                SetBool, '/execute_task/set_manual_control'
            )
            self.drive_stop_client = self.create_client(
                SetBool, '/drive_safety/set_stop'
            )
            self.task_cancel_client = self.create_client(
                CancelGoal, '/execute_task/_action/cancel_goal'
            )
            self.create_subscription(
                Bool,
                '/drive_safety/stop_latched',
                self._on_drive_stop_latched,
                transient_state_qos_profile(),
            )
        if str(self.parameter('workspace')) == 'calibration':
            self.calibration_client = ActionClient(
                self, CalibrationJob, '/calibration/job'
            )
            self.calibration_import_client = self.create_client(
                ImportCalibrationResult, '/calibration/import_result'
            )
            self.calibration_activate_client = self.create_client(
                ActivateArtifact, '/calibration/activate'
            )
            if str(self.parameter('calibration_workflow')) == 'base_geometry':
                self.base_geometry_client = self.create_client(
                    SaveBaseGeometryCalibration,
                    '/calibration/save_base_geometry',
                )
            if str(self.parameter('calibration_workflow')) == 'servo':
                self.servo_calibration_client = self.create_client(
                    ServoCalibrationStep, '/calibration/servo_step'
                )
            if str(self.parameter('calibration_workflow')) in {
                'head_camera', 'right_handeye'
            }:
                self.calibration_capture_client = self.create_client(
                    CaptureCalibrationSample, '/calibration/capture_sample'
                )
                self.calibration_solve_client = self.create_client(
                    SolveCalibrationSamples, '/calibration/solve_samples'
                )
                self.calibration_pose_client = ActionClient(
                    self, MoveCalibrationPose, '/calibration/move_pose'
                )
        if str(self.parameter('workspace')) == 'collection':
            self.collection_client = ActionClient(
                self, CollectEpisode, '/collect_episode'
            )
            self.collection_finalize_client = self.create_client(
                FinalizeEpisode, '/collect_episode/finalize'
            )
            self.review_client = self.create_client(
                ReviewEpisode, '/episodes/review'
            )
        self.create_timer(1.0, self._publish_health)

    def parameter(self, name: str):
        return self.get_parameter(name).value

    def _mapping_enabled(self) -> bool:
        return (
            self._mapping_workspace()
            and str(self.parameter('mapping_phase')) == 'build'
        )

    def _mapping_workspace(self) -> bool:
        return (
            str(self.parameter('workspace')) == 'mapping'
            and bool(self.parameter('enable_engineering_tools'))
        )

    def _mapping_validation_enabled(self) -> bool:
        return (
            self._mapping_workspace()
            and str(self.parameter('mapping_phase')) == 'validate'
        )

    def _on_voice_state(self, message: String) -> None:
        self.voice_state = message.data
        if self.voice_state == 'RECORDING':
            self.voice_transcript = ''
        self.events.publish('voice', {
            'state': self.voice_state, 'transcript': self.voice_transcript,
        })

    def _on_voice_transcript(self, message: String) -> None:
        self.voice_transcript = str(message.data).strip()
        self.events.publish('voice', {
            'state': self.voice_state, 'transcript': self.voice_transcript,
        })

    def _on_drive_stop_latched(self, message: Bool) -> None:
        self.drive_stop_latched = bool(message.data)
        self.events.publish('drive_stop', {
            'latched': self.drive_stop_latched,
            'kind': 'base_software_inhibit',
        })
        self.events.publish('health', self.health())

    @staticmethod
    def _observation_matches_capability(
        observation: dict[str, Any] | None, capability: str
    ) -> bool:
        if observation is None:
            return False
        expected = {
            'object': {'detect_object', 'grasp_object'},
            'person': {'scan_for_person', 'approach_target'},
        }
        return capability in expected.get(observation['kind'], set())

    def _activate_observation(
        self, message: PerceptionObservation, *, capture_current_frame: bool
    ) -> None:
        observation = observation_dict(message)
        if observation is None:
            return
        observation_id = observation['observation_id']
        with self._camera_lock:
            if capture_current_frame:
                source = self._camera_frames.get(observation['camera_id'])
                if source is not None:
                    self._observation_frames[observation_id] = source[1]
                    while len(self._observation_frames) > 24:
                        self._observation_frames.pop(next(iter(self._observation_frames)))
            jpeg = self._observation_frames.get(observation_id)
            if jpeg is not None:
                sequence = self._camera_frames.get('detection', (0, b''))[0] + 1
                self._camera_frames['detection'] = (sequence, jpeg)
        with self._observation_lock:
            self._perception_state = observation
        self.events.publish('perception', observation)

    def _clear_observation(self) -> None:
        with self._observation_lock:
            if self._perception_state is None:
                return
            self._perception_state = None
        self.events.publish('perception', None)

    def _on_perception_observation(
        self, message: PerceptionObservation
    ) -> None:
        self._activate_observation(message, capture_current_frame=True)

    def perception_snapshot(self) -> dict[str, Any] | None:
        with self._observation_lock:
            return json.loads(json.dumps(self._perception_state))

    def _on_task_event(self, event: TaskEvent) -> None:
        """Observe ExecuteTask goals started by voice or other stable clients."""
        terminal = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'}
        now_monotonic = time.monotonic()
        if event.status == 'RUNNING':
            self.stop_teleop()
            self.request_manual_cancel()
        event_observation = observation_dict(event.observation)
        if event_observation is not None:
            self._activate_observation(
                event.observation, capture_current_frame=False
            )
        else:
            with self._observation_lock:
                current_observation = self._perception_state
            if (
                event.status in terminal
                or not self._observation_matches_capability(
                    current_observation, event.current_capability
                )
            ):
                self._clear_observation()
        owned_terminal = None
        with self._task_lock:
            local = self.active_task
            if local is not None and local.status in terminal:
                local = None
            awaiting_identity = (
                local is not None
                and local.status in {'ACCEPTING', 'ACCEPTANCE_UNKNOWN'}
            )
            if (
                awaiting_identity
                and event.task_id not in self._owned_task_ids
                and event.object_id == local.object_id
                and event.source_place == local.source_place
                and event.recipient_id == local.recipient_id
                and event.grasp_backend == local.grasp_backend
                and bool(event.dry_run) == local.dry_run
            ):
                # ExecuteTask admits one goal at a time.  A matching TaskEvent
                # therefore recovers the server UUID even when the action
                # response is delayed or its future raises.
                self._owned_task_ids.add(event.task_id)
                self.active_server_task_id = event.task_id
            owned = event.task_id in self._owned_task_ids
            if owned:
                if local is not None and event.status in terminal:
                    owned_terminal = (
                        event.status,
                        int(event.error.code),
                        event.error.message or event.state.message,
                    )
                    record = None
                elif (
                    local is not None
                    and local.status
                    in {'ACCEPTING', 'ACCEPTANCE_UNKNOWN', 'RESULT_UNKNOWN'}
                ):
                    record = local
                    stage_started = self._stage_started_monotonic
                else:
                    return
            elif local is not None:
                return
            else:
                if (
                    self.external_task is None
                    or self.external_task.task_id != event.task_id
                ):
                    created_at = stamp_utc(event.stamp)
                    self.external_task = TaskRecord(
                        task_id=event.task_id,
                        object_id=event.object_id,
                        source_place=event.source_place,
                        recipient_id=event.recipient_id,
                        grasp_backend=event.grasp_backend,
                        dry_run=bool(event.dry_run),
                        grasp_backend_used=event.grasp_backend_used,
                        cancelable=True,
                        created_at=created_at,
                        updated_at=created_at,
                    )
                    self._external_stage_started_monotonic = now_monotonic
                record = self.external_task
                stage_started = self._external_stage_started_monotonic
            if record is not None:
                previous = record.current_capability
                if previous and previous != event.current_capability:
                    record.capability_durations[previous] = round(
                        max(0.0, now_monotonic - stage_started),
                        3,
                    )
                    stage_started = now_monotonic
                    if owned:
                        self._stage_started_monotonic = now_monotonic
                    else:
                        self._external_stage_started_monotonic = now_monotonic
                record.status = event.status
                record.current_capability = event.current_capability
                record.phase = event.state.phase
                record.progress = float(event.state.progress)
                record.message = event.error.message or event.state.message
                record.error_code = int(event.error.code)
                record.grasp_backend = event.grasp_backend
                record.grasp_backend_used = event.grasp_backend_used
                record.stage_elapsed_s = round(
                    max(0.0, now_monotonic - stage_started),
                    3,
                )
                record.updated_at = stamp_utc(event.stamp)
                if record.status in terminal:
                    record.completed_at = record.updated_at
                    if record.current_capability:
                        record.capability_durations[record.current_capability] = (
                            record.stage_elapsed_s
                        )
                self.history.save(record)
                snapshot = asdict(record)
                recent = self.history.recent()
            else:
                snapshot = None
                recent = None
        if owned_terminal is not None:
            self._finish_task(*owned_terminal)
            return
        self.events.publish('task', snapshot)
        self.events.publish('task_history', {'tasks': recent})

    def request_manual_cancel(self) -> None:
        """Signal the owning aiohttp coroutine to cancel and drain its goal."""
        self.manual_control.request_cancel()

    @property
    def drive_stop_latched(self) -> bool | None:
        return self.manual_control.drive_stop_latched

    @drive_stop_latched.setter
    def drive_stop_latched(self, enabled: bool | None) -> None:
        self.manual_control.set_drive_stop_latched(enabled)

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        now = time.monotonic()
        for status in message.status:
            self.diagnostics[status.name] = {
                'level': _uint8(status.level),
                'message': status.message,
                'hardware_id': status.hardware_id,
                'values': {entry.key: entry.value for entry in status.values},
                'received_monotonic': now,
            }
        self.events.publish('health', self.health())

    def _publish_health(self) -> None:
        """Re-evaluate freshness gates even when an upstream stream stops."""
        self.events.publish('health', self.health())

    def _on_image(self, camera_id: str, message: Image) -> None:
        now = time.monotonic()
        minimum_period = 1.0 / max(float(self.parameter('camera_max_fps')), 0.1)
        if now - self._camera_last_encoded.get(camera_id, 0.0) < minimum_period:
            return
        self._camera_last_encoded[camera_id] = now
        try:
            frame = _image_array(message)
            encoded, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not encoded:
                return
            with self._camera_lock:
                sequence = self._camera_frames.get(camera_id, (0, b''))[0] + 1
                self._camera_frames[camera_id] = (sequence, jpeg.tobytes())
                self._camera_received_at[camera_id] = now
        except (TypeError, ValueError, cv2.error) as error:
            self.get_logger().warning(
                f'Unable to encode {camera_id} camera frame: {error}',
                throttle_duration_sec=5.0,
            )

    def camera_frame(self, camera_id: str) -> tuple[int, bytes] | None:
        with self._camera_lock:
            return self._camera_frames.get(camera_id)

    def _on_map(self, message: OccupancyGrid) -> None:
        origin = message.info.origin
        map_state = {
            'frame_id': message.header.frame_id,
            'width': int(message.info.width),
            'height': int(message.info.height),
            'resolution': float(message.info.resolution),
            'origin': {
                'x': origin.position.x, 'y': origin.position.y,
                'yaw': _quaternion_yaw(origin.orientation),
            },
            'data': list(message.data),
        }
        with self._mapping_lock:
            self._mapping_state['map'] = map_state
            self._mapping_state['slam'] = 'MAPPING'
        self.events.publish('mapping', {'kind': 'map', 'map': map_state})

    def _on_scan(self, message: LaserScan) -> None:
        now = time.monotonic()
        if now - self._last_scan_event < 0.5:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', message.header.frame_id, Time()
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Unable to transform lidar preview into base_link: {error}',
                throttle_duration_sec=5.0,
            )
            return
        points = []
        stride = max(1, len(message.ranges) // 180)
        for index in range(0, len(message.ranges), stride):
            distance = float(message.ranges[index])
            if not math.isfinite(distance) or not (
                message.range_min <= distance <= message.range_max
            ):
                continue
            angle = float(message.angle_min + index * message.angle_increment)
            x, y, _ = _transform_point_xyz(
                transform.transform,
                (distance * math.cos(angle), distance * math.sin(angle), 0.0),
            )
            points.append([x, y])
        scan = {'frame_id': 'base_link', 'points_xy': points}
        with self._mapping_lock:
            self._mapping_state['scan'] = scan
        self._last_scan_event = now
        self.events.publish('mapping', {'kind': 'scan', 'scan': scan})

    def _on_navigation_path(self, message: NavigationPath) -> None:
        """Expose the current Nav2 global plan without becoming a planner."""
        source_frame = message.header.frame_id or (
            message.poses[0].header.frame_id if message.poses else ''
        )
        transform = None
        if source_frame and source_frame != 'map':
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', source_frame, Time()
                )
            except TransformException as error:
                self.get_logger().warning(
                    f'Unable to transform navigation preview into map: {error}',
                    throttle_duration_sec=5.0,
                )
                return
        stride = max(1, len(message.poses) // 300)
        points = []
        for pose in message.poses[::stride]:
            point = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            if transform is not None:
                point = _transform_point_xyz(transform.transform, point)
            points.append([float(point[0]), float(point[1])])
        path = {'frame_id': 'map', 'points_xy': points}
        with self._mapping_lock:
            self._mapping_state['path'] = path
        self.events.publish('mapping', {'kind': 'path', 'path': path})

    def _mapping_timer(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            translation = transform.transform.translation
            pose = {
                'frame_id': 'map', 'x': translation.x, 'y': translation.y,
                'yaw': _quaternion_yaw(transform.transform.rotation),
            }
            changed = False
            with self._mapping_lock:
                previous = self._mapping_state['pose']
                if previous is None or any(
                    abs(pose[key] - previous[key]) > 0.005
                    for key in ('x', 'y', 'yaw')
                ):
                    self._mapping_state['pose'] = pose
                    changed = True
            if changed:
                self.events.publish('mapping', {'kind': 'pose', 'pose': pose})
        except TransformException:
            pass
        if self._teleop_publisher is not None:
            timeout = float(self.parameter('teleop_timeout_s'))
            if (self._teleop_was_active
                    and time.monotonic() - self._teleop_last_command > timeout):
                self.stop_teleop()

    def mapping_snapshot(self) -> dict[str, Any] | None:
        if not map_visualization_enabled(str(self.parameter('workspace'))):
            return None
        with self._mapping_lock:
            return json.loads(json.dumps(self._mapping_state))

    def reset_mapping_preview(self) -> None:
        """Discard only live visualization caches after a successful SLAM reset."""
        with self._mapping_lock:
            self._mapping_state.update({
                'map': None, 'pose': None, 'scan': None, 'path': None,
                'slam': 'WAITING',
                'reset_notice': '当前地图已清除，正在重新建图。已保存的地图和地点未删除；'
                                '重建后请重新确认或记录地点，再保存新地图。',
            })
        self.events.publish('mapping', {
            'kind': 'reset', 'state': self.mapping_snapshot(),
        })

    def command_teleop(self, linear: float, angular: float, armed: bool) -> None:
        if self._teleop_publisher is None:
            raise RuntimeError('base teleop is unavailable in this launch profile')
        if not armed or not math.isfinite(linear) or not math.isfinite(angular):
            self.stop_teleop()
            return
        if (
            str(self.parameter('workspace')) == 'operator'
            and (
                self.task_active()
                or self.manual_action_active()
                or self.drive_stop_latched is not False
            )
        ):
            self.stop_teleop()
            raise RuntimeError(
                'base teleop requires an idle task runtime and an enabled base'
            )
        message = Twist()
        max_linear = float(self.parameter('teleop_max_linear_mps'))
        max_angular = float(self.parameter('teleop_max_angular_radps'))
        message.linear.x = max(-max_linear, min(max_linear, float(linear)))
        message.angular.z = max(-max_angular, min(max_angular, float(angular)))
        self._teleop_publisher.publish(message)
        self._teleop_last_command = time.monotonic()
        self._teleop_was_active = True

    def stop_teleop(self) -> None:
        if self._teleop_publisher is not None and self._teleop_was_active:
            self._teleop_publisher.publish(Twist())
        self._teleop_was_active = False

    def task_active(self) -> bool:
        terminal = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'}
        task = self.task_snapshot()
        return task is not None and task['status'] not in terminal

    def manual_action_active(self) -> bool:
        return self.manual_control.action_active()

    def claim_teleop(self, owner) -> bool:
        return self.manual_control.claim_teleop(
            owner, task_active=self.task_active()
        )

    def release_teleop(self, owner) -> None:
        self.manual_control.release_teleop(owner)

    def teleop_active(self) -> bool:
        return self.manual_control.teleop_active()

    def operator_idle(self) -> bool:
        return self.manual_control.idle(task_active=self.task_active())

    def health(self) -> dict[str, Any]:
        action_ready = self.task_client.server_is_ready()
        workspace = str(self.parameter('workspace'))
        requirements: dict[str, bool] = {}
        if workspace == 'mapping':
            scan_ready = (
                self._last_scan_event > 0.0
                and time.monotonic() - self._last_scan_event <= 2.0
            )
            services = [
                self.site_save_client, self.site_place_client,
                self.site_remove_place_client, self.site_validation_client,
                self.site_activate_client,
            ]
            phase = str(self.parameter('mapping_phase'))
            if phase == 'build':
                services.append(self.map_saver)
                navigation_ready = True
            elif phase == 'validate':
                navigation_ready = (
                    self.localize_client.server_is_ready()
                    and self.named_navigation_client.server_is_ready()
                )
            else:
                navigation_ready = False
            services_ready = all(client.service_is_ready() for client in services)
            profile_ready = scan_ready and services_ready and navigation_ready
            requirements.update({
                'lidar_scan': scan_ready,
                'site_services': services_ready,
                'navigation_actions': navigation_ready,
            })
        elif workspace == 'calibration':
            action = self.calibration_client.server_is_ready()
            services = (
                self.calibration_import_client.service_is_ready()
                and self.calibration_activate_client.service_is_ready()
            )
            profile_ready = action and services
            requirements.update({
                'calibration_job': action,
                'calibration_services': services,
            })
        elif workspace == 'collection':
            action = self.collection_client.server_is_ready()
            finalize = self.collection_finalize_client.service_is_ready()
            review = self.review_client.service_is_ready()
            profile_ready = action and finalize and review
            requirements.update({
                'collect_episode': action,
                'collection_finalize': finalize,
                'episode_review': review,
            })
        else:
            manual_reservation_ready = (
                self.manual_reservation_client.service_is_ready()
            )
            drive_stop_ready = self.drive_stop_client.service_is_ready()
            task_cancel_ready = self.task_cancel_client.service_is_ready()
            base_motion_enabled = self.drive_stop_latched is False
            task_readiness = self.diagnostics.get('xlerobot/execute_task')
            task_live_ready = bool(
                task_readiness
                and time.monotonic()
                - task_readiness['received_monotonic'] <= 2.5
                and str(
                    task_readiness.get('values', {}).get('live_ready', '')
                ).lower() == 'true'
            )
            profile_ready = (
                action_ready
                and task_live_ready
                and manual_reservation_ready
                and drive_stop_ready
                and task_cancel_ready
                and base_motion_enabled
            )
            requirements.update({
                'execute_task': action_ready,
                'execute_task_live_ready': task_live_ready,
                'manual_reservation': manual_reservation_ready,
                'drive_stop_service': drive_stop_ready,
                'task_cancel_service': task_cancel_ready,
                'base_motion_enabled': base_motion_enabled,
            })
            with self._camera_lock:
                camera_age = {
                    camera_id: time.monotonic() - self._camera_received_at.get(
                        camera_id, float('-inf')
                    )
                    for camera_id in ('head', 'wrist')
                }
            requirements.update({
                'head_camera': camera_age['head'] <= 2.0,
                'wrist_camera': camera_age['wrist'] <= 2.0,
            })
        diagnostics = {}
        levels = []
        for name, item in sorted(self.diagnostics.items()):
            public = {
                key: value for key, value in item.items()
                if key != 'received_monotonic'
            }
            effective_level = _diagnostic_level_for_readiness(name, item)
            levels.append(effective_level)
            if effective_level != item['level']:
                public['reported_level'] = item['level']
                public['level'] = effective_level
            diagnostics[name] = public
        worst = max(levels, default=_uint8(DiagnosticStatus.OK))
        if not profile_ready:
            readiness = 'BLOCKED'
        elif worst >= _uint8(DiagnosticStatus.ERROR):
            readiness = 'BLOCKED'
        elif worst == _uint8(DiagnosticStatus.WARN):
            readiness = 'DEGRADED'
        else:
            readiness = 'READY'
        return {
            'readiness': readiness,
            'workspace': workspace,
            'execute_task_available': action_ready,
            'voice_state': self.voice_state,
            'drive_stop_latched': self.drive_stop_latched,
            'diagnostics': diagnostics,
            'requirements': requirements,
        }

    def task_snapshot(self) -> dict[str, Any] | None:
        with self._task_lock:
            terminal = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'}
            if self.active_task and self.active_task.status not in terminal:
                selected = self.active_task
            elif self.external_task and self.external_task.status not in terminal:
                selected = self.external_task
            else:
                candidates = [
                    item for item in (self.active_task, self.external_task) if item
                ]
                selected = (
                    max(candidates, key=lambda item: item.updated_at)
                    if candidates else None
                )
            return asdict(selected) if selected else None

    async def submit_task(self, goal: ExecuteTask.Goal) -> dict[str, Any]:
        if self.manual_action_active() or self.teleop_active():
            raise web.HTTPConflict(text='wait for manual control to finish')
        if self.drive_stop_latched is not False:
            raise web.HTTPConflict(text='clear the base software inhibit first')
        with self._task_lock:
            terminal = {
                'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'
            }
            if (
                (self.active_task and self.active_task.status not in terminal)
                or (self.external_task and self.external_task.status not in terminal)
            ):
                raise web.HTTPConflict(text='a task is already active')
            now = utc_now()
            record = TaskRecord(
                task_id=secrets.token_hex(12),
                object_id=goal.object_id,
                source_place=goal.source_place,
                recipient_id=goal.recipient_id,
                grasp_backend=goal.grasp_backend,
                dry_run=goal.dry_run,
                created_at=now,
                updated_at=now,
            )
            self.active_task = record
            self.active_goal_handle = None
            self.active_server_task_id = None
            self._task_cancel_requested.clear()
            self._stage_started_monotonic = time.monotonic()
            self.history.save(record)
            snapshot = asdict(record)
            recent = self.history.recent()
        self.events.publish('task', snapshot)
        self.events.publish('task_history', {'tasks': recent})
        if not await _wait_until_ready(self.task_client.server_is_ready, 2.0):
            self._finish_task('FAILED', CapabilityError.UNAVAILABLE,
                              'ExecuteTask action is unavailable')
            raise web.HTTPServiceUnavailable(text='ExecuteTask action is unavailable')
        try:
            future = self.task_client.send_goal_async(
                goal, feedback_callback=self._on_task_feedback
            )
        except Exception as error:
            self._finish_task(
                'FAILED',
                CapabilityError.INTERNAL_ERROR,
                f'ExecuteTask goal could not be sent: {error}',
            )
            raise web.HTTPServiceUnavailable(
                text='ExecuteTask goal could not be sent'
            ) from error
        accept_deadline = asyncio.get_running_loop().time() + 5.0
        accept_timed_out = False
        while not future.done():
            if (
                not accept_timed_out
                and asyncio.get_running_loop().time() >= accept_deadline
            ):
                accept_timed_out = True
                self._task_cancel_requested.set()
            await asyncio.sleep(0.02)
        try:
            handle = future.result()
        except Exception as error:
            message = (
                'ExecuteTask goal acceptance could not be observed; local '
                'ownership remains blocked until a terminal TaskEvent or '
                'profile restart'
            )
            self._mark_task_acceptance_unknown(message)
            raise web.HTTPServiceUnavailable(text=message) from error
        if handle is None or not handle.accepted:
            self._finish_task(
                'REJECTED', CapabilityError.SAFETY_REJECTED, 'goal rejected'
            )
            if accept_timed_out:
                raise web.HTTPGatewayTimeout(
                    text='ExecuteTask goal response timed out; pending goal '
                         'was rejected before task ownership was released'
                )
            raise web.HTTPConflict(text='ExecuteTask goal was rejected')
        with self._task_lock:
            server_task_id = bytes(handle.goal_id.uuid).hex()
            self._owned_task_ids.add(server_task_id)
            if len(self._owned_task_ids) > 256:
                self._owned_task_ids = {server_task_id}
            already_terminal = self.active_task.status in {
                'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'
            }
            if not already_terminal:
                self.active_goal_handle = handle
                self.active_server_task_id = server_task_id
                self.active_task.status = 'RUNNING'
                self.active_task.updated_at = utc_now()
                self.history.save(self.active_task)
            snapshot = asdict(self.active_task)
        if already_terminal:
            return snapshot
        self.events.publish('task', snapshot)
        self.events.publish('task_history', {'tasks': self.history.recent()})
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_task_result)
        if self._task_cancel_requested.is_set():
            await _await_rclpy_future(handle.cancel_goal_async(), 5.0)
            terminal = await self.wait_task_terminal(record.task_id, 15.0)
            if accept_timed_out:
                raise web.HTTPGatewayTimeout(
                    text='ExecuteTask goal response timed out; accepted goal '
                         'was canceled and drained'
                )
            return terminal
        with self._task_lock:
            if (
                self.active_task is not None
                and self.active_task.task_id == record.task_id
            ):
                return asdict(self.active_task)
        return snapshot

    def _on_task_feedback(self, message) -> None:
        feedback = message.feedback
        with self._task_lock:
            if self.active_task is None:
                return
            now = time.monotonic()
            previous = self.active_task.current_capability
            if previous and previous != feedback.current_capability:
                elapsed = max(0.0, now - self._stage_started_monotonic)
                self.active_task.capability_durations[previous] = round(elapsed, 3)
                self._stage_started_monotonic = now
            self.active_task.current_capability = feedback.current_capability
            self.active_task.phase = feedback.state.phase
            self.active_task.progress = float(feedback.state.progress)
            self.active_task.message = feedback.state.message
            self.active_task.stage_elapsed_s = round(
                max(0.0, now - self._stage_started_monotonic), 3
            )
            self.active_task.updated_at = utc_now()
            self.history.save(self.active_task)
            snapshot = asdict(self.active_task)
        self.events.publish('task', snapshot)
        self.events.publish('task_history', {'tasks': self.history.recent()})

    def _on_task_result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                status = (
                    'SUCCEEDED'
                    if result.error.code == CapabilityError.NONE
                    else 'FAILED'
                )
            elif wrapped.status == GoalStatus.STATUS_CANCELED:
                status = 'CANCELED'
            elif wrapped.status == GoalStatus.STATUS_ABORTED:
                status = 'FAILED'
            else:
                self._mark_task_result_unknown(
                    f'ExecuteTask returned non-terminal action status {wrapped.status}'
                )
                return
            with self._task_lock:
                backend_used = str(getattr(result, 'grasp_backend_used', ''))
                if self.active_task is not None and backend_used:
                    self.active_task.grasp_backend_used = backend_used
            self._finish_task(status, int(result.error.code), result.error.message)
        except Exception as exc:
            self._mark_task_result_unknown(
                f'ExecuteTask terminal result could not be observed: {exc}'
            )

    def _mark_task_result_unknown(self, message: str) -> None:
        terminal = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'}
        with self._task_lock:
            if self.active_task is None or self.active_task.status in terminal:
                return
            self.active_task.status = 'RESULT_UNKNOWN'
            self.active_task.error_code = CapabilityError.INTERNAL_ERROR
            self.active_task.message = message
            self.active_task.updated_at = utc_now()
            self.history.save(self.active_task)
            snapshot = asdict(self.active_task)
        self.events.publish('task', snapshot)
        self.events.publish('task_history', {'tasks': self.history.recent()})

    def _mark_task_acceptance_unknown(self, message: str) -> None:
        terminal = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'}
        with self._task_lock:
            if self.active_task is None or self.active_task.status in terminal:
                return
            self.active_task.status = 'ACCEPTANCE_UNKNOWN'
            self.active_task.error_code = CapabilityError.INTERNAL_ERROR
            self.active_task.message = message
            self.active_task.updated_at = utc_now()
            self.history.save(self.active_task)
            snapshot = asdict(self.active_task)
        self.events.publish('task', snapshot)
        self.events.publish('task_history', {'tasks': self.history.recent()})

    def _finish_task(self, status: str, error_code: int, message: str) -> None:
        terminal = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'}
        with self._task_lock:
            if self.active_task is None:
                return
            if self.active_task.status in terminal:
                return
            self.active_task.status = status
            self.active_task.error_code = int(error_code)
            self.active_task.message = message
            self.active_task.progress = 1.0 if status == 'SUCCEEDED' else self.active_task.progress
            self.active_task.updated_at = utc_now()
            self.active_task.completed_at = self.active_task.updated_at
            current = self.active_task.current_capability
            if current and current not in self.active_task.capability_durations:
                self.active_task.capability_durations[current] = round(
                    max(0.0, time.monotonic() - self._stage_started_monotonic), 3
                )
            self.history.save(self.active_task)
            self.active_goal_handle = None
            self.active_server_task_id = None
            self._task_cancel_requested.clear()
            snapshot = asdict(self.active_task)
        self.events.publish('task', snapshot)
        self.events.publish('task_history', {'tasks': self.history.recent()})

    async def cancel_task(self, task_id: str) -> dict[str, Any]:
        with self._task_lock:
            local = (
                self.active_task is not None
                and self.active_task.task_id == task_id
            )
            external = (
                self.external_task is not None
                and self.external_task.task_id == task_id
            )
            if not local and not external:
                raise web.HTTPNotFound(text='task not found')
            record = self.active_task if local else self.external_task
            handle = self.active_goal_handle if local else None
            server_task_id = self.active_server_task_id if local else None
            terminal = record.status in {
                'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'
            }
        if terminal:
            return self.task_snapshot()
        if local:
            if handle is None:
                self._task_cancel_requested.set()
                if server_task_id is not None:
                    await self.cancel_task_uuid(server_task_id)
                else:
                    return await self.wait_task_terminal(task_id, 15.0)
            else:
                await _await_rclpy_future(handle.cancel_goal_async(), 5.0)
        else:
            await self.cancel_task_uuid(task_id)
        return await self.wait_task_terminal(task_id, 15.0)

    async def cancel_task_uuid(self, task_id: str) -> None:
        """Cancel a voice/CLI goal through the standard action service."""
        try:
            goal_info = goal_info_from_task_id(task_id)
        except ValueError as error:
            raise web.HTTPBadRequest(text=str(error)) from error
        if not await _wait_until_ready(
            self.task_cancel_client.service_is_ready, 2.0
        ):
            raise web.HTTPServiceUnavailable(
                text='ExecuteTask cancel service is unavailable'
            )
        request = CancelGoal.Request()
        request.goal_info = goal_info
        future = self.task_cancel_client.call_async(request)
        await _await_rclpy_future(future, 5.0)
        response = future.result()
        if response.return_code not in {
            CancelGoal.Response.ERROR_NONE,
            CancelGoal.Response.ERROR_GOAL_TERMINATED,
        }:
            raise web.HTTPConflict(
                text=f'ExecuteTask cancel rejected ({response.return_code})'
            )

    async def wait_task_terminal(
        self, task_id: str, timeout_s: float
    ) -> dict[str, Any]:
        terminal = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'}
        deadline = asyncio.get_running_loop().time() + timeout_s
        while True:
            with self._task_lock:
                candidates = (self.active_task, self.external_task)
                record = next(
                    (item for item in candidates
                     if item is not None and item.task_id == task_id),
                    None,
                )
                if record is None:
                    raise web.HTTPNotFound(text='task not found')
                snapshot = asdict(record)
            if snapshot['status'] in terminal:
                return snapshot
            if asyncio.get_running_loop().time() >= deadline:
                raise web.HTTPGatewayTimeout(
                    text='task cancellation did not reach a terminal state; '
                         'the base software inhibit remains latched'
                )
            await asyncio.sleep(0.02)

    def collection_snapshot(self) -> dict[str, Any] | None:
        with self._collection_lock:
            return dict(self.active_collection) if self.active_collection else None

    async def submit_collection(self, goal: CollectEpisode.Goal) -> dict[str, Any]:
        with self._collection_lock:
            if self.active_collection and self.active_collection['status'] == 'RUNNING':
                raise web.HTTPConflict(text='a collection is already active')
        if not await _wait_until_ready(
            self.collection_client.server_is_ready, 2.0
        ):
            raise web.HTTPServiceUnavailable(text='CollectEpisode is unavailable')
        future = self.collection_client.send_goal_async(
            goal, feedback_callback=self._on_collection_feedback
        )
        await _await_rclpy_future(future, 5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise web.HTTPConflict(text='collection goal rejected')
        state = {
            'dataset_id': goal.dataset_id, 'episode_id': goal.episode_id,
            'template_id': goal.template_id, 'object_id': goal.object_id,
            'dry_run': bool(goal.dry_run),
            'status': 'RUNNING', 'phase': 'PREFLIGHT', 'progress': 0.0,
            'message': '', 'elapsed_s': 0.0, 'frame_count': 0,
            'episode_uri': '', 'quality_passed': False,
        }
        with self._collection_lock:
            self.active_collection = state
            self.collection_goal_handle = handle
        self.events.publish('collection', state)
        handle.get_result_async().add_done_callback(self._on_collection_result)
        return dict(state)

    def _on_collection_feedback(self, message) -> None:
        feedback = message.feedback
        with self._collection_lock:
            if self.active_collection is None:
                return
            self.active_collection.update({
                'phase': feedback.state.phase,
                'progress': float(feedback.state.progress),
                'message': feedback.state.message,
                'elapsed_s': float(feedback.elapsed_s),
                'frame_count': int(feedback.frame_count),
            })
            state = dict(self.active_collection)
        self.events.publish('collection', state)

    def _on_collection_result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            status = (
                'SUCCEEDED'
                if (
                    wrapped.status == GoalStatus.STATUS_SUCCEEDED
                    and result.error.code == CapabilityError.NONE
                )
                else 'CANCELED'
                if (
                    wrapped.status == GoalStatus.STATUS_CANCELED
                    and result.error.code == CapabilityError.CANCELED
                )
                else 'FAILED'
            )
            update = {
                'status': status, 'message': result.error.message,
                'episode_uri': result.episode_uri,
                'quality_passed': bool(result.quality_passed),
            }
        except Exception as error:
            update = {'status': 'FAILED', 'message': str(error)}
        with self._collection_lock:
            if self.active_collection is None:
                return
            self.active_collection.update(update)
            self.collection_goal_handle = None
            state = dict(self.active_collection)
        self.events.publish('collection', state)

    async def finalize_collection(
        self, dataset_id: str, episode_id: str
    ) -> dict[str, Any]:
        with self._collection_lock:
            if (
                not self.active_collection
                or self.active_collection['dataset_id'] != dataset_id
                or self.active_collection['episode_id'] != episode_id
            ):
                raise web.HTTPNotFound(text='collection not found')
            status = self.active_collection['status']
            if status == 'SUCCEEDED':
                return dict(self.active_collection)
            if status != 'RUNNING' or self.collection_goal_handle is None:
                raise web.HTTPConflict(
                    text=f'collection cannot be finalized from {status}'
                )
        if not await _wait_until_ready(
            self.collection_finalize_client.service_is_ready, 2.0
        ):
            raise web.HTTPServiceUnavailable(
                text='collection finalize service is unavailable'
            )
        request = FinalizeEpisode.Request(
            dataset_id=dataset_id,
            episode_id=episode_id,
        )
        future = self.collection_finalize_client.call_async(request)
        await _await_rclpy_future(future, 5.0)
        response = future.result()
        if response is None:
            raise web.HTTPServiceUnavailable(
                text='collection finalize returned no response'
            )
        if response.error.code != CapabilityError.NONE:
            snapshot = self.collection_snapshot()
            if (
                snapshot is not None
                and snapshot['dataset_id'] == dataset_id
                and snapshot['episode_id'] == episode_id
                and snapshot['status'] == 'SUCCEEDED'
            ):
                return snapshot
            raise web.HTTPConflict(
                text=f'[{response.error.code}] {response.error.message}'
            )
        snapshot = self.collection_snapshot()
        if snapshot is None:
            raise web.HTTPServiceUnavailable(
                text='collection ownership was lost while finalizing'
            )
        if (
            snapshot['dataset_id'] != dataset_id
            or snapshot['episode_id'] != episode_id
        ):
            raise web.HTTPServiceUnavailable(
                text='collection identity changed while finalizing'
            )
        return snapshot

    async def cancel_collection(
        self, dataset_id: str, episode_id: str
    ) -> dict[str, Any]:
        with self._collection_lock:
            if (
                not self.active_collection
                or self.active_collection['dataset_id'] != dataset_id
                or self.active_collection['episode_id'] != episode_id
            ):
                raise web.HTTPNotFound(text='collection not found')
            status = self.active_collection['status']
            if status in {'CANCELED', 'FAILED'}:
                return dict(self.active_collection)
            if status == 'SUCCEEDED':
                raise web.HTTPConflict(
                    text='completed collection cannot be changed to incomplete'
                )
            handle = self.collection_goal_handle
        if handle is None:
            raise web.HTTPServiceUnavailable(
                text='active collection goal ownership is unavailable'
            )
        future = handle.cancel_goal_async()
        response = None
        acknowledgment_error = None
        try:
            await _await_rclpy_future(future, 5.0)
            response = future.result()
        except Exception as error:
            acknowledgment_error = error
        if (
            response is not None
            and response.return_code != CancelGoal.Response.ERROR_NONE
        ):
            snapshot = self.collection_snapshot()
            if (
                snapshot
                and snapshot['dataset_id'] == dataset_id
                and snapshot['episode_id'] == episode_id
                and snapshot['status'] in {'FAILED', 'CANCELED'}
            ):
                return snapshot
            raise web.HTTPConflict(
                text=f'collection abort was rejected ({response.return_code})'
            )
        deadline = asyncio.get_running_loop().time() + 30.0
        while True:
            snapshot = self.collection_snapshot()
            if snapshot is None:
                raise web.HTTPServiceUnavailable(
                    text='collection ownership was lost while aborting'
                )
            if (
                snapshot['dataset_id'] != dataset_id
                or snapshot['episode_id'] != episode_id
            ):
                raise web.HTTPServiceUnavailable(
                    text='collection identity changed while aborting'
                )
            if snapshot['status'] in {'CANCELED', 'FAILED'}:
                return snapshot
            if snapshot['status'] == 'SUCCEEDED':
                raise web.HTTPConflict(
                    text='collection completed before abort could retain '
                         'an incomplete episode'
                )
            if asyncio.get_running_loop().time() >= deadline:
                detail = (
                    f': {acknowledgment_error}'
                    if acknowledgment_error is not None
                    else ''
                )
                raise web.HTTPGatewayTimeout(
                    text='collection abort did not reach a terminal state'
                         f'{detail}'
                )
            await asyncio.sleep(0.02)


async def _await_rclpy_future(future, timeout_s: float) -> None:
    deadline = asyncio.get_running_loop().time() + timeout_s
    while not future.done():
        if asyncio.get_running_loop().time() >= deadline:
            raise web.HTTPGatewayTimeout(text='ROS request timed out')
        await asyncio.sleep(0.02)


async def _wait_until_ready(probe, timeout_s: float) -> bool:
    """Poll ROS discovery without blocking the aiohttp event loop."""
    deadline = asyncio.get_running_loop().time() + timeout_s
    while not probe():
        if asyncio.get_running_loop().time() >= deadline:
            return False
        await asyncio.sleep(0.02)
    return True


def _image_array(message: Image) -> np.ndarray:
    """Convert the supported ROS image encodings to OpenCV BGR/mono arrays."""
    channels = {
        'mono8': 1, 'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4,
    }.get(message.encoding.lower())
    if channels is None:
        raise ValueError(f'unsupported image encoding: {message.encoding}')
    expected = int(message.height) * int(message.step)
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if message.height <= 0 or message.width <= 0 or raw.size < expected:
        raise ValueError('image data is incomplete')
    rows = raw[:expected].reshape((message.height, message.step))
    pixels = rows[:, :message.width * channels]
    if channels == 1:
        return pixels.reshape((message.height, message.width))
    frame = pixels.reshape((message.height, message.width, channels))
    encoding = message.encoding.lower()
    if encoding == 'rgb8':
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if encoding == 'rgba8':
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    if encoding == 'bgra8':
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


def _quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


def _transform_point_xyz(transform, point) -> tuple[float, float, float]:
    """Apply a full 3-D TF transform, including an upside-down lidar roll."""
    q = transform.rotation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 0.0:
        raise ValueError('transform quaternion has zero norm')
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    px, py, pz = (float(value) for value in point)
    rotated = (
        (1.0 - 2.0 * (y * y + z * z)) * px
        + 2.0 * (x * y - z * w) * py
        + 2.0 * (x * z + y * w) * pz,
        2.0 * (x * y + z * w) * px
        + (1.0 - 2.0 * (x * x + z * z)) * py
        + 2.0 * (y * z - x * w) * pz,
        2.0 * (x * z - y * w) * px
        + 2.0 * (y * z + x * w) * py
        + (1.0 - 2.0 * (x * x + y * y)) * pz,
    )
    return (
        rotated[0] + transform.translation.x,
        rotated[1] + transform.translation.y,
        rotated[2] + transform.translation.z,
    )


class ConsoleApplication:
    """aiohttp application for the workspace selected by the launch profile."""

    def __init__(self, node: OperatorConsoleNode):
        self.node = node
        self._teleop_socket: web.WebSocketResponse | None = None
        self._teleop_token = None
        self._manual_reservation_lock = asyncio.Lock()

    def build(self) -> web.Application:
        app = web.Application(client_max_size=64 * 1024)
        app.add_routes([
            web.get('/', self.index),
            web.get('/api/v1/bootstrap', self.bootstrap),
            web.get('/api/v1/health', self.health),
            web.get('/api/v1/events', self.events),
            web.get('/api/v1/tasks', self.tasks),
            web.get('/api/v1/tasks/current', self.current_task),
            web.post('/api/v1/tasks/fetch-deliver', self.submit_task),
            web.delete('/api/v1/tasks/{task_id}', self.cancel_task),
            web.post('/api/v1/operator/localize', self.operator_localize),
            web.post('/api/v1/operator/navigate', self.operator_navigate),
            web.post('/api/v1/operator/stop-base', self.operator_stop_base),
            web.delete('/api/v1/operator/stop-base', self.operator_clear_base_stop),
            web.post('/api/v1/operator/preset', self.operator_preset),
            web.get('/api/v1/cameras/{camera_id}/stream', self.camera_stream),
            web.get('/api/v1/mapping/state', self.mapping_state),
            web.get('/api/v1/sites', self.sites),
            web.post('/api/v1/mapping/sessions', self.save_mapping_session),
            web.post('/api/v1/mapping/reset', self.reset_mapping),
            web.post('/api/v1/sites/{site_id}/places', self.set_place),
            web.delete(
                '/api/v1/sites/{site_id}/places/{place_id}', self.remove_place
            ),
            web.post('/api/v1/sites/{site_id}/validations', self.record_validation),
            web.post(
                '/api/v1/sites/{site_id}/validate-localization',
                self.validate_localization,
            ),
            web.post(
                '/api/v1/sites/{site_id}/validate-place', self.validate_place
            ),
            web.post('/api/v1/sites/{site_id}/activate', self.activate_site),
            web.get('/api/v1/teleop/base', self.base_teleop),
            web.post('/api/v1/calibrations/jobs', self.calibration_job),
            web.post('/api/v1/calibrations/imports', self.import_calibration),
            web.post(
                '/api/v1/calibrations/base-geometry',
                self.save_base_geometry_calibration,
            ),
            web.post(
                '/api/v1/calibrations/samples',
                self.capture_calibration_sample,
            ),
            web.get(
                '/api/v1/calibrations/samples',
                self.calibration_sample_coverage,
            ),
            web.post(
                '/api/v1/calibrations/solve',
                self.solve_calibration_samples,
            ),
            web.post(
                '/api/v1/calibrations/move-pose',
                self.move_calibration_pose,
            ),
            web.post(
                '/api/v1/calibrations/servo/step',
                self.servo_calibration_step,
            ),
            web.post(
                '/api/v1/calibrations/{unit_id}/activate',
                self.activate_calibration,
            ),
            web.post('/api/v1/collections/sessions', self.start_collection),
            web.get('/api/v1/collections/current', self.current_collection),
            web.post(
                '/api/v1/datasets/{dataset_id}/episodes/'
                '{episode_id}/finalize',
                self.finalize_collection,
            ),
            web.delete(
                '/api/v1/datasets/{dataset_id}/episodes/{episode_id}',
                self.cancel_collection,
            ),
            web.post(
                '/api/v1/datasets/{dataset_id}/episodes/{episode_id}/review',
                self.review_episode,
            ),
        ])
        static = self._static_root()
        if static and static.exists():
            # `colcon build --symlink-install` installs Vite's hashed assets as
            # symlinks into the source/build tree.  aiohttp rejects those files
            # by default because they resolve outside the static directory,
            # which leaves the browser with an index page but no JS or CSS.
            app.router.add_static(
                '/assets/', static / 'assets', name='assets', follow_symlinks=True
            )
        return app

    @staticmethod
    def _static_root() -> Path | None:
        try:
            return Path(get_package_share_directory('xlerobot_hmi')) / 'web_dist'
        except Exception:
            return None

    async def index(self, _request):
        root = self._static_root()
        if root and (root / 'index.html').is_file():
            return web.FileResponse(root / 'index.html')
        return web.Response(
            text='XLeRobot Operator Console frontend has not been built. '
                 'Run the documented Vite build step.',
            status=503,
        )

    async def bootstrap(self, _request):
        engineering = bool(self.node.parameter('enable_engineering_tools'))
        available = available_workspaces(str(self.node.parameter('workspace')))
        return web.json_response({
            'release': self.node.parameter('release_id'),
            'unit': self.node.parameter('unit_id'),
            'default_dataset_id': self.node.parameter('default_dataset_id'),
            'site': self.node.parameter('site_id'),
            'workspace': self.node.parameter('workspace'),
            'mapping_phase': self.node.parameter('mapping_phase'),
            'calibration_workflow': self.node.parameter('calibration_workflow'),
            'calibration_capture_only': bool(
                self.node.parameter('calibration_capture_only')
            ),
            'engineering_tools_enabled': engineering,
            'available_workspaces': available,
            'named_places': self.node.named_places,
            'active_task': self.node.task_snapshot(),
            'mapping': self.node.mapping_snapshot(),
            'perception': self.node.perception_snapshot(),
            'voice_transcript': self.node.voice_transcript,
            'collection': self.node.collection_snapshot(),
            'drive_stop_latched': self.node.drive_stop_latched,
        })

    async def health(self, _request):
        return web.json_response(self.node.health())

    async def tasks(self, request):
        return web.json_response({
            'tasks': self.node.history.recent(request.query.get('limit', 20))
        })

    async def current_task(self, _request):
        return web.json_response({'task': self.node.task_snapshot()})

    async def submit_task(self, request):
        try:
            goal = parse_task_request(await request.json())
        except (ValueError, json.JSONDecodeError) as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc
        return web.json_response(await self.node.submit_task(goal), status=202)

    async def cancel_task(self, request):
        snapshot = await self.node.cancel_task(request.match_info['task_id'])
        return web.json_response(snapshot)

    def _require_operator(self) -> None:
        self._require_workspace('operator')

    async def _request_manual_reservation(self, enabled: bool) -> None:
        request = SetBool.Request()
        request.data = enabled
        response = await self._call_service(
            self.node.manual_reservation_client, request, 5.0
        )
        if not response.success:
            if enabled:
                raise web.HTTPConflict(
                    text=response.message or 'manual control reservation rejected'
                )
            raise web.HTTPServiceUnavailable(
                text=response.message or 'manual control reservation was not released'
            )

    async def _set_manual_reservation(self, enabled: bool) -> None:
        async with self._manual_reservation_lock:
            await self._request_manual_reservation(enabled)
            self.node.manual_control.set_reserved(enabled)

    async def _release_teleop_session(self, token, reserved: bool) -> bool:
        """Release the server reservation before exposing local idle state."""
        async with self._manual_reservation_lock:
            if (
                self._teleop_token is not token
                or not self.node.manual_control.owns_teleop(token)
            ):
                return True
            if reserved:
                try:
                    await self._request_manual_reservation(False)
                    self.node.manual_control.set_reserved(False)
                except Exception as error:
                    self.node.get_logger().error(
                        'Unable to release manual teleop reservation; '
                        f'keeping local ownership fail-closed: {error}'
                    )
                    return False
            self._teleop_token = None
            self.node.release_teleop(token)
        return True

    async def _run_operator_action(
        self, client, goal, label: str, timeout_s: float
    ):
        if self.node.drive_stop_latched is not False:
            raise web.HTTPConflict(text='clear the base software inhibit first')
        if self.node.task_active():
            raise web.HTTPConflict(text='wait for the active Fetch-Deliver task first')
        if self.node.teleop_active():
            raise web.HTTPConflict(text='release base teleop first')
        return await self._run_owned_action(
            client, goal, label, timeout_s, reserve_manual_control=True
        )

    async def _run_calibration_action(
        self, client, goal, label: str, timeout_s: float
    ):
        """Run one isolated calibration action without the demo task lease."""
        return await self._run_owned_action(
            client, goal, label, timeout_s, reserve_manual_control=False
        )

    async def _run_owned_action(
        self,
        client,
        goal,
        label: str,
        timeout_s: float,
        *,
        reserve_manual_control: bool,
    ):
        owner = self.node.manual_control.begin_action()
        if owner is None:
            raise web.HTTPConflict(text='another manual operation is active')
        reserved = False
        terminal_observed = True
        response_error: web.HTTPException | None = None
        result = None
        result_status = GoalStatus.STATUS_UNKNOWN
        try:
            if reserve_manual_control:
                await self._set_manual_reservation(True)
                reserved = True
            if not await _wait_until_ready(client.server_is_ready, 2.0):
                raise web.HTTPServiceUnavailable(text=f'{label} action is unavailable')
            future = client.send_goal_async(goal)
            terminal_observed = False
            self.node.manual_control.set_goal_future(owner, future)
            accept_deadline = asyncio.get_running_loop().time() + 5.0
            while not future.done():
                if (
                    response_error is None
                    and self.node.manual_control.cancel_requested()
                ):
                    response_error = web.HTTPConflict(
                        text=f'{label} was preempted by a task or base stop'
                    )
                if (
                    response_error is None
                    and asyncio.get_running_loop().time() >= accept_deadline
                ):
                    response_error = web.HTTPGatewayTimeout(
                        text=f'{label} goal response timed out; draining pending goal'
                    )
                    self.node.manual_control.request_cancel()
                await asyncio.sleep(0.02)
            handle = future.result()
            if handle is None or not handle.accepted:
                terminal_observed = True
                if response_error is not None:
                    raise response_error
                raise web.HTTPConflict(text=f'{label} goal rejected')
            self.node.manual_control.set_goal_handle(owner, handle)
            result_future = handle.get_result_async()
            self.node.manual_control.set_result_future(owner, result_future)
            action_deadline = asyncio.get_running_loop().time() + timeout_s
            cancel_sent = False
            while not result_future.done():
                if (
                    response_error is None
                    and self.node.manual_control.cancel_requested()
                ):
                    response_error = web.HTTPConflict(
                        text=f'{label} was preempted by a task or base stop'
                    )
                if (
                    response_error is None
                    and asyncio.get_running_loop().time() >= action_deadline
                ):
                    response_error = web.HTTPGatewayTimeout(
                        text=f'{label} timed out and was canceled'
                    )
                    self.node.manual_control.request_cancel()
                if self.node.manual_control.cancel_requested() and not cancel_sent:
                    cancel_sent = True
                    try:
                        cancel_future = handle.cancel_goal_async()
                        await _await_rclpy_future(cancel_future, 5.0)
                        cancel_future.result()
                    except Exception as error:  # Keep draining the result future.
                        self.node.get_logger().error(
                            f'{label} cancel request failed while draining: {error}'
                        )
                        if response_error is None:
                            response_error = web.HTTPServiceUnavailable(
                                text=f'{label} cancellation could not be confirmed; '
                                     'waiting for terminal state'
                            )
                await asyncio.sleep(0.02)
            wrapped = result_future.result()
            result_status = wrapped.status
            if result_status not in {
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            }:
                response_error = web.HTTPServiceUnavailable(
                    text=f'{label} returned non-terminal action status '
                         f'{result_status}; restart the demo profile'
                )
            else:
                terminal_observed = True
                result = wrapped.result
        finally:
            # Never expose local idle while an accepted goal lacks a terminal
            # result. Calibration has no task-runtime lease, so this local
            # ownership marker is its only crash/failure-closed boundary.
            if terminal_observed:
                if reserved:
                    await self._set_manual_reservation(False)
                self.node.manual_control.finish_action(owner)
        if response_error is not None:
            raise response_error
        if result_status != GoalStatus.STATUS_SUCCEEDED:
            raise web.HTTPConflict(
                text=f'{label} ended with action status {result_status}'
            )
        self._require_capability_success(result.error)
        return result

    async def operator_localize(self, _request):
        self._require_operator()
        goal = AutoLocalize.Goal()
        goal.dry_run = False
        result = await self._run_operator_action(
            self.node.localize_client, goal, 'AutoLocalize', 180.0
        )
        payload = {
            'message': result.error.message,
            'position_stddev_m': result.position_stddev_m,
            'yaw_stddev_rad': result.yaw_stddev_rad,
        }
        self.node.history.audit('operator', 'manual.localize', 'success', payload)
        return web.json_response(payload)

    async def operator_navigate(self, request):
        self._require_operator()
        payload = await request.json()
        place_id = str(payload.get('place_id', '')).strip()
        if not IDENTIFIER.fullmatch(place_id):
            raise web.HTTPBadRequest(text='invalid place_id')
        goal = NavigateToNamedPlace.Goal()
        goal.place_id = place_id
        goal.dry_run = False
        result = await self._run_operator_action(
            self.node.named_navigation_client, goal, 'NavigateToNamedPlace', 180.0
        )
        evidence = {'place_id': place_id, 'message': result.error.message}
        self.node.history.audit('operator', 'manual.navigate', 'success', evidence)
        return web.json_response(evidence)

    async def operator_stop_base(self, _request):
        self._require_operator()
        await self._set_drive_stop(True)
        self.node.stop_teleop()
        if self._teleop_socket is not None:
            await self._teleop_socket.close(
                code=1001, message=b'base software inhibit'
            )
        canceled_task_ids = set()
        deadline = asyncio.get_running_loop().time() + 15.0
        idle_since = None
        try:
            while True:
                self.node.request_manual_cancel()
                task = self.node.task_snapshot()
                if task and self.node.task_active():
                    idle_since = None
                    task_id = task['task_id']
                    if task_id not in canceled_task_ids:
                        canceled_task_ids.add(task_id)
                        await self.node.cancel_task(task_id)
                elif (
                    self.node.manual_action_active()
                    or self.node.teleop_active()
                ):
                    idle_since = None
                    if (
                        self.node.teleop_active()
                        and self._teleop_socket is None
                        and self._teleop_token is not None
                    ):
                        await self._release_teleop_session(
                            self._teleop_token, reserved=True
                        )
                else:
                    now = asyncio.get_running_loop().time()
                    if idle_since is None:
                        idle_since = now
                    elif now - idle_since >= 0.5:
                        break
                if asyncio.get_running_loop().time() >= deadline:
                    raise web.HTTPGatewayTimeout(
                        text='task or manual control did not reach a stable '
                             'terminal state; the base software inhibit '
                             'remains latched'
                    )
                await asyncio.sleep(0.02)
        except web.HTTPException:
            self.node.history.audit(
                'operator', 'manual.stop_base', 'blocked',
                {
                    'canceled_goal_count': len(canceled_task_ids),
                    'latched': True,
                },
            )
            raise
        self.node.history.audit('operator', 'manual.stop_base', 'success', {
            'canceled_goal_count': len(canceled_task_ids), 'latched': True,
            'kind': 'base_software_inhibit',
        })
        return web.json_response({
            'status': 'latched',
            'latched': True,
            'kind': 'base_software_inhibit',
            'is_emergency_stop': False,
            'canceled_goal_count': len(canceled_task_ids),
        })

    async def operator_clear_base_stop(self, _request):
        self._require_operator()
        if not self.node.operator_idle():
            raise web.HTTPConflict(
                text='base software inhibit can be cleared only while idle'
            )
        await self._set_manual_reservation(True)
        try:
            await self._set_drive_stop(False)
            await self._set_manual_reservation(False)
        except Exception:
            try:
                await self._set_drive_stop(True)
            finally:
                try:
                    await self._set_manual_reservation(False)
                except web.HTTPException:
                    pass
            raise
        self.node.history.audit(
            'operator', 'manual.clear_base_stop', 'success',
            {'latched': False, 'kind': 'base_software_inhibit'},
        )
        return web.json_response({
            'status': 'cleared',
            'latched': False,
            'kind': 'base_software_inhibit',
            'is_emergency_stop': False,
        })

    async def _set_drive_stop(self, enabled: bool) -> None:
        request = SetBool.Request()
        request.data = enabled
        response = await self._call_service(
            self.node.drive_stop_client, request, 5.0
        )
        if not response.success:
            raise web.HTTPServiceUnavailable(
                text=response.message or 'base software inhibit request failed'
            )
        self.node.drive_stop_latched = enabled
        self.node.events.publish('drive_stop', {
            'latched': enabled, 'kind': 'base_software_inhibit',
        })
        self.node.events.publish('health', self.node.health())

    async def operator_preset(self, request):
        self._require_operator()
        payload = await request.json()
        preset = str(payload.get('preset', '')).strip()
        if preset not in {'arm_ready', 'head_ready', 'gripper_open'}:
            raise web.HTTPBadRequest(text='invalid maintenance preset')
        goal = SetMaintenancePreset.Goal()
        goal.preset = preset
        goal.dry_run = False
        result = await self._run_operator_action(
            self.node.maintenance_preset_client,
            goal,
            'SetMaintenancePreset',
            30.0,
        )
        evidence = {'preset': preset, 'message': result.error.message}
        self.node.history.audit('operator', 'manual.preset', 'success', evidence)
        return web.json_response(evidence)

    async def events(self, request):
        response = web.StreamResponse(
            headers={
                'Content-Type': 'text/event-stream',
                'Cache-Control': 'no-cache',
                'Connection': 'keep-alive',
            }
        )
        await response.prepare(request)
        client = self.node.events.subscribe()
        snapshot = {
            'type': 'snapshot',
            'data': {
                'health': self.node.health(),
                'task': self.node.task_snapshot(),
                'history': self.node.history.recent(),
                'mapping': self.node.mapping_snapshot(),
                'perception': self.node.perception_snapshot(),
                'voice_transcript': self.node.voice_transcript,
                'collection': self.node.collection_snapshot(),
                'drive_stop_latched': self.node.drive_stop_latched,
            },
        }
        await response.write(
            f'data: {json.dumps(snapshot, ensure_ascii=False)}\n\n'.encode()
        )
        try:
            while True:
                try:
                    event = await asyncio.wait_for(client[1].get(), timeout=15.0)
                    payload = json.dumps(event, ensure_ascii=False)
                    await response.write(f'data: {payload}\n\n'.encode())
                except asyncio.TimeoutError:
                    await response.write(b': heartbeat\n\n')
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            self.node.events.unsubscribe(client)
        return response

    async def camera_stream(self, request):
        camera_id = request.match_info['camera_id']
        if camera_id not in {'head', 'wrist', 'detection'}:
            raise web.HTTPNotFound(text='unknown camera')
        response = web.StreamResponse(headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
            'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0',
            'Pragma': 'no-cache',
            'Connection': 'keep-alive',
            'X-Accel-Buffering': 'no',
        })
        await response.prepare(request)
        last_sequence = -1
        try:
            while True:
                frame = self.node.camera_frame(camera_id)
                if frame and frame[0] != last_sequence:
                    last_sequence = frame[0]
                    jpeg = frame[1]
                    await response.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                        + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n'
                    )
                await asyncio.sleep(0.05)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    def _require_engineering_workspace(self, _request: web.Request) -> None:
        if not bool(self.node.parameter('enable_engineering_tools')):
            raise web.HTTPNotFound(text='engineering tools are disabled')

    async def mapping_state(self, request):
        self._require_engineering_workspace(request)
        state = self.node.mapping_snapshot()
        if state is None:
            raise web.HTTPNotFound(text='mapping workspace is not active')
        return web.json_response(state)

    async def sites(self, request):
        self._require_engineering_workspace(request)
        site_id = request.query.get('site_id', '').strip()
        if not site_id:
            return web.json_response({'sites': []})
        try:
            # Catalog validates the identifier before using it as a path.
            versions = self.node.catalog.versions('sites', site_id)
            active = self.node.catalog.active('sites', site_id)
        except ValueError as error:
            raise web.HTTPBadRequest(text=str(error)) from error
        draft = self.node.artifact_root / 'sites' / '.drafts' / site_id
        try:
            session_path = draft / 'mapping_session.json'
            session = json.loads(session_path.read_text()) if session_path.is_file() else {}
            validation_path = draft / 'validation.json'
            validation = (
                json.loads(validation_path.read_text())
                if validation_path.is_file() else {}
            )
            places_path = draft / 'places.yaml'
            document = (
                yaml.safe_load(places_path.read_text()) or {}
                if places_path.is_file() else {}
            )
            raw_places = document.get('named_places', {}).get('places', {})
            places = load_named_places(str(places_path))
            for place in places:
                place['dock'] = bool(raw_places[place['id']].get('dock', False))
                place['validated'] = bool(
                    validation.get('places', {}).get(place['id'], {}).get('passed', False)
                )
        except (OSError, ValueError, TypeError, AttributeError, yaml.YAMLError) as error:
            raise web.HTTPConflict(text=f'cannot read site draft: {error}') from error
        map_saved = (draft / 'map.yaml').is_file()
        return web.json_response({
            'site_id': site_id,
            'active_version': active.version if active else '',
            'versions': [
                {'version': item.version, 'uri': item.uri} for item in versions
            ],
            'draft': {
                'map_saved': map_saved,
                'map_name': session.get('map_name', ''),
                'map_saved_at': session.get('map_saved_at', ''),
                'places': places,
                'ready': bool(map_saved and places and all(
                    place['validated'] for place in places
                )),
            },
        })

    async def reset_mapping(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('mapping')
        if str(self.node.parameter('mapping_phase')) != 'build':
            raise web.HTTPNotFound(text='map reset is available only while building a map')
        payload = await request.json()
        if not isinstance(payload, dict) or payload.get('confirm') is not True:
            raise web.HTTPBadRequest(text='explicit confirm=true is required to reset the live map')
        if self.node.task_active():
            raise web.HTTPConflict(text='wait for the active task to finish')
        # Excludes teleop and other manual operations, including a second tab.
        owner = self.node.manual_control.begin_action()
        if owner is None:
            raise web.HTTPConflict(text='end teleoperation and wait for other manual operations first')
        try:
            response = await self._call_service(
                self.node.map_resetter, Reset.Request(pause_new_measurements=False), 10.0,
            )
            if response.result != Reset.Response.RESULT_SUCCESS:
                raise web.HTTPConflict(text='SLAM rejected the map reset; no success was confirmed')
            self.node.reset_mapping_preview()
            self.node.history.audit('operator', 'mapping.reset', 'success', {
                'scope': 'live_slam_only', 'saved_assets_preserved': True,
            })
            return web.json_response({
                'status': 'reset', 'saved_assets_preserved': True,
                'mapping': self.node.mapping_snapshot(),
            })
        finally:
            self.node.manual_control.finish_action(owner)

    async def save_mapping_session(self, request):
        self._require_engineering_workspace(request)
        payload = await request.json()
        site_id = str(payload.get('site_id', '')).strip()
        map_name = str(payload.get('map_name', 'map')).strip()
        if not site_id or not map_name:
            raise web.HTTPBadRequest(text='site_id and map_name are required')
        export = self.node.artifact_root / 'console' / 'map_exports' / secrets.token_hex(8)
        export.mkdir(parents=True, exist_ok=False)
        prefix = export / 'map'
        try:
            if not await _wait_until_ready(
                self.node.map_saver.service_is_ready, 2.0
            ):
                raise web.HTTPServiceUnavailable(text='SLAM map saver is unavailable')
            save = SaveMap.Request()
            save.name.data = str(prefix)
            future = self.node.map_saver.call_async(save)
            await _await_rclpy_future(future, 15.0)
            if future.result().result != SaveMap.Response.RESULT_SUCCESS:
                raise web.HTTPConflict(text='SLAM map saver reported failure')
            response = await self._call_service(
                self.node.site_save_client,
                SaveSiteMap.Request(
                    site_id=site_id, map_name=map_name,
                    source_map_yaml=str(prefix) + '.yaml',
                ),
                10.0,
            )
            self._require_capability_success(response.error)
            self.node.history.audit('operator', 'mapping.save', 'success', {
                'site_id': site_id, 'map_name': map_name,
            })
            return web.json_response({
                'site_id': site_id, 'version': response.version,
                'artifact_uri': response.artifact_uri,
            }, status=201)
        finally:
            shutil.rmtree(export, ignore_errors=True)

    async def set_place(self, request):
        self._require_engineering_workspace(request)
        payload = await request.json()
        pose = self.node.mapping_snapshot().get('pose')
        if pose is None:
            raise web.HTTPConflict(text='map to base_link pose is unavailable')
        message = SetNamedPlace.Request()
        message.site_id = request.match_info['site_id']
        message.place_id = str(payload.get('place_id', '')).strip()
        message.pose = PoseStamped()
        message.pose.header.frame_id = 'map'
        message.pose.pose.position.x = float(pose['x'])
        message.pose.pose.position.y = float(pose['y'])
        half_yaw = float(pose['yaw']) * 0.5
        message.pose.pose.orientation.z = math.sin(half_yaw)
        message.pose.pose.orientation.w = math.cos(half_yaw)
        message.nav_offset_m = float(payload.get('nav_offset_m', 0.0))
        message.dock = bool(payload.get('dock', False))
        if not message.place_id:
            raise web.HTTPBadRequest(text='place_id is required')
        response = await self._call_service(
            self.node.site_place_client, message, 5.0
        )
        self._require_capability_success(response.error)
        self.node.history.audit('operator', 'site.place.set', 'success', {
            'site_id': message.site_id, 'place_id': message.place_id,
        })
        return web.json_response({'artifact_uri': response.artifact_uri}, status=201)

    async def record_validation(self, request):
        self._require_engineering_workspace(request)
        payload = await request.json()
        message = RecordSiteValidation.Request()
        message.site_id = request.match_info['site_id']
        message.place_id = str(payload.get('place_id', '')).strip()
        for field_name in ('localization_passed', 'navigation_passed', 'dock_passed'):
            setattr(message, field_name, bool(payload.get(field_name, False)))
        for field_name in ('position_error_m', 'yaw_error_rad', 'duration_s'):
            setattr(message, field_name, float(payload.get(field_name, 0.0)))
        message.message = str(payload.get('message', ''))
        response = await self._call_service(
            self.node.site_validation_client, message, 5.0
        )
        self._require_capability_success(response.error)
        return web.json_response({
            'artifact_uri': response.artifact_uri,
            'site_ready': response.site_ready,
        })

    async def remove_place(self, request):
        self._require_engineering_workspace(request)
        message = RemoveNamedPlace.Request(
            site_id=request.match_info['site_id'],
            place_id=request.match_info['place_id'],
        )
        response = await self._call_service(
            self.node.site_remove_place_client, message, 5.0
        )
        self._require_capability_success(response.error)
        self.node.history.audit('operator', 'site.place.remove', 'success', {
            'site_id': message.site_id, 'place_id': message.place_id,
        })
        return web.json_response({'artifact_uri': response.artifact_uri})

    async def activate_site(self, request):
        self._require_engineering_workspace(request)
        payload = await request.json()
        message = ActivateArtifact.Request()
        message.artifact_type = 'site'
        message.artifact_id = request.match_info['site_id']
        message.version = str(payload.get('version', ''))
        response = await self._call_service(
            self.node.site_activate_client, message, 10.0
        )
        self._require_capability_success(response.error)
        self.node.history.audit('operator', 'site.activate', 'success', {
            'site_id': message.artifact_id, 'version': message.version or 'draft',
        })
        return web.json_response({'artifact_uri': response.artifact_uri})

    async def base_teleop(self, request):
        workspace = str(self.node.parameter('workspace'))
        if workspace == 'mapping':
            self._require_engineering_workspace(request)
            if not self.node._mapping_enabled():
                raise web.HTTPNotFound(text='mapping teleop is not active')
        elif workspace != 'operator':
            raise web.HTTPNotFound(text='base teleop is not active')
        if workspace == 'operator' and self.node.drive_stop_latched is not False:
            raise web.HTTPConflict(text='clear the base software inhibit first')
        token = object()
        if not self.node.claim_teleop(token):
            raise web.HTTPConflict(
                text='another task or manual control session is active'
            )
        reserved = False
        if workspace == 'operator':
            try:
                await self._set_manual_reservation(True)
                reserved = True
            except Exception:
                self.node.release_teleop(token)
                raise
        socket = web.WebSocketResponse(heartbeat=10.0, max_msg_size=4096)
        self._teleop_socket = socket
        self._teleop_token = token
        try:
            await socket.prepare(request)
        except Exception:
            self._teleop_socket = None
            await self._release_teleop_session(token, reserved)
            raise
        self.node.history.audit('operator', 'teleop.connect', 'success')
        try:
            while not socket.closed:
                try:
                    message = await socket.receive(
                        timeout=float(self.node.parameter('teleop_timeout_s'))
                    )
                except asyncio.TimeoutError:
                    self.node.history.audit(
                        'operator', 'teleop.watchdog', 'stopped'
                    )
                    break
                if message.type == web.WSMsgType.TEXT:
                    try:
                        command = json.loads(message.data)
                        if not bool(command.get('armed', False)):
                            break
                        self.node.command_teleop(
                            float(command.get('linear', 0.0)),
                            float(command.get('angular', 0.0)),
                            True,
                        )
                    except (
                        TypeError, ValueError, RuntimeError, json.JSONDecodeError
                    ) as error:
                        self.node.stop_teleop()
                        await socket.send_json({'error': str(error)})
                        break
                elif message.type in {
                    web.WSMsgType.CLOSE,
                    web.WSMsgType.CLOSED,
                    web.WSMsgType.ERROR,
                }:
                    break
        finally:
            self.node.stop_teleop()
            if not socket.closed:
                await socket.close()
            self._teleop_socket = None
            released = await self._release_teleop_session(token, reserved)
            self.node.history.audit(
                'operator',
                'teleop.disconnect',
                'stopped' if released else 'blocked',
                {'reservation_released': released},
            )
        return socket

    def _require_workspace(self, name: str) -> None:
        if str(self.node.parameter('workspace')) != name:
            raise web.HTTPNotFound(text=f'{name} workspace is not active')

    def _require_mapping_validation(self) -> None:
        self._require_workspace('mapping')
        if not self.node._mapping_validation_enabled():
            raise web.HTTPNotFound(text='mapping validation phase is not active')

    async def validate_localization(self, request):
        self._require_engineering_workspace(request)
        self._require_mapping_validation()
        if request.match_info['site_id'] != str(self.node.parameter('site_id')):
            raise web.HTTPBadRequest(text='site_id does not match the active draft')
        if not await _wait_until_ready(
            self.node.localize_client.server_is_ready, 2.0
        ):
            raise web.HTTPServiceUnavailable(text='AutoLocalize is unavailable')
        goal = AutoLocalize.Goal()
        goal.dry_run = False
        future = self.node.localize_client.send_goal_async(goal)
        await _await_rclpy_future(future, 5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise web.HTTPConflict(text='AutoLocalize goal rejected')
        result_future = handle.get_result_async()
        await _await_rclpy_future(result_future, 180.0)
        result = result_future.result().result
        self._require_capability_success(result.error)
        self.node._mapping_localized = True
        payload = {
            'position_stddev_m': result.position_stddev_m,
            'yaw_stddev_rad': result.yaw_stddev_rad,
            'message': result.error.message,
        }
        self.node.history.audit(
            'operator', 'site.validate.localization', 'success', payload
        )
        return web.json_response(payload)

    async def validate_place(self, request):
        self._require_engineering_workspace(request)
        self._require_mapping_validation()
        site_id = request.match_info['site_id']
        if site_id != str(self.node.parameter('site_id')):
            raise web.HTTPBadRequest(text='site_id does not match the active draft')
        if not self.node._mapping_localized:
            raise web.HTTPConflict(text='run AutoLocalize before validating a place')
        payload = await request.json()
        place_id = str(payload.get('place_id', '')).strip()
        places_path = (
            self.node.artifact_root / 'sites' / '.drafts' / site_id / 'places.yaml'
        )
        try:
            places = (yaml.safe_load(places_path.read_text(encoding='utf-8')) or {})[
                'named_places'
            ]['places']
            desired = places[place_id]
        except (FileNotFoundError, KeyError, TypeError) as error:
            raise web.HTTPBadRequest(text=f'unknown draft place: {place_id}') from error
        if not await _wait_until_ready(
            self.node.named_navigation_client.server_is_ready, 2.0
        ):
            raise web.HTTPServiceUnavailable(text='named navigation is unavailable')
        goal = NavigateToNamedPlace.Goal()
        goal.place_id = place_id
        goal.dry_run = False
        started = time.monotonic()
        future = self.node.named_navigation_client.send_goal_async(goal)
        await _await_rclpy_future(future, 5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise web.HTTPConflict(text='named navigation goal rejected')
        result_future = handle.get_result_async()
        await _await_rclpy_future(result_future, 180.0)
        result = result_future.result().result
        self._require_capability_success(result.error)
        reached = result.reached_pose.pose
        position_error = math.hypot(
            reached.position.x - float(desired['x']),
            reached.position.y - float(desired['y']),
        )
        yaw_error = abs(math.atan2(
            math.sin(_quaternion_yaw(reached.orientation) - float(desired['yaw'])),
            math.cos(_quaternion_yaw(reached.orientation) - float(desired['yaw'])),
        ))
        validation = RecordSiteValidation.Request()
        validation.site_id = site_id
        validation.place_id = place_id
        validation.localization_passed = True
        validation.navigation_passed = True
        validation.dock_passed = bool(desired.get('dock', False))
        validation.position_error_m = position_error
        validation.yaw_error_rad = yaw_error
        validation.duration_s = time.monotonic() - started
        validation.message = result.error.message
        response = await self._call_service(
            self.node.site_validation_client, validation, 5.0
        )
        self._require_capability_success(response.error)
        evidence = {
            'place_id': place_id,
            'position_error_m': position_error,
            'yaw_error_rad': yaw_error,
            'duration_s': validation.duration_s,
            'site_ready': response.site_ready,
            'artifact_uri': response.artifact_uri,
        }
        self.node.history.audit(
            'operator', 'site.validate.place', 'success', evidence
        )
        return web.json_response(evidence)

    async def calibration_job(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('calibration')
        if bool(self.node.parameter('calibration_capture_only')):
            raise web.HTTPNotFound(text='capture-only calibration does not run preflight')
        payload = await request.json()
        goal = CalibrationJob.Goal()
        goal.unit_id = str(payload.get('unit_id', '')).strip()
        goal.workflow_id = str(payload.get('workflow_id', '')).strip()
        goal.automatic = bool(payload.get('automatic', False))
        goal.dry_run = bool(payload.get('dry_run', True))
        if not goal.unit_id or goal.workflow_id not in {
            'servo', 'base_geometry', 'head_camera', 'right_handeye'
        }:
            raise web.HTTPBadRequest(text='invalid unit_id or workflow_id')
        if goal.workflow_id != str(self.node.parameter('calibration_workflow')):
            raise web.HTTPNotFound(text='requested calibration tool is not active')
        if not await _wait_until_ready(
            self.node.calibration_client.server_is_ready, 2.0
        ):
            raise web.HTTPServiceUnavailable(text='CalibrationJob is unavailable')
        future = self.node.calibration_client.send_goal_async(goal)
        await _await_rclpy_future(future, 5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise web.HTTPConflict(text='calibration job rejected')
        result_future = handle.get_result_async()
        await _await_rclpy_future(result_future, 10.0)
        result = result_future.result().result
        self._require_capability_success(result.error)
        return web.json_response({
            'artifact_uri': result.artifact_uri,
            'quality_passed': result.quality_passed,
            'message': result.error.message,
        }, status=202)

    async def import_calibration(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('calibration')
        if bool(self.node.parameter('calibration_capture_only')):
            raise web.HTTPNotFound(text='capture-only calibration does not import results')
        payload = await request.json()
        message = ImportCalibrationResult.Request(
            unit_id=str(payload.get('unit_id', '')).strip(),
            workflow_id=str(payload.get('workflow_id', '')).strip(),
            source_uri=str(payload.get('source_uri', '')).strip(),
        )
        if message.workflow_id != str(self.node.parameter('calibration_workflow')):
            raise web.HTTPNotFound(text='requested calibration tool is not active')
        response = await self._call_service(
            self.node.calibration_import_client, message, 10.0
        )
        self._require_capability_success(response.error)
        self.node.history.audit('operator', 'calibration.import', 'success', {
            'unit_id': message.unit_id, 'workflow_id': message.workflow_id,
            'quality_passed': response.quality_passed,
        })
        return web.json_response({
            'draft_uri': response.draft_uri,
            'quality_passed': response.quality_passed,
            'metrics': dict(zip(response.metric_names, response.metric_values)),
        }, status=201)

    async def activate_calibration(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('calibration')
        if bool(self.node.parameter('calibration_capture_only')):
            raise web.HTTPNotFound(text='capture-only calibration does not activate bundles')
        payload = await request.json()
        message = ActivateArtifact.Request(
            artifact_type='calibration', artifact_id=request.match_info['unit_id'],
            version=str(payload.get('version', '')).strip(),
        )
        response = await self._call_service(
            self.node.calibration_activate_client, message, 10.0
        )
        self._require_capability_success(response.error)
        self.node.history.audit('operator', 'calibration.activate', 'success', {
            'unit_id': message.artifact_id, 'version': message.version or 'draft',
        })
        return web.json_response({'artifact_uri': response.artifact_uri})

    async def save_base_geometry_calibration(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('calibration')
        if str(self.node.parameter('calibration_workflow')) != 'base_geometry':
            raise web.HTTPNotFound(text='base geometry tool is not active')
        payload = await request.json()

        def numbers(name):
            value = payload.get(name)
            if not isinstance(value, list):
                raise web.HTTPBadRequest(text=f'{name} must be an array')
            try:
                return [float(item) for item in value]
            except (TypeError, ValueError) as error:
                raise web.HTTPBadRequest(
                    text=f'{name} contains a non-number'
                ) from error

        message = SaveBaseGeometryCalibration.Request()
        message.unit_id = str(payload.get('unit_id', '')).strip()
        try:
            message.nominal_wheel_radius_m = float(
                payload.get('nominal_wheel_radius_m')
            )
            message.nominal_wheel_separation_m = float(
                payload.get('nominal_wheel_separation_m')
            )
        except (TypeError, ValueError) as error:
            raise web.HTTPBadRequest(text='nominal geometry is invalid') from error
        message.straight_commanded_m = numbers('straight_commanded_m')
        message.straight_actual_m = numbers('straight_actual_m')
        message.rotation_commanded_rad = numbers('rotation_commanded_rad')
        message.rotation_actual_rad = numbers('rotation_actual_rad')
        response = await self._call_service(
            self.node.base_geometry_client, message, 10.0
        )
        self._require_capability_success(response.error)
        evidence = {
            'unit_id': message.unit_id,
            'quality_passed': response.quality_passed,
            'metrics': dict(zip(response.metric_names, response.metric_values)),
            'draft_uri': response.draft_uri,
        }
        self.node.history.audit(
            'operator', 'calibration.base_geometry.save', 'success', evidence
        )
        return web.json_response(evidence, status=201)

    async def capture_calibration_sample(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('calibration')
        workflow = str(self.node.parameter('calibration_workflow'))
        if workflow not in {'head_camera', 'right_handeye'}:
            raise web.HTTPNotFound(text='visual calibration tool is not active')
        payload = await request.json()
        unit_id = str(payload.get('unit_id', '')).strip()
        if not unit_id:
            raise web.HTTPBadRequest(text='unit_id is required')
        response = await self._call_service(
            self.node.calibration_capture_client,
            CaptureCalibrationSample.Request(job_id=f'{unit_id}:{workflow}'),
            5.0,
        )
        self._require_capability_success(response.error)
        evidence = {
            'sample_count': response.sample_count,
            'message': response.error.message,
        }
        self.node.history.audit(
            'operator', 'calibration.sample.capture', 'success', evidence
        )
        return web.json_response(evidence, status=201)

    async def calibration_sample_coverage(self, request):
        """Expose restored visual-capture facts from the atomic sample file."""
        self._require_engineering_workspace(request)
        self._require_workspace('calibration')
        workflow = str(self.node.parameter('calibration_workflow'))
        if workflow not in {'head_camera', 'right_handeye'}:
            raise web.HTTPNotFound(
                text='visual calibration sample coverage is not active'
            )
        from xlerobot_calibration_tools.sample_set import TransformSampleSet
        from xlerobot_calibration_tools.solver import ARM_MODEL, HEAD_MODEL

        sample_file = (
            self.node.artifact_root
            / 'calibration_work'
            / workflow
            / 'samples.yaml'
        )
        if not sample_file.exists():
            return web.json_response({
                'sample_count': 0,
                'points': [],
                'spans_m': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'max_pairwise_pose_angle_deg': 0.0,
            })
        try:
            samples = TransformSampleSet.read(
                sample_file,
                expected_model=(
                    HEAD_MODEL if workflow == 'head_camera' else ARM_MODEL
                ),
            )
            coverage = samples.coverage()
        except (OSError, TypeError, ValueError) as error:
            raise web.HTTPConflict(
                text=f'{workflow} samples are invalid: {error}'
            ) from error
        return web.json_response(coverage)

    async def servo_calibration_step(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('calibration')
        if str(self.node.parameter('calibration_workflow')) != 'servo':
            raise web.HTTPNotFound(text='servo calibration tool is not active')
        payload = await request.json()
        commands = {
            'scan': ServoCalibrationStep.Request.SCAN,
            'release_torque': ServoCalibrationStep.Request.RELEASE_TORQUE,
            'capture_zero': ServoCalibrationStep.Request.CAPTURE_ZERO,
            'start_range': ServoCalibrationStep.Request.START_RANGE,
            'finish_range': ServoCalibrationStep.Request.FINISH_RANGE,
            'finalize': ServoCalibrationStep.Request.FINALIZE,
        }
        command_name = str(payload.get('command', '')).strip()
        if command_name not in commands:
            raise web.HTTPBadRequest(text='unknown servo calibration command')
        unit_id = str(payload.get('unit_id', '')).strip()
        if not unit_id:
            raise web.HTTPBadRequest(text='unit_id is required')
        message = ServoCalibrationStep.Request(
            command=commands[command_name],
            group=str(payload.get('group', '')).strip(),
            joint=str(payload.get('joint', '')).strip(),
        )
        response = await self._call_service(
            self.node.servo_calibration_client, message, 10.0
        )
        self._require_capability_success(response.error)
        evidence = {
            'phase': response.phase,
            'joint_names': list(response.joint_names),
            'raw_positions': list(response.raw_positions),
            'result_uri': response.result_uri,
        }
        if (
            command_name == 'finalize'
            and not bool(self.node.parameter('calibration_capture_only'))
        ):
            imported = await self._call_service(
                self.node.calibration_import_client,
                ImportCalibrationResult.Request(
                    unit_id=unit_id,
                    workflow_id='servo',
                    source_uri=response.result_uri,
                ),
                10.0,
            )
            self._require_capability_success(imported.error)
            evidence.update({
                'draft_uri': imported.draft_uri,
                'quality_passed': imported.quality_passed,
                'metrics': dict(zip(
                    imported.metric_names, imported.metric_values
                )),
            })
        self.node.history.audit(
            'operator', f'calibration.servo.{command_name}', 'success', evidence
        )
        return web.json_response(evidence, status=201)

    async def solve_calibration_samples(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('calibration')
        if bool(self.node.parameter('calibration_capture_only')):
            raise web.HTTPNotFound(text='capture-only calibration does not solve samples')
        workflow = str(self.node.parameter('calibration_workflow'))
        if workflow not in {'head_camera', 'right_handeye'}:
            raise web.HTTPNotFound(text='visual calibration tool is not active')
        payload = await request.json()
        unit_id = str(payload.get('unit_id', '')).strip()
        if not unit_id:
            raise web.HTTPBadRequest(text='unit_id is required')
        response = await self._call_service(
            self.node.calibration_solve_client,
            SolveCalibrationSamples.Request(unit_id=unit_id),
            60.0,
        )
        self._require_capability_success(response.error)
        evidence = {
            'draft_uri': response.draft_uri,
            'quality_passed': response.quality_passed,
            'sample_count': response.sample_count,
            'metrics': dict(zip(response.metric_names, response.metric_values)),
        }
        self.node.history.audit(
            'operator', 'calibration.samples.solve', 'success', evidence
        )
        return web.json_response(evidence, status=201)

    async def move_calibration_pose(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('calibration')
        workflow = str(self.node.parameter('calibration_workflow'))
        if workflow not in {'head_camera', 'right_handeye'}:
            raise web.HTTPNotFound(text='visual calibration tool is not active')
        payload = await request.json()
        try:
            pose_index = int(payload.get('pose_index'))
        except (TypeError, ValueError) as error:
            raise web.HTTPBadRequest(text='pose_index must be an integer') from error
        pose_count = 13 if workflow == 'head_camera' else 20
        if pose_index < 0 or pose_index >= pose_count:
            raise web.HTTPBadRequest(text='pose_index is outside the verified set')
        goal = MoveCalibrationPose.Goal()
        goal.workflow_id = workflow
        goal.pose_index = pose_index
        goal.dry_run = False
        result = await self._run_calibration_action(
            self.node.calibration_pose_client,
            goal,
            'MoveCalibrationPose',
            20.0,
        )
        evidence = {
            'pose_index': pose_index,
            'pose_name': result.pose_name,
            'pose_count': result.pose_count,
            'message': result.error.message,
        }
        self.node.history.audit(
            'operator', 'calibration.pose.move', 'success', evidence
        )
        return web.json_response(evidence)

    async def start_collection(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('collection')
        payload = await request.json()
        goal = CollectEpisode.Goal()
        goal.template_id = str(payload.get('template_id', 'pick')).strip()
        goal.dataset_id = str(payload.get('dataset_id', '')).strip()
        goal.episode_id = str(payload.get('episode_id', '')).strip()
        goal.collection_profile_id = str(
            payload.get('collection_profile_id', 'two_wheel_pick')
        ).strip()
        goal.object_id = str(payload.get('object_id', '')).strip()
        goal.language_instruction = str(payload.get('language_instruction', '')).strip()
        try:
            goal.max_duration_s = float(payload.get('max_duration_s', 30.0))
        except (TypeError, ValueError) as error:
            raise web.HTTPBadRequest(
                text='collection duration must be numeric'
            ) from error
        goal.dry_run = bool(payload.get('dry_run', False))
        if (
            goal.template_id not in {'pick', 'manual'}
            or not math.isfinite(goal.max_duration_s)
            or goal.max_duration_s <= 0
            or goal.max_duration_s > 120.0
        ):
            raise web.HTTPBadRequest(text='invalid collection template or duration')
        return web.json_response(await self.node.submit_collection(goal), status=202)

    async def current_collection(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('collection')
        return web.json_response({'collection': self.node.collection_snapshot()})

    async def cancel_collection(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('collection')
        return web.json_response(await self.node.cancel_collection(
            request.match_info['dataset_id'],
            request.match_info['episode_id'],
        ))

    async def finalize_collection(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('collection')
        return web.json_response(await self.node.finalize_collection(
            request.match_info['dataset_id'],
            request.match_info['episode_id'],
        ), status=202)

    async def review_episode(self, request):
        self._require_engineering_workspace(request)
        self._require_workspace('collection')
        payload = await request.json()
        active = self.node.collection_snapshot()
        if (
            active is None
            or active.get('dataset_id') != request.match_info['dataset_id']
            or active.get('episode_id') != request.match_info['episode_id']
            or active.get('status') != 'SUCCEEDED'
            or active.get('dry_run')
            or not active.get('episode_uri')
        ):
            raise web.HTTPConflict(
                text='only the current completed real episode can be reviewed'
            )
        message = ReviewEpisode.Request(
            dataset_id=request.match_info['dataset_id'],
            episode_id=request.match_info['episode_id'],
            status=str(payload.get('status', '')).strip(),
            failure_reason=str(payload.get('failure_reason', '')),
            notes=str(payload.get('notes', '')),
            operator_id='operator',
        )
        response = await self._call_service(self.node.review_client, message, 5.0)
        self._require_capability_success(response.error)
        self.node.history.audit('operator', 'episode.review', 'success', {
            'dataset_id': message.dataset_id,
            'episode_id': message.episode_id,
            'status': message.status,
        })
        return web.json_response({'review_uri': response.review_uri})

    async def _call_service(self, client, request, timeout_s: float):
        if not await _wait_until_ready(client.service_is_ready, 2.0):
            raise web.HTTPServiceUnavailable(text='ROS service is unavailable')
        future = client.call_async(request)
        await _await_rclpy_future(future, timeout_s)
        return future.result()

    @staticmethod
    def _require_capability_success(error) -> None:
        if error.code != CapabilityError.NONE:
            raise web.HTTPConflict(text=f'[{error.code}] {error.message}')


def main():
    """Run the ROS executor and aiohttp server as one gateway process."""
    rclpy.init()
    node = OperatorConsoleNode()
    # ROS callbacks use the default mutually-exclusive group and must not
    # block waiting for actions (those awaits run in asyncio). Thread-pool
    # contention in this gateway starved the web loop and its 100 ms teleop
    # heartbeats under map updates. Keep one ROS dispatch thread.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    app = ConsoleApplication(node).build()
    try:
        web.run_app(
            app,
            host=str(node.parameter('bind_host')),
            port=int(node.parameter('port')),
            print=None,
        )
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown(timeout_sec=3.0)
            spin_thread.join(timeout=3.0)
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
