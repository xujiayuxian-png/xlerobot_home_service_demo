"""Read-only object grounding and explicit ACT/centroid/GPD plan selection."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import re
import threading

from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from xlerobot_interfaces.action import DetectObject
from xlerobot_interfaces.msg import (
    CapabilityError,
    PerceptionObservation,
    TopGraspPlan,
)
from xlerobot_perception.detection.sam2_client import (
    ClassicalServiceError,
    PromptedSam2Client,
)
from xlerobot_perception.execution_modes import validate_dry_run_mode
from xlerobot_perception.geometry.pinhole import project_pixel_to_point
from xlerobot_perception.geometry.transforms import transform_point
from xlerobot_perception.grasp.gpd_client import GpdClient
from xlerobot_perception.grasp.refine_cloud import (
    RefinementConfig,
    refine_object_cloud,
)
from xlerobot_perception.grasp.runtime_calibration import (
    load_classical_runtime_calibration,
)
from xlerobot_perception.grasp.top_grasp import (
    TopGraspConfig,
    camera_intrinsics,
    centroid_top_grasp,
    depth_to_u16_mm,
    gpd_top_grasp,
    transform_matrix,
)
from xlerobot_perception.observations import make_observation
from xlerobot_perception.rgbd.depth import depth_median
from xlerobot_perception.rgbd.frame_buffer import RgbdFrameBuffer
from xlerobot_perception.rgbd.frame_buffer import stamp_ns
from xlerobot_perception.rgbd.images import color_to_bgr
from xlerobot_perception.vlm.lmstudio import LmStudioVlmClient


_FRAME_PATTERN = re.compile(r'^[A-Za-z][A-Za-z0-9_/]*$')
GRASP_BACKENDS = frozenset({'act', 'centroid', 'gpd'})


@dataclass
class CapabilityFailure(RuntimeError):
    """Internal exception converted to a typed action result."""

    code: int
    message: str

    def __str__(self):
        return self.message


def validate_goal_fields(object_id: str, target_frame: str, grasp_backend: str) -> None:
    """Validate identifiers before reserving the single backend slot."""
    if not object_id.strip() or len(object_id) > 128:
        raise ValueError('object_id must contain 1 to 128 characters')
    if any(ord(character) < 32 for character in object_id):
        raise ValueError('object_id must not contain control characters')
    if not _FRAME_PATTERN.fullmatch(target_frame):
        raise ValueError('target_frame must be a nonempty relative ROS frame id')
    if grasp_backend not in GRASP_BACKENDS:
        raise ValueError('grasp_backend must be one of: act, centroid, gpd')


class DetectObjectNode(Node):
    """Localize an object without publishing any command or motion topic."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__('detect_object_vlm', parameter_overrides=parameter_overrides)
        self.group = ReentrantCallbackGroup()
        self.backend_enabled = bool(self.declare_parameter('backend_enabled', False).value)
        self.dry_run_mode = str(self.declare_parameter(
            'dry_run_mode', 'contract_only'
        ).value)
        self.color_topic = str(self.declare_parameter(
            'color_topic', '/xlerobot/d455/color/image_raw'
        ).value)
        self.depth_topic = str(self.declare_parameter(
            'depth_topic', '/xlerobot/d455/aligned_depth_to_color/image_raw'
        ).value)
        self.camera_info_topic = str(self.declare_parameter(
            'camera_info_topic', '/xlerobot/d455/color/camera_info'
        ).value)
        self.sync_tolerance_s = float(self.declare_parameter(
            'rgbd_sync_tolerance_s', 0.20
        ).value)
        self.sensor_timeout_s = float(self.declare_parameter('sensor_timeout_s', 8.0).value)
        self.max_sensor_age_s = float(self.declare_parameter('max_sensor_age_s', 1.0).value)
        self.tf_timeout_s = float(self.declare_parameter('tf_timeout_s', 3.0).value)
        self.min_depth_m = float(self.declare_parameter('min_depth_m', 0.10).value)
        self.max_depth_m = float(self.declare_parameter('max_depth_m', 4.0).value)
        self.classical_base_url = str(self.declare_parameter(
            'classical_base_url', 'http://127.0.0.1:8765'
        ).value)
        self.transforms_file = str(self.declare_parameter(
            'transforms_file', ''
        ).value)
        self.grasp_alignment_file = str(self.declare_parameter(
            'grasp_alignment_file', ''
        ).value)
        self.classical_timeout_s = float(self.declare_parameter(
            'classical_timeout_s', 90.0
        ).value)
        self.gpd_timeout_s = float(self.declare_parameter('gpd_timeout_s', 120.0).value)
        self.gpd_top_k = int(self.declare_parameter('gpd_top_k', 10).value)
        self.planning_frame = str(self.declare_parameter(
            'classical_planning_frame', 'base_link'
        ).value)
        self.top_down_xyzw = tuple(float(value) for value in self.declare_parameter(
            'top_down_orientation_xyzw', [0.70710678, 0.0, 0.0, 0.70710678]
        ).value)
        self.wrist_yaw_min_rad = float(self.declare_parameter(
            'wrist_yaw_min_rad', -math.pi
        ).value)
        self.wrist_yaw_max_rad = float(self.declare_parameter(
            'wrist_yaw_max_rad', math.pi
        ).value)
        self.top_config = TopGraspConfig(
            pregrasp_above_top_m=float(self.declare_parameter(
                'pregrasp_above_top_m', 0.08
            ).value),
            grasp_below_top_m=float(self.declare_parameter(
                'grasp_below_top_m', 0.012
            ).value),
            lift_above_top_m=float(self.declare_parameter(
                'lift_above_top_m', 0.10
            ).value),
            min_table_clearance_m=float(self.declare_parameter(
                'min_table_clearance_m', 0.005
            ).value),
            min_points=int(self.declare_parameter('min_object_points', 80).value),
            workspace_x_m=tuple(float(value) for value in self.declare_parameter(
                'workspace_x_m', [-0.06, 0.18]
            ).value),
            workspace_y_m=tuple(float(value) for value in self.declare_parameter(
                'workspace_y_m', [-0.10, 0.15]
            ).value),
            workspace_z_m=tuple(float(value) for value in self.declare_parameter(
                'workspace_z_m', [0.0, 1.6]
            ).value),
            min_grasp_width_m=float(self.declare_parameter(
                'min_grasp_width_m', 0.015
            ).value),
            max_grasp_width_m=float(self.declare_parameter(
                'max_grasp_width_m', 0.085
            ).value),
            width_margin_m=float(self.declare_parameter('width_margin_m', 0.008).value),
            gpd_max_centroid_distance_m=float(self.declare_parameter(
                'gpd_max_centroid_distance_m', 0.10
            ).value),
        )
        self.refinement_config = RefinementConfig(
            table_roi_y_start_fraction=float(self.declare_parameter(
                'table_roi_y_start_fraction', 0.35
            ).value),
            table_plane_distance_m=float(self.declare_parameter(
                'table_plane_distance_m', 0.003
            ).value),
            table_ransac_iterations=int(self.declare_parameter(
                'table_ransac_iterations', 400
            ).value),
            table_min_inliers=int(self.declare_parameter(
                'table_min_inliers', 200
            ).value),
            body_cluster_epsilon_m=float(self.declare_parameter(
                'body_cluster_epsilon_m', 0.015
            ).value),
            body_cluster_min_points=int(self.declare_parameter(
                'body_cluster_min_points', 40
            ).value),
            sampling_stride=int(self.declare_parameter(
                'refinement_sampling_stride', 2
            ).value),
        )
        self._validate_parameters()
        self.vlm = LmStudioVlmClient(
            base_url=str(self.declare_parameter(
                'vlm_base_url', 'http://127.0.0.1:1234'
            ).value),
            model=str(self.declare_parameter('vlm_model', 'qwen/qwen3-vl-4b').value),
            timeout_s=float(self.declare_parameter('vlm_timeout_s', 30.0).value),
            bbox_format=str(self.declare_parameter('vlm_bbox_format', 'norm1000').value),
            jpeg_quality=int(self.declare_parameter('vlm_jpeg_quality', 90).value),
            request_long_edge_px=int(self.declare_parameter(
                'vlm_request_long_edge_px', 1280
            ).value),
        )
        self.sam2 = PromptedSam2Client(
            self.classical_base_url, timeout_s=self.classical_timeout_s
        )
        self.gpd = GpdClient(self.classical_base_url, timeout_s=self.gpd_timeout_s)

        self.frames = RgbdFrameBuffer()
        self._goal_lock = threading.Lock()
        self._goal_active = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.observation_publisher = self.create_publisher(
            PerceptionObservation, '/perception/observations', 10
        )
        self.create_subscription(
            Image, self.color_topic, self._on_color, qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.create_subscription(
            Image, self.depth_topic, self._on_depth, qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_info, qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.server = ActionServer(
            self,
            DetectObject,
            'detect_object',
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group,
        )
        mode = 'enabled' if self.backend_enabled else 'disabled (dry-run only)'
        self.get_logger().info(f'DetectObject VLM action ready; backend {mode}')

    def _validate_parameters(self):
        positive = {
            'rgbd_sync_tolerance_s': self.sync_tolerance_s,
            'sensor_timeout_s': self.sensor_timeout_s,
            'max_sensor_age_s': self.max_sensor_age_s,
            'tf_timeout_s': self.tf_timeout_s,
            'min_depth_m': self.min_depth_m,
            'max_depth_m': self.max_depth_m,
            'classical_timeout_s': self.classical_timeout_s,
            'gpd_timeout_s': self.gpd_timeout_s,
            'pregrasp_above_top_m': self.top_config.pregrasp_above_top_m,
            'grasp_below_top_m': self.top_config.grasp_below_top_m,
            'lift_above_top_m': self.top_config.lift_above_top_m,
            'min_table_clearance_m': self.top_config.min_table_clearance_m,
            'min_grasp_width_m': self.top_config.min_grasp_width_m,
            'max_grasp_width_m': self.top_config.max_grasp_width_m,
            'gpd_max_centroid_distance_m': self.top_config.gpd_max_centroid_distance_m,
            'table_plane_distance_m': self.refinement_config.table_plane_distance_m,
            'body_cluster_epsilon_m': self.refinement_config.body_cluster_epsilon_m,
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            raise ValueError(f'parameters must be positive: {invalid}')
        if self.max_depth_m <= self.min_depth_m:
            raise ValueError('max_depth_m must be greater than min_depth_m')
        if (
            self.gpd_top_k <= 0
            or self.top_config.min_points <= 0
            or self.refinement_config.table_ransac_iterations <= 0
            or self.refinement_config.table_min_inliers <= 0
            or self.refinement_config.body_cluster_min_points <= 0
            or self.refinement_config.sampling_stride <= 0
            or not 0.0 < self.refinement_config.table_roi_y_start_fraction < 1.0
            or len(self.top_config.workspace_x_m) != 2
            or len(self.top_config.workspace_y_m) != 2
            or len(self.top_config.workspace_z_m) != 2
            or self.top_config.workspace_x_m[0] >= self.top_config.workspace_x_m[1]
            or self.top_config.workspace_y_m[0] >= self.top_config.workspace_y_m[1]
            or self.top_config.workspace_z_m[0] >= self.top_config.workspace_z_m[1]
            or self.top_config.min_grasp_width_m >= self.top_config.max_grasp_width_m
            or len(self.top_down_xyzw) != 4
            or not all(math.isfinite(value) for value in self.top_down_xyzw)
            or self.wrist_yaw_min_rad >= self.wrist_yaw_max_rad
            or not _FRAME_PATTERN.fullmatch(self.planning_frame)
        ):
            raise ValueError('classical grasp parameters are invalid')
        validate_dry_run_mode(self.dry_run_mode)

    def goal_callback(self, request):
        try:
            validate_goal_fields(
                request.object_id, request.target_frame, request.grasp_backend
            )
            if (
                request.grasp_backend in {'centroid', 'gpd'}
                and request.target_frame != self.planning_frame
            ):
                raise ValueError(
                    f'classical grasp target_frame must be {self.planning_frame}'
                )
        except ValueError as exc:
            self.get_logger().warning(f'Rejecting DetectObject goal: {exc}')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_active:
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _on_color(self, message):
        self._store('color', message)

    def _on_depth(self, message):
        self._store('depth', message)

    def _on_info(self, message):
        self._store('info', message)

    def _store(self, key, message):
        self.frames.store(key, message)

    def execute(self, goal_handle):
        result = DetectObject.Result()
        backend = goal_handle.request.grasp_backend
        result.backend_used = backend
        try:
            self._feedback(goal_handle, 'validating', 0.05, 'goal accepted')
            if goal_handle.request.dry_run and self.dry_run_mode == 'contract_only':
                self._set_contract_only_result(result, goal_handle.request)
                result.error.code = CapabilityError.NONE
                result.error.message = (
                    f'dry-run validated for {backend}; no sensor or remote request made'
                )
                result.confidence = 0.0
                goal_handle.succeed()
                self._feedback(goal_handle, 'complete', 1.0, result.error.message)
                return result
            if not self.backend_enabled:
                raise CapabilityFailure(
                    CapabilityError.SAFETY_REJECTED,
                    'VLM backend is disabled; set backend_enabled=true explicitly',
                )

            self._feedback(
                goal_handle, 'waiting_fresh_frame', 0.20,
                'waiting for post-head-arrival RGBD frame',
            )
            color, depth, info = self._snapshot(goal_handle)
            self.get_logger().info(
                'using newest post-head-arrival RGBD frame: '
                f'color_stamp_ns={stamp_ns(color)}, depth_stamp_ns={stamp_ns(depth)}, '
                f'info_stamp_ns={stamp_ns(info)}'
            )
            self._feedback(goal_handle, 'detecting', 0.35, 'requesting VLM grounding')
            image_bgr = color_to_bgr(color)
            try:
                detections, latency_s = self.vlm.detect(
                    image_bgr, goal_handle.request.object_id.strip()
                )
            except Exception as exc:
                raise CapabilityFailure(CapabilityError.BACKEND_FAILURE, str(exc)) from exc
            self._check_canceled(goal_handle)
            if not detections:
                raise CapabilityFailure(
                    CapabilityError.NOT_FOUND,
                    f'object not found: {goal_handle.request.object_id.strip()}',
                )
            detection = max(detections, key=lambda item: item.confidence)
            camera_frame = info.header.frame_id or depth.header.frame_id
            if not camera_frame:
                raise CapabilityFailure(CapabilityError.UNAVAILABLE, 'camera frame id is empty')

            if backend == 'act':
                self._feedback(
                    goal_handle, 'transforming', 0.80,
                    'projecting VLM center with timestamped TF',
                )
                result.target = self._act_target(
                    detection, depth, info, camera_frame,
                    goal_handle.request.target_frame,
                )
                result.grasp_plan.valid = False
                result.grasp_plan.backend = 'act'
                result.grasp_plan.method = 'vlm_bbox_center_for_act'
                result.grasp_plan.selection_reason = (
                    'ACT produces streamed joint commands after local pregrasp'
                )
            else:
                geometry = self._classical_geometry(
                    goal_handle,
                    backend,
                    detection,
                    image_bgr,
                    depth,
                    info,
                    camera_frame,
                )
                result.target = self._point_from_xyz(
                    geometry.grasp_m, self.planning_frame, depth.header.stamp
                )
            result.observation = make_observation(
                kind='object',
                label=detection.label,
                camera_id='head',
                image=color,
                bbox=detection.bbox_px,
                confidence=detection.confidence,
                target=result.target,
            )
            if backend == 'act':
                result.grasp_plan.observation_id = result.observation.observation_id
            else:
                result.grasp_plan = self._plan_message(
                    geometry,
                    result.observation.observation_id,
                    self.planning_frame,
                    depth.header.stamp,
                )
            self.observation_publisher.publish(result.observation)
            self.get_logger().info(
                f'grounded {detection.label} with backend={backend} '
                f'bbox={detection.bbox_px} target_{result.target.header.frame_id}=('
                f'{result.target.point.x:.5f},{result.target.point.y:.5f},'
                f'{result.target.point.z:.5f})'
            )
            result.confidence = float(detection.confidence)
            result.error.code = CapabilityError.NONE
            result.error.message = (
                f'localized {detection.label} for {backend}; '
                f'VLM latency {latency_s:.2f}s'
            )
            goal_handle.succeed()
            self._feedback(goal_handle, 'complete', 1.0, result.error.message)
            return result
        except CapabilityFailure as exc:
            result.error.code = int(exc.code)
            result.error.message = exc.message
            if exc.code == CapabilityError.CANCELED or goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self._feedback(goal_handle, 'failed', 1.0, exc.message)
            return result
        except Exception as exc:
            result.error.code = CapabilityError.INTERNAL_ERROR
            result.error.message = str(exc)
            goal_handle.abort()
            self._feedback(goal_handle, 'failed', 1.0, str(exc))
            return result
        finally:
            with self._goal_lock:
                self._goal_active = False

    def _set_contract_only_result(self, result, request) -> None:
        if request.grasp_backend == 'act':
            result.target.header.frame_id = request.target_frame
            result.target.point.x = 0.10
            result.target.point.y = 0.0
            result.target.point.z = 0.17
            result.grasp_plan.valid = False
            result.grasp_plan.backend = 'act'
            result.grasp_plan.method = 'contract_only'
            result.grasp_plan.selection_reason = 'ACT contract-only dry run'
            return
        config, calibration = self._classical_config()
        grasp_xyz = np.asarray([
            0.5 * sum(config.workspace_x_m),
            0.5 * sum(config.workspace_y_m),
            0.5 * sum(config.workspace_z_m),
        ])
        result.target = self._point_from_xyz(
            grasp_xyz, request.target_frame, result.target.header.stamp
        )
        plan = result.grasp_plan
        plan.valid = True
        plan.observation_id = f'contract-only:{request.grasp_backend}'
        plan.backend = request.grasp_backend
        plan.method = 'contract_only'
        plan.score = 0.0
        plan.pregrasp = self._pose_from_xyz(
            grasp_xyz + np.array([0.0, 0.0, config.pregrasp_above_top_m]),
            request.target_frame,
            None,
        )
        plan.grasp = self._pose_from_xyz(
            grasp_xyz, request.target_frame, None
        )
        plan.lift = self._pose_from_xyz(
            grasp_xyz + np.array([0.0, 0.0, config.lift_above_top_m]),
            request.target_frame,
            None,
        )
        plan.table_height_m = float(grasp_xyz[2] - 0.05)
        plan.object_height_m = 0.05
        plan.grasp_width_m = 0.04
        plan.wrist_yaw_seed_rad = 0.0
        plan.wrist_yaw_min_rad = self.wrist_yaw_min_rad
        plan.wrist_yaw_max_rad = self.wrist_yaw_max_rad
        plan.selection_reason = (
            'deterministic geometry for contract-only dry run; '
            f'{calibration.provenance}'
        )

    def _act_target(self, detection, depth, info, camera_frame, target_frame):
        x1, y1, x2, y2 = detection.bbox_px
        u = 0.5 * (x1 + x2)
        v = 0.5 * (y1 + y2)
        # Preserve the verified ACT route: depth comes only from an expanding
        # window around the VLM box center, never the box edges.
        depth_m = depth_median(depth, u, v)
        if depth_m is None or not self.min_depth_m <= depth_m <= self.max_depth_m:
            raise CapabilityFailure(
                CapabilityError.NOT_FOUND,
                'detection has no depth inside the configured range',
            )
        camera_point = project_pixel_to_point(
            u, v, depth_m, info, frame_id=camera_frame, stamp=depth.header.stamp
        )
        return self._target_point(camera_point, target_frame)

    def _classical_geometry(
        self, goal_handle, backend, detection, image_bgr, depth, info, camera_frame
    ):
        config, calibration = self._classical_config()
        self._feedback(
            goal_handle, 'segmenting', 0.52, 'requesting prompted SAM2 mask'
        )
        depth_u16 = depth_to_u16_mm(depth)
        intrinsics = camera_intrinsics(info)
        try:
            instance = self.sam2.segment(
                image_bgr,
                depth_u16,
                label=detection.label,
                bbox_px=tuple(int(value) for value in detection.bbox_px),
            )
        except ClassicalServiceError as exc:
            raise CapabilityFailure(
                CapabilityError.BACKEND_FAILURE, f'prompted SAM2 failed: {exc}'
            ) from exc
        self._check_canceled(goal_handle)
        transform = self._lookup_transform(
            self.planning_frame, camera_frame, depth.header.stamp
        )
        target_from_camera = transform_matrix(transform)
        self._feedback(
            goal_handle, 'refining_depth', 0.64,
            'fitting table plane and retaining the main 3D object body',
        )
        try:
            refined = refine_object_cloud(
                depth_u16,
                instance.mask,
                intrinsics,
                target_from_camera,
                min_depth_m=self.min_depth_m,
                max_depth_m=self.max_depth_m,
                config=self.refinement_config,
            )
        except ValueError as exc:
            raise CapabilityFailure(
                CapabilityError.NOT_FOUND,
                f'RGB-D table/body refinement failed without fallback: {exc}',
            ) from exc
        if len(refined.points_target) < config.min_points:
            raise CapabilityFailure(
                CapabilityError.NOT_FOUND,
                'refined object cloud has only '
                f'{len(refined.points_target)} points; need {config.min_points}',
            )
        self._feedback(goal_handle, 'planning_grasp', 0.72, f'planning {backend} top grasp')
        if backend == 'centroid':
            try:
                geometry = centroid_top_grasp(
                    refined.points_target,
                    confidence=float(detection.confidence),
                    config=config,
                    table_plane=refined.table_plane,
                )
                return replace(
                    geometry,
                    selection_reason=(
                        f'{geometry.selection_reason}; fitted-table-inliers='
                        f'{refined.table_plane.inlier_count}; main-body-points='
                        f'{len(refined.points_target)}; {calibration.provenance}'
                    ),
                )
            except ValueError as exc:
                raise CapabilityFailure(CapabilityError.NOT_FOUND, str(exc)) from exc

        try:
            candidates = self.gpd.infer(
                image_bgr,
                depth_u16,
                refined.mask,
                intrinsics,
                label=detection.label,
                bbox_px=instance.bbox_px,
                top_k=self.gpd_top_k,
            )
            self._check_canceled(goal_handle)
            geometry = gpd_top_grasp(
                candidates,
                points_target=refined.points_target,
                pixels_uv=refined.pixels_uv,
                mask=refined.mask,
                intrinsics=intrinsics,
                target_from_camera=target_from_camera,
                config=config,
                table_plane=refined.table_plane,
            )
            return replace(
                geometry,
                selection_reason=(
                    f'{geometry.selection_reason}; fitted-table-inliers='
                    f'{refined.table_plane.inlier_count}; main-body-points='
                    f'{len(refined.points_target)}; {calibration.provenance}'
                ),
            )
        except (ClassicalServiceError, ValueError) as exc:
            # Deliberately no centroid fallback: the selected backend remains
            # observable and a failed GPD experiment fails as GPD.
            raise CapabilityFailure(
                CapabilityError.BACKEND_FAILURE, f'GPD failed without fallback: {exc}'
            ) from exc

    def _classical_config(self):
        try:
            calibration = load_classical_runtime_calibration(
                self.transforms_file, self.grasp_alignment_file
            )
        except (OSError, ValueError) as exc:
            raise CapabilityFailure(
                CapabilityError.UNAVAILABLE,
                f'classical runtime calibration is unavailable: {exc}',
            ) from exc
        return replace(
            self.top_config,
            workspace_x_m=(
                calibration.workspace_min_m[0], calibration.workspace_max_m[0]
            ),
            workspace_y_m=(
                calibration.workspace_min_m[1], calibration.workspace_max_m[1]
            ),
            workspace_z_m=(
                calibration.workspace_min_m[2], calibration.workspace_max_m[2]
            ),
        ), calibration

    def _plan_message(self, geometry, observation_id, frame_id, stamp):
        plan = TopGraspPlan()
        plan.valid = True
        plan.observation_id = str(observation_id)
        plan.backend = geometry.backend
        plan.method = geometry.method
        plan.score = float(geometry.score)
        plan.pregrasp = self._pose_from_xyz(geometry.pregrasp_m, frame_id, stamp)
        plan.grasp = self._pose_from_xyz(geometry.grasp_m, frame_id, stamp)
        plan.lift = self._pose_from_xyz(geometry.lift_m, frame_id, stamp)
        plan.table_height_m = float(geometry.table_height_m)
        plan.object_height_m = float(geometry.object_height_m)
        plan.grasp_width_m = float(geometry.grasp_width_m)
        plan.wrist_yaw_seed_rad = float(geometry.wrist_yaw_rad)
        plan.wrist_yaw_min_rad = self.wrist_yaw_min_rad
        plan.wrist_yaw_max_rad = self.wrist_yaw_max_rad
        plan.selection_reason = geometry.selection_reason
        plan.debug_asset_uri = ''
        return plan

    def _pose_from_xyz(self, xyz, frame_id, stamp):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        if stamp is not None:
            pose.header.stamp = stamp
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.x = self.top_down_xyzw[0]
        pose.pose.orientation.y = self.top_down_xyzw[1]
        pose.pose.orientation.z = self.top_down_xyzw[2]
        pose.pose.orientation.w = self.top_down_xyzw[3]
        return pose

    @staticmethod
    def _point_from_xyz(xyz, frame_id, stamp):
        point = PointStamped()
        point.header.frame_id = frame_id
        point.header.stamp = stamp
        point.point.x = float(xyz[0])
        point.point.y = float(xyz[1])
        point.point.z = float(xyz[2])
        return point

    def _snapshot(self, goal_handle):
        # The task moves the head before it sends DetectObject.  Do not reuse a
        # still-valid buffered frame captured during that motion: require the
        # complete RGBD tuple to have been produced after this action started.
        action_started_ns = self.get_clock().now().nanoseconds
        try:
            return self.frames.wait_snapshot(
                timeout_s=self.sensor_timeout_s,
                sync_tolerance_s=self.sync_tolerance_s,
                max_age_s=self.max_sensor_age_s,
                min_stamp_ns=action_started_ns,
                now_ns=lambda: self.get_clock().now().nanoseconds,
                check_interrupt=lambda: self._check_canceled(goal_handle),
            )
        except TimeoutError as exc:
            raise CapabilityFailure(CapabilityError.TIMEOUT, str(exc)) from exc

    def _target_point(self, camera_point, target_frame):
        if camera_point.header.frame_id == target_frame:
            camera_point.header.frame_id = target_frame
            return camera_point
        transform = self._lookup_transform(
            target_frame, camera_point.header.frame_id, camera_point.header.stamp
        )
        return transform_point(camera_point, transform)

    def _lookup_transform(self, target_frame, source_frame, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException as exc:
            raise CapabilityFailure(
                CapabilityError.UNAVAILABLE,
                f'missing timestamped TF {source_frame}->{target_frame}: {exc}',
            ) from exc

    @staticmethod
    def _check_canceled(goal_handle):
        if goal_handle.is_cancel_requested:
            raise CapabilityFailure(CapabilityError.CANCELED, 'detect object canceled')

    @staticmethod
    def _feedback(goal_handle, phase, progress, message):
        feedback = DetectObject.Feedback()
        feedback.state.phase = phase
        feedback.state.progress = float(progress)
        feedback.state.message = message
        goal_handle.publish_feedback(feedback)


def main():
    """Run the read-only detection action server."""
    rclpy.init()
    node = DetectObjectNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
