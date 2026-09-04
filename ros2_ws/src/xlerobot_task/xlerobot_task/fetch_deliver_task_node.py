from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from xlerobot_interfaces.action import (
    ApproachTarget,
    AutoLocalize,
    DetectObject,
    ExecuteTask,
    GraspObject,
    HandoverObject,
    NavigateToNamedPlace,
    ScanForPerson,
    SpeakText,
)
from xlerobot_interfaces.msg import (
    CapabilityError,
    PerceptionObservation,
    ScanMapConsistency,
    TaskEvent,
)
from xlerobot_task.flow import CAPABILITY_SEQUENCE, FetchDeliverRequest, validate_request


READINESS_JOINT_NAMES = (
    "right_arm_shoulder_pan",
    "right_arm_shoulder_lift",
    "right_arm_elbow_flex",
    "right_arm_wrist_flex",
    "right_arm_wrist_roll",
    "right_arm_gripper",
    "head_pan_joint",
    "head_tilt_joint",
)


@dataclass
class TaskFailure(RuntimeError):
    code: int
    message: str

    def __str__(self) -> str:
        return self.message


@dataclass
class ChildOperation:
    """One child action from goal request through terminal result."""

    label: str
    send_future: object
    handle: object | None = None
    result_future: object | None = None


def joints_at_target(message, joint_names, targets, tolerance_rad) -> bool:
    """Return true only when every requested measured joint is near target."""
    positions = dict(zip(message.name, message.position))
    return all(
        name in positions
        and math.isfinite(float(positions[name]))
        and abs(float(positions[name]) - float(target)) <= tolerance_rad
        for name, target in zip(joint_names, targets)
    )


def readiness_joint_state_valid(message) -> bool:
    """Require a finite measured position for every joint used by the demo."""
    positions = dict(zip(message.name, message.position))
    return all(
        name in positions and math.isfinite(float(positions[name]))
        for name in READINESS_JOINT_NAMES
    )


def update_head_stable_since(stable_since, sample_time, at_target):
    """Track a continuous in-tolerance interval across fresh joint samples."""
    if not at_target:
        return None
    return sample_time if stable_since is None else stable_since


def should_request_reached_place_skip(
    *, localization_was_ready: bool, source_place: str, dry_run: bool
) -> bool:
    """Request a live pose check only for a trusted, real table task."""
    return localization_was_ready and source_place == "table" and not dry_run


class FetchDeliverTaskNode(Node):
    """Compose task-level actions without importing any backend package."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("fetch_deliver_task", parameter_overrides=parameter_overrides)
        self.group = ReentrantCallbackGroup()
        self.task_timeout_s = float(self.declare_parameter("task_timeout_s", 360.0).value)
        self.stage_timeouts = {
            "auto_localize": float(self.declare_parameter("localization_timeout_s", 70.0).value),
            "navigate_to_named_place": float(
                self.declare_parameter("navigation_timeout_s", 180.0).value
            ),
            "detect_object": float(self.declare_parameter("detection_timeout_s", 20.0).value),
            "grasp_object": float(self.declare_parameter("grasp_timeout_s", 90.0).value),
            "scan_for_person": float(
                self.declare_parameter("person_scan_timeout_s", 180.0).value
            ),
            "approach_target": float(self.declare_parameter("approach_timeout_s", 125.0).value),
            "handover_object": float(self.declare_parameter("handover_timeout_s", 30.0).value),
            "speak_text": float(self.declare_parameter("speech_timeout_s", 10.0).value),
        }
        self.localization_position_stddev_m = float(
            self.declare_parameter("localization_position_stddev_m", 0.20).value
        )
        self.localization_yaw_stddev_rad = float(
            self.declare_parameter("localization_yaw_stddev_rad", 0.20).value
        )
        self.scan_map_max_age_s = float(
            self.declare_parameter("scan_map_max_age_s", 1.0).value
        )
        self.scan_map_settle_timeout_s = float(
            self.declare_parameter("scan_map_settle_timeout_s", 3.0).value
        )
        self.child_cancel_timeout_s = float(
            self.declare_parameter("child_cancel_timeout_s", 3.0).value
        )
        self.joint_state_max_age_s = float(
            self.declare_parameter("joint_state_max_age_s", 0.5).value
        )
        self.camera_max_age_s = float(
            self.declare_parameter("camera_max_age_s", 1.0).value
        )
        self.diagnostic_max_age_s = float(
            self.declare_parameter("diagnostic_max_age_s", 3.0).value
        )
        self.head_camera_topic = str(
            self.declare_parameter(
                "head_camera_topic", "/xlerobot/d455/color/image_raw"
            ).value
        )
        self.head_depth_topic = str(
            self.declare_parameter(
                "head_depth_topic",
                "/xlerobot/d455/aligned_depth_to_color/image_raw",
            ).value
        )
        self.head_camera_info_topic = str(
            self.declare_parameter(
                "head_camera_info_topic",
                "/xlerobot/d455/color/camera_info",
            ).value
        )
        self.wrist_camera_topic = str(
            self.declare_parameter(
                "wrist_camera_topic", "/right_wrist_camera/image_raw"
            ).value
        )
        positive_values = (
            self.task_timeout_s,
            *self.stage_timeouts.values(),
            self.localization_position_stddev_m,
            self.localization_yaw_stddev_rad,
            self.scan_map_max_age_s,
            self.scan_map_settle_timeout_s,
            self.child_cancel_timeout_s,
            self.joint_state_max_age_s,
            self.camera_max_age_s,
            self.diagnostic_max_age_s,
        )
        if (
            not all(
                math.isfinite(value) and value > 0.0
                for value in positive_values
            )
            or not self.head_camera_topic
            or not self.head_depth_topic
            or not self.head_camera_info_topic
            or not self.wrist_camera_topic
        ):
            raise ValueError(
                "task timeouts, thresholds, and readiness parameters must be "
                "finite and positive"
            )
        self.handover_text = str(self.declare_parameter("handover_text", "给你").value)
        self.default_standoff_m = float(self.declare_parameter("standoff_m", 0.5).value)
        self.detection_head_positions = [
            float(value) for value in self.declare_parameter(
                "detection_head_positions", [0.0, 0.8]
            ).value
        ]
        self.detection_head_move_s = float(
            self.declare_parameter("detection_head_move_s", 1.5).value
        )
        self.detection_head_tolerance_rad = float(
            self.declare_parameter("detection_head_tolerance_rad", 0.06).value
        )
        self.detection_head_settle_timeout_s = float(
            self.declare_parameter("detection_head_settle_timeout_s", 3.5).value
        )
        self.detection_head_stable_duration_s = float(
            self.declare_parameter("detection_head_stable_duration_s", 0.5).value
        )
        self.detection_head_post_settle_s = float(
            self.declare_parameter("detection_head_post_settle_s", 0.25).value
        )
        if (
            len(self.detection_head_positions) != 2
            or not all(
                math.isfinite(value)
                for value in (
                    *self.detection_head_positions,
                    self.default_standoff_m,
                    self.detection_head_move_s,
                    self.detection_head_tolerance_rad,
                    self.detection_head_settle_timeout_s,
                    self.detection_head_stable_duration_s,
                    self.detection_head_post_settle_s,
                )
            )
            or self.default_standoff_m <= 0.0
            or self.detection_head_move_s <= 0.0
            or self.detection_head_tolerance_rad <= 0.0
            or self.detection_head_settle_timeout_s <= 0.0
            or self.detection_head_stable_duration_s <= 0.0
            or self.detection_head_post_settle_s < 0.0
            or (
                self.detection_head_stable_duration_s
                + self.detection_head_post_settle_s
                >= self.detection_head_settle_timeout_s
            )
        ):
            raise ValueError("detection head preset is invalid")

        self.localize_client = ActionClient(
            self, AutoLocalize, "auto_localize", callback_group=self.group
        )
        self.navigate_client = ActionClient(
            self, NavigateToNamedPlace, "navigate_to_named_place", callback_group=self.group
        )
        self.detect_client = ActionClient(
            self, DetectObject, "detect_object", callback_group=self.group
        )
        self.head_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/head_controller/follow_joint_trajectory",
            callback_group=self.group,
        )
        self.grasp_client = ActionClient(
            self, GraspObject, "grasp_object", callback_group=self.group
        )
        self.scan_client = ActionClient(
            self, ScanForPerson, "scan_for_person", callback_group=self.group
        )
        self.approach_client = ActionClient(
            self, ApproachTarget, "approach_target", callback_group=self.group
        )
        self.handover_client = ActionClient(
            self, HandoverObject, "handover_object", callback_group=self.group
        )
        self.speak_client = ActionClient(
            self, SpeakText, "speak_text", callback_group=self.group
        )

        self._lock = threading.Lock()
        self._goal_active = False
        self._manual_reserved = False
        self._blocked_reason = ""
        self._stop_latched = False
        self._stop_state_received = False
        self._required_diagnostics = {
            "xlerobot/startup_ready": (
                DiagnosticStatus.STALE,
                "startup ready status not received",
                0.0,
            ),
            "xlerobot/drive_safety": (
                DiagnosticStatus.STALE,
                "drive safety status not received",
                0.0,
            ),
            "xlerobot/person_search": (
                DiagnosticStatus.STALE,
                "person search status not received",
                0.0,
            ),
        }
        self._latest_amcl_pose = None
        self._scan_map_condition = threading.Condition()
        self._latest_scan_map_consistency = None
        self._latest_scan_map_received_monotonic = 0.0
        self._head_condition = threading.Condition()
        self._latest_joint_state = None
        self._latest_joint_state_sequence = 0
        self._latest_joint_state_received_monotonic = 0.0
        self._readiness_joint_state_received_monotonic = 0.0
        self._camera_received_monotonic = {
            "head_color": 0.0,
            "head_depth": 0.0,
            "head_camera_info": 0.0,
            "wrist": 0.0,
        }
        self._task_event_lock = threading.Lock()
        self._task_event_capability = ""
        self._task_event_phase = ""
        self._task_event_progress = 0.0
        self._task_event_message = ""
        self._task_event_observation = PerceptionObservation()
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.task_event_publisher = self.create_publisher(
            TaskEvent, "/task/events", state_qos
        )
        self.readiness_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.manual_control_service = self.create_service(
            SetBool,
            "/execute_task/set_manual_control",
            self._set_manual_control,
            callback_group=self.group,
        )
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            10,
            callback_group=self.group,
        )
        self.scan_map_subscription = self.create_subscription(
            ScanMapConsistency,
            "/localization/scan_map_consistency",
            self._on_scan_map_consistency,
            10,
            callback_group=self.group,
        )
        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            20,
            callback_group=self.group,
        )
        self.head_camera_subscription = self.create_subscription(
            Image,
            self.head_camera_topic,
            lambda _message: self._on_camera("head_color"),
            qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.head_depth_subscription = self.create_subscription(
            Image,
            self.head_depth_topic,
            lambda _message: self._on_camera("head_depth"),
            qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.head_camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.head_camera_info_topic,
            lambda _message: self._on_camera("head_camera_info"),
            qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.wrist_camera_subscription = self.create_subscription(
            Image,
            self.wrist_camera_topic,
            lambda _message: self._on_camera("wrist"),
            qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.stop_state_subscription = self.create_subscription(
            Bool,
            "/drive_safety/stop_latched",
            self._on_stop_state,
            state_qos,
            callback_group=self.group,
        )
        self.diagnostic_subscription = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self._on_diagnostics,
            20,
            callback_group=self.group,
        )
        self.readiness_timer = self.create_timer(
            1.0, self._publish_readiness, callback_group=self.group
        )
        self.server = ActionServer(
            self,
            ExecuteTask,
            "execute_task",
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group,
        )
        self.get_logger().info("Fetch-and-deliver task action is ready")

    def goal_callback(self, request: ExecuteTask.Goal) -> GoalResponse:
        try:
            validate_request(
                FetchDeliverRequest(
                    object_id=request.object_id,
                    source_place=request.source_place,
                    recipient_id=request.recipient_id,
                    dry_run=request.dry_run,
                )
            )
        except ValueError as exc:
            self.get_logger().warn(f"Rejecting invalid task: {exc}")
            return GoalResponse.REJECT

        with self._lock:
            if self._goal_active or self._manual_reserved or self._blocked_reason:
                return GoalResponse.REJECT
            self._goal_active = True
        if not request.dry_run:
            try:
                ready, reasons = self._live_readiness()
            except Exception:
                with self._lock:
                    self._goal_active = False
                raise
            with self._lock:
                stop_blocked = (
                    not self._stop_state_received
                    or self._stop_latched
                    or bool(self._blocked_reason)
                )
                if not ready or stop_blocked:
                    self._goal_active = False
            if not ready or stop_blocked:
                if stop_blocked and ready:
                    reasons = ["base stop state changed while accepting the task"]
                self.get_logger().warning(
                    "Rejecting live task while not ready: " + "; ".join(reasons)
                )
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        with self._task_event_lock:
            self._task_event_phase = "canceling"
            self._task_event_message = "waiting for the active capability to stop"
        self._publish_task_event(goal_handle, "RUNNING")
        return CancelResponse.ACCEPT

    def _set_manual_control(self, request, response):
        # Internal single-client protocol: operator_console serializes acquire
        # and release.  A future independent manual client requires a typed
        # owner/lease contract rather than sharing this anonymous SetBool.
        with self._lock:
            if request.data:
                if self._goal_active:
                    response.success = False
                    response.message = "a task is active"
                    return response
                if self._manual_reserved:
                    response.success = False
                    response.message = "manual control is already reserved"
                    return response
                if self._blocked_reason:
                    response.success = False
                    response.message = self._blocked_reason
                    return response
                self._manual_reserved = True
                response.success = True
                response.message = "manual control reserved"
                return response
            self._manual_reserved = False
            response.success = True
            response.message = "manual control released"
            return response

    def execute(self, goal_handle):
        request = goal_handle.request
        result = ExecuteTask.Result()
        deadline = time.monotonic() + self.task_timeout_s
        with self._task_event_lock:
            self._task_event_capability = ""
            self._task_event_phase = "starting"
            self._task_event_progress = 0.0
            self._task_event_message = ""
            self._task_event_observation = PerceptionObservation()
        self._publish_task_event(goal_handle, "RUNNING")

        try:
            localization_was_ready = self._localization_ready()
            if localization_was_ready:
                self._feedback(
                    goal_handle,
                    "auto_localize",
                    "already_localized",
                    1.0 / len(CAPABILITY_SEQUENCE),
                )
            else:
                localize_goal = AutoLocalize.Goal()
                localize_goal.dry_run = request.dry_run
                self._require_success(
                    self._call(
                        self.localize_client,
                        localize_goal,
                        "auto_localize",
                        goal_handle,
                        deadline,
                        0,
                        self.stage_timeouts["auto_localize"],
                    ).error
                )
                if not request.dry_run:
                    self._wait_for_scan_map_consistency(goal_handle, deadline)
            self._feedback(
                goal_handle,
                "navigate_to_named_place",
                "starting",
                1.0 / len(CAPABILITY_SEQUENCE),
            )
            nav_goal = NavigateToNamedPlace.Goal()
            nav_goal.place_id = request.source_place
            nav_goal.dry_run = request.dry_run
            nav_goal.skip_if_already_reached = should_request_reached_place_skip(
                localization_was_ready=localization_was_ready,
                source_place=request.source_place,
                dry_run=request.dry_run,
            )
            self._require_success(
                self._call(
                    self.navigate_client,
                    nav_goal,
                    "navigate_to_named_place",
                    goal_handle,
                    deadline,
                    1,
                    self.stage_timeouts["navigate_to_named_place"],
                ).error
            )

            if not request.dry_run:
                self._move_head_for_detection(goal_handle, deadline)

            detect_goal = DetectObject.Goal()
            detect_goal.object_id = request.object_id
            detect_goal.target_frame = "map"
            detect_goal.dry_run = request.dry_run
            detect_result = self._call(
                self.detect_client,
                detect_goal,
                "detect_object",
                goal_handle,
                deadline,
                2,
                self.stage_timeouts["detect_object"],
            )
            self._require_success(detect_result.error)
            self._set_task_observation(detect_result.observation)
            self._publish_task_event(goal_handle, "RUNNING")

            grasp_goal = GraspObject.Goal()
            grasp_goal.object_id = request.object_id
            grasp_goal.target = detect_result.target
            grasp_goal.backend = ""
            grasp_goal.dry_run = request.dry_run
            self._require_success(
                self._call(
                    self.grasp_client,
                    grasp_goal,
                    "grasp_object",
                    goal_handle,
                    deadline,
                    3,
                    self.stage_timeouts["grasp_object"],
                ).error
            )

            self._clear_task_observation()
            scan_goal = ScanForPerson.Goal()
            scan_goal.recipient_id = request.recipient_id
            scan_goal.dry_run = request.dry_run
            scan_result = self._call(
                self.scan_client,
                scan_goal,
                "scan_for_person",
                goal_handle,
                deadline,
                4,
                self.stage_timeouts["scan_for_person"],
            )
            self._require_success(scan_result.error)
            self._set_task_observation(scan_result.observation)
            self._publish_task_event(goal_handle, "RUNNING")

            approach_goal = ApproachTarget.Goal()
            approach_goal.target = scan_result.person
            approach_goal.standoff_m = self.default_standoff_m
            approach_goal.dry_run = request.dry_run
            self._require_success(
                self._call(
                    self.approach_client,
                    approach_goal,
                    "approach_target",
                    goal_handle,
                    deadline,
                    5,
                    self.stage_timeouts["approach_target"],
                ).error
            )

            self._clear_task_observation()
            speak_goal = SpeakText.Goal()
            speak_goal.text = self.handover_text
            speak_goal.voice = ""
            speak_goal.dry_run = request.dry_run
            self._require_success(
                self._call(
                    self.speak_client,
                    speak_goal,
                    "speak_text",
                    goal_handle,
                    deadline,
                    6,
                    self.stage_timeouts["speak_text"],
                ).error
            )

            handover_goal = HandoverObject.Goal()
            handover_goal.object_id = request.object_id
            handover_goal.recipient_id = request.recipient_id
            handover_goal.dry_run = request.dry_run
            self._require_success(
                self._call(
                    self.handover_client,
                    handover_goal,
                    "handover_object",
                    goal_handle,
                    deadline,
                    7,
                    self.stage_timeouts["handover_object"],
                ).error
            )

            result.error.code = CapabilityError.NONE
            result.error.message = "fetch-and-deliver completed"
            result.completed_object_id = request.object_id
            self._feedback(goal_handle, "complete", "done", 1.0)
            self._check_parent(goal_handle, deadline)
            self._publish_task_event(
                goal_handle, "SUCCEEDED", CapabilityError.NONE,
                result.error.message,
            )
            goal_handle.succeed()
            return result
        except TaskFailure as exc:
            parent_canceled = bool(goal_handle.is_cancel_requested)
            error_code = int(exc.code)
            error_message = exc.message
            if parent_canceled:
                error_code = CapabilityError.CANCELED
                error_message = "task canceled"
            elif error_code == CapabilityError.CANCELED:
                error_code = CapabilityError.BACKEND_FAILURE
                error_message = (
                    "a child capability reported cancellation without a "
                    f"parent cancellation request: {exc.message}"
                )
            result.error.code = error_code
            result.error.message = error_message
            result.completed_object_id = ""
            if error_code == CapabilityError.CANCELED:
                self._publish_task_event(
                    goal_handle, "CANCELED", error_code, error_message
                )
                goal_handle.canceled()
            else:
                self._publish_task_event(
                    goal_handle, "FAILED", error_code, error_message
                )
                goal_handle.abort()
            return result
        except Exception as exc:  # Defensive conversion at the action boundary.
            result.error.code = CapabilityError.INTERNAL_ERROR
            result.error.message = str(exc)
            result.completed_object_id = ""
            self._publish_task_event(
                goal_handle, "FAILED", CapabilityError.INTERNAL_ERROR, str(exc)
            )
            goal_handle.abort()
            return result
        finally:
            with self._lock:
                if not self._blocked_reason:
                    self._goal_active = False

    def _move_head_for_detection(self, parent_goal, task_deadline):
        stage_deadline = min(
            task_deadline,
            time.monotonic() + self.stage_timeouts["detect_object"],
        )
        self._feedback(
            parent_goal,
            "detect_object",
            "head_preset",
            2.0 / float(len(CAPABILITY_SEQUENCE)),
            "moving head to verified table detection view",
        )
        if not self._wait_for_action_server(
            self.head_client, parent_goal, stage_deadline, 1.0
        ):
            raise TaskFailure(
                CapabilityError.UNAVAILABLE, "head controller is unavailable"
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]
        point = JointTrajectoryPoint()
        point.positions = list(self.detection_head_positions)
        seconds = int(self.detection_head_move_s)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int(
            (self.detection_head_move_s - seconds) * 1.0e9
        )
        goal.trajectory.points = [point]

        wrapped = self._run_child_action(
            self.head_client,
            goal,
            parent_goal,
            stage_deadline,
            "detection head preset",
        )
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise TaskFailure(
                CapabilityError.BACKEND_FAILURE,
                "detection head preset failed",
            )
        self._wait_for_detection_head(parent_goal, stage_deadline)

    def _wait_for_detection_head(self, parent_goal, stage_deadline):
        joint_names = ["head_pan_joint", "head_tilt_joint"]
        settle_deadline = min(
            stage_deadline,
            time.monotonic() + self.detection_head_settle_timeout_s,
        )
        # Discard a pre-result sample so arrival is proved by a joint-state
        # measurement received after the trajectory action completed.
        with self._head_condition:
            self._latest_joint_state = None
        stable_since = None
        last_sequence = -1
        while rclpy.ok():
            if parent_goal.is_cancel_requested:
                raise TaskFailure(CapabilityError.CANCELED, "task canceled")
            if time.monotonic() >= settle_deadline:
                with self._head_condition:
                    message = self._latest_joint_state
                positions = {} if message is None else dict(
                    zip(message.name, message.position)
                )
                actual = [positions.get(name) for name in joint_names]
                raise TaskFailure(
                    CapabilityError.TIMEOUT,
                    "detection head did not reach measured preset: "
                    f"target={self.detection_head_positions}, actual={actual}",
                )
            with self._head_condition:
                message = self._latest_joint_state
                sequence = self._latest_joint_state_sequence
                received_at = self._latest_joint_state_received_monotonic
                if message is not None and sequence != last_sequence:
                    last_sequence = sequence
                    at_target = joints_at_target(
                        message,
                        joint_names,
                        self.detection_head_positions,
                        self.detection_head_tolerance_rad,
                    )
                    stable_since = update_head_stable_since(
                        stable_since, received_at, at_target
                    )
                    if (
                        stable_since is not None
                        and received_at - stable_since
                        >= self.detection_head_stable_duration_s
                    ):
                        break
                self._head_condition.wait(
                    timeout=min(0.05, self._remaining(settle_deadline))
                )
        else:
            raise TaskFailure(CapabilityError.INTERNAL_ERROR, "head settling interrupted")

        quiet_deadline = min(
            settle_deadline,
            time.monotonic() + self.detection_head_post_settle_s,
        )
        while time.monotonic() < quiet_deadline:
            if parent_goal.is_cancel_requested:
                raise TaskFailure(CapabilityError.CANCELED, "task canceled")
            time.sleep(min(0.02, self._remaining(quiet_deadline)))

    def _call(
        self,
        client,
        child_goal,
        label,
        parent_goal,
        task_deadline,
        step_index,
        stage_timeout_s,
    ):
        stage_deadline = min(task_deadline, time.monotonic() + stage_timeout_s)
        self._check_parent(parent_goal, stage_deadline)
        self._feedback(
            parent_goal,
            label,
            "waiting_for_server",
            step_index / float(len(CAPABILITY_SEQUENCE)),
        )
        if not self._wait_for_action_server(
            client, parent_goal, stage_deadline, 1.0
        ):
            raise TaskFailure(CapabilityError.UNAVAILABLE, f"{label} action is unavailable")

        wrapped = self._run_child_action(
            client,
            child_goal,
            parent_goal,
            stage_deadline,
            label,
            feedback_callback=lambda msg: self._relay_feedback(
                parent_goal, label, step_index, msg.feedback.state
            ),
        )

        if wrapped.status == GoalStatus.STATUS_CANCELED:
            if parent_goal.is_cancel_requested:
                raise TaskFailure(CapabilityError.CANCELED, f"{label} was canceled")
            raise TaskFailure(
                CapabilityError.BACKEND_FAILURE,
                f"{label} was canceled without a parent cancellation request",
            )
        payload_error = getattr(wrapped.result, "error", None)
        if (
            payload_error is not None
            and int(payload_error.code) == CapabilityError.CANCELED
        ):
            if parent_goal.is_cancel_requested:
                raise TaskFailure(CapabilityError.CANCELED, f"{label} was canceled")
            raise TaskFailure(
                CapabilityError.BACKEND_FAILURE,
                f"{label} reported cancellation without a parent "
                "cancellation request",
            )
        if wrapped.status == GoalStatus.STATUS_ABORTED:
            error = payload_error
            code = (
                int(error.code)
                if error is not None
                and int(error.code) != CapabilityError.NONE
                else CapabilityError.BACKEND_FAILURE
            )
            message = (
                str(error.message).strip()
                if error is not None and str(error.message).strip()
                else f"{label} action aborted"
            )
            raise TaskFailure(code, message)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise TaskFailure(
                CapabilityError.BACKEND_FAILURE,
                f"{label} returned unexpected status {wrapped.status}",
            )
        return wrapped.result

    def _run_child_action(
        self,
        client,
        child_goal,
        parent_goal,
        deadline,
        label,
        feedback_callback=None,
    ):
        self._check_parent(parent_goal, deadline)
        options = {}
        if feedback_callback is not None:
            options["feedback_callback"] = feedback_callback
        send_future = client.send_goal_async(child_goal, **options)
        operation = ChildOperation(label=label, send_future=send_future)
        try:
            child_handle = self._wait_future(
                send_future,
                parent_goal,
                self._remaining(deadline),
                f"{label} goal response",
                operation,
            )
            operation.handle = child_handle
            if child_handle is None or not child_handle.accepted:
                raise TaskFailure(
                    CapabilityError.BACKEND_FAILURE, f"{label} rejected the goal"
                )
            operation.result_future = child_handle.get_result_async()
            return self._wait_future(
                operation.result_future,
                parent_goal,
                self._remaining(deadline),
                label,
                operation,
            )
        except TaskFailure:
            raise
        except Exception as exc:
            self._block_after_incomplete_cancel(label)
            self._drain_operation_until_terminal(operation, label)
            raise TaskFailure(
                CapabilityError.BACKEND_FAILURE,
                f"{label} result tracking failed; restart the demo profile: {exc}",
            ) from exc

    def _wait_future(self, future, parent_goal, timeout_s, label, operation=None):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        deadline = time.monotonic() + max(0.01, timeout_s)
        while rclpy.ok() and not event.wait(0.02):
            if parent_goal.is_cancel_requested:
                if operation is not None and not self._cancel_operation_and_wait(
                    operation, label
                ):
                    self._block_after_incomplete_cancel(label)
                    self._drain_operation_until_terminal(operation, label)
                    raise TaskFailure(
                        CapabilityError.BACKEND_FAILURE,
                        f"{label} did not reach terminal state after cancellation; "
                        "restart the demo profile before starting another task",
                    )
                raise TaskFailure(CapabilityError.CANCELED, "task canceled")
            if time.monotonic() >= deadline:
                if operation is not None and not self._cancel_operation_and_wait(
                    operation, label
                ):
                    self._block_after_incomplete_cancel(label)
                    self._drain_operation_until_terminal(operation, label)
                    raise TaskFailure(
                        CapabilityError.BACKEND_FAILURE,
                        f"{label} timed out and did not reach terminal state; "
                        "restart the demo profile before starting another task",
                    )
                raise TaskFailure(CapabilityError.TIMEOUT, f"{label} timed out")
        if not event.is_set():
            raise TaskFailure(CapabilityError.INTERNAL_ERROR, f"{label} interrupted")
        if parent_goal.is_cancel_requested:
            if operation is not None and not self._cancel_operation_and_wait(
                operation, label
            ):
                self._block_after_incomplete_cancel(label)
                self._drain_operation_until_terminal(operation, label)
                raise TaskFailure(
                    CapabilityError.BACKEND_FAILURE,
                    f"{label} did not reach terminal state after cancellation; "
                    "restart the demo profile before starting another task",
                )
            raise TaskFailure(CapabilityError.CANCELED, "task canceled")
        if time.monotonic() >= deadline:
            if operation is not None and not self._cancel_operation_and_wait(
                operation, label
            ):
                self._block_after_incomplete_cancel(label)
                self._drain_operation_until_terminal(operation, label)
                raise TaskFailure(
                    CapabilityError.BACKEND_FAILURE,
                    f"{label} timed out and did not reach terminal state; "
                    "restart the demo profile before starting another task",
                )
            raise TaskFailure(CapabilityError.TIMEOUT, f"{label} timed out")
        result = future.result()
        if (
            operation is not None
            and future is operation.result_future
            and not self._terminal_result_observed(result)
        ):
            self._block_after_incomplete_cancel(label)
            self._drain_operation_until_terminal(operation, label)
            raise TaskFailure(
                CapabilityError.BACKEND_FAILURE,
                f"{label} returned a non-terminal action status; "
                "restart the demo profile",
            )
        return result

    @staticmethod
    def _wait_done(future, deadline) -> bool:
        if future is None:
            return False
        if future.done():
            return True
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        return event.wait(max(0.0, deadline - time.monotonic()))

    def _cancel_operation_and_wait(self, operation, label, timeout_s=None) -> bool:
        """Wait for a pending goal response, cancel acceptance, and terminal result."""
        deadline = time.monotonic() + float(
            self.child_cancel_timeout_s if timeout_s is None else timeout_s
        )
        if operation.handle is None:
            if not self._wait_done(operation.send_future, deadline):
                self.get_logger().warning(
                    f"Timed out waiting for {label} goal response before cancellation"
                )
                return False
            try:
                operation.handle = operation.send_future.result()
            except Exception as exc:
                self.get_logger().warning(
                    f"{label} goal response failed while canceling: {exc}"
                )
                return False
        child_handle = operation.handle
        if child_handle is None or not child_handle.accepted:
            return True
        if operation.result_future is None:
            try:
                operation.result_future = child_handle.get_result_async()
            except Exception as exc:
                self.get_logger().warning(
                    f"Unable to observe {label} terminal result: {exc}"
                )
                return False
        try:
            cancel_future = child_handle.cancel_goal_async()
        except Exception as exc:  # Defensive logging at the action boundary.
            self.get_logger().warning(f"Unable to cancel {label}: {exc}")
            return False
        cancel_acknowledged = self._wait_done(cancel_future, deadline)
        if not cancel_acknowledged:
            self.get_logger().warning(
                f"Timed out waiting for {label} cancellation acknowledgement"
            )
        else:
            try:
                cancel_future.result()
            except Exception as exc:
                self.get_logger().warning(
                    f"{label} cancellation acknowledgement failed: {exc}"
                )
        if not self._wait_done(operation.result_future, deadline):
            self.get_logger().warning(
                f"Timed out waiting for {label} terminal result after cancellation"
            )
            return False
        try:
            wrapped = operation.result_future.result()
        except Exception as exc:
            self.get_logger().warning(
                f"{label} terminal result could not be observed: {exc}"
            )
            return False
        if not self._terminal_result_observed(wrapped):
            self.get_logger().warning(
                f"{label} returned a non-terminal result status"
            )
            return False
        return True

    def _drain_operation_until_terminal(self, operation, label) -> None:
        """Keep the parent action non-terminal until the child is proven stopped."""
        while rclpy.ok() and not operation.send_future.done():
            time.sleep(0.02)
        if not operation.send_future.done():
            return
        if operation.handle is None:
            try:
                operation.handle = operation.send_future.result()
            except Exception as exc:
                self.get_logger().error(
                    f"Cannot prove whether {label} was accepted: {exc}; "
                    "holding task ownership until profile restart"
                )
                while rclpy.ok():
                    time.sleep(0.1)
                return
        child_handle = operation.handle
        if child_handle is None or not child_handle.accepted:
            return
        if operation.result_future is None:
            try:
                operation.result_future = child_handle.get_result_async()
            except Exception as exc:
                self.get_logger().error(
                    f"Cannot observe {label} terminal result: {exc}; "
                    "holding task ownership until profile restart"
                )
                while rclpy.ok():
                    time.sleep(0.1)
                return
        if not operation.result_future.done():
            try:
                child_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(
                    f"Unable to re-request {label} cancellation: {exc}"
                )
        while rclpy.ok() and not operation.result_future.done():
            time.sleep(0.02)
        if operation.result_future.done():
            try:
                wrapped = operation.result_future.result()
            except Exception as exc:
                self.get_logger().error(
                    f"Cannot prove {label} reached terminal state: {exc}; "
                    "holding task ownership until profile restart"
                )
                while rclpy.ok():
                    time.sleep(0.1)
                return
            if not self._terminal_result_observed(wrapped):
                self.get_logger().error(
                    f"Cannot prove {label} reached terminal state: "
                    f"status={getattr(wrapped, 'status', 'missing')}; "
                    "holding task ownership until profile restart"
                )
                while rclpy.ok():
                    time.sleep(0.1)

    @staticmethod
    def _terminal_result_observed(wrapped) -> bool:
        return getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN) in {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }

    def _block_after_incomplete_cancel(self, label):
        reason = f"{label} cancellation did not reach terminal state"
        with self._lock:
            self._blocked_reason = reason
        self.get_logger().error(reason)

    def _relay_feedback(self, goal_handle, capability, step_index, state) -> None:
        progress = (
            step_index + min(max(float(state.progress), 0.0), 1.0)
        ) / float(len(CAPABILITY_SEQUENCE))
        self._feedback(goal_handle, capability, state.phase, progress, state.message)

    def _feedback(self, goal_handle, capability, phase, progress, message="") -> None:
        feedback = ExecuteTask.Feedback()
        feedback.current_capability = capability
        feedback.state.phase = phase
        feedback.state.progress = float(progress)
        feedback.state.message = message
        with self._task_event_lock:
            self._task_event_capability = str(capability)
            self._task_event_phase = str(phase)
            self._task_event_progress = float(progress)
            self._task_event_message = str(message)
        goal_handle.publish_feedback(feedback)
        self._publish_task_event(goal_handle, "RUNNING")

    @staticmethod
    def _task_id(goal_handle) -> str:
        return bytes(goal_handle.goal_id.uuid).hex()

    def _publish_task_event(
        self, goal_handle, status: str, error_code: int = 0,
        error_message: str = "",
    ) -> None:
        request = goal_handle.request
        event = TaskEvent()
        event.stamp = self.get_clock().now().to_msg()
        event.task_id = self._task_id(goal_handle)
        event.object_id = request.object_id
        event.source_place = request.source_place
        event.recipient_id = request.recipient_id
        event.dry_run = request.dry_run
        event.status = status
        with self._task_event_lock:
            event.current_capability = self._task_event_capability
            event.state.phase = self._task_event_phase
            event.state.progress = self._task_event_progress
            event.state.message = self._task_event_message
            event.observation = self._task_event_observation
        event.error.code = int(error_code)
        event.error.message = str(error_message)
        self.task_event_publisher.publish(event)

    def _set_task_observation(self, observation: PerceptionObservation) -> None:
        with self._task_event_lock:
            self._task_event_observation = observation

    def _clear_task_observation(self) -> None:
        with self._task_event_lock:
            self._task_event_observation = PerceptionObservation()

    @staticmethod
    def _require_success(error) -> None:
        if int(error.code) != CapabilityError.NONE:
            raise TaskFailure(int(error.code), error.message or "capability failed")

    @staticmethod
    def _remaining(deadline) -> float:
        return max(0.01, deadline - time.monotonic())

    def _check_parent(self, goal_handle, deadline) -> None:
        if goal_handle.is_cancel_requested:
            raise TaskFailure(CapabilityError.CANCELED, "task canceled")
        if time.monotonic() >= deadline:
            raise TaskFailure(CapabilityError.TIMEOUT, "task timed out")

    def _wait_for_action_server(
        self, client, parent_goal, stage_deadline, max_wait_s
    ) -> bool:
        availability_deadline = min(
            stage_deadline, time.monotonic() + max(0.0, max_wait_s)
        )
        while rclpy.ok():
            self._check_parent(parent_goal, stage_deadline)
            if client.server_is_ready():
                return True
            if time.monotonic() >= availability_deadline:
                return False
            time.sleep(0.02)
        raise TaskFailure(
            CapabilityError.UNAVAILABLE,
            "task interrupted while waiting for action server",
        )

    def _on_amcl_pose(self, message) -> None:
        self._latest_amcl_pose = message

    def _on_joint_state(self, message) -> None:
        with self._head_condition:
            self._latest_joint_state = message
            self._latest_joint_state_sequence += 1
            self._latest_joint_state_received_monotonic = time.monotonic()
            if readiness_joint_state_valid(message):
                self._readiness_joint_state_received_monotonic = (
                    self._latest_joint_state_received_monotonic
                )
            self._head_condition.notify_all()

    def _on_camera(self, camera: str) -> None:
        self._camera_received_monotonic[camera] = time.monotonic()

    def _on_stop_state(self, message) -> None:
        with self._lock:
            self._stop_latched = bool(message.data)
            self._stop_state_received = True

    def _on_diagnostics(self, message) -> None:
        received_at = time.monotonic()
        for status in message.status:
            if status.name in self._required_diagnostics:
                with self._lock:
                    self._required_diagnostics[status.name] = (
                        int(status.level),
                        str(status.message),
                        received_at,
                    )

    def _live_readiness(self):
        now_s = time.monotonic()
        reasons = []
        with self._lock:
            stop_latched = self._stop_latched
            stop_received = self._stop_state_received
            blocked_reason = self._blocked_reason
            diagnostics = dict(self._required_diagnostics)
        if blocked_reason:
            reasons.append(blocked_reason)
        if not stop_received:
            reasons.append("drive stop state not received")
        elif stop_latched:
            reasons.append("base software stop is latched")
        for name, (level, message, received_at) in diagnostics.items():
            label = name.removeprefix("xlerobot/").replace("_", " ")
            age_s = now_s - received_at if received_at > 0.0 else math.inf
            if age_s > self.diagnostic_max_age_s:
                reasons.append(f"{label} diagnostic is stale ({age_s:.2f}s)")
            elif level != DiagnosticStatus.OK:
                reasons.append(message or f"{label} is not ready")
        joint_age_s = (
            now_s - self._readiness_joint_state_received_monotonic
            if self._readiness_joint_state_received_monotonic > 0.0
            else math.inf
        )
        if joint_age_s > self.joint_state_max_age_s:
            reasons.append(f"joint state is stale ({joint_age_s:.2f}s)")
        for camera in (
            "head_color",
            "head_depth",
            "head_camera_info",
            "wrist",
        ):
            received_at = self._camera_received_monotonic[camera]
            age_s = now_s - received_at if received_at > 0.0 else math.inf
            if age_s > self.camera_max_age_s:
                reasons.append(f"{camera} camera is stale ({age_s:.2f}s)")
        clients = {
            "auto_localize": self.localize_client,
            "navigate_to_named_place": self.navigate_client,
            "detect_object": self.detect_client,
            "head_controller": self.head_client,
            "grasp_object": self.grasp_client,
            "scan_for_person": self.scan_client,
            "approach_target": self.approach_client,
            "handover_object": self.handover_client,
            "speak_text": self.speak_client,
        }
        unavailable = [
            name for name, client in clients.items() if not client.server_is_ready()
        ]
        if unavailable:
            reasons.append("action servers unavailable: " + ", ".join(unavailable))
        return not reasons, reasons

    def _publish_readiness(self) -> None:
        ready, reasons = self._live_readiness()
        with self._lock:
            task_active = self._goal_active
            manual_reserved = self._manual_reserved
            stop_latched = self._stop_latched
            blocked_reason = self._blocked_reason
        status = DiagnosticStatus()
        status.name = "xlerobot/execute_task"
        status.hardware_id = "two_wheel_reference"
        status.level = (
            DiagnosticStatus.ERROR
            if blocked_reason
            else DiagnosticStatus.OK
            if ready
            else DiagnosticStatus.WARN
        )
        status.message = (
            "task execution ready" if ready else "; ".join(reasons)
        )
        for key, value in (
            ("live_ready", ready),
            ("task_active", task_active),
            ("manual_reserved", manual_reserved),
            ("base_stop_latched", stop_latched),
        ):
            status.values.append(KeyValue(key=key, value=str(value).lower()))
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.readiness_publisher.publish(array)

    def _on_scan_map_consistency(self, message) -> None:
        with self._scan_map_condition:
            self._latest_scan_map_consistency = message
            self._latest_scan_map_received_monotonic = time.monotonic()
            self._scan_map_condition.notify_all()

    def _scan_map_ready(self, *, log=True) -> bool:
        with self._scan_map_condition:
            message = self._latest_scan_map_consistency
            received_at = self._latest_scan_map_received_monotonic
        age_s = time.monotonic() - received_at if received_at > 0.0 else math.inf
        ready = bool(
            message is not None
            and message.valid
            and message.consistent
            and age_s <= self.scan_map_max_age_s
        )
        if log:
            if message is None:
                detail = "no result received"
            else:
                detail = (
                    f"valid={message.valid}, consistent={message.consistent}, "
                    f"score={float(message.score):.3f}, age={age_s:.2f}s, "
                    f"detail={message.message}"
                )
            self.get_logger().info(
                f"scan-map consistency: {detail}, auto_localize={not ready}"
            )
        return ready

    def _wait_for_scan_map_consistency(self, parent_goal, task_deadline) -> None:
        settle_deadline = min(
            task_deadline,
            time.monotonic() + self.scan_map_settle_timeout_s,
        )
        while rclpy.ok():
            self._check_parent(parent_goal, task_deadline)
            if self._scan_map_ready(log=False):
                self._scan_map_ready(log=True)
                return
            if time.monotonic() >= settle_deadline:
                with self._scan_map_condition:
                    message = self._latest_scan_map_consistency
                detail = "no valid result" if message is None else message.message
                raise TaskFailure(
                    CapabilityError.BACKEND_FAILURE,
                    "scan-map consistency did not recover after AutoLocalize: "
                    f"{detail}",
                )
            with self._scan_map_condition:
                self._scan_map_condition.wait(
                    timeout=min(0.05, self._remaining(settle_deadline))
                )
        raise TaskFailure(
            CapabilityError.INTERNAL_ERROR,
            "scan-map consistency wait interrupted",
        )

    def _localization_ready(self) -> bool:
        if self._latest_amcl_pose is None:
            self.get_logger().info('localization quality: no AMCL pose received')
            return False
        covariance = self._latest_amcl_pose.pose.covariance
        position_stddev = math.sqrt(
            max(float(covariance[0]), float(covariance[7]), 0.0)
        )
        yaw_stddev = math.sqrt(max(float(covariance[35]), 0.0))
        covariance_ready = (
            position_stddev <= self.localization_position_stddev_m
            and yaw_stddev <= self.localization_yaw_stddev_rad
        )
        scan_map_ready = self._scan_map_ready()
        ready = covariance_ready and scan_map_ready
        self.get_logger().info(
            f'localization quality: position_stddev={position_stddev:.3f}m '
            f'(limit={self.localization_position_stddev_m:.3f}m), '
            f'yaw_stddev={yaw_stddev:.3f}rad '
            f'(limit={self.localization_yaw_stddev_rad:.3f}rad), '
            f'covariance_ready={covariance_ready}, auto_localize={not ready}'
        )
        return ready


def main() -> None:
    rclpy.init()
    node = FetchDeliverTaskNode()
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


if __name__ == "__main__":
    main()
