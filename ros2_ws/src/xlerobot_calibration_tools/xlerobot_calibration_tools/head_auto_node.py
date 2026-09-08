"""One local automatic head-camera sweep; remote services never own motion."""

from collections import deque
import hashlib
from pathlib import Path
import re
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from xlerobot_interfaces.action import CalibrationJob, MoveCalibrationPose
from xlerobot_interfaces.msg import CalibrationTargetObservation, CapabilityError, HeadCalibrationStatus
from xlerobot_interfaces.srv import CaptureCalibrationSample

from .bundle import UnitCalibrationStore
from .head_auto import HeadSession, load_poses
from .workflow import solve_transform_samples


class SweepPaused(Exception):
    pass


def flattened_metrics(values, prefix=''):
    result = {}
    for key, value in values.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            result.update(flattened_metrics(value, name + '.'))
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            result[name] = float(value)
    return result


def stable_observations(stamps, now, stable_after):
    """Require three distinct current frames, not samples spanning a stream gap."""
    return (len(stamps) >= 3 and stamps[0] >= stable_after
            and all(left < right for left, right in zip(stamps, stamps[1:]))
            and 0 <= now - stamps[-1] <= 0.25
            and 0 <= now - stamps[0] <= 0.5
            and stamps[-1] - stamps[0] <= 0.35)


class HeadCalibrationNode(Node):
    def __init__(self):
        super().__init__('auto_head_calibration')
        self.enabled = bool(self.declare_parameter('execution_enabled', False).value)
        unit = str(self.declare_parameter('unit_id', '').value)
        if re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]{0,63}', unit) is None:
            raise ValueError('unit_id is not a valid reference-unit identifier')
        state_root = Path(str(self.declare_parameter('state_root', '.xlerobot').value)).expanduser().resolve()
        artifact_root = Path(str(self.declare_parameter('artifact_root', '').value)).expanduser().resolve()
        expected_capture = state_root / 'units' / unit / 'capture'
        if not unit or artifact_root != expected_capture:
            raise ValueError('artifact_root must be state_root/units/<unit_id>/capture')
        repo_root = Path(str(self.declare_parameter('repo_root', str(Path.cwd())).value))
        default_poses = Path(get_package_share_directory('xlerobot_calibration_tools')) / 'config/head_camera_poses.yaml'
        pose_file = str(self.declare_parameter('pose_file', str(default_poses)).value)
        poses, pose_hash = load_poses(pose_file)
        servo_file = state_root / 'units' / unit / 'draft/components/servo.yaml'
        servo_hash = hashlib.sha256(servo_file.read_bytes()).hexdigest()
        self.session = HeadSession(artifact_root / 'calibration_work/head_camera', unit, poses, pose_hash,
                                   servo_hash=servo_hash)
        self.store = UnitCalibrationStore(state_root, repo_root)
        self.lock = threading.RLock()
        self.busy = False
        self.motion_unconfirmed = False
        self.observations = deque(maxlen=3)
        self.callbacks = ReentrantCallbackGroup()
        self.move_client = ActionClient(self, MoveCalibrationPose, '/calibration/move_pose', callback_group=self.callbacks)
        self.capture_client = self.create_client(CaptureCalibrationSample, '/calibration/capture_sample', callback_group=self.callbacks)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.status_publisher = self.create_publisher(HeadCalibrationStatus, '/calibration/head_status', qos)
        self.create_subscription(CalibrationTargetObservation, '/calibration/target_observation',
                                 self.on_observation, qos_profile_sensor_data,
                                 callback_group=self.callbacks)
        self.server = ActionServer(self, CalibrationJob, '/calibration/head_auto',
                                   execute_callback=self.execute, goal_callback=self.goal,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.callbacks)
        self.create_timer(1.0, self.publish_status, callback_group=self.callbacks)
        self.publish_status()

    def on_observation(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        with self.lock:
            if not message.accepted:
                self.observations.clear()
            elif not self.observations or stamp > self.observations[-1]:
                self.observations.append(stamp)

    def publish_status(self):
        with self.lock:
            document = self.session.document
            message = HeadCalibrationStatus()
            message.header.stamp = self.get_clock().now().to_msg()
            message.unit_id = self.session.unit
            message.running = self.busy
            message.phase = document['phase']
            message.message = document['message']
            message.pose_index = int(document['pose_index'])
            message.pose_count = len(self.session.poses)
            message.sample_count = int(document['sample_count'])
            message.target_sample_count = 12
            message.pose_states = list(document['pose_states'])
            message.pose_pan = [float(row['pan']) for row in self.session.poses]
            message.pose_tilt = [float(row['tilt']) for row in self.session.poses]
            message.result_uri = document['result_uri']
            message.quality_passed = bool(document['quality_passed'])
            metrics = flattened_metrics(document['metrics'])
            message.metric_names = list(metrics)
            message.metric_values = list(metrics.values())
        self.status_publisher.publish(message)

    def goal(self, goal):
        with self.lock:
            if not self.enabled or self.busy or self.motion_unconfirmed \
                    or self.session.document['phase'] == 'COMPLETED' \
                    or goal.unit_id != self.session.unit or goal.workflow_id != 'head_camera' \
                    or not goal.automatic or goal.dry_run:
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def transition(self, phase, message, index=None):
        with self.lock:
            self.session.phase(phase, message, index)
        self.publish_status()

    def _wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('calibration service/action response timed out')
            time.sleep(0.02)
        if not future.done():
            raise SweepPaused('calibration process stopping')
        return future.result()

    def move(self, handle, index):
        if handle.is_cancel_requested:
            raise SweepPaused('paused before movement')
        if not self.move_client.wait_for_server(timeout_sec=3.0):
            raise SweepPaused('head motion controller is unavailable')
        request = MoveCalibrationPose.Goal(workflow_id='head_camera', pose_index=index, dry_run=False)
        # An unanswered goal request may already have reached the controller.
        self.motion_unconfirmed = True
        goal_handle = self._wait(self.move_client.send_goal_async(request), 5.0)
        if goal_handle is None or not goal_handle.accepted:
            self.motion_unconfirmed = False
            raise SweepPaused('head motion request was rejected')
        future = goal_handle.get_result_async()
        deadline = time.monotonic() + 15.0
        canceling = False
        while not future.done():
            if not canceling and (handle.is_cancel_requested or not rclpy.ok()
                                  or time.monotonic() >= deadline):
                goal_handle.cancel_goal_async()
                canceling = True
                deadline = time.monotonic() + 8.0
            if canceling and time.monotonic() >= deadline:
                raise SweepPaused('head stop is unconfirmed; restart only after checking hardware')
            time.sleep(0.02)
        wrapped = future.result()
        self.motion_unconfirmed = False
        if canceling or handle.is_cancel_requested:
            raise SweepPaused('paused; head controller reached a terminal state')
        if wrapped.status != 4 or wrapped.result.error.code != CapabilityError.NONE:
            raise SweepPaused(wrapped.result.error.message or 'head motion failed')

    def wait_target(self, handle):
        stable_after = self.get_clock().now().nanoseconds / 1e9 + 0.5
        deadline = time.monotonic() + 3.0
        with self.lock:
            self.observations.clear()
        while time.monotonic() < deadline:
            if handle.is_cancel_requested or not rclpy.ok():
                raise SweepPaused('paused while waiting for the board')
            now = self.get_clock().now().nanoseconds / 1e9
            with self.lock:
                stamps = list(self.observations)
            if stable_observations(stamps, now, stable_after):
                return True
            time.sleep(0.02)
        return False

    def execute(self, handle):
        result = CalibrationJob.Result()
        try:
            with self.lock:
                pending = self.session.begin()
            for index in pending:
                self.transition('MOVING', f'moving to pose {index + 1}/{len(self.session.poses)}', index)
                self.move(handle, index)
                self.transition('WAITING', 'waiting for 16 stable tags', index)
                if not self.wait_target(handle):
                    with self.lock:
                        self.session.skipped(index, f'pose {index + 1}: board incomplete or unstable for 3 seconds')
                    self.publish_status()
                    continue
                if not self.capture_client.wait_for_service(timeout_sec=2.0):
                    raise SweepPaused('sample collector is unavailable')
                with self.lock:
                    job_id = self.session.prepare_capture(index)
                self.publish_status()
                response = self._wait(self.capture_client.call_async(
                    CaptureCalibrationSample.Request(job_id=job_id)), 5.0)
                with self.lock:
                    if response.error.code == CapabilityError.NONE:
                        self.session.captured(index)
                    elif 'too close to an existing' in response.error.message:
                        self.session.skipped(index, 'duplicate pose not counted as a new sample')
                    else:
                        self.session.reconcile()
                        self.session.skipped(index, response.error.message)
                self.publish_status()
                feedback = CalibrationJob.Feedback()
                feedback.state.phase = 'CAPTURING'
                feedback.state.progress = float((index + 1) / len(self.session.poses))
                feedback.sample_count = int(self.session.document['sample_count'])
                feedback.target_sample_count = 12
                handle.publish_feedback(feedback)
            if handle.is_cancel_requested:
                raise SweepPaused('paused before solving')
            self.transition('SOLVING', 'checking sample coverage and solving camera extrinsics')
            sample_count = int(self.session.document['sample_count'])
            if sample_count < 12:
                raise ValueError(f'only {sample_count}/12 independent samples; keep the full board visible and retry skipped poses')
            document = solve_transform_samples(self.session.sample_path, 'head_camera')
            if handle.is_cancel_requested or not rclpy.ok():
                raise SweepPaused('paused before saving the solved draft')
            self.store.save_component(self.session.unit, 'head_camera', document)
            uri = (self.store.draft_components(self.session.unit) / 'head_camera.yaml').as_uri()
            with self.lock:
                self.session.complete(uri, document['metrics'])
            result.error.code = CapabilityError.NONE
            result.error.message = self.session.document['message']
            result.artifact_uri = uri
            result.quality_passed = True
            handle.succeed()
        except (SweepPaused, TimeoutError) as error:
            with self.lock:
                self.session.pause(str(error))
            result.error.code = CapabilityError.CANCELED if handle.is_cancel_requested else CapabilityError.BACKEND_FAILURE
            result.error.message = str(error)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        except Exception as error:
            self.transition('ERROR', str(error))
            result.error.code = CapabilityError.INVALID_GOAL
            result.error.message = str(error)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        finally:
            with self.lock:
                self.busy = False
            self.publish_status()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = HeadCalibrationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=10.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
