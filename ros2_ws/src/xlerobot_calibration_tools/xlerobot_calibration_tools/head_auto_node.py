"""Shared local head-camera / hand-eye sweep; remote services never own motion."""

from collections import deque
import hashlib
from pathlib import Path
import re
import threading
import time
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from xlerobot_interfaces.action import CalibrationJob, MoveCalibrationPose
from xlerobot_interfaces.msg import CalibrationTargetObservation, CapabilityError, HeadCalibrationStatus
from xlerobot_interfaces.msg import HandeyeCalibrationStatus
from xlerobot_interfaces.srv import CaptureCalibrationSample

from .bundle import UnitCalibrationStore
from .head_auto import HeadSession, load_poses
from .handeye_auto import HandeyeSession, load_poses as load_handeye_poses, FIT_COUNT, VALIDATION_COUNT
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
    """Shared local sweep executor; only the evidence policy differs by workflow."""

    def __init__(self, workflow='head_camera'):
        if workflow not in {'head_camera', 'right_handeye'}:
            raise ValueError('unsupported automatic visual calibration workflow')
        self.workflow = workflow
        self.handeye = workflow == 'right_handeye'
        self.required = FIT_COUNT + VALIDATION_COUNT if self.handeye else 12
        endpoint = 'handeye' if self.handeye else 'head'
        super().__init__(f'auto_{endpoint}_calibration')
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
        default_poses = Path(get_package_share_directory('xlerobot_calibration_tools')) / f'config/{workflow}_poses.yaml'
        pose_file = str(self.declare_parameter('pose_file', str(default_poses)).value)
        self.pose_file = Path(pose_file)
        poses, pose_hash = (load_handeye_poses if self.handeye else load_poses)(pose_file)
        servo_file = state_root / 'units' / unit / 'draft/components/servo.yaml'
        servo_hash = hashlib.sha256(servo_file.read_bytes()).hexdigest()
        if self.handeye:
            head_file = servo_file.with_name('head_camera.yaml')
            servo_hash = hashlib.sha256(servo_file.read_bytes() + head_file.read_bytes()).hexdigest()
        self.predecessors = [servo_file, head_file] if self.handeye else [servo_file]
        self.predecessor_bytes = [path.read_bytes() for path in self.predecessors]
        self.draft_path = servo_file.with_name(workflow + '.yaml')
        session_class = HandeyeSession if self.handeye else HeadSession
        self.session = session_class(artifact_root / f'calibration_work/{workflow}', unit, poses, pose_hash,
                                   servo_hash=servo_hash)
        self.store = UnitCalibrationStore(state_root, repo_root)
        self.lock = threading.RLock()
        self.busy = False
        self.motion_unconfirmed = False
        self.capture_unconfirmed = False
        self.capture_future = None
        self.observations = deque(maxlen=3)
        self.callbacks = ReentrantCallbackGroup()
        self.move_client = ActionClient(self, MoveCalibrationPose, '/calibration/move_pose', callback_group=self.callbacks)
        self.capture_client = self.create_client(CaptureCalibrationSample, '/calibration/capture_sample', callback_group=self.callbacks)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.status_type = HandeyeCalibrationStatus if self.handeye else HeadCalibrationStatus
        self.status_publisher = self.create_publisher(self.status_type, f'/calibration/{endpoint}_status', qos)
        self.create_subscription(CalibrationTargetObservation, '/calibration/target_observation',
                                 self.on_observation, qos_profile_sensor_data,
                                 callback_group=self.callbacks)
        self.server = ActionServer(self, CalibrationJob, f'/calibration/{endpoint}_auto',
                                   execute_callback=self.execute, goal_callback=self.goal,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.callbacks)
        self.create_timer(1.0, self.publish_status, callback_group=self.callbacks)
        self.create_service(Trigger, f'/calibration/{endpoint}_reset', self.reset_session,
                            callback_group=self.callbacks)
        self.publish_status()

    def reset_session(self, request, response):
        with self.lock:
            if self.busy or self.motion_unconfirmed or self.capture_unconfirmed:
                response.success = False
                response.message = '请先暂停，等待运动和采样完全结束。'
                return response
            try:
                if hashlib.sha256(self.pose_file.read_bytes()).hexdigest() != self.session.pose_hash:
                    raise ValueError('采样姿态配置已改变，请重新启动工具。')
                if any(p.read_bytes() != before for p, before in
                       zip(self.predecessors, self.predecessor_bytes)):
                    raise ValueError('前序标定已改变，请重新启动工具以加载新配置。')
                if self.session.document['phase'] == 'IDLE' and not self.session.document['sample_count']:
                    response.success = True
                    response.message = '已经是空白会话；请确认运动后点击开始标定。'
                    return response
                self.session, archive = self.session.restart(self.draft_path)
                self.observations.clear()
                response.success = True
                response.message = ('新会话已准备好；未触发运动，草稿和生效标定不变。'
                                    + (f' 旧会话归档：{archive}' if archive else ''))
            except Exception as error:
                response.success = False
                response.message = str(error)
        self.publish_status()
        return response

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
            message = self.status_type()
            message.header.stamp = self.get_clock().now().to_msg()
            message.unit_id = self.session.unit
            message.running = self.busy
            message.phase = document['phase']
            message.message = document['message']
            message.pose_index = int(document['pose_index'])
            message.pose_count = len(self.session.poses)
            message.sample_count = int(document['sample_count'])
            message.target_sample_count = self.required
            message.pose_states = list(document['pose_states'])
            if self.handeye:
                message.pose_roles = ['fit' if i < FIT_COUNT else 'validation'
                                      for i in range(len(self.session.poses))]
                message.pose_messages = list(document['pose_messages'])
                message.report_uri = self.session.report_path.as_uri() if self.session.report_path.exists() else ''
            else:
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
            if not self.enabled or self.busy or self.motion_unconfirmed or self.capture_unconfirmed \
                    or self.session.document['phase'] == 'COMPLETED' \
                    or goal.unit_id != self.session.unit or goal.workflow_id != self.workflow \
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

    def capture_finished(self, future):
        # A service timeout is not proof that the collector did not save a row.
        # Retain ownership until a response arrives; reconcile its job ID on resume.
        try:
            future.result()
        except Exception:
            return
        with self.lock:
            if future is self.capture_future:
                self.capture_unconfirmed = False

    def move(self, handle, index, *, return_ready=False):
        if handle.is_cancel_requested:
            raise SweepPaused('paused before movement')
        if not self.move_client.wait_for_server(timeout_sec=3.0):
            raise SweepPaused('calibration motion controller is unavailable')
        request = MoveCalibrationPose.Goal(workflow_id=self.workflow, pose_index=index,
                                           dry_run=False, return_ready=return_ready)
        # An unanswered goal request may already have reached the controller.
        self.motion_unconfirmed = True
        goal_handle = self._wait(self.move_client.send_goal_async(request), 5.0)
        if goal_handle is None or not goal_handle.accepted:
            self.motion_unconfirmed = False
            raise SweepPaused('calibration motion request was rejected')
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
                raise SweepPaused('motion stop is unconfirmed; restart only after checking hardware')
            time.sleep(0.02)
        wrapped = future.result()
        self.motion_unconfirmed = False
        if canceling or handle.is_cancel_requested:
            raise SweepPaused('paused; motion controller reached a terminal state')
        if wrapped.status != 4 or wrapped.result.error.code != CapabilityError.NONE:
            raise SweepPaused(wrapped.result.error.message or 'calibration motion failed')

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
            if [path.read_bytes() for path in self.predecessors] != self.predecessor_bytes:
                raise ValueError('predecessor calibration changed; restart with --fresh and re-render runtime')
            fit_frozen = False
            for index in pending:
                if self.handeye and index >= FIT_COUNT and not fit_frozen:
                    self.transition('SOLVING', 'freezing the 20-pose fit before independent validation')
                    with self.lock:
                        self.session.freeze_fit()
                    fit_frozen = True
                self.transition('MOVING', f'moving to pose {index + 1}/{len(self.session.poses)}', index)
                self.move(handle, index)
                self.transition('WAITING', 'waiting for stable Tag 23' if self.handeye else 'waiting for 16 stable tags', index)
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
                with self.lock:
                    self.capture_unconfirmed = True
                capture_future = self.capture_client.call_async(CaptureCalibrationSample.Request(job_id=job_id))
                with self.lock:
                    self.capture_future = capture_future
                capture_future.add_done_callback(self.capture_finished)
                response = self._wait(capture_future, 5.0)
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
                feedback.target_sample_count = self.required
                handle.publish_feedback(feedback)
            if handle.is_cancel_requested:
                raise SweepPaused('paused before solving')
            self.transition('SOLVING', 'checking sample coverage and solving camera extrinsics')
            sample_count = int(self.session.document['sample_count'])
            if sample_count < self.required:
                raise ValueError(f'only {sample_count}/{self.required} independent samples; keep the target visible and retry skipped poses')
            if self.handeye:
                self.transition('VALIDATING', 'comparing unseen Tag poses against frozen visual/FK transforms; no refitting')
                with self.lock:
                    document = self.session.finish_validation()
            else:
                document = solve_transform_samples(self.session.sample_path, self.workflow)
            if handle.is_cancel_requested or not rclpy.ok():
                raise SweepPaused('paused before saving the solved draft')
            if [path.read_bytes() for path in self.predecessors] != self.predecessor_bytes:
                raise ValueError('predecessor calibration changed during capture; draft unchanged')
            self.store.save_component(self.session.unit, self.workflow, document)
            uri = (self.store.draft_components(self.session.unit) / f'{self.workflow}.yaml').as_uri()
            self.transition('RETURNING', '标定草稿已保存，正在回到 ready 姿态')
            try:
                self.move(handle, 0, return_ready=True)
            except Exception as error:
                raise SweepPaused(f'标定草稿已保存，但回 ready 未完成：{error}；检查后可继续') from error
            with self.lock:
                self.session.complete(uri, self.session.document['metrics'] if self.handeye else document['metrics'])
                self.session.document['message'] = '标定通过，草稿已保存，已回到 ready 姿态；生效配置未改变'
                self.session.save()
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


def main(args=None, workflow='head_camera'):
    rclpy.init(args=args)
    node = HeadCalibrationNode(workflow)
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


def main_handeye(args=None):
    main(args, workflow='right_handeye')
