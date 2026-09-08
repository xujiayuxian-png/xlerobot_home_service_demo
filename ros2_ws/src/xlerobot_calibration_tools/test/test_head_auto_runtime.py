"""Real ROS orchestration with local fake I/O; never start a hardware process.

The node, ROS messages, action cancellation, sample persistence, strict solver,
and draft store are real.  Only motion/detector/collector I/O is substituted.
Domain 76 and localhost discovery isolate this test from the reference robot.
"""

from contextlib import contextmanager
from pathlib import Path
import threading
import time

from action_msgs.msg import GoalStatus
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Header
import yaml

from xlerobot_calibration_tools.head_auto_node import HeadCalibrationNode
from xlerobot_calibration_tools.sample_set import TransformSampleSet
from xlerobot_calibration_tools.solver import HEAD_MODEL
from xlerobot_interfaces.action import CalibrationJob, MoveCalibrationPose
from xlerobot_interfaces.msg import CalibrationTargetObservation, CapabilityError, HeadCalibrationStatus
from xlerobot_interfaces.srv import CaptureCalibrationSample


REPO = Path(__file__).resolve().parents[4]
PACKAGE = Path(__file__).resolve().parents[1]
UNIT = 'head-ros-integration-test'


def wait_for(predicate, timeout=10.0, description='condition'):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError(f'timed out waiting for {description}')


def completed(future, timeout=10.0):
    wait_for(future.done, timeout, 'ROS future')
    return future.result()


class FakeCalibrationIO(Node):
    """No driver imports: record move requests and persist public fixture rows."""

    def __init__(self, sample_path, *, hold_pose=None):
        super().__init__('head_calibration_test_io')
        self.sample_path = sample_path
        self.hold_pose = hold_pose
        self.hold_entered = threading.Event()
        self.stopping = threading.Event()
        self.moves = []
        self.captures = []
        self.events = []
        self.statuses = []
        self.motion_active = 0
        self.max_motion_active = 0
        self.last_completed_pose = None
        self.io_errors = []
        self.rows = yaml.safe_load(
            (REPO / 'examples/calibration/head-camera/samples.yaml').read_text())['samples']
        self.samples = TransformSampleSet(
            UNIT, HEAD_MODEL, 'base_link', 'head_tilt_link',
            'head_camera_link', 'calibration_target')
        group = ReentrantCallbackGroup()
        self.move_server = ActionServer(
            self, MoveCalibrationPose, '/calibration/move_pose',
            execute_callback=self.move, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.capture_server = self.create_service(
            CaptureCalibrationSample, '/calibration/capture_sample', self.capture,
            callback_group=group)
        self.observation_publisher = self.create_publisher(
            CalibrationTargetObservation, '/calibration/target_observation', qos_profile_sensor_data)
        self.create_timer(0.04, self.publish_observation, callback_group=group)
        self.create_subscription(
            HeadCalibrationStatus, '/calibration/head_status', self.statuses.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE), callback_group=group)
        self.client = ActionClient(
            self, CalibrationJob, '/calibration/head_auto', callback_group=group)

    def publish_observation(self):
        self.observation_publisher.publish(CalibrationTargetObservation(
            header=Header(
                stamp=self.get_clock().now().to_msg(), frame_id='head_camera_link'),
            accepted=True, tag_count=16, reprojection_rmse_px=0.2,
            detail='software-only fixture observation'))

    def move(self, handle):
        index = handle.request.pose_index
        self.moves.append(index)
        self.events.append(('move', index))
        self.motion_active += 1
        self.max_motion_active = max(self.max_motion_active, self.motion_active)
        result = MoveCalibrationPose.Result(pose_name=f'fixture-{index}', pose_count=12)
        try:
            if handle.request.workflow_id != 'head_camera' or handle.request.dry_run:
                raise AssertionError('unexpected fake motion request')
            if index == self.hold_pose:
                self.hold_entered.set()
                while not handle.is_cancel_requested and not self.stopping.is_set():
                    time.sleep(0.01)
            if handle.is_cancel_requested:
                result.error.code = CapabilityError.CANCELED
                self.events.append(('canceled', index))
                handle.canceled()
            else:
                self.last_completed_pose = index
                self.events.append(('arrived', index))
                result.error.code = CapabilityError.NONE
                handle.succeed()
            return result
        finally:
            self.motion_active -= 1

    def capture(self, request, response):
        try:
            index = int(request.job_id.rsplit(':', 1)[1])
            if self.motion_active or self.last_completed_pose != index:
                raise AssertionError('capture raced movement or used the wrong pose')
            if index in self.captures:
                raise AssertionError('completed pose was captured twice')
            row = self.rows[index]
            self.samples.append_and_write_atomic(
                self.sample_path, np.asarray(row['moving_in_base']),
                np.asarray(row['target_in_camera']), float(index),
                {**row['quality'], 'capture_job_id': request.job_id})
            self.captures.append(index)
            self.events.append(('capture', index))
            response.error.code = CapabilityError.NONE
            response.sample_count = len(self.captures)
        except Exception as error:
            self.io_errors.append(str(error))
            response.error.code = CapabilityError.BACKEND_FAILURE
            response.error.message = str(error)
        return response


class Runtime:
    def __init__(self, node, io, unit_root):
        self.node = node
        self.io = io
        self.unit_root = unit_root
        self.latest_goal = None

    def start(self, *, dry_run=False):
        assert self.io.client.wait_for_server(timeout_sec=5.0)
        self.latest_goal = completed(self.io.client.send_goal_async(CalibrationJob.Goal(
            unit_id=UNIT, workflow_id='head_camera', automatic=True, dry_run=dry_run)))
        return self.latest_goal


@contextmanager
def runtime(tmp_path, monkeypatch, *, enabled=True, hold_pose=None):
    assert not rclpy.ok(), 'this integration test requires its own ROS context'
    monkeypatch.setenv('ROS_DOMAIN_ID', '76')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    state_root = tmp_path / 'state'
    unit_root = state_root / 'units' / UNIT
    components = unit_root / 'draft/components'
    components.mkdir(parents=True)
    (components / 'servo.yaml').write_bytes((REPO / 'examples/calibration/servo.yaml').read_bytes())
    previous = unit_root / 'versions/existing-test-version'
    previous.mkdir(parents=True)
    (previous / 'manifest.yaml').write_text('test-only previously active bundle\n')
    (unit_root / 'active').symlink_to('versions/existing-test-version', target_is_directory=True)
    runtime_yaml = unit_root / 'runtime/geometry.yaml'
    runtime_yaml.parent.mkdir()
    runtime_yaml.write_text('test-only previously rendered geometry\n')
    pose_document = yaml.safe_load((PACKAGE / 'config/head_camera_poses.yaml').read_text())
    pose_document['poses'] = pose_document['poses'][:12]
    pose_file = tmp_path / 'head-test-poses.yaml'
    pose_file.write_text(yaml.safe_dump(pose_document))
    params = {
        'execution_enabled': str(enabled).lower(), 'unit_id': UNIT,
        'state_root': str(state_root), 'artifact_root': str(unit_root / 'capture'),
        'repo_root': str(REPO), 'pose_file': str(pose_file),
    }
    arguments = ['--ros-args']
    for name, value in params.items():
        arguments.extend(['-p', f'{name}:={value}'])
    rclpy.init(args=arguments, domain_id=76)
    executor = MultiThreadedExecutor(num_threads=8)
    node = io = worker = running = None
    try:
        node = HeadCalibrationNode()
        io = FakeCalibrationIO(node.session.sample_path, hold_pose=hold_pose)
        executor.add_node(node)
        executor.add_node(io)
        worker = threading.Thread(target=executor.spin, daemon=True)
        worker.start()
        running = Runtime(node, io, unit_root)
        yield running
    finally:
        if io is not None:
            io.stopping.set()
        if running is not None and running.latest_goal is not None and running.latest_goal.accepted:
            goal = running.latest_goal
            if node.busy:
                completed(goal.cancel_goal_async(), 5.0)
                completed(goal.get_result_async(), 10.0)
        executor.shutdown(timeout_sec=5.0)
        if worker is not None:
            worker.join(timeout=5.0)
        if io is not None:
            io.move_server.destroy()
            io.client.destroy()
            io.destroy_node()
        if node is not None:
            node.server.destroy()
            node.move_client.destroy()
            node.destroy_node()
        rclpy.shutdown()


def assert_completed_draft(running):
    result = completed(running.latest_goal.get_result_async(), 30.0)
    assert result.status == GoalStatus.STATUS_SUCCEEDED, result.result.error.message
    assert result.result.error.code == CapabilityError.NONE
    assert result.result.quality_passed
    draft = running.unit_root / 'draft/components/head_camera.yaml'
    assert result.result.artifact_uri == draft.as_uri()
    document = yaml.safe_load(draft.read_text())
    assert document['schema'] == 'xlerobot_transform_calibration/v1'
    assert document['calibration_id'] == UNIT
    assert document['metrics']['sample_count'] == 12
    assert document['metrics']['translation_rmse_mm'] < 0.001
    assert document['metrics']['rotation_rmse_deg'] < 0.001
    assert document['metrics']['observability']['rotation_rank'] == 3
    assert document['x']['xyz_m'] == pytest.approx([0.031, 0.048, 0.034], abs=1e-6)
    assert (running.unit_root / 'active').readlink() == Path('versions/existing-test-version')
    assert (running.unit_root / 'active/manifest.yaml').read_text() == 'test-only previously active bundle\n'
    assert (running.unit_root / 'runtime/geometry.yaml').read_text() == 'test-only previously rendered geometry\n'
    assert running.io.captures == list(range(12))
    assert running.io.max_motion_active == 1
    assert not running.io.io_errors
    progress = yaml.safe_load(running.node.session.path.read_text())
    assert progress['phase'] == 'COMPLETED'
    assert progress['pose_states'] == ['captured'] * 12
    assert progress['sample_count'] == 12
    assert len(progress['servo_sha256']) == 64
    wait_for(lambda: any(status.phase == 'COMPLETED' and status.quality_passed
                         and not status.running for status in running.io.statuses),
             description='final status topic')


def test_real_ros_head_sweep_saves_strict_draft_and_preserves_active(tmp_path, monkeypatch):
    with runtime(tmp_path, monkeypatch) as running:
        assert running.start().accepted
        assert_completed_draft(running)
        assert running.io.moves == list(range(12))
        assert running.io.events == [
            item for index in range(12)
            for item in [('move', index), ('arrived', index), ('capture', index)]]


def test_real_ros_cancel_is_confirmed_then_resume_keeps_completed_samples(tmp_path, monkeypatch):
    with runtime(tmp_path, monkeypatch, hold_pose=3) as running:
        goal = running.start()
        assert goal.accepted
        wait_for(running.io.hold_entered.is_set, timeout=12.0, description='fourth fake motion')
        assert running.io.captures == [0, 1, 2]
        saved_prefix = running.node.session.sample_path.read_bytes()
        canceled = completed(goal.cancel_goal_async())
        assert canceled.goals_canceling
        result = completed(goal.get_result_async())
        assert result.status == GoalStatus.STATUS_CANCELED
        assert result.result.error.code == CapabilityError.CANCELED
        assert ('canceled', 3) in running.io.events
        assert running.node.session.document['phase'] == 'PAUSED'
        assert running.node.session.sample_path.read_bytes() == saved_prefix
        assert not (running.unit_root / 'draft/components/head_camera.yaml').exists()
        assert running.node.motion_unconfirmed is False
        running.io.hold_pose = None
        assert running.start().accepted
        assert_completed_draft(running)
        assert running.io.moves == [0, 1, 2, 3, *range(3, 12)]


def test_real_solver_quality_failure_does_not_publish_a_draft(tmp_path, monkeypatch):
    with runtime(tmp_path, monkeypatch) as running:
        for row in running.io.rows:
            row['quality']['reprojection_rmse_px'] = 20.0
        assert running.start().accepted
        result = completed(running.latest_goal.get_result_async(), 30.0)
        assert result.status == GoalStatus.STATUS_ABORTED
        assert not result.result.quality_passed
        assert 'reprojection' in result.result.error.message
        assert running.node.session.document['phase'] == 'ERROR'
        assert running.io.captures == list(range(12))
        assert not (running.unit_root / 'draft/components/head_camera.yaml').exists()
        assert (running.unit_root / 'active').readlink() == Path('versions/existing-test-version')
        assert (running.unit_root / 'runtime/geometry.yaml').read_text() == 'test-only previously rendered geometry\n'


@pytest.mark.parametrize('enabled,dry_run', [(False, False), (True, True)])
def test_disabled_or_dry_run_goal_never_sends_motion(tmp_path, monkeypatch, enabled, dry_run):
    with runtime(tmp_path, monkeypatch, enabled=enabled) as running:
        assert not running.start(dry_run=dry_run).accepted
        wait_for(lambda: bool(running.io.statuses), description='idle status topic')
        assert running.io.moves == []
        assert running.io.captures == []
        assert not running.node.session.sample_path.exists()
        assert not (running.unit_root / 'draft/components/head_camera.yaml').exists()
