import threading
import time
from types import SimpleNamespace

from rclpy.action import CancelResponse, GoalResponse

from xlerobot_dataset_tools import record_episode_node
from xlerobot_dataset_tools.record_episode_node import RecordEpisodeNode
from xlerobot_interfaces.action import RecordEpisode
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_interfaces.srv import FinalizeEpisode, MarkEpisodeEvent


class FakeWriter:
    def __init__(self):
        self.manifest = {
            'teleop_enabled_at': '',
            'teleop_disabled_at': '',
        }
        self.states = []
        self.timestamps = []
        self.aborted = []
        self.finished = []
        self.appended = []

    def begin(self):
        return None

    def abort(self, reason):
        self.aborted.append(reason)

    def append(self, state, images, elapsed):
        self.appended.append((state, images, elapsed))
        self.states.append(state)

    def mark_teleop_enabled(self):
        self.manifest['teleop_enabled_at'] = 'test'

    def mark_teleop_disabled(self):
        self.manifest['teleop_disabled_at'] = 'test'

    def finish(self, reason):
        self.finished.append(reason)
        raise AssertionError('unsafe episode must not be finalized')


def recorder_state(now: float):
    node = object.__new__(RecordEpisodeNode)
    node.lock = threading.Lock()
    node.io_lock = threading.Lock()
    node.input_freshness_s = 0.5
    node.camera_topics = {
        'head': '/head/image',
        'wrist': '/wrist/image',
    }
    node.latest_state = {
        name: float(index)
        for index, name in enumerate(record_episode_node.JOINT_NAMES)
    }
    node.latest_images = {'head': object(), 'wrist': object()}
    node.input_received_at = {
        'joint_state': now,
        'head': now,
        'wrist': now,
    }
    node.stop = threading.Event()
    node.cancel_requested = False
    node.committing = False
    node.dataset_id = 'dataset-001'
    return node


def valid_goal():
    goal = RecordEpisode.Goal()
    goal.dataset_id = 'dataset-001'
    goal.episode_id = 'episode-001'
    goal.collection_profile_id = 'two_wheel_pick'
    goal.max_duration.sec = 30
    return goal


def test_goal_admission_reserves_atomically():
    node = object.__new__(RecordEpisodeNode)
    node.lock = threading.Lock()
    node.active = False
    node.goal_reserved = False
    node.profile = {'profile_id': 'two_wheel_pick'}

    assert RecordEpisodeNode.goal(node, valid_goal()) == GoalResponse.ACCEPT
    assert node.goal_reserved is True
    assert RecordEpisodeNode.goal(node, valid_goal()) == GoalResponse.REJECT


def test_goal_rejects_unbounded_or_excessive_duration():
    node = object.__new__(RecordEpisodeNode)
    node.lock = threading.Lock()
    node.active = False
    node.goal_reserved = False
    node.profile = {'profile_id': 'two_wheel_pick'}
    goal = valid_goal()

    goal.max_duration.sec = 0
    assert RecordEpisodeNode.goal(node, goal) == GoalResponse.REJECT
    goal.max_duration.sec = 181
    assert RecordEpisodeNode.goal(node, goal) == GoalResponse.REJECT
    assert node.goal_reserved is False


def test_cancel_is_accepted_before_commit_and_rejected_during_commit():
    node = object.__new__(RecordEpisodeNode)
    node.lock = threading.Lock()
    node.active = True
    node.goal_reserved = True
    node.cancel_requested = False
    node.committing = False

    assert RecordEpisodeNode.cancel(node, object()) == CancelResponse.ACCEPT
    assert node.cancel_requested is True

    node.cancel_requested = False
    node.committing = True
    assert RecordEpisodeNode.cancel(node, object()) == CancelResponse.REJECT
    assert node.cancel_requested is False


def test_start_event_timeout_covers_the_bounded_pre_motion_workflow():
    assert record_episode_node.START_EVENT_TIMEOUT_S >= 120.0


def test_ready_requires_fresh_joint_state_and_both_cameras():
    now = time.monotonic()
    node = recorder_state(now)
    assert RecordEpisodeNode._ready(node, now=now + 0.1)

    node.input_received_at['wrist'] = now - 1.0
    assert not RecordEpisodeNode._ready(node, now=now + 0.1)

    node.input_received_at['wrist'] = now
    del node.latest_state[record_episode_node.JOINT_NAMES[-1]]
    assert not RecordEpisodeNode._ready(node, now=now + 0.1)


def test_sampling_stops_when_a_live_input_becomes_stale():
    now = time.monotonic()
    node = recorder_state(now)
    node.active = True
    node.armed = True
    node.stop = threading.Event()
    node.writer = object()
    node.sample_error = ''
    node.input_received_at['joint_state'] = now - 1.0

    RecordEpisodeNode._sample(node)

    assert node.stop.is_set()
    assert 'became stale' in node.sample_error


def test_sampling_failure_is_latched_before_commit_can_begin():
    now = time.monotonic()
    node = recorder_state(now)
    node.active = True
    node.armed = True
    node.episode_id = 'episode-001'
    node.sample_error = ''
    node.started_at = now
    node.frame_count = 0
    node.goal_handle = None
    node.latest_images = {
        'head': SimpleNamespace(copy=lambda: object()),
        'wrist': SimpleNamespace(copy=lambda: object()),
    }
    errors = []
    node.get_logger = lambda: SimpleNamespace(error=errors.append)

    class FailingWriter(FakeWriter):
        def append(self, _state, _images, _elapsed):
            raise RuntimeError('disk write failed')

    node.writer = FailingWriter()

    RecordEpisodeNode._sample(node)

    assert node.stop.is_set()
    assert node.armed is False
    assert node.sample_error == 'disk write failed'
    assert errors == ['episode sample failed: disk write failed']


def test_sample_rechecks_stop_after_waiting_for_io_lock():
    now = time.monotonic()
    node = recorder_state(now)
    node.active = True
    node.armed = True
    node.stop = threading.Event()
    node.writer = FakeWriter()
    node.writer.manifest.update({
        'teleop_enabled_at': 'test',
        'teleop_disabled_at': 'test',
    })
    node.episode_id = 'episode-001'
    node.sample_error = ''
    node.started_at = now
    node.frame_count = 0
    node.goal_handle = None
    node.latest_images = {
        'head': SimpleNamespace(copy=lambda: object()),
        'wrist': SimpleNamespace(copy=lambda: object()),
    }

    class Gate:
        def __init__(self):
            self.waiting = threading.Event()
            self.release = threading.Event()

        def __enter__(self):
            self.waiting.set()
            assert self.release.wait(timeout=2.0)

        def __exit__(self, *_args):
            return False

    gate = Gate()
    node.io_lock = gate
    worker = threading.Thread(target=RecordEpisodeNode._sample, args=(node,))
    worker.start()
    assert gate.waiting.wait(timeout=2.0)

    response = RecordEpisodeNode.finalize(
        node,
        FinalizeEpisode.Request(
            dataset_id='dataset-001', episode_id='episode-001'
        ),
        FinalizeEpisode.Response(),
    )
    gate.release.set()
    worker.join(timeout=2.0)

    assert response.error.code == CapabilityError.NONE
    assert node.armed is False
    assert node.stop.is_set()
    assert node.writer.appended == []


def test_start_event_rechecks_inputs_and_arms_transactionally():
    now = time.monotonic()
    node = recorder_state(now)
    node.io_lock = threading.Lock()
    node.active = True
    node.armed = False
    node.episode_id = 'episode-001'
    node.writer = FakeWriter()
    feedback = []
    node.goal_handle = SimpleNamespace(
        publish_feedback=lambda message: feedback.append(message)
    )
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: object())
    )
    request = MarkEpisodeEvent.Request(
        dataset_id='dataset-001',
        episode_id='episode-001',
        event='teleop_enabled',
    )

    response = RecordEpisodeNode.mark_event(
        node, request, MarkEpisodeEvent.Response()
    )

    assert response.error.code == CapabilityError.NONE
    assert node.armed is True
    assert node.writer.manifest['teleop_enabled_at'] == 'test'
    assert feedback[-1].state.phase == 'ARMED'


def test_start_event_rejects_stale_input_without_arming():
    now = time.monotonic()
    node = recorder_state(now)
    node.io_lock = threading.Lock()
    node.active = True
    node.armed = False
    node.episode_id = 'episode-001'
    node.writer = FakeWriter()
    node.goal_handle = None
    node.input_received_at['wrist'] = now - 1.0
    request = MarkEpisodeEvent.Request(
        dataset_id='dataset-001',
        episode_id='episode-001',
        event='teleop_enabled',
    )

    response = RecordEpisodeNode.mark_event(
        node, request, MarkEpisodeEvent.Response()
    )

    assert response.error.code == CapabilityError.UNAVAILABLE
    assert node.armed is False
    assert node.writer.manifest['teleop_enabled_at'] == ''


def test_disable_event_stops_sampling_and_allows_scoped_finalize():
    now = time.monotonic()
    node = recorder_state(now)
    node.active = True
    node.armed = True
    node.episode_id = 'episode-001'
    node.writer = FakeWriter()
    node.writer.manifest['teleop_enabled_at'] = 'test'
    node.goal_handle = None
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: object())
    )

    boundary = RecordEpisodeNode.mark_event(
        node,
        MarkEpisodeEvent.Request(
            dataset_id='dataset-001',
            episode_id='episode-001',
            event='teleop_disabled',
        ),
        MarkEpisodeEvent.Response(),
    )

    assert boundary.error.code == CapabilityError.NONE
    assert node.armed is False
    assert not node.stop.is_set()
    assert node.writer.manifest['teleop_disabled_at'] == 'test'

    finalized = RecordEpisodeNode.finalize(
        node,
        FinalizeEpisode.Request(
            dataset_id='dataset-001', episode_id='episode-001'
        ),
        FinalizeEpisode.Response(),
    )
    assert finalized.error.code == CapabilityError.NONE
    assert node.stop.is_set()


def test_finalize_without_disable_boundary_is_rejected_without_stopping():
    now = time.monotonic()
    node = recorder_state(now)
    node.active = True
    node.armed = True
    node.episode_id = 'episode-001'
    node.writer = FakeWriter()
    node.writer.manifest['teleop_enabled_at'] = 'test'

    response = RecordEpisodeNode.finalize(
        node,
        FinalizeEpisode.Request(
            dataset_id='dataset-001', episode_id='episode-001'
        ),
        FinalizeEpisode.Response(),
    )

    assert response.error.code == CapabilityError.INVALID_GOAL
    assert node.armed is True
    assert not node.stop.is_set()


def test_start_event_is_rejected_after_finalize_begins():
    now = time.monotonic()
    node = recorder_state(now)
    node.active = True
    node.armed = False
    node.episode_id = 'episode-001'
    node.writer = FakeWriter()
    node.goal_handle = None
    node.stop.set()
    request = MarkEpisodeEvent.Request(
        dataset_id='dataset-001',
        episode_id='episode-001',
        event='teleop_enabled',
    )

    response = RecordEpisodeNode.mark_event(
        node, request, MarkEpisodeEvent.Response()
    )

    assert response.error.code == CapabilityError.INVALID_GOAL
    assert node.armed is False
    assert node.writer.manifest['teleop_enabled_at'] == ''


def test_late_event_from_another_dataset_cannot_arm_current_episode():
    now = time.monotonic()
    node = recorder_state(now)
    node.active = True
    node.armed = False
    node.episode_id = 'episode-001'
    node.writer = FakeWriter()
    node.goal_handle = None
    request = MarkEpisodeEvent.Request(
        dataset_id='previous-dataset',
        episode_id='episode-001',
        event='teleop_enabled',
    )

    response = RecordEpisodeNode.mark_event(
        node, request, MarkEpisodeEvent.Response()
    )

    assert response.error.code == CapabilityError.NOT_FOUND
    assert node.armed is False
    assert node.writer.manifest['teleop_enabled_at'] == ''


def test_late_finalize_from_another_dataset_cannot_stop_current_episode():
    now = time.monotonic()
    node = recorder_state(now)
    node.active = True
    node.armed = True
    node.episode_id = 'episode-001'
    node.writer = FakeWriter()

    response = RecordEpisodeNode.finalize(
        node,
        FinalizeEpisode.Request(
            dataset_id='previous-dataset', episode_id='episode-001'
        ),
        FinalizeEpisode.Response(),
    )

    assert response.error.code == CapabilityError.NOT_FOUND
    assert node.armed is True
    assert not node.stop.is_set()


def test_early_execute_failure_releases_goal_reservation(
    tmp_path, monkeypatch
):
    node = object.__new__(RecordEpisodeNode)
    node.lock = threading.Lock()
    node.io_lock = threading.Lock()
    node.dataset_root = tmp_path
    node.minimum_free_bytes = 10
    node.goal_reserved = True
    node.active = False
    node.armed = False
    node.dataset_id = ''
    node.episode_id = ''
    node.writer = None
    node.goal_handle = None
    monkeypatch.setattr(
        record_episode_node.shutil,
        'disk_usage',
        lambda _path: SimpleNamespace(free=0),
    )

    events = []
    handle = SimpleNamespace(
        request=valid_goal(),
        abort=lambda: events.append('aborted'),
    )
    result = RecordEpisodeNode.execute(node, handle)

    assert result.error.message == 'insufficient disk space'
    assert events == ['aborted']
    assert node.goal_reserved is False


def execute_state(tmp_path):
    node = object.__new__(RecordEpisodeNode)
    node.lock = threading.Lock()
    node.io_lock = threading.Lock()
    node.dataset_root = tmp_path
    node.minimum_free_bytes = 0
    node.profile = {
        'fps': 30,
        'observation': {'topic': '/joint_states'},
    }
    node.revision = 'test-revision'
    node.unit_id = 'unit-test'
    node.calibration_version = 'calibration-test'
    node.camera_topics = {
        'head': '/head/image',
        'wrist': '/wrist/image',
    }
    node.stop = threading.Event()
    node.goal_reserved = True
    node.active = False
    node.armed = False
    node.dataset_id = ''
    node.episode_id = ''
    node.writer = None
    node.goal_handle = None
    node.frame_count = 0
    node.sample_error = ''
    node.cancel_requested = False
    node.committing = False
    node._ready = lambda: True
    return node


def test_failsafe_timeout_aborts_instead_of_publishing_episode(
    tmp_path, monkeypatch
):
    node = execute_state(tmp_path)
    writer = FakeWriter()
    monkeypatch.setattr(
        record_episode_node, 'EpisodeWriter', lambda *_args, **_kwargs: writer
    )
    monkeypatch.setattr(record_episode_node.rclpy, 'ok', lambda: True)
    goal = valid_goal()
    goal.max_duration.sec = 0
    goal.max_duration.nanosec = 1
    events = []

    def feedback(message):
        if message.state.phase == 'READY':
            with node.lock:
                writer.manifest['teleop_enabled_at'] = 'test'
                node.started_at = time.monotonic()
                node.armed = True

    handle = SimpleNamespace(
        request=goal,
        is_cancel_requested=False,
        publish_feedback=feedback,
        abort=lambda: events.append('aborted'),
        canceled=lambda: events.append('canceled'),
        succeed=lambda: events.append('succeeded'),
    )

    result = RecordEpisodeNode.execute(node, handle)

    assert result.error.code != 0
    assert 'fail-safe deadline expired' in result.error.message
    assert events == ['aborted']
    assert writer.finished == []
    assert writer.aborted


def test_finalize_before_recording_start_keeps_episode_incomplete(
    tmp_path, monkeypatch
):
    node = execute_state(tmp_path)
    writer = FakeWriter()
    monkeypatch.setattr(
        record_episode_node, 'EpisodeWriter', lambda *_args, **_kwargs: writer
    )
    monkeypatch.setattr(record_episode_node.rclpy, 'ok', lambda: True)
    events = []

    def feedback(message):
        if message.state.phase == 'READY':
            node.stop.set()

    handle = SimpleNamespace(
        request=valid_goal(),
        is_cancel_requested=False,
        publish_feedback=feedback,
        abort=lambda: events.append('aborted'),
        canceled=lambda: events.append('canceled'),
        succeed=lambda: events.append('succeeded'),
    )

    result = RecordEpisodeNode.execute(node, handle)

    assert result.error.code != 0
    assert 'finalize arrived before recording started' in result.error.message
    assert events == ['aborted']
    assert writer.finished == []
    assert writer.aborted


def test_ros_shutdown_before_arming_keeps_episode_incomplete(
    tmp_path, monkeypatch
):
    node = execute_state(tmp_path)
    writer = FakeWriter()
    monkeypatch.setattr(
        record_episode_node, 'EpisodeWriter', lambda *_args, **_kwargs: writer
    )
    monkeypatch.setattr(record_episode_node.rclpy, 'ok', lambda: False)
    events = []
    handle = SimpleNamespace(
        request=valid_goal(),
        is_cancel_requested=False,
        publish_feedback=lambda _feedback: None,
        abort=lambda: events.append('aborted'),
        canceled=lambda: events.append('canceled'),
        succeed=lambda: events.append('succeeded'),
    )

    result = RecordEpisodeNode.execute(node, handle)

    assert 'ROS shutdown before recorder armed' in result.error.message
    assert events == ['aborted']
    assert writer.finished == []
    assert writer.aborted


def test_start_event_wait_is_bounded(tmp_path, monkeypatch):
    node = execute_state(tmp_path)
    writer = FakeWriter()
    monkeypatch.setattr(
        record_episode_node, 'EpisodeWriter', lambda *_args, **_kwargs: writer
    )
    monkeypatch.setattr(record_episode_node.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(record_episode_node, 'START_EVENT_TIMEOUT_S', 0.0)
    events = []
    handle = SimpleNamespace(
        request=valid_goal(),
        is_cancel_requested=False,
        publish_feedback=lambda _feedback: None,
        abort=lambda: events.append('aborted'),
        canceled=lambda: events.append('canceled'),
        succeed=lambda: events.append('succeeded'),
    )

    result = RecordEpisodeNode.execute(node, handle)

    assert 'recording start event timed out' in result.error.message
    assert events == ['aborted']
    assert writer.finished == []
    assert writer.aborted
