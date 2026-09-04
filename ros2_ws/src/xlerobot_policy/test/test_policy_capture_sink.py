from pathlib import Path

import numpy as np
import pytest
import rclpy
from rclpy.action import GoalResponse
from rclpy.parameter import Parameter
from xlerobot_interfaces.action import ExecutePolicyStream
from xlerobot_interfaces.msg import PolicyJointChunk
from xlerobot_policy.policy_capture_sink import (
    CaptureSession,
    PolicyCaptureSink,
    write_replay,
)
import yaml


JOINTS = ['joint_a', 'joint_b']


def chunk(sequence, rows, *, final=False):
    message = PolicyJointChunk()
    message.session_id = 'session-test'
    message.source_id = 'act:test'
    message.sequence = sequence
    message.generated_at.sec = 10
    message.sample_period.nanosec = 50_000_000
    message.final_chunk = final
    message.joint_names = JOINTS
    message.sample_count = len(rows)
    message.positions = [value for row in rows for value in row]
    return message


def session():
    return CaptureSession(
        session_id='session-test',
        source_id='act:test',
        joint_names=JOINTS,
        start_positions=[0.0, 0.1],
        start_velocities=[0.0, 0.0],
    )


def test_capture_document_is_explicitly_nonexecuting(tmp_path: Path):
    capture = session()
    capture.accept(chunk(0, [[0.01, 0.11]]), received_at_ns=10_010_000_000)
    capture.accept(
        chunk(1, [[0.02, 0.12]], final=True),
        received_at_ns=10_015_000_000,
    )
    output = tmp_path / 'captured.yaml'
    write_replay(output, capture)

    document = yaml.safe_load(output.read_text(encoding='utf-8'))
    assert document['format'] == 'xlerobot_policy_replay/v1'
    assert document['provenance']['kind'] == 'arbitrary_pose_transport_capture'
    assert 'No motor command' in document['provenance']['note']
    assert len(document['chunks']) == 2
    assert capture.sample_count == 2


def test_capture_preserves_verified_pregrasp_context():
    capture = CaptureSession(
        session_id='session-test',
        source_id='act:test',
        joint_names=JOINTS,
        start_positions=[0.0, 0.1],
        start_velocities=[0.0, 0.0],
        start_context={
            'context_id': 'pregrasp-test',
            'established_at_ns': 9_900_000_000,
            'joint_names': JOINTS,
            'positions': [0.0, 0.1],
        },
    )
    capture.accept(
        chunk(0, [[0.01, 0.11]], final=True),
        received_at_ns=10_010_000_000,
    )
    document = capture.document()
    assert document['provenance']['kind'] == 'post_pregrasp_policy_capture'
    assert document['start_context']['context_id'] == 'pregrasp-test'


def test_capture_refuses_sequence_dimensions_and_nonfinite_values():
    capture = session()
    with pytest.raises(ValueError, match='sequence'):
        capture.accept(chunk(1, [[0.0, 0.0]]), received_at_ns=10_001_000_000)
    malformed = chunk(0, [[0.0, 0.0]])
    malformed.sample_count = 2
    with pytest.raises(ValueError, match='dimensions'):
        capture.accept(malformed, received_at_ns=10_001_000_000)
    invalid = chunk(0, [[np.nan, 0.0]])
    with pytest.raises(ValueError, match='nonfinite'):
        capture.accept(invalid, received_at_ns=10_001_000_000)


def test_replay_requires_final_chunk_and_never_overwrites(tmp_path: Path):
    capture = session()
    capture.accept(chunk(0, [[0.01, 0.11]]), received_at_ns=10_010_000_000)
    with pytest.raises(ValueError, match='final chunk'):
        write_replay(tmp_path / 'incomplete.yaml', capture)

    capture.accept(
        chunk(1, [[0.02, 0.12]], final=True),
        received_at_ns=10_015_000_000,
    )
    output = tmp_path / 'complete.yaml'
    write_replay(output, capture)
    with pytest.raises(FileExistsError):
        write_replay(output, capture)


def test_capture_sink_accepts_only_explicit_capture_only_goal(tmp_path: Path):
    rclpy.init()
    node = PolicyCaptureSink(
        parameter_overrides=[
            Parameter('output_path', value=str(tmp_path / 'capture.yaml'))
        ]
    )
    try:
        goal = ExecutePolicyStream.Goal()
        goal.source_id = 'act:test'
        goal.joint_names = JOINTS
        goal.max_duration.sec = 2
        assert node._goal_callback(goal) == GoalResponse.REJECT
        goal.capture_only = True
        assert node._goal_callback(goal) == GoalResponse.REJECT
        goal.session_id = 'capture-goal-test'
        assert node._goal_callback(goal) == GoalResponse.ACCEPT
    finally:
        node.destroy_node()
        rclpy.shutdown()
