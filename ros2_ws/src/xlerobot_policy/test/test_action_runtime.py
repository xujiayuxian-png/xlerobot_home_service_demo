import os
import threading
import time

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, JointState
from xlerobot_interfaces.action import ExecuteLearnedPolicy, ExecutePolicyStream
from xlerobot_interfaces.msg import CapabilityError, PolicyJointChunk
from xlerobot_policy.act_policy_node import ActPolicyNode, JOINTS


def wait_future(future, timeout_s=8.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done()
    return future.result()


class FakeStreamingExecutor(Node):
    def __init__(self):
        super().__init__('fake_streaming_executor')
        self.group = ReentrantCallbackGroup()
        self.condition = threading.Condition()
        self.goal_handle = None
        self.chunks = []
        self.stall = False
        self.canceled = threading.Event()
        self.server = ActionServer(
            self,
            ExecutePolicyStream,
            'execute_policy_stream',
            execute_callback=self.execute,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=self.group,
        )
        self.subscription = self.create_subscription(
            PolicyJointChunk,
            'policy_joint_chunks',
            self.on_chunk,
            10,
            callback_group=self.group,
        )

    def execute(self, goal_handle):
        result = ExecutePolicyStream.Result()
        assert not goal_handle.request.capture_only
        session_id = goal_handle.request.session_id
        assert session_id
        if goal_handle.request.dry_run:
            result.error.code = CapabilityError.NONE
            result.error.message = 'fake executor dry-run valid'
            result.session_id = session_id
            goal_handle.succeed()
            return result
        with self.condition:
            self.goal_handle = goal_handle
            self.session_id = session_id
            feedback = ExecutePolicyStream.Feedback()
            feedback.state.phase = 'armed'
            feedback.state.progress = 0.1
            feedback.session_id = session_id
            goal_handle.publish_feedback(feedback)
            deadline = time.monotonic() + 6.0
            while (
                (
                    self.stall
                    or not any(chunk.final_chunk for chunk in self.chunks)
                )
                and not goal_handle.is_cancel_requested
                and time.monotonic() < deadline
            ):
                self.condition.wait(timeout=0.02)
            if goal_handle.is_cancel_requested:
                result.error.code = CapabilityError.CANCELED
                result.error.message = 'fake executor canceled'
                self.canceled.set()
                goal_handle.canceled()
                return result
            if not any(chunk.final_chunk for chunk in self.chunks):
                result.error.code = CapabilityError.TIMEOUT
                result.error.message = 'fake executor timed out'
                goal_handle.abort()
                return result
            result.error.code = CapabilityError.NONE
            result.error.message = 'fake stream complete'
            result.session_id = session_id
            result.accepted_chunks = len(self.chunks)
            result.executed_samples = sum(chunk.sample_count for chunk in self.chunks)
            goal_handle.succeed()
            return result

    def on_chunk(self, message):
        with self.condition:
            assert message.session_id == self.session_id
            assert message.source_id == 'act:act_xlerobot_box_wrist_only'
            assert message.sequence == len(self.chunks)
            self.chunks.append(message)
            feedback = ExecutePolicyStream.Feedback()
            feedback.state.phase = 'running'
            feedback.state.progress = 0.5
            feedback.session_id = self.session_id
            feedback.accepted_chunks = len(self.chunks)
            feedback.executed_samples = sum(
                chunk.sample_count for chunk in self.chunks
            )
            feedback.queued_horizon_s = 0.0
            self.goal_handle.publish_feedback(feedback)
            self.condition.notify_all()


def policy_goal(*, dry_run, node=None, with_context=True):
    goal = ExecuteLearnedPolicy.Goal()
    goal.policy_id = 'act_xlerobot_box_wrist_only'
    goal.object_id = 'camping_lamp'
    goal.max_duration.sec = 10
    goal.dry_run = dry_run
    if not dry_run and with_context:
        goal.start_context.context_id = 'pregrasp-test'
        goal.start_context.established_at = node.get_clock().now().to_msg()
        goal.start_context.joint_names = list(JOINTS)
        goal.start_context.positions = [0.25, 0.0, 0.0, 0.0, 0.0, 1.0]
    return goal


def test_dry_run_and_session_chunks_never_need_a_controller_or_network():
    os.environ['ROS_DOMAIN_ID'] = '84'
    rclpy.init()
    fake = FakeStreamingExecutor()
    policy = ActPolicyNode(parameter_overrides=[Parameter('backend_enabled', value=True)])
    client_node = Node('act_policy_runtime_client')
    client = ActionClient(client_node, ExecuteLearnedPolicy, 'execute_learned_policy')
    head_pub = client_node.create_publisher(
        Image, '/xlerobot/d455/color/image_raw', 10
    )
    wrist_pub = client_node.create_publisher(Image, '/right_wrist_camera/image_raw', 10)
    joint_pub = client_node.create_publisher(JointState, '/joint_states', 10)
    calls = []
    prediction_mode = {'name': 'close', 'count': 0}

    def predict(**_observation):
        mode = prediction_mode['name']
        prediction_mode['count'] += 1
        calls.append(mode)
        if mode == 'refill' and prediction_mode['count'] == 1:
            return [
                [0.9, -index / 30.0, 0.2, 0.3, 0.4, 1.0]
                for index in range(30)
            ]
        rows = []
        for index in range(40):
            close_step = 15 if mode == 'refill' else 25
            lift = -min(index, close_step) / float(close_step)
            gripper = 1.0 if index < close_step else 0.0
            rows.append([0.9, lift, 0.2, 0.3, 0.4, gripper])
        return rows

    policy.http.predict = predict

    def publish_observation():
        stamp = client_node.get_clock().now().to_msg()
        image = Image()
        image.header.stamp = stamp
        image.height = 4
        image.width = 4
        image.encoding = 'bgr8'
        image.step = 12
        image.data = bytes(48)
        head_pub.publish(image)
        wrist_pub.publish(image)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = list(JOINTS)
        joints.position = [0.25, 0.0, 0.0, 0.0, 0.0, 1.0]
        joint_pub.publish(joints)

    timer = client_node.create_timer(0.05, publish_observation)
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (fake, policy, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=3.0)
        dry_handle = wait_future(client.send_goal_async(policy_goal(dry_run=True)))
        dry_result = wait_future(dry_handle.get_result_async()).result
        assert dry_result.error.code == CapabilityError.NONE
        assert not calls
        assert not fake.chunks

        missing_handle = wait_future(
            client.send_goal_async(
                policy_goal(dry_run=False, node=client_node, with_context=False)
            )
        )
        missing = wait_future(missing_handle.get_result_async())
        assert missing.status == GoalStatus.STATUS_ABORTED
        assert missing.result.error.code == CapabilityError.SAFETY_REJECTED
        assert 'pregrasp start context' in missing.result.error.message
        assert not calls
        assert not fake.chunks

        handle = wait_future(
            client.send_goal_async(policy_goal(dry_run=False, node=client_node))
        )
        wrapped = wait_future(handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.error.code == CapabilityError.NONE
        assert calls == ['close']
        assert len(fake.chunks) == 7
        assert [chunk.sequence for chunk in fake.chunks] == list(range(7))
        assert all(chunk.sample_count == 10 for chunk in fake.chunks)
        assert not any(chunk.final_chunk for chunk in fake.chunks[:-1])
        assert fake.chunks[-1].final_chunk
        assert all(
            value == 0.25
            for chunk in fake.chunks
            for value in chunk.positions[0::len(JOINTS)]
        )
        assert fake.chunks[-1].positions[-1] == 0.0

        with fake.condition:
            fake.chunks.clear()
        prediction_mode.update(name='refill', count=0)
        refill_handle = wait_future(
            client.send_goal_async(policy_goal(dry_run=False, node=client_node))
        )
        refill_result = wait_future(refill_handle.get_result_async())
        assert refill_result.status == GoalStatus.STATUS_SUCCEEDED
        assert refill_result.result.error.code == CapabilityError.NONE
        refill_predictions = prediction_mode['count']
        assert refill_predictions >= 2
        assert len(calls[1:]) == refill_predictions
        assert set(calls[1:]) == {'refill'}
        total_samples = sum(chunk.sample_count for chunk in fake.chunks)
        assert 90 <= total_samples <= 100
        assert [chunk.sequence for chunk in fake.chunks] == list(
            range(len(fake.chunks))
        )
        assert not any(chunk.final_chunk for chunk in fake.chunks[:-1])
        assert fake.chunks[-1].final_chunk
        assert fake.chunks[-1].positions[-1] == 0.0

        with fake.condition:
            fake.chunks.clear()
            fake.stall = True
        prediction_mode.update(name='close', count=0)
        cancel_handle = wait_future(
            client.send_goal_async(policy_goal(dry_run=False, node=client_node))
        )
        deadline = time.monotonic() + 3.0
        while len(fake.chunks) < 7 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(fake.chunks) == 7
        cancel_response = wait_future(cancel_handle.cancel_goal_async())
        assert len(cancel_response.goals_canceling) == 1
        canceled = wait_future(cancel_handle.get_result_async())
        assert canceled.status == GoalStatus.STATUS_CANCELED
        assert canceled.result.error.code == CapabilityError.CANCELED
        assert fake.canceled.wait(timeout=2.0)
        cancel_predictions = calls[1 + refill_predictions:]
        assert calls[0] == 'close'
        assert cancel_predictions
        assert set(cancel_predictions) == {'close'}
    finally:
        client_node.destroy_timer(timer)
        executor.shutdown(timeout_sec=3.0)
        thread.join(timeout=3.0)
        client.destroy()
        client_node.destroy_node()
        policy.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()
