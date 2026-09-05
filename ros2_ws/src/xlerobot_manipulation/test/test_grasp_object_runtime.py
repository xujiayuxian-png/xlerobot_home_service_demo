import os
from pathlib import Path
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from xlerobot_interfaces.action import (
    ExecuteLearnedPolicy, GraspObject, PrepareGrasp, VerifyGrasp,
)
from xlerobot_interfaces.msg import CapabilityError


JOINTS = [
    'right_arm_shoulder_pan',
    'right_arm_shoulder_lift',
    'right_arm_elbow_flex',
    'right_arm_wrist_flex',
    'right_arm_wrist_roll',
]
POLICY_JOINTS = JOINTS + ['right_arm_gripper']


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '73'
    server = Node(
        package='xlerobot_manipulation',
        executable='grasp_object_server',
        name='grasp_object_server',
        parameters=[
            {
                'execution_enabled': True,
                'joint_state_timeout_s': 1.0,
                'head_move_duration_s': 0.1,
                'pregrasp_gripper_duration_s': 0.1,
                'return_ready_duration_s': 0.1,
                'pregrasp_settle_timeout_s': 0.4,
                'pregrasp_settle_hold_s': 0.05,
                'grasp_alignment_file': str(
                    Path(__file__).parent / 'data' / 'grasp_alignment.yaml'
                ),
            }
        ],
        output='screen',
    )
    return LaunchDescription([server, launch_testing.actions.ReadyToTest()])


class FakeGraspBackends(RclpyNode):
    def __init__(self):
        super().__init__('fake_grasp_backends')
        self.group = ReentrantCallbackGroup()
        self.events = []
        self.stall_policy = False
        self.verification_results = []
        self.policy_gripper_positions = []
        self.arm_updates_state = True
        self.gripper_updates_state = True
        self.policy_canceled = threading.Event()
        self.positions = [0.0] * 5 + [1.0]
        self.policy_contexts = []
        self.arm_trajectories = []
        self.ik_requests = []
        self.ik_service = self.create_service(
            GetPositionIK,
            '/compute_ik',
            self.on_ik,
            callback_group=self.group,
        )
        self.plan_service = self.create_service(
            GetMotionPlan,
            '/plan_kinematic_path',
            self.on_plan,
            callback_group=self.group,
        )
        self.head_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory',
            execute_callback=lambda handle: self.follow(handle, 'head'),
            callback_group=self.group,
        )
        self.arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory',
            execute_callback=lambda handle: self.follow(handle, 'arm'),
            callback_group=self.group,
        )
        self.gripper_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/right_gripper_controller/follow_joint_trajectory',
            execute_callback=lambda handle: self.follow(handle, 'gripper'),
            callback_group=self.group,
        )
        self.policy_server = ActionServer(
            self,
            ExecuteLearnedPolicy,
            '/execute_learned_policy',
            execute_callback=self.policy,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
            callback_group=self.group,
        )
        self.verification_server = ActionServer(
            self,
            VerifyGrasp,
            '/verify_grasp',
            execute_callback=self.verify_grasp,
            callback_group=self.group,
        )
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_state)

    def on_ik(self, request, response):
        self.events.append('ik')
        self.ik_requests.append(request)
        response.error_code.val = MoveItErrorCodes.SUCCESS
        response.solution.joint_state.name = list(JOINTS)
        response.solution.joint_state.position = [
            self.positions[0] + 0.1,
            self.positions[1],
            self.positions[2],
            1.2,
            self.positions[4],
        ]
        return response

    def on_plan(self, request, response):
        self.events.append('plan')
        plan = response.motion_plan_response
        plan.error_code.val = MoveItErrorCodes.SUCCESS
        plan.group_name = 'right_arm'
        trajectory = plan.trajectory.joint_trajectory
        trajectory.joint_names = list(JOINTS)
        start = JointTrajectoryPoint()
        start.positions = list(self.positions[:5])
        finish = JointTrajectoryPoint()
        finish.positions = [
            self.positions[0] + 0.1,
            self.positions[1],
            self.positions[2],
            1.2,
            self.positions[4],
        ]
        finish.time_from_start.sec = 1
        trajectory.points = [start, finish]
        return response

    def follow(self, goal_handle, label):
        self.events.append(label)
        trajectory = goal_handle.request.trajectory
        if label == 'arm':
            self.arm_trajectories.append(trajectory)
        if (
            label in ('arm', 'gripper')
            and (
                (label == 'arm' and self.arm_updates_state)
                or (label == 'gripper' and self.gripper_updates_state)
            )
            and trajectory.points
        ):
            final = trajectory.points[-1].positions
            positions = dict(zip(trajectory.joint_names, final))
            for index, name in enumerate(JOINTS):
                if name in positions:
                    self.positions[index] = positions[name]
            if 'right_arm_gripper' in positions:
                self.positions[-1] = positions['right_arm_gripper']
                self.uncorrectable_goal_tolerance_once = False
            # A successful controller result must follow, not race ahead of,
            # the joint-state evidence consumed by GraspObject.
            time.sleep(0.08)
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result

    def policy(self, goal_handle):
        self.events.append('policy_dry' if goal_handle.request.dry_run else 'policy')
        if not goal_handle.request.dry_run:
            context = goal_handle.request.start_context
            assert context.context_id.startswith('pregrasp-')
            assert list(context.joint_names) == POLICY_JOINTS
            assert list(context.positions) == self.positions
            self.policy_contexts.append(context)
        result = ExecuteLearnedPolicy.Result()
        if self.stall_policy and not goal_handle.request.dry_run:
            deadline = time.monotonic() + 5.0
            while (
                not goal_handle.is_cancel_requested
                and time.monotonic() < deadline
            ):
                time.sleep(0.01)
            if goal_handle.is_cancel_requested:
                result.error.code = CapabilityError.CANCELED
                result.error.message = 'fake policy canceled'
                goal_handle.canceled()
                self.policy_canceled.set()
                return result
        if not goal_handle.request.dry_run:
            self.positions[-1] = (
                self.policy_gripper_positions.pop(0)
                if self.policy_gripper_positions else 0.20
            )
        result.error.code = CapabilityError.NONE
        result.error.message = 'fake policy complete'
        goal_handle.succeed()
        return result

    def verify_grasp(self, goal_handle):
        self.events.append('verify')
        feedback = VerifyGrasp.Feedback()
        feedback.state.phase = 'verifying'
        feedback.state.progress = 0.4
        goal_handle.publish_feedback(feedback)
        result = VerifyGrasp.Result()
        verification_succeeds = (
            self.verification_results.pop(0)
            if self.verification_results else True
        )
        if not verification_succeeds:
            result.error.code = CapabilityError.NOT_FOUND
            result.error.message = 'fake wrist verification failed'
            result.grasped = False
            result.confidence = 0.0
            result.reason = 'empty gripper'
            goal_handle.abort()
            return result
        result.error.code = CapabilityError.NONE
        result.error.message = 'fake wrist verification passed'
        result.grasped = True
        result.confidence = 0.95
        result.reason = 'object held between fingers'
        goal_handle.succeed()
        return result

    def publish_state(self):
        stamp = self.get_clock().now().to_msg()
        state = JointState()
        state.header.stamp = stamp
        state.name = list(POLICY_JOINTS)
        state.position = list(self.positions)
        state.velocity = [0.0] * len(POLICY_JOINTS)
        self.joint_pub.publish(state)


class GraspObjectRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.fake = FakeGraspBackends()
        cls.client_node = rclpy.create_node('grasp_object_runtime_client')
        cls.client = ActionClient(cls.client_node, GraspObject, '/grasp_object')
        cls.prepare_client = ActionClient(
            cls.client_node, PrepareGrasp, '/prepare_grasp'
        )
        cls.executor = MultiThreadedExecutor(num_threads=8)
        cls.executor.add_node(cls.fake)
        cls.executor.add_node(cls.client_node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        assert cls.client.wait_for_server(timeout_sec=5.0)
        assert cls.prepare_client.wait_for_server(timeout_sec=5.0)
        time.sleep(0.2)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown(timeout_sec=3.0)
        cls.thread.join(timeout=3.0)
        cls.client.destroy()
        cls.prepare_client.destroy()
        cls.client_node.destroy_node()
        cls.fake.destroy_node()
        rclpy.shutdown()

    def test_01_dry_run_plans_and_checks_policy_without_controller_goals(self):
        self.fake.events.clear()
        self.fake.ik_requests.clear()
        wrapped = self._run(self._goal(dry_run=True))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(self.fake.events, ['ik', 'plan', 'policy_dry'])
        request = self.fake.ik_requests[0].ik_request
        self.assertEqual(
            list(request.robot_state.joint_state.position[:5]),
            [-0.10278, -0.53076, -0.5645, 1.2, 0.06981317],
        )
        self.assertAlmostEqual(request.pose_stamped.pose.position.x, 0.2055)
        self.assertAlmostEqual(request.pose_stamped.pose.position.y, -0.1943)
        self.assertAlmostEqual(request.pose_stamped.pose.position.z, 0.5245)
        self.assertAlmostEqual(request.pose_stamped.pose.orientation.x, 0.70710678)
        self.assertAlmostEqual(request.pose_stamped.pose.orientation.w, 0.70710678)

    def test_01a_prepare_grasp_reuses_the_same_plan_without_starting_policy(self):
        self.fake.events.clear()
        grasp_goal = self._goal(dry_run=True)
        goal = PrepareGrasp.Goal()
        goal.object_id = grasp_goal.object_id
        goal.target = grasp_goal.target
        goal.dry_run = True
        handle = self._wait(self.prepare_client.send_goal_async(goal), 3.0)
        self.assertTrue(handle.accepted)
        wrapped = self._wait(handle.get_result_async(), 5.0)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(self.fake.events, ['ik', 'plan'])
        self.assertEqual(wrapped.result.start_context.context_id, '')

    def test_01b_classical_split_compensation_and_sag_are_applied_once(self):
        self.fake.events.clear()
        self.fake.ik_requests.clear()
        goal = self._goal(dry_run=True)
        goal.backend = 'centroid'
        plan = goal.grasp_plan
        plan.valid = True
        plan.observation_id = 'public-fixture-observation'
        plan.backend = 'centroid'
        plan.method = 'sam2_rgbd_top_centroid'
        plan.score = 0.9
        plan.table_height_m = 0.80
        plan.object_height_m = 0.06
        plan.grasp_width_m = 0.04
        plan.wrist_yaw_seed_rad = 0.0
        plan.wrist_yaw_min_rad = -3.0
        plan.wrist_yaw_max_rad = 3.0
        plan.selection_reason = 'shared public RGB-D fixture'
        for pose, xyz in (
            (plan.pregrasp, (0.10, -0.05, 0.88)),
            (plan.grasp, (0.10, -0.05, 0.86)),
            (plan.lift, (0.10, -0.05, 0.89)),
        ):
            pose.header.frame_id = 'base_link'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
            pose.pose.orientation.x = 0.70710678
            pose.pose.orientation.w = 0.70710678
        wrapped = self._run(goal)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.backend_used, 'centroid')
        self.assertEqual(self.fake.events, ['ik', 'plan', 'ik', 'plan', 'ik', 'plan'])
        poses = [
            request.ik_request.pose_stamped.pose.position
            for request in self.fake.ik_requests
        ]
        # pregrasp/lift add split vision compensation and gravity sag.
        self.assertAlmostEqual(poses[0].x, 0.1055)
        self.assertAlmostEqual(poses[0].y, -0.0443)
        self.assertAlmostEqual(poses[0].z, 0.9245)
        # The contact grasp adds vision compensation only.
        self.assertAlmostEqual(poses[1].z, 0.8665)
        self.assertAlmostEqual(poses[2].z, 0.9345)

    def test_01c_gpd_accepts_finite_backend_native_ranking_scores(self):
        for score in (-0.25, 2.75):
            with self.subTest(score=score):
                self.fake.events.clear()
                goal = self._goal(dry_run=True)
                goal.backend = 'gpd'
                plan = goal.grasp_plan
                plan.valid = True
                plan.observation_id = 'gpd-native-score'
                plan.backend = 'gpd'
                plan.method = 'sam2_rgbd_gpd_top'
                plan.score = score
                plan.table_height_m = 0.80
                plan.object_height_m = 0.06
                plan.grasp_width_m = 0.04
                plan.wrist_yaw_seed_rad = 0.0
                plan.wrist_yaw_min_rad = -3.0
                plan.wrist_yaw_max_rad = 3.0
                plan.selection_reason = 'native GPD logit-margin rank'
                for pose, xyz in (
                    (plan.pregrasp, (0.10, -0.05, 0.88)),
                    (plan.grasp, (0.10, -0.05, 0.86)),
                    (plan.lift, (0.10, -0.05, 0.89)),
                ):
                    pose.header.frame_id = 'base_link'
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
                    pose.pose.orientation.x = 0.70710678
                    pose.pose.orientation.w = 0.70710678
                wrapped = self._run(goal)
                self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
                self.assertEqual(wrapped.result.backend_used, 'gpd')
                self.assertEqual(
                    self.fake.events, ['ik', 'plan', 'ik', 'plan', 'ik', 'plan']
                )

    def test_01d_classical_uses_shared_workspace_not_calibration_sample_region(self):
        for x, succeeds in ((0.10, True), (0.85, False)):
            with self.subTest(x=x):
                goal = self._goal(dry_run=True)
                goal.backend = 'centroid'
                plan = goal.grasp_plan
                plan.valid = True
                plan.observation_id = 'shared-runtime-workspace'
                plan.backend = 'centroid'
                plan.method = 'sam2_rgbd_top_centroid'
                plan.score = 0.9
                plan.table_height_m = 0.70
                plan.object_height_m = 0.04
                plan.grasp_width_m = 0.03
                plan.wrist_yaw_min_rad = -3.0
                plan.wrist_yaw_max_rad = 3.0
                plan.selection_reason = 'same runtime workspace as ACT'
                for pose, z in ((plan.pregrasp, 0.82), (plan.grasp, 0.73),
                                (plan.lift, 0.84)):
                    pose.header.frame_id = 'base_link'
                    pose.pose.position.x = x
                    pose.pose.position.y = -0.05
                    pose.pose.position.z = z
                    pose.pose.orientation.x = 0.70710678
                    pose.pose.orientation.w = 0.70710678
                wrapped = self._run(goal)
                if succeeds:
                    self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
                else:
                    self.assertEqual(wrapped.result.error.code, CapabilityError.SAFETY_REJECTED)
                    self.assertIn('outside workspace', wrapped.result.error.message)

    def test_02_live_sequence_is_pregrasp_policy_return_ready_then_head_up(self):
        self.fake.events.clear()
        self.fake.policy_contexts.clear()
        self.fake.arm_trajectories.clear()
        wrapped = self._run(self._goal(dry_run=False))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(
            self.fake.events,
            ['ik', 'plan', 'gripper', 'arm', 'policy', 'arm', 'verify', 'head'],
        )
        self.assertEqual(len(self.fake.policy_contexts), 1)
        pregrasp = self.fake.arm_trajectories[0]
        self.assertEqual(list(pregrasp.joint_names), JOINTS)
        self.assertEqual(len(pregrasp.points), 1)
        self.assertEqual(list(pregrasp.points[0].velocities), [0.0] * len(JOINTS))
        self.assertEqual(list(pregrasp.points[0].accelerations), [0.0] * len(JOINTS))
        self.assertEqual(pregrasp.points[0].time_from_start.sec, 4)
        self.assertEqual(pregrasp.points[0].time_from_start.nanosec, 0)

    def test_02a_failed_verification_retries_the_complete_grasp_once(self):
        self.fake.events.clear()
        self.fake.verification_results = [False, True]
        self.fake.policy_gripper_positions = [0.0, 0.20]
        wrapped = self._run(self._goal(dry_run=False))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(
            self.fake.events,
            [
                'ik', 'plan', 'gripper', 'arm', 'policy', 'arm', 'verify',
                'ik', 'plan', 'gripper', 'arm', 'policy', 'arm', 'verify', 'head',
            ],
        )

    def test_02b_two_failed_verifications_abort_after_the_single_retry(self):
        self.fake.events.clear()
        self.fake.verification_results = [False, False]
        self.fake.policy_gripper_positions = [0.0, 0.0]
        wrapped = self._run(self._goal(dry_run=False))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NOT_FOUND)
        self.assertEqual(self.fake.events.count('policy'), 2)
        self.assertEqual(self.fake.events.count('verify'), 2)
        self.assertEqual(self.fake.events[-1], 'head')

    def test_02c_vlm_false_negative_does_not_retry_a_blocked_gripper(self):
        self.fake.events.clear()
        self.fake.verification_results = [False]
        self.fake.policy_gripper_positions = [0.20]
        wrapped = self._run(self._goal(dry_run=False))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(self.fake.events.count('policy'), 1)
        self.assertEqual(self.fake.events.count('verify'), 1)
        self.assertEqual(self.fake.events[-1], 'head')

    def test_02d_vlm_positive_accepts_a_fully_closed_gripper(self):
        self.fake.events.clear()
        self.fake.verification_results = [True]
        self.fake.policy_gripper_positions = [0.0]
        wrapped = self._run(self._goal(dry_run=False))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.NONE)
        self.assertEqual(self.fake.events.count('policy'), 1)
        self.assertEqual(self.fake.events.count('verify'), 1)

    def test_03_outer_cancel_propagates_to_policy(self):
        self.fake.events.clear()
        self.fake.stall_policy = True
        self.fake.policy_canceled.clear()
        handle = self._wait(self.client.send_goal_async(self._goal(dry_run=False)), 3.0)
        deadline = time.monotonic() + 3.0
        while 'policy' not in self.fake.events and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertIn('policy', self.fake.events)
        canceled = self._wait(handle.cancel_goal_async(), 2.0)
        self.assertGreater(len(canceled.goals_canceling), 0)
        wrapped = self._wait(handle.get_result_async(), 3.0)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(wrapped.result.error.code, CapabilityError.CANCELED)
        self.assertTrue(self.fake.policy_canceled.wait(timeout=2.0))
        self.fake.stall_policy = False

    def test_05_controller_success_without_measured_pregrasp_never_starts_act(self):
        self.fake.events.clear()
        time.sleep(0.1)
        self.fake.arm_updates_state = False
        wrapped = self._run(self._goal(dry_run=False))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            wrapped.result.error.code, CapabilityError.SAFETY_REJECTED
        )
        self.assertIn('did not settle', wrapped.result.error.message)
        self.assertNotIn('policy', self.fake.events)
        self.fake.arm_updates_state = True

    def test_06_controller_success_without_open_gripper_never_starts_act(self):
        self.fake.events.clear()
        self.fake.positions[-1] = 1.0
        self.fake.gripper_updates_state = False
        time.sleep(0.1)
        wrapped = self._run(self._goal(dry_run=False))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            wrapped.result.error.code, CapabilityError.SAFETY_REJECTED
        )
        self.assertIn('did not settle', wrapped.result.error.message)
        self.assertIn('gripper', self.fake.events)
        self.assertNotIn('policy', self.fake.events)
        self.fake.gripper_updates_state = True
        self.fake.positions[-1] = 1.64

    def _goal(self, *, dry_run):
        goal = GraspObject.Goal()
        goal.object_id = 'camping_lamp'
        goal.backend = 'act'
        goal.dry_run = dry_run
        goal.target.header.frame_id = 'base_link'
        goal.target.header.stamp = self.client_node.get_clock().now().to_msg()
        goal.target.point.x = 0.2
        goal.target.point.y = -0.2
        goal.target.point.z = 0.4
        return goal

    def _run(self, goal):
        handle = self._wait(self.client.send_goal_async(goal), 3.0)
        self.assertTrue(handle.accepted)
        return self._wait(handle.get_result_async(), 10.0)

    @staticmethod
    def _wait(future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise AssertionError('future timed out')
        return future.result()
