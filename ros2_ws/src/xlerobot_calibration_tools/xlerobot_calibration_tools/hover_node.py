"""Local hover validation coordinator; one existing ros2_control action owner."""
from __future__ import annotations

import asyncio
import hashlib
import json
from pathlib import Path
import threading
import time
import uuid

from aiohttp import web
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation
from xlerobot_interfaces.action import MoveCalibrationPose

from .bundle import _atomic_yaml
from .hover import ArmGeometry, HoverDetector, JOINTS, measurement, same_board_pose
from .hover_results import TARGETS, summarize_arrival, suite_result
from .quality import validate_component
from .workflow import load_yaml
from .workbench import errors


class HoverNode(Node):
    def __init__(self):
        super().__init__('hover_validation')
        for name, default in [('execution_enabled', False), ('state_root', '.xlerobot'),
                              ('unit_id', 'reference-two-wheel'), ('port', 8082)]:
            self.declare_parameter(name, default)
        self.root = Path(self.get_parameter('state_root').value) / 'units' / self.get_parameter('unit_id').value
        self.inputs = [self.root / 'draft/components' / f'{n}.yaml'
                       for n in ('servo', 'head_camera', 'right_handeye')]
        self.input_bytes = [p.read_bytes() for p in self.inputs]
        cal = load_yaml(self.inputs[-1])
        validate_component('right_handeye', cal)
        self.x, self.y = np.array(cal['x']['matrix']), np.array(cal['y']['matrix'])
        self.provenance = {p.stem: hashlib.sha256(b).hexdigest() for p, b in zip(self.inputs, self.input_bytes)}
        self.data_lock = threading.Lock()
        self.latest = {}
        self.geometry = None
        self.detector, self.bridge = HoverDetector(), CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.plan = None
        self.result = None
        self.phase, self.message = 'IDLE', '先准备观测姿态，再预览高位目标；预览不移动机器人'
        self.motion_handle = None
        self.motion_unconfirmed = False
        self.operation_lock = asyncio.Lock()
        self.auto_task = None
        self.cancel_requested = False
        self.auto_progress = None
        self.suite = None
        self.create_subscription(Image, '/xlerobot/d455/color/image_raw',
            lambda m: self.receive('image', m), QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(CameraInfo, '/xlerobot/d455/color/camera_info',
            lambda m: self.receive('info', m), qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states',
            lambda m: self.receive('joints', m), qos_profile_sensor_data)
        self.create_subscription(String, '/robot_description', self.description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = ActionClient(self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory')
        self.prepare_client = ActionClient(self, MoveCalibrationPose, '/calibration/move_pose')

    def check_cancel(self):
        if self.cancel_requested:
            raise ValueError('已停止自动验证；已完成的数据保留，不自动回位')

    async def prepare_observation(self, *, return_ready=False):
        self.check_cancel()
        if not self.get_parameter('execution_enabled').value:
            raise ValueError('hardware execution is disabled')
        self.check_inputs()
        if not self.prepare_client.server_is_ready():
            raise ValueError('观测姿态服务未就绪，请稍候')
        self.plan = self.result = None
        self.phase, self.message = 'PREPARING', '头部低头、右臂移动到已验证的手眼观测姿态'
        self.motion_unconfirmed = True
        pending = self.prepare_client.send_goal_async(MoveCalibrationPose.Goal(
            workflow_id='right_handeye', pose_index=3, dry_run=False, return_ready=return_ready))
        try:
            self.motion_handle = await self.wait(pending, 5)
        except BaseException:
            def late(future):
                handle = future.result()
                if handle and handle.accepted:
                    handle.cancel_goal_async()
            pending.add_done_callback(late)
            raise
        if not self.motion_handle.accepted:
            self.motion_unconfirmed = False
            raise ValueError('观测姿态请求被拒绝')
        result_future = self.motion_handle.get_result_async()
        if self.cancel_requested:
            await self.wait(self.motion_handle.cancel_goal_async(), 5)
        try:
            result = await self.wait(result_future, 25)
        except BaseException:
            await self.wait(self.motion_handle.cancel_goal_async(), 5)
            await self.wait(result_future, 8)
            self.motion_unconfirmed = False
            raise
        self.motion_unconfirmed = False
        if result.status != 4 or result.result.error.code != 0:
            raise ValueError('观测准备未完成：' + result.result.error.message)
        self.check_cancel()
        self.phase, self.message = 'IDLE', '观测姿态已到位，请确认桌面板和 Tag 23 同时可见，再预览悬停点'

    def receive(self, name, message):
        with self.data_lock:
            self.latest[name] = (message, time.monotonic())

    def description(self, message):
        try:
            self.geometry = ArmGeometry(message.data)
        except (ValueError, KeyError) as error:
            self.message = str(error)

    def check_inputs(self):
        if [p.read_bytes() for p in self.inputs] != self.input_bytes:
            raise ValueError('标定草稿已变化，请结束并重新启动悬停会话')

    def joints(self, entry=None):
        if entry is None:
            with self.data_lock:
                entry = self.latest.get('joints')
        if entry is None or time.monotonic() - entry[1] > .5:
            raise ValueError('关节反馈过旧，请检查控制器')
        msg = entry[0]
        age = self.get_clock().now().nanoseconds / 1e9 - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        values = dict(zip(msg.name, msg.position))
        if not 0 <= age <= .5 or not all(n in values and np.isfinite(values[n]) for n in JOINTS):
            raise ValueError('关节时间戳 / 数据无效')
        if any(abs(values.get(n, 999) - expected) > .025 for n, expected in
               [('head_pan_joint', 0.), ('head_tilt_joint', .796136)]):
            raise ValueError('头部不在手眼标定固定姿态；请先点击准备观测姿态，不能直接沿用固定相机变换')
        return values

    def observe(self):
        # A button press can land between camera frames or just after motion.
        # Wait for current evidence, as the calibration collector does, instead
        # of turning a single stale/blurred frame into a failed operation.
        # Keep the original timestamp, age and image/joint alignment limits.
        deadline = time.monotonic() + 1.0
        while True:
            try:
                return self.observe_once()
            except ValueError as error:
                if time.monotonic() >= deadline:
                    raise ValueError(f'{error}（等待新观测 1 秒后仍未就绪）') from error
                time.sleep(.04)

    def observe_once(self):
        self.check_inputs()
        if self.geometry is None:
            raise ValueError('等待机器人模型')
        with self.data_lock:
            latest = dict(self.latest)
        if not all(n in latest for n in ('image', 'info', 'joints')):
            raise ValueError('等待相机图像、内参和关节反馈')
        msg, received = latest['image']
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if time.monotonic() - received > .5 or not 0 <= self.get_clock().now().nanoseconds / 1e9 - stamp <= .5:
            age = self.get_clock().now().nanoseconds / 1e9 - stamp
            raise ValueError(f'相机观测过旧或时间戳无效：图像年龄 {age:.3f}s，接收间隔 {time.monotonic() - received:.3f}s')
        joints = self.joints(latest['joints'])
        joint_msg = latest['joints'][0]
        joint_stamp = joint_msg.header.stamp.sec + joint_msg.header.stamp.nanosec / 1e9
        if abs(joint_stamp - stamp) > .25:
            raise ValueError('图像与关节反馈不同步，请稍后重试')
        info = latest['info'][0]
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        board, tag = self.detector.detect(image, np.array(info.k).reshape(3, 3), np.array(info.d))
        try:
            head_tf = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
        except TransformException as error:
            raise ValueError('等待图像时刻的头部标定 TF') from error
        head_camera = np.eye(4)
        head_camera[:3, :3] = Rotation.from_quat([
            head_tf.rotation.x, head_tf.rotation.y, head_tf.rotation.z, head_tf.rotation.w]).as_matrix()
        head_camera[:3, 3] = [head_tf.translation.x, head_tf.translation.y, head_tf.translation.z]
        if self.get_clock().now().nanoseconds / 1e9 - stamp > .5:
            raise ValueError('图像处理后观测已过旧，请等下一帧')
        return {'stamp': stamp, 'camera_from_board': board.target_in_camera.tolist(),
                'camera_from_tag': tag.target_in_camera.tolist(), 'joints': joints,
                'scene_base_from_board': (head_camera @ board.target_in_camera).tolist(),
                'board_ids': list(board.tag_ids), 'board_reprojection_px': board.reprojection_rmse_px,
                'tag_reprojection_px': tag.reprojection_rmse_px}

    async def wait(self, future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done():
            if time.monotonic() >= deadline:
                raise ValueError('控制器响应超时；未确认停止前不可再次执行')
            await asyncio.sleep(.02)
        return future.result()

    async def execute(self, plan_id):
        self.check_cancel()
        if not self.get_parameter('execution_enabled').value:
            raise ValueError('hardware execution is disabled')
        if not self.plan or self.plan['id'] != plan_id or time.monotonic() - self.plan_time > 60:
            raise ValueError('预览已过期，请重新预览')
        row = await asyncio.to_thread(self.observe)
        if np.max(np.abs(np.array([row['joints'][n] for n in JOINTS]) - self.plan['start'])) > .04:
            raise ValueError('机械臂姿态变化，请重新预览')
        current_board = self.x @ np.array(row['camera_from_board'])
        if not same_board_pose(current_board, self.plan['base_from_board']):
            raise ValueError('桌面板或机器人位置变化，请重新预览')
        if not same_board_pose(row['scene_base_from_board'], self.plan['scene_base_from_board']):
            raise ValueError('头部场景定位变化，请重新预览')
        if not self.client.server_is_ready():
            raise ValueError('本地机械臂控制器未就绪')
        self.check_cancel()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        start, finish = np.array(self.plan['start']), np.array(self.plan['goal'])
        for i in range(31):
            u = i / 30
            point = JointTrajectoryPoint()
            point.positions = (start + (3 * u * u - 2 * u**3) * (finish - start)).tolist()
            point.velocities = ((6 * u - 6 * u * u) / 12 * (finish - start)).tolist()
            ticks = round(u * 12 * 1e9)
            point.time_from_start = Duration(sec=ticks // 10**9, nanosec=ticks % 10**9)
            goal.trajectory.points.append(point)
        self.phase, self.message = 'MOVING', '移动到预览目标；不移动头部、轮子或夹爪'
        self.motion_unconfirmed = True
        pending = self.client.send_goal_async(goal)
        try:
            self.motion_handle = await self.wait(pending, 5)
        except Exception:
            # A late accepted goal must be canceled, never abandoned.
            def late(future):
                handle = future.result()
                if handle and handle.accepted:
                    handle.cancel_goal_async()
            pending.add_done_callback(late)
            raise
        if not self.motion_handle.accepted:
            self.motion_unconfirmed = False
            raise ValueError('控制器拒绝目标')
        result_future = self.motion_handle.get_result_async()
        deadline = time.monotonic() + 20
        try:
            while not result_future.done():
                self.check_cancel()
                self.joints()
                if time.monotonic() > deadline:
                    raise ValueError('运动超时')
                await asyncio.sleep(.05)
            result = result_future.result()
            self.motion_unconfirmed = False
            if result.status != 4 or result.result.error_code != 0:
                raise ValueError('移动已取消或失败；请检查机械臂后重新预览')
            self.plan['hardware_executed'] = True
            self.phase, self.message = 'ARRIVED', '控制器动作结束；点击测量验证真实到位误差'
        except BaseException:
            await self.wait(self.motion_handle.cancel_goal_async(), 5)
            await self.wait(result_future, 5)
            self.motion_unconfirmed = False
            raise

    async def measure(self):
        if not self.plan or not self.plan['hardware_executed']:
            raise ValueError('先完成当前预览目标的移动')
        rows, seen = [], set()
        deadline = time.monotonic() + 15
        last_error = ''
        while len(rows) < 20 and time.monotonic() < deadline:
            self.check_cancel()
            try:
                row = await asyncio.to_thread(self.observe)
                if row['stamp'] not in seen:
                    if rows and max(abs(row['joints'][n] - rows[0]['joints'][n]) for n in JOINTS) > .01:
                        raise ValueError('机械臂仍在运动，请到位静止后测量')
                    if not same_board_pose(self.x @ np.array(row['camera_from_board']),
                                           self.plan['base_from_board']):
                        raise ValueError('桌面板已移动，请重新预览')
                    if not same_board_pose(row['scene_base_from_board'], self.plan['scene_base_from_board']):
                        raise ValueError('头部场景定位变化，请重新预览')
                    rows.append(row)
                    seen.add(row['stamp'])
            except ValueError as error:
                last_error = str(error)
            await asyncio.sleep(.1)
        if len(rows) != 20:
            raise ValueError(f'有效静止观测 {len(rows)}/20；{last_error}')
        metrics = [measurement(r, self.plan, self.geometry, self.x, self.y) for r in rows]
        summary = {n: float(np.mean([m[n] for m in metrics])) for n in (
            'planar_error_mm', 'height_shortfall_mm', 'height_mm', 'error_3d_mm', 'visual_fk_closure_mm')}
        summary['joint_tracking_error_deg'] = np.mean([m['joint_tracking_error_deg'] for m in metrics], axis=0).tolist()
        self.result = {'schema': 'xlerobot_hover_validation/v1', 'id': uuid.uuid4().hex,
                       'predecessors': self.provenance, 'plan': self.plan, 'summary': summary,
                       'observations': rows, 'frames': 20, 'independent_arrivals': 1,
                       'stationary_board_tolerance': {'translation_mm': 5., 'rotation_deg': 1.},
                       'claim': 'visual metrology, not independent ground truth or repeatability'}
        path = self.root / 'capture/calibration_work/hover' / f'{self.result["id"]}.yaml'
        _atomic_yaml(path, self.result)
        _atomic_yaml(self.root / 'workbench/hover.yaml', {
            'predecessors': self.provenance, 'report_path': str(path), 'summary': summary,
            'quality_passed': False, 'acceptance': 'measurement_only_no_accuracy_claim'})
        self.phase, self.message = 'MEASURED', '结果已保存；20 帧是一处悬停观测，不是 20 次重复到位'

    async def preview(self, target):
        self.check_cancel()
        self.plan = self.result = None
        row = await asyncio.to_thread(self.observe)
        plan = await asyncio.to_thread(self.geometry.plan,
            [row['joints'][n] for n in JOINTS], np.array(row['camera_from_board']),
            self.x, self.y, target, scene_base_from_board=row['scene_base_from_board'])
        self.plan = dict(plan, id=uuid.uuid4().hex)
        self.plan_time = time.monotonic()
        self.phase, self.message = 'PREVIEW', '预览完成，检查空间后再确认移动'

    def save_suite(self, run_id, arrivals, status, message):
        self.suite = suite_result(run_id, arrivals, status, message=message)
        session = self.root / 'workbench/session.yaml'
        if session.is_file():
            self.suite['device_generation'] = load_yaml(session).get('generation')
        _atomic_yaml(self.root / 'capture/calibration_work/hover/runs' / f'{run_id}.yaml', self.suite)
        _atomic_yaml(self.root / 'workbench/hover_latest.yaml', self.suite)

    async def automatic(self):
        run_id, arrivals = uuid.uuid4().hex, []
        async with self.operation_lock:
            try:
                self.save_suite(run_id, arrivals, 'RUNNING', '正在准备观测姿态')
                self.auto_progress = {'completed': 0, 'total': 3, 'target': 'prepare'}
                await self.prepare_observation()
                for target in TARGETS:
                    self.check_cancel()
                    self.auto_progress['target'] = target
                    await self.preview(target)
                    await self.execute(self.plan['id'])
                    await asyncio.sleep(.5)
                    await self.measure()
                    arrivals.append(summarize_arrival(self.result))
                    self.auto_progress['completed'] = len(arrivals)
                    self.save_suite(run_id, arrivals, 'RUNNING', f'已完成 {len(arrivals)}/3 个点')
                self.auto_progress['target'] = 'return_ready'
                # Retrace the verified preparation corridor before parking.
                await self.prepare_observation()
                await self.prepare_observation(return_ready=True)
                self.phase, self.message = 'COMPLETED', '三点验证完成，已回 ready；结果与建议补偿已保存，尚未应用补偿'
                self.save_suite(run_id, arrivals, 'COMPLETED', self.message)
            except (Exception, asyncio.CancelledError) as error:
                self.phase, self.message = 'ERROR', str(error) or '自动验证中断，已完成数据保留'
                self.save_suite(run_id, arrivals, 'INTERRUPTED', self.message)

    def app(self):
        app = web.Application(middlewares=[errors])

        async def status(request):
            return web.json_response({'phase': self.phase, 'message': self.message,
                'busy': self.operation_lock.locked() or bool(self.auto_task and not self.auto_task.done()), 'motion_unconfirmed': self.motion_unconfirmed,
                'ready': self.prepare_client.server_is_ready() and self.client.server_is_ready() and self.geometry is not None,
                'auto_progress': self.auto_progress, 'suite': self.suite,
                'plan': self.plan, 'result': self.result, 'provenance': self.provenance})

        async def command(request):
            body = await request.json()
            if not isinstance(body, dict):
                raise ValueError('JSON object required')
            operation = request.match_info['operation']
            if operation == 'cancel':
                self.cancel_requested = True
                if self.motion_unconfirmed and self.motion_handle and self.motion_handle.accepted:
                    await self.wait(self.motion_handle.cancel_goal_async(), 5)
                return web.json_response({'message': '已请求停止；请等待控制器确认'})
            if self.operation_lock.locked() or self.motion_unconfirmed or (self.auto_task and not self.auto_task.done()):
                raise ValueError('另一操作尚未结束或运动状态未确认')
            self.cancel_requested = False
            if operation == 'auto':
                if body.get('confirmed') is not True or not self.get_parameter('execution_enabled').value:
                    raise ValueError('自动验证需要硬件启用和明确确认')
                self.check_inputs()
                self.auto_task = asyncio.create_task(self.automatic())
                return web.json_response({'accepted': True}, status=202)
            async with self.operation_lock:
                try:
                    if operation == 'prepare':
                        if body.get('confirmed') is not True:
                            raise ValueError('请确认头部和右臂路径无障碍')
                        await self.prepare_observation()
                    elif operation == 'preview':
                        await self.preview(body.get('target'))
                    elif operation == 'execute':
                        if body.get('confirmed') is not True:
                            raise ValueError('请确认机械臂路径及桌面周围无障碍')
                        await self.execute(body.get('plan_id'))
                    elif operation == 'measure':
                        await self.measure()
                    else:
                        raise ValueError('unknown hover command')
                except Exception as error:
                    self.phase, self.message = 'ERROR', str(error)
                    raise
            return await status(request)

        async def cleanup(app):
            self.cancel_requested = True
            if self.motion_unconfirmed and self.motion_handle and self.motion_handle.accepted:
                self.motion_handle.cancel_goal_async()
            if self.auto_task and not self.auto_task.done():
                try:
                    await asyncio.wait_for(asyncio.shield(self.auto_task), 8)
                except asyncio.TimeoutError:
                    self.auto_task.cancel()
                    await self.auto_task
        app.on_cleanup.append(cleanup)
        app.router.add_get('/api/v1/hover/status', status)
        app.router.add_post('/api/v1/hover/{operation}', command)
        return app


def main():
    rclpy.init()
    node = HoverNode()
    # These ROS callbacks only receive data / resolve action futures. Detection,
    # planning and HTTP run outside this executor. A thread pool contends on the
    # default callback group and starves joint feedback under camera traffic.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        web.run_app(node.app(), host='127.0.0.1', port=node.get_parameter('port').value)
    finally:
        if node.motion_handle and node.motion_handle.accepted and node.motion_unconfirmed:
            node.motion_handle.cancel_goal_async()
            time.sleep(.2)
        executor.shutdown(timeout_sec=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
