"""Calibration action, solver, and versioned result boundary."""

from datetime import datetime, timezone
import math
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from xlerobot_calibration_tools.sample_set import TransformSampleSet
from xlerobot_calibration_tools.solver import ARM_MODEL, HEAD_MODEL, solve
from xlerobot_calibration_tools.transforms import to_dict
from xlerobot_interfaces.action import CalibrationJob
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_interfaces.srv import (
    ActivateArtifact,
    ImportCalibrationResult,
    SaveBaseGeometryCalibration,
    SolveCalibrationSamples,
)

from .calibration_assets import UnitCalibrationStore, WORKFLOWS
from .site_assets import _id


_SAMPLE_CONTRACTS = {
    'head_camera': {
        'model': HEAD_MODEL,
        'minimum': 12,
        'frames': {
            'base': 'base_link',
            'moving': 'head_tilt_link',
            'camera': 'head_camera_link',
            'target': 'calibration_target',
        },
    },
    'right_handeye': {
        'model': ARM_MODEL,
        'minimum': 20,
        'frames': {
            'base': 'base_link',
            'moving': 'right_arm_fixed_jaw_link',
            'camera': 'd455_color_optical_frame',
            'target': 'calibration_target',
        },
    },
}


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_workbench')
        root = Path(str(self.declare_parameter(
            'artifact_root', '/var/lib/xlerobot'
        ).value)).expanduser()
        revision = str(self.declare_parameter('software_revision', 'development').value)
        self.workflow_id = str(self.declare_parameter('workflow_id', '').value)
        if self.workflow_id not in WORKFLOWS:
            raise ValueError('workflow_id must select exactly one calibration tool')
        self.store = UnitCalibrationStore(root, revision)
        sample_file = str(self.declare_parameter('sample_file', '').value)
        self.sample_file = Path(sample_file).expanduser() if sample_file else None
        self.create_service(
            ImportCalibrationResult, '/calibration/import_result', self.import_result
        )
        self.create_service(
            ActivateArtifact, '/calibration/activate', self.activate
        )
        if self.workflow_id == 'base_geometry':
            self.create_service(
                SaveBaseGeometryCalibration,
                '/calibration/save_base_geometry',
                self.save_base_geometry,
            )
        if self.workflow_id in {'head_camera', 'right_handeye'}:
            if self.sample_file is None:
                raise ValueError('visual calibration requires sample_file')
            self.create_service(
                SolveCalibrationSamples,
                '/calibration/solve_samples',
                self.solve_samples,
            )
        self.server = ActionServer(
            self, CalibrationJob, '/calibration/job', execute_callback=self.execute,
            goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
        )

    def goal(self, goal):
        if (
            goal.workflow_id != self.workflow_id or not goal.unit_id.strip()
            or goal.automatic or not goal.dry_run
        ):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, handle):
        goal = handle.request
        feedback = CalibrationJob.Feedback()
        feedback.state.phase = 'PREFLIGHT'
        feedback.state.progress = 0.5
        feedback.state.message = 'checking existing draft and workflow boundary'
        feedback.target_sample_count = 0
        handle.publish_feedback(feedback)
        result = CalibrationJob.Result()
        component = self.store.draft(goal.unit_id) / 'components' / f'{goal.workflow_id}.yaml'
        result.artifact_uri = self.store.draft(goal.unit_id).as_uri()
        result.quality_passed = component.exists()
        result.error.code = CapabilityError.NONE
        result.error.message = 'dry-run preflight complete'
        handle.succeed()
        return result

    def import_result(self, request, response):
        try:
            if request.workflow_id != self.workflow_id:
                raise ValueError('result belongs to a different calibration tool')
            draft, quality, metrics = self.store.import_result(
                request.unit_id, request.workflow_id, request.source_uri
            )
            response.error.code = CapabilityError.NONE
            response.error.message = 'calibration component imported into draft'
            response.draft_uri = draft.as_uri()
            response.quality_passed = quality
            response.metric_names = list(metrics)
            response.metric_values = [metrics[name] for name in response.metric_names]
        except Exception as error:
            response.error.code = CapabilityError.INVALID_GOAL
            response.error.message = str(error)
        return response

    def activate(self, request, response):
        try:
            if request.artifact_type != 'calibration':
                raise ValueError('expected artifact_type=calibration')
            if request.version:
                reference = self.store.catalog.activate(
                    'units', request.artifact_id, request.version
                )
            else:
                reference = self.store.finalize_and_activate(request.artifact_id)
            response.error.code = CapabilityError.NONE
            response.error.message = 'unit calibration bundle activated'
            response.artifact_uri = reference.uri
        except Exception as error:
            response.error.code = CapabilityError.INVALID_GOAL
            response.error.message = str(error)
        return response

    def save_base_geometry(self, request, response):
        try:
            draft, quality, metrics = self.store.save_base_geometry_measurements(
                request.unit_id,
                nominal_radius=request.nominal_wheel_radius_m,
                nominal_separation=request.nominal_wheel_separation_m,
                straight_commanded=list(request.straight_commanded_m),
                straight_actual=list(request.straight_actual_m),
                rotation_commanded=list(request.rotation_commanded_rad),
                rotation_actual=list(request.rotation_actual_rad),
            )
            response.error.code = CapabilityError.NONE
            response.error.message = 'base geometry fitted into unit draft'
            response.draft_uri = draft.as_uri()
            response.quality_passed = quality
            response.metric_names = list(metrics)
            response.metric_values = [metrics[name] for name in response.metric_names]
        except Exception as error:
            response.error.code = CapabilityError.INVALID_GOAL
            response.error.message = str(error)
        return response

    def solve_samples(self, request, response):
        try:
            contract = _SAMPLE_CONTRACTS[self.workflow_id]
            model = contract['model']
            unit_id = _id(request.unit_id, 'unit_id')
            sample_set = TransformSampleSet.read(
                self.sample_file,
                expected_model=model,
                expected_calibration_id=unit_id,
                expected_frames=contract['frames'],
            )
            samples = sample_set.samples
            if len(samples) < contract['minimum']:
                raise ValueError(
                    f'{self.workflow_id} requires at least '
                    f'{contract["minimum"]} samples'
                )
            reprojections = []
            for index, quality in enumerate(sample_set.qualities):
                value = quality.get('reprojection_rmse_px')
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                ):
                    raise ValueError(
                        f'sample {index} must contain finite numeric '
                        'reprojection_rmse_px'
                    )
                reprojections.append(float(value))
            solution = solve(samples, model)
            metrics = dict(solution.metrics)
            metrics['reprojection_rmse_px'] = sum(reprojections) / len(reprojections)
            document = {
                'schema': 'xlerobot_transform_calibration/v1',
                'model': model,
                'calibration_id': sample_set.calibration_id,
                'created_at': datetime.now(timezone.utc).isoformat(),
                'sample_count': len(samples),
                'method': solution.method,
                'metrics': metrics,
                'x_semantics': (
                    'mount_from_camera' if model == HEAD_MODEL
                    else 'base_from_camera'
                ),
                'y_semantics': (
                    'base_from_target' if model == HEAD_MODEL
                    else 'gripper_from_target'
                ),
                'x': to_dict(solution.x),
                'y': to_dict(solution.y),
            }
            draft, quality, checked = self.store.save_component(
                unit_id, self.workflow_id, document
            )
            response.error.code = CapabilityError.NONE
            response.error.message = 'samples solved into unit draft'
            response.draft_uri = draft.as_uri()
            response.quality_passed = quality
            response.sample_count = len(samples)
            response.metric_names = list(checked)
            response.metric_values = [checked[name] for name in checked]
        except Exception as error:
            response.error.code = CapabilityError.INVALID_GOAL
            response.error.message = str(error)
        return response


def main():
    rclpy.init()
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
