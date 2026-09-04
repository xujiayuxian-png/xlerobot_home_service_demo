from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

import xlerobot_commissioning.calibration_node as calibration_module
from xlerobot_commissioning.calibration_node import CalibrationNode
from xlerobot_calibration_tools.sample_set import TransformSampleSet
from xlerobot_calibration_tools.solver import ARM_MODEL, HEAD_MODEL
from xlerobot_interfaces.msg import CapabilityError


EXPECTED_CONTRACTS = {
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


def pose(*, x=0.0, y=0.0, z=0.0):
    value = np.eye(4)
    value[:3, 3] = [x, y, z]
    return value


def write_samples(
    path: Path,
    workflow: str,
    *,
    calibration_id: str = 'unit-001',
) -> dict:
    contract = EXPECTED_CONTRACTS[workflow]
    frames = contract['frames']
    sample_set = TransformSampleSet(
        calibration_id=calibration_id,
        model=contract['model'],
        base_frame=frames['base'],
        moving_frame=frames['moving'],
        camera_frame=frames['camera'],
        target_frame=frames['target'],
    )
    for index in range(contract['minimum']):
        sample_set.append(
            pose(x=0.01 * index),
            pose(x=0.5, z=0.2),
            float(index + 1),
            {'reprojection_rmse_px': 0.4 + 0.01 * index},
        )
    sample_set.write_atomic(path)
    return sample_set.to_dict()


def response():
    return SimpleNamespace(
        error=SimpleNamespace(code=999, message=''),
        draft_uri='',
        quality_passed=False,
        sample_count=0,
        metric_names=[],
        metric_values=[],
    )


class Store:
    def __init__(self, draft: Path):
        self.draft = draft
        self.calls = []

    def save_component(self, unit_id, workflow, document):
        self.calls.append((unit_id, workflow, document))
        return (
            self.draft,
            True,
            {
                'sample_count': float(document['sample_count']),
                **document['metrics'],
            },
        )


def node_for(tmp_path: Path, workflow: str) -> CalibrationNode:
    node = CalibrationNode.__new__(CalibrationNode)
    node.workflow_id = workflow
    node.sample_file = tmp_path / 'samples.yaml'
    node.store = Store(tmp_path / 'draft')
    return node


@pytest.mark.parametrize('workflow', ['head_camera', 'right_handeye'])
def test_solve_uses_strict_sample_authority_for_each_workflow(
    tmp_path, monkeypatch, workflow
):
    node = node_for(tmp_path, workflow)
    write_samples(node.sample_file, workflow)
    contract = EXPECTED_CONTRACTS[workflow]
    assert calibration_module._SAMPLE_CONTRACTS[workflow] == contract
    solved = []

    def fake_solve(samples, model):
        solved.append((samples, model))
        return SimpleNamespace(
            metrics={
                'translation_rmse_mm': 2.0,
                'translation_max_mm': 4.0,
                'rotation_rmse_deg': 0.5,
                'rotation_max_deg': 1.0,
            },
            method='test',
            x=np.eye(4),
            y=np.eye(4),
        )

    monkeypatch.setattr(calibration_module, 'solve', fake_solve)
    result = node.solve_samples(
        SimpleNamespace(unit_id='unit-001'),
        response(),
    )

    assert result.error.code == CapabilityError.NONE
    assert result.sample_count == contract['minimum']
    assert len(solved) == 1
    assert solved[0][1] == contract['model']
    assert len(solved[0][0]) == contract['minimum']
    assert len(node.store.calls) == 1
    unit_id, saved_workflow, document = node.store.calls[0]
    assert (unit_id, saved_workflow) == ('unit-001', workflow)
    assert document['calibration_id'] == 'unit-001'
    assert document['sample_count'] == contract['minimum']
    assert document['metrics']['reprojection_rmse_px'] == pytest.approx(
        sum(0.4 + 0.01 * index for index in range(contract['minimum']))
        / contract['minimum']
    )


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        (
            lambda document: document.update(
                model='fixed_camera_moving_target'
            ),
            'expected model',
        ),
        (
            lambda document: document.update(calibration_id='unit-002'),
            'expected calibration_id',
        ),
        (
            lambda document: document['frames'].update(camera='wrong_camera'),
            'frames do not match',
        ),
        (
            lambda document: document['samples'][0][
                'moving_in_base'
            ][0].__setitem__(3, float('nan')),
            'invalid transform data',
        ),
        (
            lambda document: document['samples'][1].update(index=9),
            'contiguous',
        ),
        (
            lambda document: document.update(extra='not-in-schema'),
            'fields do not match',
        ),
        (
            lambda document: document['samples'][0]['quality'].update(
                reprojection_rmse_px='nan'
            ),
            'finite numeric reprojection',
        ),
    ],
)
def test_solve_rejects_any_non_authoritative_sample_document(
    tmp_path, monkeypatch, mutation, message
):
    node = node_for(tmp_path, 'head_camera')
    document = write_samples(node.sample_file, 'head_camera')
    mutation(document)
    node.sample_file.write_text(
        yaml.safe_dump(document, sort_keys=False),
        encoding='utf-8',
    )

    def should_not_solve(_samples, _model):
        raise AssertionError('solver must not receive an invalid sample set')

    monkeypatch.setattr(calibration_module, 'solve', should_not_solve)
    result = node.solve_samples(
        SimpleNamespace(unit_id='unit-001'),
        response(),
    )

    assert result.error.code == CapabilityError.INVALID_GOAL
    assert message in result.error.message
    assert node.store.calls == []


def test_solve_rejects_invalid_request_unit_before_solver(tmp_path, monkeypatch):
    node = node_for(tmp_path, 'head_camera')
    write_samples(node.sample_file, 'head_camera')

    def should_not_solve(_samples, _model):
        raise AssertionError('solver must not receive an invalid unit request')

    monkeypatch.setattr(calibration_module, 'solve', should_not_solve)
    result = node.solve_samples(
        SimpleNamespace(unit_id='../unit-001'),
        response(),
    )

    assert result.error.code == CapabilityError.INVALID_GOAL
    assert 'invalid unit_id' in result.error.message
    assert node.store.calls == []
