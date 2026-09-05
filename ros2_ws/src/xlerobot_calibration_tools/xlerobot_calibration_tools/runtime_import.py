"""Preserve an existing unit's runtime without inventing solver evidence."""

import math
from pathlib import Path

import numpy as np

from .quality import validate_grasp_alignment_values, validate_servo
from .workflow import load_yaml


FILES = ('geometry.yaml', 'servos.yaml', 'controllers.yaml',
         'transforms.yaml', 'grasp_alignment.yaml')
SCHEMA = 'xlerobot_imported_runtime_bundle/v1'


def validate_documents(documents: dict, repo: Path) -> None:
    if set(documents) != set(FILES):
        raise ValueError('existing runtime must contain all five configuration files')
    geometry = documents['geometry.yaml']
    reference = load_yaml(repo / 'ros2_ws/src/xlerobot_description/config/'
                          'two_wheel_reference_geometry.yaml')

    def check_shape(value, expected):
        if isinstance(expected, dict):
            if not isinstance(value, dict) or not set(expected) <= set(value):
                raise ValueError('imported geometry is incomplete')
            for key in expected:
                check_shape(value[key], expected[key])
        elif isinstance(expected, list):
            if not isinstance(value, list) or len(value) != len(expected):
                raise ValueError('imported geometry vector has wrong size')
            for item, template in zip(value, expected):
                check_shape(item, template)
        elif isinstance(expected, str):
            values = np.asarray(str(value).split(), dtype=float)
            if len(values) != len(expected.split()) or not np.isfinite(values).all():
                raise ValueError('imported geometry pose must be finite')
        elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('imported geometry must be finite')

    check_shape(geometry, reference)
    validate_servo({'schema': 'xlerobot_servo_calibration/v1', **documents['servos.yaml']})
    controller = documents['controllers.yaml']['base_controller']['ros__parameters']
    for key in ('wheel_radius', 'wheel_separation'):
        value = geometry['base'][key]
        if value <= 0 or not math.isclose(value, controller[key], abs_tol=1e-10):
            raise ValueError(f'imported controller and geometry disagree on {key}')
    transforms = documents['transforms.yaml']
    if transforms.get('schema') != 'xlerobot_runtime_transforms/v1':
        raise ValueError('imported transforms schema is invalid')
    if not transforms.get('provenance'):
        raise ValueError('imported runtime requires recorded source provenance')
    alignment = documents['grasp_alignment.yaml']
    validate_grasp_alignment_values(alignment)
    if alignment.get('validation') != 'existing_unit_runtime' or not alignment.get('provenance'):
        raise ValueError('imported alignment must identify the existing unit runtime')
