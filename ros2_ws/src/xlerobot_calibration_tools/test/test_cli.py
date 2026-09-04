from pathlib import Path

import numpy as np
import pytest

from xlerobot_calibration_tools.cli import load_samples, SAMPLE_SCHEMA
from xlerobot_calibration_tools.solver import HEAD_MODEL
import yaml


def write_samples(path: Path, model: str) -> None:
    document = {
        'schema': SAMPLE_SCHEMA,
        'model': model,
        'calibration_id': 'test_calibration',
        'samples': [
            {
                'index': 1,
                'moving_in_base': np.eye(4).tolist(),
                'target_in_camera': np.eye(4).tolist(),
            }
        ],
    }
    path.write_text(yaml.safe_dump(document), encoding='utf-8')


def test_loads_only_the_expected_versioned_model(tmp_path):
    path = tmp_path / 'samples.yaml'
    write_samples(path, HEAD_MODEL)
    document, samples = load_samples(path, HEAD_MODEL)
    assert document['calibration_id'] == 'test_calibration'
    assert len(samples) == 1
    assert np.array_equal(samples[0].moving_in_base, np.eye(4))

    with pytest.raises(ValueError, match='expected model'):
        load_samples(path, 'different_model')


def test_rejects_unversioned_samples(tmp_path):
    path = tmp_path / 'samples.yaml'
    path.write_text('samples: []\n', encoding='utf-8')
    with pytest.raises(ValueError, match='expected schema'):
        load_samples(path, HEAD_MODEL)
