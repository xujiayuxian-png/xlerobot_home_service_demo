"""Offline adapter contracts; never import the SDK or open accelerator devices."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

source = next(p / 'tools/lib/npu_bench.py' for p in Path(__file__).resolve().parents
              if (p / 'tools/lib/npu_bench.py').is_file())
spec = importlib.util.spec_from_file_location('npu_bench', source)
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


def test_person_decode_filters_invalid_and_other_classes_and_suppresses_overlap():
    scores = np.zeros((80, 8400), np.float32)
    boxes = np.zeros((4, 8400), np.float32)
    scores[0, :6] = [0.9, 0.8, 0.7, np.nan, 0.9, 0.9]
    scores[1, 6] = 1.0
    boxes[:, :7] = np.array([[10, 10, 20, 20], [11, 10, 20, 20],
                            [90, 50, 40, 40], [10, 10, 20, 20],
                            [np.inf, 10, 20, 20], [10, 10, -1, 20],
                            [50, 50, 20, 20]]).T
    result = bench.person_boxes(scores, boxes, (120, 200, 3), 2)
    assert len(result) == 2
    assert result[0]['bbox_px'] == [0, 0, 40, 40]
    assert result[1]['bbox_px'] == [140, 60, 200, 120]
    assert bench.person_boxes(np.zeros_like(scores), boxes, (120, 200), 2) == []


def test_decoder_uses_float_host_ids_and_unmasks_only_available_positions():
    for position in (0, 3, 199):
        token, index, mask = bench.decoder_controls(50260, position)
        assert token.dtype == index.dtype == mask.dtype == np.float32
        assert token.tolist() == [[50260]] and index.tolist() == [position]
        assert mask.shape == (1, 1, 1, 200)
        assert np.count_nonzero(mask == 0) == position + 1
        assert np.all(mask.flatten()[:199-position] == -100)
    for position in (-1, 200):
        with pytest.raises(ValueError):
            bench.decoder_controls(1, position)


def test_failed_qnn_load_releases_interpreter(monkeypatch):
    destroyed = []
    interpreter = SimpleNamespace(init=lambda: 0, load_model=lambda: 42,
                                  destroy=lambda: destroyed.append(True))
    sdk = SimpleNamespace(
        Model=SimpleNamespace(create_instance=lambda _: object()),
        Config=SimpleNamespace(create_instance=SimpleNamespace),
        FrameworkType=SimpleNamespace(TYPE_QNN236=236),
        AccelerateType=SimpleNamespace(TYPE_DSP=1),
        InterpreterBuilder=SimpleNamespace(
            build_interpreter_from_model_and_config=lambda *_: interpreter))
    monkeypatch.setitem(sys.modules, 'aidlite', sdk)
    with pytest.raises(RuntimeError, match='42'):
        bench.load_qnn('unused-model', '236')
    assert destroyed == [True]


def test_token_probability_is_stable_for_large_logits_and_rejects_invalid_output():
    assert np.exp(bench.token_log_probability([10000, 10000], 0)) == pytest.approx(0.5)
    assert bench.token_log_probability([10000, -10000], 0) == pytest.approx(0)
    with pytest.raises(ValueError):
        bench.token_log_probability([np.nan, 0], 0)


def test_asr_screen_rejects_silence_unfinished_and_missing_evidence():
    sample = dict(finished=True, text='拿杯子', no_speech_probability=0.01,
                  mean_token_log_probability=-0.1)
    assert bench.asr_candidate_passes(sample)
    for overrides in [dict(finished=False), dict(text=' '), dict(no_speech_probability=0.96),
                      dict(no_speech_probability=None), dict(mean_token_log_probability=np.nan),
                      dict(mean_token_log_probability=-2.2)]:
        assert not bench.asr_candidate_passes({**sample, **overrides})
