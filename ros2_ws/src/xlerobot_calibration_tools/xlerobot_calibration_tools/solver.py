"""Offline solvers for the two reference calibration geometries."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .transforms import from_parameters, invert, to_parameters, validate_transform


HEAD_MODEL = 'moving_camera_fixed_target'
ARM_MODEL = 'fixed_camera_moving_target'


@dataclass(frozen=True)
class CalibrationSample:
    """One timestamp-resolved kinematic and visual transform pair."""

    index: int
    moving_in_base: np.ndarray
    target_in_camera: np.ndarray

    def validate(self) -> None:
        validate_transform(self.moving_in_base, f'sample {self.index} moving_in_base')
        validate_transform(
            self.target_in_camera, f'sample {self.index} target_in_camera'
        )


@dataclass(frozen=True)
class CalibrationSolution:
    """Solved X/Y transforms and residual evidence."""

    model: str
    x: np.ndarray
    y: np.ndarray
    method: str
    metrics: dict


def _sample_delta(
    sample: CalibrationSample, model: str, x_value: np.ndarray, y_value: np.ndarray
) -> np.ndarray:
    if model == HEAD_MODEL:
        estimate = sample.moving_in_base @ x_value @ sample.target_in_camera
        return invert(y_value) @ estimate
    if model == ARM_MODEL:
        visual = x_value @ sample.target_in_camera
        kinematic = sample.moving_in_base @ y_value
        return invert(kinematic) @ visual
    raise ValueError(f'unknown calibration model: {model}')


def residual_metrics(
    samples: Sequence[CalibrationSample],
    model: str,
    x_value: np.ndarray,
    y_value: np.ndarray,
) -> dict:
    """Compute translation and rotation closure errors for every sample."""
    translations = []
    rotations = []
    for sample in samples:
        delta = _sample_delta(sample, model, x_value, y_value)
        translations.append(float(np.linalg.norm(delta[:3, 3])))
        rotations.append(
            float(np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec()))
        )
    translation_array = np.asarray(translations)
    rotation_array = np.asarray(rotations)
    return {
        'translation_rmse_mm': float(
            np.sqrt(np.mean(translation_array**2)) * 1000.0
        ),
        'translation_max_mm': float(np.max(translation_array) * 1000.0),
        'rotation_rmse_deg': float(
            np.degrees(np.sqrt(np.mean(rotation_array**2)))
        ),
        'rotation_max_deg': float(np.degrees(np.max(rotation_array))),
        'per_sample_translation_m': translations,
        'per_sample_rotation_rad': rotations,
    }


def _score(
    samples: Sequence[CalibrationSample], model: str, x_value: np.ndarray, y_value: np.ndarray
) -> float:
    metrics = residual_metrics(samples, model, x_value, y_value)
    return metrics['translation_rmse_mm'] + metrics['rotation_rmse_deg']


def _head_seeds(samples: Sequence[CalibrationSample]):
    moving_to_base = [invert(sample.moving_in_base) for sample in samples]
    moving_rotations = [value[:3, :3] for value in moving_to_base]
    moving_translations = [value[:3, 3].reshape(3, 1) for value in moving_to_base]
    target_rotations = [sample.target_in_camera[:3, :3] for sample in samples]
    target_translations = [
        sample.target_in_camera[:3, 3].reshape(3, 1) for sample in samples
    ]
    methods = [
        ('tsai', cv2.CALIB_HAND_EYE_TSAI),
        ('park', cv2.CALIB_HAND_EYE_PARK),
        ('horaud', cv2.CALIB_HAND_EYE_HORAUD),
        ('daniilidis', cv2.CALIB_HAND_EYE_DANIILIDIS),
    ]
    for method_name, method in methods:
        try:
            rotation, translation = cv2.calibrateHandEye(
                moving_rotations,
                moving_translations,
                target_rotations,
                target_translations,
                method=method,
            )
        except cv2.error:
            continue
        raw = np.eye(4)
        raw[:3, :3] = rotation
        raw[:3, 3] = translation.reshape(3)
        try:
            inverse = invert(raw)
        except ValueError:
            continue
        for direction, x_value in [('raw', raw), ('inverse', inverse)]:
            estimates = [
                sample.moving_in_base @ x_value @ sample.target_in_camera
                for sample in samples
            ]
            y_value = _average_transforms(estimates)
            yield x_value, y_value, f'{method_name}:{direction}'


def _arm_seeds(samples: Sequence[CalibrationSample]):
    world_to_camera_rotations = [
        sample.target_in_camera[:3, :3] for sample in samples
    ]
    world_to_camera_translations = [
        sample.target_in_camera[:3, 3].reshape(3, 1) for sample in samples
    ]
    base_to_gripper_rotations = [sample.moving_in_base[:3, :3] for sample in samples]
    base_to_gripper_translations = [
        sample.moving_in_base[:3, 3].reshape(3, 1) for sample in samples
    ]
    for method_name, method in [
        ('shah', cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH),
        ('li', cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI),
    ]:
        try:
            first_rotation, first_translation, second_rotation, second_translation = (
                cv2.calibrateRobotWorldHandEye(
                    world_to_camera_rotations,
                    world_to_camera_translations,
                    base_to_gripper_rotations,
                    base_to_gripper_translations,
                    method=method,
                )
            )
        except cv2.error:
            continue
        first = np.eye(4)
        first[:3, :3] = first_rotation
        first[:3, 3] = first_translation.reshape(3)
        second = np.eye(4)
        second[:3, :3] = second_rotation
        second[:3, 3] = second_translation.reshape(3)
        try:
            candidates = [
                (invert(second), invert(first)),
                (second, first),
                (invert(first), invert(second)),
                (first, second),
            ]
        except ValueError:
            continue
        for index, (x_value, y_value) in enumerate(candidates):
            yield x_value, y_value, f'{method_name}:candidate_{index}'


def _average_transforms(transforms: Sequence[np.ndarray]) -> np.ndarray:
    translations = np.asarray([transform[:3, 3] for transform in transforms])
    rotations = Rotation.from_matrix([transform[:3, :3] for transform in transforms])
    result = np.eye(4)
    result[:3, :3] = rotations.mean().as_matrix()
    result[:3, 3] = np.mean(translations, axis=0)
    return result


def _refine(
    samples: Sequence[CalibrationSample],
    model: str,
    x_initial: np.ndarray,
    y_initial: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    def residuals(parameters: np.ndarray) -> np.ndarray:
        x_value = from_parameters(parameters[:6])
        y_value = from_parameters(parameters[6:])
        values = []
        for sample in samples:
            delta = _sample_delta(sample, model, x_value, y_value)
            values.extend(delta[:3, 3])
            values.extend(Rotation.from_matrix(delta[:3, :3]).as_rotvec())
        return np.asarray(values)

    initial = np.concatenate([to_parameters(x_initial), to_parameters(y_initial)])
    result = least_squares(residuals, initial, method='lm', max_nfev=8000)
    if not result.success or not np.all(np.isfinite(result.x)):
        raise RuntimeError(f'calibration refinement failed: {result.message}')
    return from_parameters(result.x[:6]), from_parameters(result.x[6:])


def solve(samples: Sequence[CalibrationSample], model: str) -> CalibrationSolution:
    """Solve and refine one validated calibration dataset."""
    if model not in {HEAD_MODEL, ARM_MODEL}:
        raise ValueError(f'unknown calibration model: {model}')
    if len(samples) < 4:
        raise ValueError('at least four calibration samples are required')
    for sample in samples:
        sample.validate()
    seeds = _head_seeds(samples) if model == HEAD_MODEL else _arm_seeds(samples)
    candidates = []
    for x_value, y_value, method in seeds:
        if not np.all(np.isfinite(x_value)) or not np.all(np.isfinite(y_value)):
            continue
        try:
            score = _score(samples, model, x_value, y_value)
        except ValueError:
            continue
        candidates.append((score, x_value, y_value, method))
    if not candidates:
        raise RuntimeError('no finite calibration seed was produced')
    _, x_initial, y_initial, method = min(candidates, key=lambda item: item[0])
    x_value, y_value = _refine(samples, model, x_initial, y_initial)
    return CalibrationSolution(
        model=model,
        x=x_value,
        y=y_value,
        method=method,
        metrics=residual_metrics(samples, model, x_value, y_value),
    )
