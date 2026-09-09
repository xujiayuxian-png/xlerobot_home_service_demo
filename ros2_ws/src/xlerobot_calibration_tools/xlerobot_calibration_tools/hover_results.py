"""Compact, persistent hover results and advisory-only signed target offsets."""
from datetime import datetime, timezone

import numpy as np

from .hover import same_board_pose
from .transforms import validate_transform

TARGETS = ('center', 'right', 'left')


def summarize_arrival(report):
    if report.get('schema') != 'xlerobot_hover_validation/v1':
        raise ValueError('expected a hover measurement report')
    plan, observations = report['plan'], report['observations']
    if not plan['hardware_executed'] or len(observations) != 20:
        raise ValueError('a completed arrival with 20 observations is required')
    if len({r['stamp'] for r in observations}) != 20:
        raise ValueError('duplicate observation timestamps')
    target = np.asarray(plan['target_board_xyz'], dtype=float)
    if target.shape != (3,) or not np.isfinite(target).all() or plan['target'] not in TARGETS:
        raise ValueError('invalid bounded hover target')
    xyz = np.array([(np.linalg.inv(validate_transform(r['camera_from_board'])) @
                     validate_transform(r['camera_from_tag']))[:3, 3] for r in observations])
    error = (xyz - target) * 1000
    joints = np.asarray(report['summary']['joint_tracking_error_deg'], dtype=float)
    if joints.shape != (5,) or not np.isfinite(joints).all():
        raise ValueError('invalid joint tracking evidence')
    return {
        'target': plan['target'], 'report_id': report['id'], 'frames': 20,
        'stamp': max(r['stamp'] for r in observations),
        'predecessors': report['predecessors'],
        'target_xyz_mm': (target * 1000).tolist(), 'observed_xyz_mm': (xyz.mean(axis=0) * 1000).tolist(),
        'error_xyz_mm': error.mean(axis=0).tolist(),
        'frame_std_xyz_mm': error.std(axis=0).tolist(),
        'planar_error_mm': float(np.linalg.norm(error[:, :2], axis=1).mean()),
        'height_mm': float(-xyz[:, 2].mean() * 1000),
        'height_shortfall_mm': float(error[:, 2].mean()),
        'max_joint_error_deg': float(np.abs(joints).max()),
        'base_from_board': validate_transform(plan['base_from_board']).tolist(),
        'scene_base_from_board': validate_transform(plan['scene_base_from_board']).tolist(),
    }


def suite_result(run_id, arrivals, status, *, source='automatic', message=''):
    """Each arrival has equal weight. 60 frames are never 60 independent trials."""
    result = {'schema': 'xlerobot_hover_suite/v1', 'id': run_id, 'source': source,
              'updated_at': datetime.now(timezone.utc).isoformat(), 'status': status,
              'message': message, 'arrivals': arrivals, 'suggestion': None,
              'independent_arrivals': len(arrivals), 'frames': sum(r['frames'] for r in arrivals)}
    if not arrivals:
        return result
    result['predecessors'] = arrivals[0]['predecessors']
    if len(arrivals) != 3 or {r['target'] for r in arrivals} != set(TARGETS):
        return result
    if any(r['predecessors'] != result['predecessors'] for r in arrivals):
        result['message'] = '测量混用了不同标定版本，不生成补偿建议'
        return result
    if any(not same_board_pose(r['scene_base_from_board'], arrivals[0]['scene_base_from_board']) for r in arrivals):
        result['message'] = '三点之间桌面姿态变化超出范围，不生成统一补偿建议'
        return result
    errors = np.asarray([r['error_xyz_mm'] for r in arrivals])
    if errors.shape != (3, 3) or not np.isfinite(errors).all():
        raise ValueError('invalid signed errors')
    # Negative observed error is added to a future target, NOT to measured data.
    correction = -errors.mean(axis=0)
    residual = errors + correction
    result['summary'] = {
        'mean_planar_error_mm': float(np.mean([r['planar_error_mm'] for r in arrivals])),
        'mean_height_shortfall_mm': float(errors[:, 2].mean()),
        'height_shortfall_range_mm': [float(errors[:, 2].min()), float(errors[:, 2].max())],
    }
    result['suggestion'] = {
        'schema': 'xlerobot_hover_offset_suggestion/v1', 'status': 'unverified_not_applied',
        'frame': 'calibration_board', 'units': 'm',
        'add_to_target_xyz_m': (correction / 1000).tolist(),
        'raise_target_mm': float(-correction[2]),
        'estimated_remaining_error_xyz_mm': residual.tolist(),
        'between_pose_error_std_mm': errors.std(axis=0).tolist(),
        'predecessors': result['predecessors'],
        'scope': 'same Tag mounting, head pose, load and three nearby 200 mm hover targets only',
        'advice': '先按建议修正目标并重新验证；改变目标后必须重新规划，不能复用旧轨迹。'
                  '这是总偏移初值，不能拆成 vision_fk_compensation_m 和 gravity_sag_z_m，'
                  '也不能直接用于 ACT 或覆盖手眼矩阵。',
    }
    return result
