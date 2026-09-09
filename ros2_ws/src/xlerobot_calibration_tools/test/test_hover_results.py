import numpy as np
import pytest

from xlerobot_calibration_tools.hover_results import summarize_arrival, suite_result


def arrival(target='center', error=(3., -4., 40.)):
    xyz = {'center': [0, 0, -.2], 'right': [.03, 0, -.2], 'left': [-.03, 0, -.2]}[target]
    tag = np.eye(4)
    tag[:3, 3] = np.array(xyz) + np.array(error) / 1000
    report = {'schema': 'xlerobot_hover_validation/v1', 'id': target,
              'predecessors': {'servo': 'a', 'head_camera': 'b', 'right_handeye': 'c'},
              'plan': {'hardware_executed': True, 'target': target, 'target_board_xyz': xyz,
                       'base_from_board': np.eye(4).tolist(), 'scene_base_from_board': np.eye(4).tolist()},
              'summary': {'joint_tracking_error_deg': [0.] * 5},
              'observations': [{'stamp': i, 'camera_from_board': np.eye(4).tolist(),
                                'camera_from_tag': tag.tolist()} for i in range(20)]}
    return report


def test_signed_offset_is_negative_observed_error_and_three_arrivals_not_sixty():
    rows = [summarize_arrival(arrival(t)) for t in ('center', 'right', 'left')]
    result = suite_result('run', rows, 'COMPLETED')
    assert result['frames'] == 60 and result['independent_arrivals'] == 3
    assert result['suggestion']['add_to_target_xyz_m'] == pytest.approx([-.003, .004, -.04])
    assert result['suggestion']['raise_target_mm'] == pytest.approx(40)
    assert result['suggestion']['status'] == 'unverified_not_applied'
    assert result['suggestion']['frame'] == 'calibration_board'
    assert np.allclose(result['suggestion']['estimated_remaining_error_xyz_mm'], 0)


@pytest.mark.parametrize('invalid', ['missing', 'duplicate_target', 'predecessor', 'scene'])
def test_never_combines_incomplete_mixed_or_moved_runs(invalid):
    rows = [summarize_arrival(arrival(t)) for t in ('center', 'right', 'left')]
    if invalid == 'missing': rows.pop()
    if invalid == 'duplicate_target': rows[2]['target'] = 'center'
    if invalid == 'predecessor': rows[2]['predecessors']['servo'] = 'new'
    if invalid == 'scene': rows[2]['scene_base_from_board'][0][3] = .03
    assert suite_result('run', rows, 'COMPLETED')['suggestion'] is None


def test_repeated_or_nan_observations_are_not_suggestions():
    report = arrival()
    report['observations'][-1]['stamp'] = 0
    with pytest.raises(ValueError): summarize_arrival(report)
    report = arrival()
    report['observations'][0]['camera_from_tag'][0][3] = float('nan')
    with pytest.raises(ValueError): summarize_arrival(report)
