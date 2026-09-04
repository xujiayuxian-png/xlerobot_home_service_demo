import pytest

from xlerobot_hmi.fetch_deliver_cli import argument_parser, build_goal


def test_cli_defaults_to_dry_run_and_named_defaults():
    arguments = argument_parser().parse_args(['露营灯'])
    goal = build_goal(arguments)
    assert goal.object_id == '露营灯'
    assert goal.source_place == 'table'
    assert goal.recipient_id == 'nearest_person'
    assert goal.grasp_backend == 'act'
    assert goal.dry_run


def test_execute_requires_explicit_flag():
    arguments = argument_parser().parse_args([
        'lamp', '--source', 'desk', '--recipient', 'lisa',
        '--backend', 'centroid', '--execute'
    ])
    goal = build_goal(arguments)
    assert goal.source_place == 'desk'
    assert goal.recipient_id == 'lisa'
    assert goal.grasp_backend == 'centroid'
    assert not goal.dry_run


@pytest.mark.parametrize('argv', [[' '], ['lamp', '--timeout', '0']])
def test_invalid_request_is_rejected_before_ros(argv):
    with pytest.raises(ValueError):
        build_goal(argument_parser().parse_args(argv))
