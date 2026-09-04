from pathlib import Path

import yaml


def test_reference_config_preserves_verified_policy_semantics():
    path = Path(__file__).parents[1] / 'config' / 'act_box_wrist_only.yaml'
    config = yaml.safe_load(path.read_text())
    assert config['format'] == 'xlerobot_act_adapter_config/v1'
    assert config['policy_id'] == 'act_xlerobot_box_wrist_only'
    assert config['action']['raw_chunk_mode']
    assert config['action']['freeze_shoulder_pan']
    assert config['action']['control_hz'] == 30.0
    assert config['action']['request_hz'] == 3.0
    assert config['action']['queued_action_steps'] == 60
    assert config['action']['realtime_refill_threshold_steps'] == 15
    assert config['action']['realtime_blend_steps'] == 10
    assert config['start_contract']['producer'] == 'GraspObject'
    assert config['start_contract']['context_required_for_execution']
    assert config['start_contract']['max_context_age_s'] == 5.0
    assert config['start_contract']['position_tolerance_rad'] == 0.05
    assert not config['start_contract']['capture_only_mode_default']
    assert config['gripper_governor']['close_lookahead_steps'] == 100
    assert config['gripper_governor']['close_hold_s'] == 1.0
    assert config['gripper_governor']['lock_closed']
