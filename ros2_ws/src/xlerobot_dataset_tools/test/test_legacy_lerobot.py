import json
from pathlib import Path

import numpy as np
import pytest

from xlerobot_dataset_tools.legacy_lerobot import (
    JOINT_NAMES,
    load_episode,
    supervised_arrays,
)


def arrays(frames=4):
    states = np.arange(frames * 6, dtype=np.float64).reshape(frames, 6) / 10.0
    timestamps = np.arange(frames, dtype=np.float64) / 30.0
    return states, timestamps


def test_recorded_mode_preserves_even_stationary_follower_targets():
    states, timestamps = arrays()
    observations, actions, _, provenance = supervised_arrays(
        states, states.copy(), timestamps, action_mode='recorded'
    )
    np.testing.assert_allclose(observations, states)
    np.testing.assert_allclose(actions, states)
    assert provenance == 'recorded_action'


def test_legacy_mode_is_explicit_and_drops_only_last_state():
    states, timestamps = arrays()
    observations, actions, selected_timestamps, provenance = supervised_arrays(
        states,
        states.copy(),
        timestamps,
        action_mode='legacy_next_state',
    )
    np.testing.assert_allclose(observations, states[:-1])
    np.testing.assert_allclose(actions, states[1:])
    np.testing.assert_allclose(selected_timestamps, timestamps[:-1])
    assert provenance == 'follower_next_state_reconstructed'
    assert observations.dtype == np.float32


def test_recorded_mode_preserves_true_action_and_all_frames():
    states, timestamps = arrays()
    actions = states + 0.1
    observations, selected_actions, selected_timestamps, provenance = (
        supervised_arrays(
            states,
            actions,
            timestamps,
            action_mode='recorded',
        )
    )
    np.testing.assert_allclose(observations, states)
    np.testing.assert_allclose(selected_actions, actions)
    np.testing.assert_allclose(selected_timestamps, timestamps)
    assert provenance == 'recorded_action'


def test_episode_contract_requires_joint_order_and_videos(tmp_path: Path):
    states, timestamps = arrays()
    episode = tmp_path / 'ep_001'
    (episode / 'videos').mkdir(parents=True)
    (episode / 'meta.json').write_text(
        json.dumps(
            {
                'language_instruction': 'pick the object',
                'fps': 30.0,
                'joint_names': JOINT_NAMES,
            }
        ),
        encoding='utf-8',
    )
    np.savez(
        episode / 'data.npz',
        observation_state=states,
        action=states,
        timestamp=timestamps,
        frame_index=np.arange(len(states)),
    )
    with pytest.raises(ValueError, match='MP4'):
        load_episode(episode, action_mode='legacy_next_state')
