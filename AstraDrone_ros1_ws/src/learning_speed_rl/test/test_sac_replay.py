#!/usr/bin/env python3

import unittest

import numpy as np

from learning_speed_rl.training.sac_replay import (
    ActionMapping,
    OBSERVATION_DIM,
    SacReplayBuffer,
    flatten_policy_input,
)


def policy_input(value=0.0):
    return {
        "lidar_surrogate": [value] * 3200,
        "future_positions_body": [[value, value, value]] * 20,
        "actual_velocity_body": [value, value, value],
        "tracking_error_body": [value, value, value],
        "previous_applied_v_max": 0.30,
    }


def transition(step, normalized, mapping):
    requested = mapping.to_v_max(normalized)
    episode = "training_episode_000001"
    return {
        "episode_id": episode,
        "step_index": step,
        "request_id": step + 1,
        "truncated": False,
        "terminal_reason": "",
        "transition": {
            "state_t": policy_input(float(step)),
            "state_t_plus_1": policy_input(float(step + 1)),
            "action_t": {
                "requested_v_max": requested,
                "filtered_v_max": requested,
                "applied_v_max": requested,
            },
            "reward_context": {
                "provenance": {"run_id": "test", "episode_id": episode}
            },
            "reward": 1.0,
            "reward_components": {"reward_total": 1.0},
            "terminated": False,
        },
    }


class SacReplayTest(unittest.TestCase):
    def test_policy_input_dimension_is_frozen(self):
        vector = flatten_policy_input(policy_input())
        self.assertEqual(vector.shape, (OBSERVATION_DIM,))
        with self.assertRaises(ValueError):
            broken = policy_input()
            broken["lidar_surrogate"] = broken["lidar_surrogate"][:-1]
            flatten_policy_input(broken)

    def test_action_mapping_round_trip(self):
        mapping = ActionMapping(0.30, 1.75)
        for normalized in (-1.0, -0.4, 0.0, 0.8, 1.0):
            self.assertAlmostEqual(
                mapping.to_normalized(mapping.to_v_max(normalized)), normalized
            )
        self.assertAlmostEqual(mapping.to_v_max(-1.0), 0.30, places=12)
        self.assertAlmostEqual(mapping.to_v_max(0.0), 1.025, places=12)
        self.assertAlmostEqual(mapping.to_v_max(1.0), 1.75, places=12)

    def test_replay_preserves_identity_and_truncation(self):
        mapping = ActionMapping(0.30, 1.75)
        replay = SacReplayBuffer(16, mapping)
        for step, action in enumerate((-0.8, 0.0, 0.7)):
            replay.add(transition(step, action, mapping), action, 0)
        replay.mark_episode_boundary(
            "training_episode_000001", True, "max_episode_time"
        )
        audit = replay.audit()
        self.assertTrue(audit["passed"], audit)
        self.assertEqual(audit["truncated_count"], 1)
        batch = replay.sample(2, np.random.default_rng(1))
        self.assertEqual(batch["observations"].shape, (2, 3267))
        self.assertIn("terminated", batch)
        self.assertIn("truncated", batch)

    def test_safety_intervention_is_audited(self):
        mapping = ActionMapping(0.30, 1.75)
        replay = SacReplayBuffer(4, mapping, intervention_tolerance_mps=0.001)
        record = transition(0, 0.0, mapping)
        record["transition"]["action_t"]["applied_v_max"] += 0.01
        replay.add(record, 0.0, 0)
        self.assertIn("safety_intervention", replay.audit()["failures"])

    def test_large_logical_capacity_grows_storage_on_demand(self):
        mapping = ActionMapping(0.30, 1.75)
        replay = SacReplayBuffer(
            100000, mapping, initial_allocation=2
        )
        for step in range(3):
            replay.add(transition(step, 0.0, mapping), 0.0, 0)
        audit = replay.audit()
        self.assertEqual(audit["logical_capacity"], 100000)
        self.assertEqual(audit["allocated_capacity"], 4)
        self.assertEqual(audit["size"], 3)


if __name__ == "__main__":
    unittest.main()
