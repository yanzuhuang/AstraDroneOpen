#!/usr/bin/env python3

import unittest

from hector_ego_training_backend.episode_reset_contract import (
    EpisodeBinding,
    EpisodeIdentityLedger,
    action_matches,
    observation_matches,
    sac_closure_matches,
    trajectory_matches,
)


class EpisodeResetContractTest(unittest.TestCase):
    def test_identity_advances_exactly_once_per_reset(self):
        ledger = EpisodeIdentityLedger()
        self.assertEqual(ledger.current, EpisodeBinding(1, 0))
        self.assertEqual(ledger.current.episode_key, "training_episode_000001")
        self.assertEqual(ledger.advance_after_reset(1), EpisodeBinding(2, 1))
        with self.assertRaises(ValueError):
            ledger.advance_after_reset(3)

    def test_action_requires_exact_episode_identity(self):
        binding = EpisodeBinding(7, 6)
        self.assertTrue(action_matches(binding, binding.episode_key, 0, 7))
        self.assertFalse(action_matches(binding, binding.episode_key, 1, 7))
        self.assertFalse(action_matches(binding, "training_episode_000006", 0, 7))

    def test_trajectory_requires_event_and_source_barriers(self):
        binding = EpisodeBinding(2, 1)
        self.assertTrue(
            trajectory_matches(binding, 12, 20.1, 101, 100, 20.0, 19.0, 11)
        )
        self.assertFalse(
            trajectory_matches(binding, 12, 20.1, 99, 100, 20.0, 19.0, 11)
        )
        self.assertFalse(
            trajectory_matches(binding, 11, 20.1, 101, 100, 20.0, 19.0, 11)
        )

    def test_sac_closure_requires_exact_episode_generation_and_identity(self):
        binding = EpisodeBinding(4, 3)
        payload = {
            "episode_id": binding.episode_key,
            "reset_generation": 3,
            "terminal_transition_closed": True,
            "transition_count": 101,
            "last_step_index": 100,
            "last_request_id": 101,
            "status": "completed",
        }
        self.assertTrue(sac_closure_matches(binding, payload))
        payload["reset_generation"] = 2
        self.assertFalse(sac_closure_matches(binding, payload))
        payload["reset_generation"] = 3
        payload["last_request_id"] = 100
        self.assertFalse(sac_closure_matches(binding, payload))

    def test_observation_requires_current_generation_and_trajectory(self):
        binding = EpisodeBinding(3, 2)
        self.assertTrue(observation_matches(binding, True, 2, 2, 30.0, 29.0, 8, 8))
        self.assertFalse(observation_matches(binding, True, 1, 2, 30.0, 29.0, 8, 8))
        self.assertFalse(observation_matches(binding, True, 2, 2, 28.0, 29.0, 8, 8))
        self.assertFalse(observation_matches(binding, True, 2, 2, 30.0, 29.0, 7, 8))


if __name__ == "__main__":
    unittest.main()
