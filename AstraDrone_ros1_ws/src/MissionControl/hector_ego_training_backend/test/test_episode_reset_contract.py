#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml

from hector_ego_training_backend.episode_reset_contract import (
    EpisodeBinding,
    EpisodeIdentityLedger,
    RandomResetConfig,
    RandomResetSampler,
    ResetCandidate,
    ResetSamplingError,
    StaticObstacleXY,
    action_matches,
    episode_count_stop_due,
    observation_matches,
    sac_closure_matches,
    training_target_matches,
    trajectory_matches,
    validate_reset_candidate,
)


def random_config(seed=1001, attempts=8, obstacles=()):
    return RandomResetConfig(
        enabled=True,
        center_x=0.0,
        center_y=0.0,
        x_min_offset=-1.0,
        x_max_offset=1.0,
        y_min_offset=-1.0,
        y_max_offset=1.0,
        z=3.0,
        yaw=0.0,
        seed=seed,
        max_sampling_attempts=attempts,
        uav_collision_radius_xy=0.395567,
        ego_obstacles_inflation=0.30,
        additional_static_clearance=0.50,
        static_obstacles=tuple(obstacles),
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

    def test_training_target_requires_exact_count_and_identity(self):
        binding = EpisodeBinding(21, 20)
        payload = {
            "episode_id": binding.episode_key,
            "reset_generation": 20,
            "valid_transition_count": 10000,
            "target_valid_transitions": 10000,
            "reason": "training_target_reached",
        }
        self.assertTrue(training_target_matches(binding, payload, 10000))
        payload["valid_transition_count"] = 10001
        self.assertFalse(training_target_matches(binding, payload, 10000))
        payload["valid_transition_count"] = 10000
        payload["reset_generation"] = 19
        self.assertFalse(training_target_matches(binding, payload, 10000))

    def test_training_ignores_episode_count_and_evaluation_stops_at_three(self):
        self.assertFalse(episode_count_stop_due("training", 20, 0))
        self.assertFalse(episode_count_stop_due("training", 21, 0))
        self.assertFalse(episode_count_stop_due("evaluation", 2, 3))
        self.assertTrue(episode_count_stop_due("evaluation", 3, 3))
        with self.assertRaises(ValueError):
            episode_count_stop_due("training", 20, 20)

    def test_observation_requires_current_generation_and_trajectory(self):
        binding = EpisodeBinding(3, 2)
        self.assertTrue(observation_matches(binding, True, 2, 2, 30.0, 29.0, 8, 8))
        self.assertFalse(observation_matches(binding, True, 1, 2, 30.0, 29.0, 8, 8))
        self.assertFalse(observation_matches(binding, True, 2, 2, 28.0, 29.0, 8, 8))
        self.assertFalse(observation_matches(binding, True, 2, 2, 30.0, 29.0, 7, 8))

    def test_random_reset_seed_reproduces_sequence_and_bounds(self):
        left = RandomResetSampler(random_config(seed=1001))
        right = RandomResetSampler(random_config(seed=1001))
        left_sequence = [left.sample() for _ in range(20)]
        right_sequence = [right.sample() for _ in range(20)]
        for left_sample, right_sample in zip(left_sequence, right_sequence):
            self.assertEqual(left_sample["candidate"], right_sample["candidate"])
            candidate = left_sample["candidate"]
            self.assertGreaterEqual(candidate.x, -1.0)
            self.assertLessEqual(candidate.x, 1.0)
            self.assertGreaterEqual(candidate.y, -1.0)
            self.assertLessEqual(candidate.y, 1.0)
            self.assertEqual(candidate.z, 3.0)
            self.assertEqual(candidate.yaw, 0.0)

    def test_unsafe_candidate_is_rejected_then_resampled(self):
        sampler = RandomResetSampler(random_config(attempts=4))
        calls = []

        def reject_first(candidate, _config):
            calls.append(candidate)
            return {
                "valid": len(calls) > 1,
                "reasons": [] if len(calls) > 1 else ["test_unsafe"],
            }

        sample = sampler.sample(validator=reject_first)
        self.assertEqual(sample["attempt_count"], 2)
        self.assertEqual(len(sample["sampling_attempts"]), 2)
        self.assertFalse(
            sample["sampling_attempts"][0]["validation"]["valid"]
        )

    def test_max_attempts_fail_closed(self):
        sampler = RandomResetSampler(random_config(attempts=3))

        def reject_all(_candidate, _config):
            return {"valid": False, "reasons": ["test_unsafe"]}

        with self.assertRaises(ResetSamplingError) as context:
            sampler.sample(validator=reject_all)
        self.assertEqual(len(context.exception.attempts), 3)

    def test_worksite_clearance_rejects_unsafe_static_candidate(self):
        obstacle = StaticObstacleXY("test_tree", 0.0, 0.0, 10.0)
        sampler = RandomResetSampler(
            random_config(attempts=2, obstacles=(obstacle,))
        )
        with self.assertRaises(ResetSamplingError):
            sampler.sample()

    def test_random_sampling_does_not_change_episode_generation(self):
        ledger = EpisodeIdentityLedger()
        sampler = RandomResetSampler(random_config())
        sampler.sample()
        self.assertEqual(ledger.current, EpisodeBinding(1, 0))
        self.assertEqual(ledger.advance_after_reset(1), EpisodeBinding(2, 1))

    def test_worksite_profile_has_safe_complete_square(self):
        path = Path(__file__).resolve().parents[1] / "config/worksite_training_reset.yaml"
        values = yaml.safe_load(path.read_text(encoding="utf-8"))["random_start"]
        obstacles = tuple(
            StaticObstacleXY(
                item["name"], item["x"], item["y"], item["radius_at_hover_z"]
            )
            for item in values["static_obstacles"]
        )
        config = RandomResetConfig(
            enabled=True,
            center_x=values["center_x"],
            center_y=values["center_y"],
            x_min_offset=values["x_min_offset"],
            x_max_offset=values["x_max_offset"],
            y_min_offset=values["y_min_offset"],
            y_max_offset=values["y_max_offset"],
            z=values["z"],
            yaw=values["yaw"],
            seed=values["seed"],
            max_sampling_attempts=values["max_sampling_attempts"],
            uav_collision_radius_xy=values["uav_collision_radius_xy"],
            ego_obstacles_inflation=values["ego_obstacles_inflation"],
            additional_static_clearance=values["additional_static_clearance"],
            static_obstacles=obstacles,
        )
        minimum = float("inf")
        for x in (-1.0, 1.0):
            for y in (-1.0, 1.0):
                result = validate_reset_candidate(
                    ResetCandidate(x, y, 3.0, 0.0), config
                )
                self.assertTrue(result["valid"], result)
                minimum = min(minimum, result["minimum_static_clearance_m"])
        self.assertGreater(minimum, 1.33)


if __name__ == "__main__":
    unittest.main()
