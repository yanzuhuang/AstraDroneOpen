#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml

from learning_speed_rl.training.formal_training_contract import (
    FormalTrainingSchedule,
    checkpoint_filename,
    infrastructure_terminal_reason,
    learning_started,
    mode_uses_training_replay,
    validate_training_episode_target,
)


class FormalTrainingContractTest(unittest.TestCase):
    def test_only_explicit_infrastructure_prefix_requests_fail_closed(self):
        self.assertEqual(
            infrastructure_terminal_reason(
                "mission_failure",
                "infrastructure:observation_c_producer_stall",
            ),
            "infrastructure:observation_c_producer_stall",
        )
        for ordinary in (
            "collision",
            "planner_failure:NO_FEASIBLE_TRAJECTORY",
            "max_episode_time",
            "invalid_observation:source_stale",
        ):
            with self.subTest(reason=ordinary):
                self.assertEqual(
                    infrastructure_terminal_reason(ordinary), ""
                )

    def test_timing_qualification_is_explicit_and_capped_at_ten(self):
        self.assertEqual(
            validate_training_episode_target(
                10, timing_qualification=True
            ),
            "rl_timing_qualification",
        )
        with self.assertRaisesRegex(ValueError, "1..10"):
            validate_training_episode_target(11, timing_qualification=True)
        with self.assertRaisesRegex(ValueError, "exclusive"):
            validate_training_episode_target(
                10, smoke_test=True, timing_qualification=True
            )
        self.assertEqual(
            validate_training_episode_target(31, smoke_test=True),
            "forest_31episode_smoke",
        )
        self.assertEqual(
            validate_training_episode_target(10000), "formal_training"
        )

    def test_early_learning_observation_is_explicit_and_exactly_100(self):
        self.assertEqual(
            validate_training_episode_target(
                100, early_learning_observation=True
            ),
            "early_learning_100episode_observation",
        )
        for invalid in (99, 101, 10000):
            with self.subTest(total=invalid):
                with self.assertRaisesRegex(ValueError, "exactly 100"):
                    validate_training_episode_target(
                        invalid, early_learning_observation=True
                    )
        with self.assertRaisesRegex(ValueError, "exclusive"):
            validate_training_episode_target(
                100,
                timing_qualification=True,
                early_learning_observation=True,
            )

    def setUp(self):
        self.checkpoints = tuple(range(500, 10001, 500))
        self.schedule = FormalTrainingSchedule(
            total_training_episodes=10000,
            checkpoint_episodes=self.checkpoints,
        )

    def test_episode_9999_cannot_normally_stop_training(self):
        self.schedule.validate()
        self.assertFalse(self.schedule.stop_due(9999))
        self.assertEqual(self.schedule.next_episode_number(9999), 10000)

    def test_episode_10000_stops_exactly_and_episode_10001_is_forbidden(self):
        self.assertTrue(self.schedule.checkpoint_due(10000))
        self.assertTrue(self.schedule.stop_due(10000))
        with self.assertRaises(ValueError):
            self.schedule.stop_due(10001)
        with self.assertRaises(ValueError):
            self.schedule.next_episode_number(10000)

    def test_checkpoint_schedule_is_episode_based_and_exact(self):
        emitted = set()
        events = []
        for episode in range(1, 10001):
            if self.schedule.checkpoint_due(episode) and episode not in emitted:
                emitted.add(episode)
                events.append(episode)
        self.assertEqual(events, list(self.checkpoints))
        self.assertEqual(len(events), 20)
        self.assertEqual(
            [checkpoint_filename(events[index]) for index in (0, -1)],
            [
                "sac_checkpoint_episode_0500.pt",
                "sac_checkpoint_episode_10000.pt",
            ],
        )

    def test_success_collision_and_truncation_each_count_once_after_closure(self):
        completed = 0
        for _terminal_outcome in ("success", "collision", "truncated"):
            completed = self.schedule.complete_episode(completed, True)
        self.assertEqual(completed, 3)
        with self.assertRaises(ValueError):
            self.schedule.complete_episode(completed, False)

    def test_evaluation_has_no_training_replay_or_updates(self):
        self.assertFalse(mode_uses_training_replay("evaluation"))
        self.assertTrue(mode_uses_training_replay("training"))

    def test_learning_starts_at_1000_transitions_not_episodes(self):
        self.assertFalse(learning_started(999, 1000))
        self.assertTrue(learning_started(1000, 1000))
        self.assertTrue(learning_started(1001, 1000))

    def test_formal_yaml_freezes_10000_episode_schedule(self):
        path = Path(__file__).resolve().parents[1] / "config/sac_training_v1.yaml"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        training = data["training"]
        self.assertEqual(training["mode"], "training")
        self.assertEqual(training["total_training_episodes"], 10000)
        self.assertEqual(
            training["checkpoint_episodes"], list(self.checkpoints)
        )
        self.assertNotIn("target_valid_transitions", training)
        self.assertNotIn("checkpoint_steps", training)
        self.assertNotIn("max_training_episodes", training)
        self.assertEqual(
            data["training"]["completion_reason"],
            "training_episode_count_reached",
        )
        self.assertEqual(data["replay"]["capacity"], 100000)
        self.assertEqual(data["training"]["learning_starts"], 1000)
        self.assertEqual(data["episode"]["max_steps"], 500)
        self.assertEqual(data["sac"]["batch_size"], 64)
        self.assertEqual(data["sac"]["critic_warmup_updates"], 100)
        self.assertEqual(data["sac"]["policy_learning_rate"], 1.0e-5)
        self.assertEqual(data["sac"]["critic_learning_rate"], 1.0e-3)
        self.assertEqual(data["sac"]["alpha_learning_rate"], 1.0e-3)
        self.assertEqual(data["sac"]["gamma"], 0.99)
        self.assertEqual(data["sac"]["tau"], 0.005)
        self.assertEqual(data["sac"]["target_entropy"], -1.0)
        self.assertEqual(data["sac"]["log_std_min"], -3.0)
        self.assertEqual(data["sac"]["log_std_max"], -1.0)
        self.assertEqual(data["action"]["expected_v_max_min"], 0.30)
        self.assertEqual(data["action"]["expected_v_max_max"], 1.75)
        self.assertEqual(data["sac"]["seed"], 1)

    def test_evaluation_is_independent_and_fixed_at_100_episodes(self):
        path = Path(__file__).resolve().parents[1] / "config/sac_training_v1.yaml"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.assertEqual(
            set(data["evaluation"]),
            {"episode_count", "checkpoint_path", "checkpoint_episode"},
        )
        self.assertEqual(data["evaluation"]["episode_count"], 100)
        self.assertNotIn("evaluation", data["training"])

    def test_formal_action_mapping_is_exact(self):
        from learning_speed_rl.training.sac_replay import ActionMapping

        mapping = ActionMapping(0.30, 1.75)
        self.assertAlmostEqual(mapping.to_v_max(-1.0), 0.30, places=12)
        self.assertAlmostEqual(mapping.to_v_max(0.0), 1.025, places=12)
        self.assertAlmostEqual(mapping.to_v_max(1.0), 1.75, places=12)


if __name__ == "__main__":
    unittest.main()
