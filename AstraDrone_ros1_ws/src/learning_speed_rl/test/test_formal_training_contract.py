#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml

from learning_speed_rl.training.formal_training_contract import (
    FormalTrainingSchedule,
    mode_updates_networks,
    mode_uses_training_replay,
)


class FormalTrainingContractTest(unittest.TestCase):
    def setUp(self):
        self.schedule = FormalTrainingSchedule(
            target_valid_transitions=10000,
            checkpoint_steps=(5000, 10000),
            checkpoint_interval=5000,
            evaluation_during_training=False,
        )

    def test_checkpoint_at_5000(self):
        self.schedule.validate()
        self.assertTrue(self.schedule.checkpoint_due(5000))
        self.assertFalse(self.schedule.stop_due(5000))

    def test_checkpoint_and_stop_at_10000(self):
        self.assertTrue(self.schedule.checkpoint_due(10000))
        self.assertTrue(self.schedule.stop_due(10000))
        with self.assertRaises(ValueError):
            self.schedule.stop_due(10001)

    def test_twenty_short_episodes_do_not_complete_training(self):
        completed = sum([400] * 20)
        self.assertEqual(completed, 8000)
        self.assertFalse(self.schedule.stop_due(completed))
        self.assertEqual(self.schedule.next_transition_step(completed), 8001)
        self.assertEqual(self.schedule.episode_step_budget(completed, 500), 500)

    def test_episode_twenty_one_and_later_continue_to_exact_target(self):
        completed = 8000
        episode_count = 20
        checkpoint_events = []
        replay_steps = []
        while not self.schedule.stop_due(completed):
            episode_count += 1
            budget = self.schedule.episode_step_budget(completed, 500)
            for _ in range(budget):
                completed = self.schedule.next_transition_step(completed)
                replay_steps.append(completed)
                if self.schedule.checkpoint_due(completed):
                    checkpoint_events.append(completed)
        self.assertEqual(episode_count, 24)
        self.assertEqual(completed, 10000)
        self.assertEqual(replay_steps[-1], 10000)
        self.assertNotIn(10001, replay_steps)
        self.assertEqual(checkpoint_events, [10000])
        with self.assertRaises(ValueError):
            self.schedule.next_transition_step(completed)

    def test_checkpoint_schedule_is_exactly_once(self):
        emitted = set()
        events = []
        for step in range(1, 10001):
            if self.schedule.checkpoint_due(step) and step not in emitted:
                emitted.add(step)
                events.append(step)
        self.assertEqual(events, [5000, 10000])

    def test_training_evaluation_is_disabled(self):
        invalid = FormalTrainingSchedule(10000, (5000, 10000), 5000, True)
        with self.assertRaises(ValueError):
            invalid.validate()

    def test_evaluation_has_no_training_replay_or_updates(self):
        self.assertFalse(mode_uses_training_replay("evaluation"))
        self.assertFalse(mode_updates_networks("evaluation"))
        self.assertTrue(mode_uses_training_replay("training"))
        self.assertTrue(mode_updates_networks("training"))

    def test_formal_yaml_freezes_first_10k_schedule(self):
        path = Path(__file__).resolve().parents[1] / "config/sac_training_v1.yaml"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        training = data["training"]
        self.assertEqual(training["mode"], "training")
        self.assertEqual(training["target_valid_transitions"], 10000)
        self.assertEqual(training["checkpoint_steps"], [5000, 10000])
        self.assertEqual(training["checkpoint_interval"], 5000)
        self.assertFalse(training["evaluation_during_training"])
        self.assertEqual(data["training"]["completion_reason"], "training_target_reached")
        self.assertEqual(data["replay"]["capacity"], 100000)
        self.assertEqual(data["sac"]["seed"], 1)


if __name__ == "__main__":
    unittest.main()
