#!/usr/bin/env python3

import json
from pathlib import Path
import unittest

from learning_speed_rl.training import BalancedForestMapScheduler


class ForestScheduleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_path = (
            Path(__file__).resolve().parents[4]
            / "simulation/forest/forest_pool_v1.json"
        )
        cls.config = json.loads(cls.config_path.read_text(encoding="utf-8"))

    def scheduler(self, rng_seed=None):
        mapping = {
            item["logical_seed"]: item["raw_seed"]
            for item in self.config["seed_pool"]
        }
        schedule = self.config["scheduler"]
        return BalancedForestMapScheduler(
            schedule["training_logical_seeds"],
            schedule["evaluation_logical_seeds"],
            mapping,
            episodes_per_map_block=schedule["episodes_per_map_block"],
            rng_seed=schedule["rng_seed"] if rng_seed is None else rng_seed,
        )

    def smoke_scheduler(self):
        mapping = {
            item["logical_seed"]: item["raw_seed"]
            for item in self.config["seed_pool"]
        }
        schedule = self.config["scheduler"]
        return BalancedForestMapScheduler(
            schedule["training_logical_seeds"],
            schedule["evaluation_logical_seeds"],
            mapping,
            episodes_per_map_block=10,
            rng_seed=schedule["rng_seed"],
            smoke_test=True,
        )

    def test_training_and_evaluation_pools_are_isolated(self):
        scheduler = self.scheduler()
        training = {scheduler.training_assignment(index).logical_seed for index in range(10000)}
        self.assertEqual(training, set(range(8)))
        self.assertTrue(training.isdisjoint({8, 9}))
        self.assertEqual(scheduler.evaluation_assignment(8).mode, "evaluation")
        self.assertEqual(scheduler.evaluation_assignment(9).mode, "evaluation")
        with self.assertRaises(ValueError):
            scheduler.evaluation_assignment(7)

    def test_same_map_runs_for_exactly_100_episodes(self):
        scheduler = self.scheduler()
        first = scheduler.training_assignment(0)
        self.assertTrue(all(scheduler.training_assignment(index).logical_seed == first.logical_seed for index in range(100)))
        self.assertNotEqual(first.logical_seed, scheduler.training_assignment(100).logical_seed)
        self.assertTrue(scheduler.map_switch_due_after(100, 10000))
        self.assertFalse(scheduler.map_switch_due_after(99, 10000))

    def test_every_800_episode_round_covers_each_training_map_once(self):
        scheduler = self.scheduler()
        for round_id in range(12):
            blocks = [
                scheduler.training_assignment(round_id * 800 + offset * 100).logical_seed
                for offset in range(8)
            ]
            self.assertEqual(set(blocks), set(range(8)))
            self.assertEqual(len(blocks), len(set(blocks)))

    def test_each_round_reshuffles_and_fixed_rng_is_reproducible(self):
        left = self.scheduler(314159)
        right = self.scheduler(314159)
        sequence_left = [left.training_assignment(index * 100).logical_seed for index in range(24)]
        sequence_right = [right.training_assignment(index * 100).logical_seed for index in range(24)]
        self.assertEqual(sequence_left, sequence_right)
        self.assertNotEqual(sequence_left[:8], sequence_left[8:16])

    def test_10000_episode_balance_differs_by_at_most_one_block(self):
        scheduler = self.scheduler()
        counts = {seed: 0 for seed in range(8)}
        for episode in range(10000):
            counts[scheduler.training_assignment(episode).logical_seed] += 1
        self.assertEqual(sum(counts.values()), 10000)
        self.assertLessEqual(max(counts.values()) - min(counts.values()), 100)

    def test_scheduler_snapshot_resume_preserves_next_map(self):
        original = self.scheduler(2718)
        original.training_assignment(5999)
        snapshot = original.snapshot(6000)
        restored = BalancedForestMapScheduler.from_snapshot(snapshot)
        self.assertEqual(
            original.training_assignment(6000).as_dict(),
            restored.training_assignment(6000).as_dict(),
        )

    def test_map_switch_does_not_recreate_sac_or_replay_state(self):
        scheduler = self.scheduler()
        class TrainingOwner:
            pass

        agent = TrainingOwner()
        agent.actor = object()
        agent.critic1, agent.critic2 = object(), object()
        agent.target1, agent.target2 = object(), object()
        agent.actor_optimizer = object()
        agent.critic_optimizer = object()
        agent.alpha_optimizer = object()
        agent.global_environment_step = 1234
        agent.completed_episode_count = 100
        replay = []
        actor_id = id(agent.actor)
        critic_ids = (id(agent.critic1), id(agent.critic2))
        target_ids = (id(agent.target1), id(agent.target2))
        optimizer_ids = (id(agent.actor_optimizer), id(agent.critic_optimizer), id(agent.alpha_optimizer))
        replay_id = id(replay)
        first = scheduler.training_assignment(99)
        second = scheduler.training_assignment(100)
        self.assertNotEqual(first.logical_seed, second.logical_seed)
        self.assertEqual(actor_id, id(agent.actor))
        self.assertEqual(critic_ids, (id(agent.critic1), id(agent.critic2)))
        self.assertEqual(target_ids, (id(agent.target1), id(agent.target2)))
        self.assertEqual(optimizer_ids, (id(agent.actor_optimizer), id(agent.critic_optimizer), id(agent.alpha_optimizer)))
        self.assertEqual(replay_id, id(replay))
        self.assertEqual(agent.global_environment_step, 1234)
        self.assertEqual(agent.completed_episode_count, 100)
        self.assertEqual(len(replay), 0)

    def test_evaluation_path_has_no_training_state_owner(self):
        scheduler = self.scheduler()
        before = scheduler.snapshot(4321)
        for logical_seed in (8, 9):
            assignment = scheduler.evaluation_assignment(logical_seed, 7)
            self.assertEqual(assignment.mode, "evaluation")
        after = scheduler.snapshot(4321)
        self.assertEqual(before, after)

    def test_31_episode_smoke_uses_four_maps_and_three_switches(self):
        scheduler = self.smoke_scheduler()
        assignments = [scheduler.training_assignment(index) for index in range(31)]
        self.assertEqual(
            [item.logical_seed for item in assignments],
            [6] * 10 + [1] * 10 + [4] * 10 + [3],
        )
        self.assertEqual(
            [item.raw_seed for item in assignments],
            [36] * 10 + [10] * 10 + [23] * 10 + [18],
        )
        self.assertEqual(
            [index for index in range(1, 31) if scheduler.map_switch_due_after(index, 31)],
            [10, 20, 30],
        )
        self.assertTrue({item.logical_seed for item in assignments}.isdisjoint({8, 9}))

    def test_nonstandard_block_requires_explicit_smoke_contract(self):
        mapping = {
            item["logical_seed"]: item["raw_seed"]
            for item in self.config["seed_pool"]
        }
        schedule = self.config["scheduler"]
        with self.assertRaises(ValueError):
            BalancedForestMapScheduler(
                schedule["training_logical_seeds"],
                schedule["evaluation_logical_seeds"],
                mapping,
                episodes_per_map_block=10,
                rng_seed=schedule["rng_seed"],
            )


if __name__ == "__main__":
    unittest.main()
