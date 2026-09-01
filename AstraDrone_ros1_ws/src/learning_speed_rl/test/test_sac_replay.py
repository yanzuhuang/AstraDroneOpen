#!/usr/bin/env python3

import unittest
import threading
import time

import numpy as np

from learning_speed_rl.training.sac_replay import (
    ActionMapping,
    OBSERVATION_DIM,
    SacReplayBuffer,
    flatten_policy_input,
)
from learning_speed_rl.training import (
    ImmutableTransitionRecord,
    OrderedTransitionWriter,
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
    def test_formed_transition_is_deeply_immutable(self):
        mapping = ActionMapping(0.30, 1.75)
        source = transition(0, 0.0, mapping)
        formed = ImmutableTransitionRecord(source)
        source["transition"]["reward"] = 99.0
        self.assertEqual(formed["transition"]["reward"], 1.0)
        with self.assertRaises(TypeError):
            formed["transition"]["reward"] = 2.0
        with self.assertRaises(TypeError):
            formed["transition"]["state_t"]["lidar_surrogate"][0] = 2.0

    def test_ordered_writer_delay_does_not_change_submission_order(self):
        mapping = ActionMapping(0.30, 1.75)
        release = threading.Event()
        started = threading.Event()
        persisted = []

        def persist(record, _payload):
            if int(record["step_index"]) == 0:
                started.set()
                self.assertTrue(release.wait(1.0))
            persisted.append(int(record["step_index"]))

        writer = OrderedTransitionWriter(persist)
        writer.submit(transition(0, 0.0, mapping), {"reset_generation": 7})
        self.assertTrue(started.wait(1.0))
        writer.submit(transition(1, 0.0, mapping), {"reset_generation": 7})
        self.assertEqual(writer.submitted_count, 2)
        self.assertEqual(writer.persisted_count, 0)
        self.assertGreaterEqual(writer.pending_count, 2)
        release.set()
        writer.close(timeout_sec=1.0)
        self.assertEqual(persisted, [0, 1])
        self.assertEqual(writer.persisted_count, 2)

    def test_ordered_writer_rejects_holes_generation_change_and_post_terminal(self):
        mapping = ActionMapping(0.30, 1.75)
        persisted = []
        writer = OrderedTransitionWriter(
            lambda record, _payload: persisted.append(record["step_index"])
        )
        writer.submit(transition(0, 0.0, mapping), {"reset_generation": 4})
        with self.assertRaisesRegex(ValueError, "not contiguous"):
            writer.submit(transition(2, 0.0, mapping), {"reset_generation": 4})
        with self.assertRaisesRegex(ValueError, "generation changed"):
            writer.submit(transition(1, 0.0, mapping), {"reset_generation": 5})
        terminal = transition(1, 0.0, mapping)
        terminal["terminated"] = True
        terminal["transition"]["terminated"] = True
        terminal["terminal_reason"] = "collision"
        writer.submit(terminal, {"reset_generation": 4})
        with self.assertRaisesRegex(ValueError, "after terminal boundary"):
            writer.submit(transition(2, 0.0, mapping), {"reset_generation": 4})
        writer.close(timeout_sec=1.0)
        self.assertEqual(persisted, [0, 1])

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
            "training_episode_000001",
            terminated=False,
            truncated=True,
            terminal_reason="max_episode_time",
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

    def test_ordered_commit_rejects_gap_and_accepts_only_next_step(self):
        mapping = ActionMapping(0.30, 1.75)
        replay = SacReplayBuffer(16, mapping)
        replay.add(transition(0, 0.0, mapping), 0.0, 7)
        with self.assertRaisesRegex(ValueError, "not contiguous"):
            replay.add(transition(2, 0.0, mapping), 0.0, 7)
        replay.add(transition(1, 0.0, mapping), 0.0, 7)
        self.assertEqual(replay.audit()["size"], 2)

    def test_terminal_boundary_is_unique_and_blocks_future_commit(self):
        mapping = ActionMapping(0.30, 1.75)
        replay = SacReplayBuffer(16, mapping)
        replay.add(transition(0, 0.0, mapping), 0.0, 3)
        terminal = transition(1, 0.0, mapping)
        terminal["transition"]["terminated"] = True
        terminal["terminal_reason"] = "collision"
        replay.add(terminal, 0.0, 3)
        replay.mark_episode_boundary(
            "training_episode_000001",
            terminated=True,
            truncated=False,
            terminal_reason="collision",
        )
        with self.assertRaisesRegex(ValueError, "after terminal boundary"):
            replay.add(transition(2, 0.0, mapping), 0.0, 3)
        audit = replay.audit()
        self.assertTrue(audit["passed"], audit)
        self.assertEqual(audit["terminated_count"], 1)
        self.assertEqual(audit["closed_episode_count"], 1)

    def test_terminal_between_actions_promotes_only_last_real_replay_row(self):
        mapping = ActionMapping(0.30, 1.75)
        replay = SacReplayBuffer(16, mapping)
        replay.add(transition(0, 0.0, mapping), 0.0, 3)
        replay.mark_episode_boundary(
            "training_episode_000001",
            terminated=True,
            truncated=False,
            terminal_reason="planner_failure:NO_FEASIBLE_TRAJECTORY",
        )
        audit = replay.audit()
        self.assertTrue(audit["passed"], audit)
        self.assertEqual(audit["size"], 1)
        self.assertEqual(audit["terminated_count"], 1)
        self.assertEqual(audit["closed_episode_count"], 1)
        with self.assertRaisesRegex(ValueError, "after terminal boundary"):
            replay.add(transition(1, 0.0, mapping), 0.0, 3)

    def test_generation_mismatch_is_rejected_before_commit(self):
        mapping = ActionMapping(0.30, 1.75)
        replay = SacReplayBuffer(16, mapping)
        replay.add(transition(0, 0.0, mapping), 0.0, 4)
        with self.assertRaisesRegex(ValueError, "generation changed"):
            replay.add(transition(1, 0.0, mapping), 0.0, 5)


if __name__ == "__main__":
    unittest.main()
