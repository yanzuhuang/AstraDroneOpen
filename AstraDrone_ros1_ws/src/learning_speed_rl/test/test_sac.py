#!/usr/bin/env python3

import tempfile
import unittest

import numpy as np

try:
    import torch  # noqa: F401
    from learning_speed_rl.training.sac import SacAgent, SacConfig
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


@unittest.skipUnless(TORCH_AVAILABLE, "optional training-only PyTorch unavailable")
class SacLearnerTest(unittest.TestCase):
    def test_update_and_checkpoint_round_trip(self):
        config = SacConfig(
            batch_size=8,
            hidden_dim=32,
            torch_num_threads=1,
            critic_warmup_updates=0,
        )
        agent = SacAgent(config)
        rng = np.random.default_rng(4)
        batch = {
            "observations": rng.normal(size=(8, 3267)).astype(np.float32),
            "next_observations": rng.normal(size=(8, 3267)).astype(np.float32),
            "actions": rng.uniform(-1.0, 1.0, size=(8, 1)).astype(np.float32),
            "rewards": rng.normal(size=8).astype(np.float32),
            "terminated": np.zeros(8, dtype=np.float32),
            "truncated": np.ones(8, dtype=np.float32),
        }
        metric = agent.update(batch)
        self.assertTrue(metric["gradient_finite"])
        self.assertTrue(np.isfinite(metric["critic1_loss"]))
        self.assertGreaterEqual(metric["actor_log_std_min"], -3.0)
        self.assertLessEqual(metric["actor_log_std_max"], -1.0)
        self.assertTrue(np.isfinite(metric["actor_sample_std_mean"]))
        self.assertTrue(
            np.isfinite(metric["actor_deterministic_action_mean"])
        )
        self.assertTrue(metric["actor_update_enabled"])
        self.assertTrue(metric["actor_update_applied"])
        self.assertGreater(agent.parameter_update_audit()["critic1_l2_delta"], 0.0)
        with tempfile.NamedTemporaryFile(suffix=".pt") as handle:
            agent.save_checkpoint(handle.name, {"test": True})
            restored = SacAgent(config)
            restored.load_checkpoint(handle.name)
            self.assertTrue(agent.state_equal(restored))

    def test_actor_action_is_finite_and_bounded(self):
        agent = SacAgent(SacConfig(hidden_dim=32, batch_size=8))
        action = agent.sample_action(np.zeros(3267, dtype=np.float32))
        self.assertTrue(np.isfinite(action))
        self.assertGreaterEqual(action, -1.0)
        self.assertLessEqual(action, 1.0)

    def test_critic_warmup_defers_actor_and_alpha_updates(self):
        config = SacConfig(
            batch_size=8,
            hidden_dim=32,
            torch_num_threads=1,
            critic_warmup_updates=2,
        )
        agent = SacAgent(config)
        rng = np.random.default_rng(5)
        batch = {
            "observations": rng.normal(size=(8, 3267)).astype(np.float32),
            "next_observations": rng.normal(size=(8, 3267)).astype(np.float32),
            "actions": rng.uniform(-1.0, 1.0, size=(8, 1)).astype(np.float32),
            "rewards": rng.normal(size=8).astype(np.float32),
            "terminated": np.zeros(8, dtype=np.float32),
            "truncated": np.zeros(8, dtype=np.float32),
        }
        first = agent.update(batch)
        second = agent.update(batch)
        third = agent.update(batch)
        self.assertFalse(first["actor_update_enabled"])
        self.assertFalse(second["actor_update_enabled"])
        self.assertTrue(third["actor_update_enabled"])
        self.assertTrue(third["actor_update_applied"])
        self.assertIsNone(first["actor_loss"])
        self.assertIsNotNone(third["actor_loss"])


if __name__ == "__main__":
    unittest.main()
