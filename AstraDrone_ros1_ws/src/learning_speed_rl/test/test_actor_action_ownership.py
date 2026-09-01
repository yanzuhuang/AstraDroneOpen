"""Deterministic unit coverage for candidate/accepted Actor ownership."""

import unittest

from learning_speed_rl.training import ActorActionOwnership


class ActorActionOwnershipTest(unittest.TestCase):
    def test_terminal_before_accept_discards_candidate(self):
        ownership = ActorActionOwnership()
        ownership.propose(95, 0.25)
        self.assertEqual(ownership.candidate_count, 1)
        self.assertIsNone(ownership.resolve_acceptance(95, False))
        self.assertEqual(ownership.candidate_count, 0)
        self.assertEqual(ownership.active_interval_count, 0)
        ownership.require_closed()

    def test_accept_then_transition_consumes_exact_action(self):
        ownership = ActorActionOwnership()
        ownership.propose(7, -0.4)
        accepted = ownership.resolve_acceptance(7, True)
        self.assertEqual(accepted.step_index, 7)
        self.assertEqual(ownership.active_interval_count, 1)
        self.assertAlmostEqual(ownership.consume_transition(7), -0.4)
        ownership.require_closed()

    def test_mismatched_or_overlapping_ownership_fails_closed(self):
        ownership = ActorActionOwnership()
        ownership.propose(2, 0.0)
        with self.assertRaisesRegex(RuntimeError, "second candidate"):
            ownership.propose(2, 0.1)
        with self.assertRaisesRegex(RuntimeError, "disagrees"):
            ownership.resolve_acceptance(3, True)
        ownership.resolve_acceptance(2, True)
        with self.assertRaisesRegex(RuntimeError, "ACTION INTERVAL is active"):
            ownership.propose(3, 0.1)
        with self.assertRaisesRegex(RuntimeError, "still active"):
            ownership.require_closed()
        ownership.consume_transition(2)
        ownership.require_closed()

    def test_candidate_never_owns_active_interval_before_acceptance(self):
        ownership = ActorActionOwnership()
        ownership.propose(4, 0.2)
        self.assertEqual(ownership.candidate_count, 1)
        self.assertEqual(ownership.active_interval_count, 0)
        ownership.resolve_acceptance(4, False)
        self.assertEqual(ownership.candidate_count, 0)
        self.assertEqual(ownership.active_interval_count, 0)
        ownership.require_closed()


if __name__ == "__main__":
    unittest.main()
