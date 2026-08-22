"""Unit/offline coverage for Episode v0.1 and its 10 Hz causal scheduler."""

import math
import threading
import time
import unittest

import numpy as np

from learning_speed_rl.training import (
    AstraDroneEnv,
    AstraDroneEpisodeConfig,
    AstraDroneStepConfig,
    OfficialTrajectoryIdentity,
    PolicyStateProvenance,
    PolicyStateV1,
    ProgressContextState,
    Stage1Reward,
)


class MutableRosClock:
    def __init__(self, value):
        self._lock = threading.Lock()
        self._value = float(value)

    def __call__(self):
        with self._lock:
            return self._value

    def advance(self, delta):
        with self._lock:
            self._value += float(delta)
            return self._value


class AsyncEpisodeHarness:
    def __init__(
        self, *, acknowledge=True, terminal_after=None, terminal_kind="success"
    ):
        self.clock = MutableRosClock(10.0)
        self.trajectory = OfficialTrajectoryIdentity(
            7, 9.0, "uav1/camera_init"
        )
        self.acknowledge = acknowledge
        self.terminal_after = terminal_after
        self.terminal_kind = terminal_kind
        self.published = []
        self.stop = threading.Event()
        self.env = AstraDroneEnv(
            publish_requested_v_max=self.publish,
            ros_clock=self.clock,
            reward=Stage1Reward(),
            config=AstraDroneStepConfig(
                duration_sec=0.1,
                action_timeout_sec=0.05,
                applied_timeout_sec=0.05,
                hold_timeout_sec=0.10,
                next_observation_timeout_sec=0.05,
            ),
            run_id="unit_episode_run",
            episode_id="episode_0001",
        )
        self.env.record_official_trajectory(self.trajectory, receive_sec=9.5)
        self.env.record_mission_state("NAVIGATING", receive_sec=9.8)
        self.env.record_bridge_state("TRACK_EGO")
        self.env.record_planner_safety(
            current_position_in_collision=False,
            emergency_stop_active=False,
            planner_state="EXEC_TRAJ",
            receive_sec=9.9,
        )
        self._record_observation(9.95)
        self.thread = threading.Thread(target=self._pump, daemon=True)

    def _state(self, stamp):
        return PolicyStateV1(
            lidar_surrogate=np.ones(3200, dtype=np.float32),
            future_positions_body=np.zeros((20, 3), dtype=np.float32),
            actual_velocity_body=np.asarray([1.2, -0.3, 0.4]),
            tracking_error_body=np.zeros(3, dtype=np.float32),
            previous_applied_v_max=1.25,
            provenance=PolicyStateProvenance(
                observation_stamp_sec=stamp,
                observation_receive_sec=stamp,
                body_frame="uav1/body",
                observation_version="scheme_c_trajectory_fusion_v1.0",
                official_trajectory=self.trajectory,
            ),
        )

    @staticmethod
    def _context(receive):
        return ProgressContextState(
            mission_progress=min(1.0, max(0.0, (receive - 9.0) / 100.0)),
            progress_receipt_ros_time_sec=receive - 0.002,
            mission_state="NAVIGATING",
            mission_state_receipt_ros_time_sec=receive - 0.003,
            observation_receive_sec=receive,
        )

    @staticmethod
    def _reward_observation():
        return {
            "nearest_obstacle_distance_m": 4.25,
            "known_obstacle_bin_fraction": 0.06,
            "unknown_bin_count": 400,
        }

    def _record_observation(self, stamp):
        self.env.record_observation(
            self._state(stamp),
            self._context(stamp),
            self._reward_observation(),
        )

    def publish(self, request):
        now = self.clock()
        self.published.append((now, request))
        if not self.acknowledge:
            return
        # A foreign identity must never be consumed by this pending request.
        self.env.record_action_stamped(
            stamp_sec=request.publish_ros_time_sec,
            episode_id="foreign_episode",
            step_index=request.step_index,
            request_id=request.request_id,
            source_mode="mock",
            requested_v_max=request.requested_v_max,
            filtered_v_max=request.requested_v_max,
            receive_sec=now + 0.001,
        )
        self.env.record_action_stamped(
            stamp_sec=request.publish_ros_time_sec,
            episode_id=request.episode_id,
            step_index=request.step_index,
            request_id=request.request_id,
            source_mode="mock",
            requested_v_max=request.requested_v_max,
            filtered_v_max=request.requested_v_max,
            receive_sec=now + 0.002,
        )
        self.env.record_applied_stamped(
            stamp_sec=request.publish_ros_time_sec + 1.0e-7,
            episode_id=request.episode_id,
            step_index=request.step_index,
            request_id=request.request_id,
            applied_v_max=request.requested_v_max,
            receive_sec=now + 0.004,
        )
        if self.terminal_after is not None and len(self.published) == self.terminal_after:
            if self.terminal_kind == "success":
                self.env.record_mission_success(True, receive_sec=now + 0.025)
                self.env.record_mission_done(True, receive_sec=now + 0.025)
            elif self.terminal_kind == "failure":
                self.env.record_mission_failure(True, receive_sec=now + 0.025)
                self.env.record_mission_done(True, receive_sec=now + 0.025)
            elif self.terminal_kind == "collision":
                self.env.record_planner_safety(
                    current_position_in_collision=True,
                    emergency_stop_active=False,
                    planner_state="EXEC_TRAJ",
                    receive_sec=now + 0.025,
                )
            else:
                raise AssertionError("unsupported test terminal kind")

    def _pump(self):
        next_observation = 10.05
        while not self.stop.is_set():
            now = self.clock.advance(0.01)
            while now + 1.0e-12 >= next_observation:
                self._record_observation(next_observation)
                next_observation += 0.1
            time.sleep(0.001)

    def run(self, *, max_steps=8, max_duration=None):
        self.thread.start()
        try:
            return self.env.run_episode(
                lambda _state, _index: 0.75,
                AstraDroneEpisodeConfig(
                    max_steps=max_steps,
                    max_duration_sec=max_duration,
                    start_timeout_sec=0.1,
                    completion_timeout_sec=0.2,
                    deadline_tolerance_sec=0.025,
                    maximum_in_flight=8,
                ),
            )
        finally:
            self.stop.set()
            self.thread.join(timeout=1.0)


class AstraDroneEnvEpisodeTest(unittest.TestCase):
    def test_overlapping_scheduler_closes_unique_causal_transitions(self):
        harness = AsyncEpisodeHarness()
        result = harness.run(max_steps=8)

        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["metrics"]["scheduled_action_count"], 8)
        self.assertEqual(result["metrics"]["transition_count"], 8)
        self.assertEqual(result["metrics"]["accepted_action_count"], 16)
        self.assertEqual(result["metrics"]["accepted_applied_ack_count"], 8)
        self.assertTrue(result["metrics"]["identity_chain_strict_1_to_1"])
        self.assertTrue(result["metrics"]["closed_request_ids_strictly_monotonic"])
        self.assertEqual(result["metrics"]["timeout_count"], 0)
        self.assertEqual(result["metrics"]["causal_mismatch_count"], 0)
        self.assertGreaterEqual(result["metrics"]["maximum_observed_in_flight"], 2)
        self.assertTrue(result["episode"]["truncated"])
        self.assertFalse(result["episode"]["terminated"])
        self.assertEqual(result["episode"]["terminal_reason"], "max_episode_steps")
        self.assertEqual(
            [step["step_index"] for step in result["steps"]], list(range(8))
        )
        self.assertEqual(
            [step["request_id"] for step in result["steps"]], list(range(1, 9))
        )
        self.assertEqual(
            len({step["timing"]["action_stamp_sec"] for step in result["steps"]}),
            8,
        )
        self.assertEqual(
            len(
                {
                    step["timing"]["state_t_plus_1_receive_sec"]
                    for step in result["steps"]
                }
            ),
            8,
        )
        for step in result["steps"]:
            timing = step["timing"]
            transition = step["transition"]
            self.assertLessEqual(
                timing["state_t_receive_sec"], timing["action_stamp_sec"]
            )
            self.assertGreater(
                timing["state_t_plus_1_stamp_sec"], timing["state_t_stamp_sec"]
            )
            self.assertGreaterEqual(
                timing["state_t_plus_1_receive_sec"],
                timing["hold_target_ros_time_sec"],
            )
            action = transition["action_t"]
            self.assertAlmostEqual(
                action["requested_v_max"], action["filtered_v_max"]
            )
            self.assertAlmostEqual(
                action["filtered_v_max"], action["applied_v_max"]
            )
            self.assertTrue(math.isfinite(step["reward"]))
            self.assertFalse(step["transition_truncated"])

        expected_return = sum(step["reward"] for step in result["steps"])
        self.assertAlmostEqual(result["episode"]["episode_return"], expected_return)
        self.assertEqual(result["episode"]["step_index"], 7)
        self.assertTrue(result["steps"][-1]["truncated"])
        self.assertFalse(result["steps"][-1]["transition_truncated"])
        intervals = [
            later[0] - earlier[0]
            for earlier, later in zip(harness.published, harness.published[1:])
        ]
        self.assertTrue(all(abs(value - 0.1) <= 0.02 for value in intervals))

    def test_mission_success_terminates_without_truncation(self):
        harness = AsyncEpisodeHarness(terminal_after=4)
        result = harness.run(max_steps=20)

        self.assertEqual(result["status"], "completed")
        self.assertTrue(result["episode"]["terminated"])
        self.assertFalse(result["episode"]["truncated"])
        self.assertIn("mission_success", result["episode"]["terminal_reason"])
        self.assertTrue(any(step["terminated"] for step in result["steps"]))
        self.assertLess(result["metrics"]["scheduled_action_count"], 20)

    def test_max_duration_is_an_episode_truncation_boundary(self):
        harness = AsyncEpisodeHarness()
        result = harness.run(max_steps=100, max_duration=0.45)

        self.assertEqual(result["status"], "completed")
        self.assertFalse(result["episode"]["terminated"])
        self.assertTrue(result["episode"]["truncated"])
        self.assertEqual(
            result["episode"]["terminal_reason"], "max_episode_duration"
        )
        self.assertGreater(result["metrics"]["transition_count"], 0)

    def test_existing_failure_and_danger_signals_are_terminated(self):
        for terminal_kind, expected_reason, dangerous in (
            ("failure", "mission_failure", False),
            ("collision", "collision_proxy", True),
        ):
            harness = AsyncEpisodeHarness(
                terminal_after=4, terminal_kind=terminal_kind
            )
            result = harness.run(max_steps=20)
            self.assertTrue(result["episode"]["terminated"])
            self.assertFalse(result["episode"]["truncated"])
            self.assertIn(expected_reason, result["episode"]["terminal_reason"])
            terminal_steps = [
                step for step in result["steps"] if step["terminated"]
            ]
            self.assertTrue(terminal_steps)
            self.assertEqual(
                terminal_steps[-1]["transition"]["dangerous_terminal"],
                dangerous,
            )

    def test_missing_action_ack_times_out_fail_closed(self):
        harness = AsyncEpisodeHarness(acknowledge=False)
        result = harness.run(max_steps=3)

        self.assertEqual(result["status"], "timeout")
        self.assertEqual(result["metrics"]["timeout_count"], 1)
        self.assertEqual(result["metrics"]["transition_count"], 0)
        self.assertFalse(result["episode"]["terminated"])
        self.assertFalse(result["episode"]["truncated"])

    def test_reset_remains_explicitly_unavailable(self):
        clock = MutableRosClock(10.0)
        env = AstraDroneEnv(
            publish_requested_v_max=lambda _value: None,
            ros_clock=clock,
            reward=Stage1Reward(),
            config=AstraDroneStepConfig(),
        )
        with self.assertRaisesRegex(NotImplementedError, "does not implement"):
            env.reset()

    def test_episode_requires_exact_0p1_policy_period(self):
        with self.assertRaisesRegex(ValueError, "must be 0.1"):
            AstraDroneEpisodeConfig(policy_period_sec=0.2)
        with self.assertRaisesRegex(ValueError, "must be 0.1"):
            AstraDroneStepConfig(duration_sec=0.2)

    def test_policy_state_selection_rejects_one_nanosecond_future_receipt(self):
        harness = AsyncEpisodeHarness()
        future = harness.env.record_observation(
            harness._state(10.000000001),
            harness._context(10.000000001),
            harness._reward_observation(),
        )
        with harness.env._condition:
            selected = harness.env._latest_strictly_causal_observation_locked(10.0)
        self.assertIsNot(selected, future)
        self.assertLessEqual(
            selected.state.provenance.observation_receive_sec, 10.0
        )

    def test_wrong_request_id_is_not_consumed(self):
        harness = AsyncEpisodeHarness(acknowledge=False)
        state = harness.env._observations[-1]
        pending = harness.env._begin_pending_step(0.75, 10.0, 0, state)
        harness.env.record_action_stamped(
            stamp_sec=10.0,
            episode_id=pending.episode_id,
            step_index=0,
            request_id=99,
            source_mode="mock",
            requested_v_max=0.75,
            filtered_v_max=0.75,
            receive_sec=10.001,
        )
        with harness.env._condition:
            completed = harness.env._advance_pending_locked([pending], set())
        self.assertEqual(completed, [])
        self.assertIsNone(pending.action_event)

    def test_duplicate_applied_id_cannot_close_twice(self):
        harness = AsyncEpisodeHarness(acknowledge=False)
        state = harness.env._observations[-1]
        pending = harness.env._begin_pending_step(0.75, 10.0, 0, state)
        harness.env.record_action_stamped(
            stamp_sec=10.0,
            episode_id=pending.episode_id,
            step_index=0,
            request_id=1,
            source_mode="mock",
            requested_v_max=0.75,
            filtered_v_max=0.75,
            receive_sec=10.001,
        )
        first = harness.env.record_applied_stamped(
            stamp_sec=10.0,
            episode_id=pending.episode_id,
            step_index=0,
            request_id=1,
            applied_v_max=0.75,
            receive_sec=10.002,
        )
        duplicate = harness.env.record_applied_stamped(
            stamp_sec=10.0,
            episode_id=pending.episode_id,
            step_index=0,
            request_id=1,
            applied_v_max=0.75,
            receive_sec=10.003,
        )
        self.assertIsNotNone(first)
        self.assertIsNone(duplicate)
        self.assertEqual(harness.env._duplicate_applied_count, 1)

    def test_source_stamped_applied_before_action_fails_closed(self):
        harness = AsyncEpisodeHarness(acknowledge=False)
        state = harness.env._observations[-1]
        pending = harness.env._begin_pending_step(0.75, 10.0, 0, state)
        harness.env.record_applied_stamped(
            stamp_sec=9.999,
            episode_id=pending.episode_id,
            step_index=0,
            request_id=1,
            applied_v_max=0.75,
            receive_sec=10.001,
        )
        harness.env.record_action_stamped(
            stamp_sec=10.0,
            episode_id=pending.episode_id,
            step_index=0,
            request_id=1,
            source_mode="mock",
            requested_v_max=0.75,
            filtered_v_max=0.75,
            receive_sec=10.002,
        )
        with harness.env._condition:
            with self.assertRaisesRegex(
                Exception, "timestamp order is invalid"
            ):
                harness.env._advance_pending_locked([pending], set())

    def test_callback_reordering_preserves_valid_source_order(self):
        harness = AsyncEpisodeHarness(acknowledge=False)
        state = harness.env._observations[-1]
        pending = harness.env._begin_pending_step(0.75, 10.0, 0, state)
        harness.env.record_applied_stamped(
            stamp_sec=10.0001,
            episode_id=pending.episode_id,
            step_index=0,
            request_id=1,
            applied_v_max=0.75,
            receive_sec=10.002,
        )
        harness.env.record_action_stamped(
            stamp_sec=10.0,
            episode_id=pending.episode_id,
            step_index=0,
            request_id=1,
            source_mode="mock",
            requested_v_max=0.75,
            filtered_v_max=0.75,
            receive_sec=10.003,
        )
        with harness.env._condition:
            completed = harness.env._advance_pending_locked([pending], set())
        self.assertEqual(completed, [])
        self.assertIsNotNone(pending.action_event)
        self.assertIsNotNone(pending.applied_event)

    def test_future_action_timestamp_is_rejected(self):
        harness = AsyncEpisodeHarness(acknowledge=False)
        rejected = harness.env.record_action_stamped(
            stamp_sec=10.1,
            episode_id="episode_0001",
            step_index=0,
            request_id=1,
            source_mode="mock",
            requested_v_max=0.75,
            filtered_v_max=0.75,
            receive_sec=10.0,
        )
        self.assertIsNone(rejected)
        self.assertIn("future", harness.env._identity_violation)


if __name__ == "__main__":
    unittest.main()
