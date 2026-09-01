"""Unit/offline coverage for Episode v0.1 and its 10 Hz causal scheduler."""

import math
import threading
import time
import unittest

import numpy as np

from learning_speed_rl.training import (
    ActorActionOwnership,
    AstraDroneEnv,
    AstraDroneEpisodeConfig,
    AstraDroneStepConfig,
    OfficialTrajectoryIdentity,
    PolicyStateProvenance,
    PolicyStateV1,
    ProgressContextState,
    LearningSpeedReward,
    OrderedTransitionWriter,
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
        self,
        *,
        acknowledge=True,
        terminal_after=None,
        terminal_kind="success",
        previous_applied_v_max=0.75,
        force_replan_on_first_action=False,
        advance_after_force_replan=False,
        keep_pre_replan_selected_trajectory=False,
        observation_period_sec=0.1,
        duplicate_terminal_callback=False,
        trajectory_end_sec=100.0,
        terminal_after_trajectory_end=False,
        replacement_trajectory_at_sec=None,
    ):
        self.clock = MutableRosClock(10.0)
        self.trajectory = OfficialTrajectoryIdentity(
            7, 9.0, "uav1/camera_init"
        )
        self.acknowledge = acknowledge
        self.terminal_after = terminal_after
        self.terminal_kind = terminal_kind
        self.previous_applied_v_max = float(previous_applied_v_max)
        self.force_replan_on_first_action = bool(force_replan_on_first_action)
        self.advance_after_force_replan = bool(advance_after_force_replan)
        self.keep_pre_replan_selected_trajectory = bool(
            keep_pre_replan_selected_trajectory
        )
        self.latest_lookup_trajectory = self.trajectory
        self.observation_period_sec = float(observation_period_sec)
        self.duplicate_terminal_callback = bool(duplicate_terminal_callback)
        self.trajectory_end_sec = float(trajectory_end_sec)
        self.terminal_after_trajectory_end = bool(
            terminal_after_trajectory_end
        )
        self.replacement_trajectory_at_sec = (
            None
            if replacement_trajectory_at_sec is None
            else float(replacement_trajectory_at_sec)
        )
        self._trajectory_end_terminal_latched = False
        self._replacement_trajectory_published = False
        self.published = []
        self.stop = threading.Event()
        self.suppress_observations = threading.Event()
        self.terminal_receipt_sec = None
        self.env = AstraDroneEnv(
            publish_requested_v_max=self.publish,
            ros_clock=self.clock,
            reward=LearningSpeedReward(),
            config=AstraDroneStepConfig(
                action_timeout_sec=0.05,
                applied_timeout_sec=0.05,
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
            previous_applied_v_max=self.previous_applied_v_max,
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
        return self.env.record_observation(
            self._state(stamp),
            self._context(stamp),
            self._reward_observation(),
            telemetry={
                "selected_trajectory_id": self.trajectory.trajectory_id,
                "selected_trajectory_start_stamp_sec": (
                    self.trajectory.start_time_sec
                ),
                "selected_trajectory_start_stamp_secs": (
                    self.trajectory.start_time_secs
                ),
                "selected_trajectory_start_stamp_nsecs": (
                    self.trajectory.start_time_nsecs
                ),
                "selected_trajectory_end_stamp_sec": self.trajectory_end_sec,
                "latest_trajectory_id_at_lookup": (
                    self.latest_lookup_trajectory.trajectory_id
                ),
                "latest_trajectory_start_stamp_sec": (
                    self.latest_lookup_trajectory.start_time_sec
                ),
            },
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
        if self.force_replan_on_first_action and len(self.published) == 1:
            pre_replan_trajectory = self.trajectory
            post_replan_trajectory = OfficialTrajectoryIdentity(
                self.trajectory.trajectory_id + 1,
                request.publish_ros_time_sec + 0.010,
                self.trajectory.source_frame,
            )
            self.env.record_official_trajectory(
                post_replan_trajectory,
                receive_sec=now + 0.020,
            )
            self.latest_lookup_trajectory = post_replan_trajectory
            self.trajectory = (
                pre_replan_trajectory
                if self.keep_pre_replan_selected_trajectory
                else post_replan_trajectory
            )
            if self.advance_after_force_replan:
                advanced_trajectory = OfficialTrajectoryIdentity(
                    post_replan_trajectory.trajectory_id + 1,
                    request.publish_ros_time_sec + 0.030,
                    post_replan_trajectory.source_frame,
                )
                self.env.record_official_trajectory(
                    advanced_trajectory,
                    receive_sec=now + 0.040,
                )
                self.latest_lookup_trajectory = advanced_trajectory
                if not self.keep_pre_replan_selected_trajectory:
                    self.trajectory = advanced_trajectory
        if self.terminal_after is not None and len(self.published) == self.terminal_after:
            if self.terminal_kind == "success":
                self.terminal_receipt_sec = now + 0.025
                self.env.record_mission_success(True, receive_sec=now + 0.025)
                self.env.record_mission_done(True, receive_sec=now + 0.025)
            elif self.terminal_kind in ("failure", "planner_failure", "stale"):
                self.terminal_receipt_sec = now + 0.025
                self.env.record_mission_failure(True, receive_sec=now + 0.025)
                self.env.record_mission_done(True, receive_sec=now + 0.025)
            elif self.terminal_kind == "collision":
                self.terminal_receipt_sec = now + 0.025
                self.env.record_planner_safety(
                    current_position_in_collision=True,
                    emergency_stop_active=False,
                    planner_state="EXEC_TRAJ",
                    receive_sec=now + 0.025,
                )
            else:
                raise AssertionError("unsupported test terminal kind")
            if self.duplicate_terminal_callback:
                if self.terminal_kind == "success":
                    self.env.record_mission_success(True, receive_sec=now + 0.026)
                    self.env.record_mission_done(True, receive_sec=now + 0.026)
                else:
                    self.env.record_mission_failure(True, receive_sec=now + 0.026)
                    self.env.record_mission_done(True, receive_sec=now + 0.026)

    def _pump(self):
        next_observation = 10.05
        while not self.stop.is_set():
            now = self.clock.advance(0.01)
            if (
                self.replacement_trajectory_at_sec is not None
                and not self._replacement_trajectory_published
                and now + 1.0e-12 >= self.replacement_trajectory_at_sec
            ):
                replacement = OfficialTrajectoryIdentity(
                    self.trajectory.trajectory_id + 1,
                    self.replacement_trajectory_at_sec,
                    self.trajectory.source_frame,
                )
                self.env.record_official_trajectory(
                    replacement, receive_sec=now
                )
                self.trajectory = replacement
                self.latest_lookup_trajectory = replacement
                self.trajectory_end_sec = now + 10.0
                self._replacement_trajectory_published = True
            if (
                self.terminal_after_trajectory_end
                and now > self.trajectory_end_sec
            ):
                self.suppress_observations.set()
                if (
                    not self._trajectory_end_terminal_latched
                    and now + 1.0e-12 >= self.trajectory_end_sec + 0.5
                ):
                    self.terminal_receipt_sec = now
                    self.env.record_mission_failure(True, receive_sec=now)
                    self.env.record_mission_done(True, receive_sec=now)
                    self._trajectory_end_terminal_latched = True
            while now + 1.0e-12 >= next_observation:
                if not self.suppress_observations.is_set():
                    self._record_observation(next_observation)
                next_observation += self.observation_period_sec
            time.sleep(0.001)

    def run(
        self,
        *,
        max_steps=8,
        max_duration=None,
        max_steps_reason="max_episode_steps",
    ):
        self.thread.start()
        try:
            return self.env.run_episode(
                lambda _state, _index: 0.75,
                AstraDroneEpisodeConfig(
                    max_steps=max_steps,
                    max_duration_sec=max_duration,
                    max_steps_reason=max_steps_reason,
                    start_timeout_sec=0.1,
                    completion_timeout_sec=0.2,
                    deadline_tolerance_sec=0.025,
                ),
            )
        finally:
            self.stop.set()
            self.thread.join(timeout=1.0)


class AstraDroneEnvEpisodeTest(unittest.TestCase):
    def test_trajectory_identity_uses_exact_native_ros_stamp(self):
        first = OfficialTrajectoryIdentity(
            42, 12.345678899, "uav1/camera_init", 12, 345678899
        )
        same = OfficialTrajectoryIdentity(
            42, 12.345678899, "uav1/camera_init", 12, 345678899
        )
        next_nanosecond = OfficialTrajectoryIdentity(
            42, 12.345678900, "uav1/camera_init", 12, 345678900
        )
        self.assertEqual(first.identity_key, same.identity_key)
        self.assertNotEqual(first.identity_key, next_nanosecond.identity_key)
        self.assertLess(first.identity_key, next_nanosecond.identity_key)

    @staticmethod
    def _restore_active_episode(harness, observation_stamp=10.0):
        harness.env.record_mission_state(
            "TRAINING_EPISODE_ACTIVE", receive_sec=observation_stamp - 0.003
        )
        harness.env.record_mission_progress(
            0.0, receive_sec=observation_stamp - 0.002
        )
        harness.env.record_bridge_state("TRACK_EGO")
        harness.env.record_planner_safety(
            current_position_in_collision=False,
            emergency_stop_active=False,
            planner_state="EXEC_TRAJ",
            receive_sec=observation_stamp - 0.001,
        )
        harness._record_observation(observation_stamp)

    @staticmethod
    def _run_with_actor_ownership(
        harness,
        *,
        max_steps=20,
        provider_hook=None,
        acceptance_hook=None,
    ):
        ownership = ActorActionOwnership()
        committed = []

        def action_provider(_state, step_index):
            ownership.propose(step_index, 0.0)
            if provider_hook is not None:
                provider_hook(step_index)
            return 0.75

        def action_acceptance_recorder(step_index, accepted):
            if acceptance_hook is not None:
                acceptance_hook(step_index, accepted)
            ownership.resolve_acceptance(step_index, accepted)

        def transition_submitter(record):
            ownership.consume_transition(record["step_index"])
            committed.append(record)

        harness.thread.start()
        try:
            result = harness.env.run_episode(
                action_provider,
                AstraDroneEpisodeConfig(
                    max_steps=max_steps,
                    start_timeout_sec=0.1,
                    completion_timeout_sec=0.2,
                    deadline_tolerance_sec=0.025,
                ),
                transition_submitter=transition_submitter,
                action_acceptance_recorder=action_acceptance_recorder,
            )
        finally:
            harness.stop.set()
            harness.thread.join(timeout=1.0)
        ownership.require_closed()
        return result, committed, ownership

    def test_external_episode_first_action_uses_coordinator_fresh_trajectory(self):
        harness = AsyncEpisodeHarness()
        fresh = OfficialTrajectoryIdentity(8, 9.8, "uav1/camera_init")
        harness.env.record_official_trajectory(fresh, receive_sec=9.9)
        harness.trajectory = fresh

        harness.env.begin_external_episode(
            "forest_smoke",
            "training_episode_000001",
            1,
            official_trajectory_id=fresh.trajectory_id,
            official_trajectory_start_time=fresh.start_time_sec,
            official_trajectory_start_secs=fresh.start_time_secs,
            official_trajectory_start_nsecs=fresh.start_time_nsecs,
        )
        self._restore_active_episode(harness)
        result = harness.run(max_steps=1)

        self.assertEqual(result["status"], "completed")
        provenance = result["steps"][0]["transition"]["action_t"][
            "latest_official_trajectory"
        ]
        self.assertEqual(provenance["trajectory_id"], fresh.trajectory_id)
        self.assertAlmostEqual(provenance["start_time_sec"], fresh.start_time_sec)

    def test_force_replan_trajectory_is_diagnostic_not_action_pairing(self):
        harness = AsyncEpisodeHarness(
            previous_applied_v_max=1.25,
            force_replan_on_first_action=True,
            advance_after_force_replan=True,
        )
        result = harness.run(max_steps=1)

        self.assertEqual(result["status"], "completed")
        step = result["steps"][0]
        self.assertTrue(step["timing"]["force_replan_expected"])
        self.assertEqual(
            step["timing"]["policy_trajectory_role"],
            "POLICY_INPUT",
        )
        self.assertEqual(
            step["transition"]["action_t"]["latest_official_trajectory"][
                "trajectory_id"
            ],
            7,
        )
        self.assertEqual(
            step["timing"]["planner_diagnostic_trajectory_id"], 8
        )
        self.assertEqual(
            step["timing"]["planner_provenance_role"], "DIAGNOSTIC"
        )
        self.assertEqual(step["timing"]["planner_diagnostic_status"], "observed")

    def test_missing_planner_diagnostic_does_not_miss_tick(self):
        harness = AsyncEpisodeHarness(previous_applied_v_max=1.25)
        result = harness.run(max_steps=2)

        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["metrics"]["actor_action_count"], 2)
        self.assertEqual(result["metrics"]["transition_count"], 2)
        self.assertEqual(
            result["metrics"]["accepted_policy_tick_indices"], [0, 1]
        )
        self.assertEqual(
            result["steps"][0]["timing"]["planner_diagnostic_status"],
            "pending",
        )

    def test_force_replan_diagnostic_does_not_replace_policy_trajectory(self):
        harness = AsyncEpisodeHarness(
            previous_applied_v_max=1.25,
            force_replan_on_first_action=True,
            keep_pre_replan_selected_trajectory=True,
        )
        result = harness.run(max_steps=1)

        self.assertEqual(result["status"], "completed")
        step = result["steps"][0]
        self.assertTrue(step["timing"]["force_replan_expected"])
        self.assertEqual(
            step["timing"]["planner_diagnostic_status"],
            "observed",
        )
        self.assertEqual(
            step["transition"]["state_t_plus_1"]["provenance"][
                "official_trajectory"
            ]["trajectory_id"],
            7,
        )
        self.assertGreater(
            step["timing"]["state_t_plus_1_stamp_sec"],
            step["timing"]["action_stamp_sec"],
        )

    def test_external_episode_rejects_old_generation_trajectory(self):
        harness = AsyncEpisodeHarness()
        expected = OfficialTrajectoryIdentity(8, 10.1, "uav1/camera_init")
        harness.env.begin_external_episode(
            "forest_smoke",
            "training_episode_000002",
            2,
            official_trajectory_id=expected.trajectory_id,
            official_trajectory_start_time=expected.start_time_sec,
            official_trajectory_start_secs=expected.start_time_secs,
            official_trajectory_start_nsecs=expected.start_time_nsecs,
        )
        harness.trajectory = expected
        self._restore_active_episode(harness, observation_stamp=10.2)

        with harness.env._condition:
            self.assertFalse(harness.env._episode_ready_locked())
        stale = harness.env.record_official_trajectory(
            OfficialTrajectoryIdentity(7, 9.0, "uav1/camera_init"),
            receive_sec=10.21,
        )
        self.assertIsNone(stale)
        with harness.env._condition:
            self.assertFalse(harness.env._episode_ready_locked())

        accepted = harness.env.record_official_trajectory(
            expected, receive_sec=10.22
        )
        self.assertIsNotNone(accepted)
        with harness.env._condition:
            self.assertTrue(harness.env._episode_ready_locked())

    def test_sequential_scheduler_closes_unique_causal_transitions(self):
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
        self.assertEqual(
            result["metrics"]["maximum_active_action_interval_count"], 1
        )
        self.assertEqual(
            result["metrics"]["active_action_interval_count_at_closure"], 0
        )
        self.assertEqual(result["metrics"]["scheduler_state"], "CLOSED")
        self.assertEqual(result["metrics"]["nominal_decision_period_sec"], 0.1)
        self.assertEqual(result["metrics"]["actor_action_count"], 8)
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
                timing["applied_receive_ros_time_sec"],
            )
            self.assertEqual(
                timing["timing_contract"],
                "absolute_action_interval_async_replay_v2",
            )
            self.assertEqual(timing["close_trigger"], "policy_tick")
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
        scheduled = [
            step["scheduled_ros_time_sec"] for step in result["steps"]
        ]
        self.assertTrue(
            all(
                abs(later - earlier - 0.1) <= 1.0e-9
                for earlier, later in zip(scheduled, scheduled[1:])
            )
        )
        self.assertLessEqual(
            result["metrics"]["effective_policy_rate_hz"], 10.0 + 1.0e-9
        )
        self.assertTrue(
            result["metrics"]["policy_tick_indices_strictly_monotonic"]
        )
        self.assertEqual(
            result["metrics"]["accepted_policy_tick_indices"], list(range(8))
        )
        # The next state was already available at the fixed boundary; there is
        # no action_stamp + 0.1 post-action hold gate anymore.
        self.assertLess(
            result["steps"][0]["timing"]["state_t_plus_1_receive_sec"],
            result["steps"][0]["timing"]["action_stamp_sec"] + 0.1,
        )

    def test_slow_ordered_writer_does_not_block_actor_action_cadence(self):
        harness = AsyncEpisodeHarness()
        ownership = ActorActionOwnership()
        persisted = []

        def persist(record, _payload):
            time.sleep(0.08)
            persisted.append(int(record["step_index"]))

        writer = OrderedTransitionWriter(persist)

        def action_provider(_state, step_index):
            ownership.propose(step_index, 0.0)
            return 0.75

        def accept(step_index, accepted):
            ownership.resolve_acceptance(step_index, accepted)

        def handoff(record):
            ownership.consume_transition(record["step_index"])
            writer.submit(record, {"reset_generation": 1})

        harness.thread.start()
        try:
            result = harness.env.run_episode(
                action_provider,
                AstraDroneEpisodeConfig(
                    max_steps=5,
                    start_timeout_sec=0.1,
                    completion_timeout_sec=0.2,
                ),
                transition_submitter=handoff,
                action_acceptance_recorder=accept,
            )
            writer.close(timeout_sec=2.0)
        finally:
            harness.stop.set()
            harness.thread.join(timeout=1.0)

        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["metrics"]["accepted_policy_tick_indices"], list(range(5)))
        self.assertEqual(persisted, list(range(5)))
        self.assertGreater(writer.maximum_pending_count, 1)
        scheduled = [
            step["scheduled_ros_time_sec"] for step in result["steps"]
        ]
        self.assertTrue(
            all(
                abs(later - earlier - 0.1) <= 1.0e-9
                for earlier, later in zip(scheduled, scheduled[1:])
            )
        )
        ownership.require_closed()

    def test_normal_two_step_order_is_form_submit_then_next_action(self):
        harness = AsyncEpisodeHarness()
        committed = []
        harness.thread.start()
        try:
            result = harness.env.run_episode(
                lambda _state, _index: 0.75,
                AstraDroneEpisodeConfig(
                    max_steps=2,
                    start_timeout_sec=0.1,
                    completion_timeout_sec=0.2,
                ),
                transition_submitter=lambda record: committed.append(
                    int(record["step_index"])
                ),
            )
        finally:
            harness.stop.set()
            harness.thread.join(timeout=1.0)
        self.assertEqual(committed, [0, 1])
        self.assertEqual([item[1].step_index for item in harness.published], [0, 1])

    def test_candidate_terminal_before_accept_is_discarded_without_new_step(self):
        harness = AsyncEpisodeHarness()

        def terminal_before_accept(step_index):
            if step_index == 1:
                harness.env.record_mission_failure(
                    True, receive_sec=harness.clock()
                )

        result, committed, ownership = self._run_with_actor_ownership(
            harness,
            provider_hook=terminal_before_accept,
        )
        self.assertEqual(result["status"], "completed")
        self.assertEqual(len(harness.published), 1)
        self.assertEqual(len(committed), 1)
        self.assertEqual(result["metrics"]["actor_action_count"], 1)
        self.assertEqual(result["metrics"]["transition_count"], 1)
        self.assertEqual(result["steps"][0]["step_index"], 0)
        self.assertTrue(result["steps"][0]["terminated"])
        self.assertIn("mission_failure", result["steps"][0]["terminal_reason"])
        self.assertEqual(ownership.candidate_count, 0)
        self.assertEqual(ownership.active_interval_count, 0)

    def test_external_truncation_before_accept_is_not_promoted_to_terminated(self):
        harness = AsyncEpisodeHarness()

        def truncate_before_accept(step_index):
            if step_index == 1:
                harness.env.record_external_truncation(
                    "max_episode_time", receive_sec=harness.clock()
                )

        result, committed, ownership = self._run_with_actor_ownership(
            harness,
            provider_hook=truncate_before_accept,
        )
        self.assertEqual(result["status"], "completed")
        self.assertEqual(len(harness.published), 1)
        self.assertEqual(len(committed), 1)
        self.assertFalse(result["episode"]["terminated"])
        self.assertTrue(result["episode"]["truncated"])
        self.assertEqual(result["episode"]["terminal_reason"], "max_episode_time")
        ownership.require_closed()

    def test_terminal_after_accept_closes_the_accepted_action(self):
        harness = AsyncEpisodeHarness(
            terminal_after=1,
            terminal_kind="planner_failure",
        )
        result, committed, ownership = self._run_with_actor_ownership(harness)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(len(harness.published), 1)
        self.assertEqual(len(committed), 1)
        self.assertTrue(committed[0]["terminated"])
        self.assertEqual(
            sum(bool(step["terminated"]) for step in result["steps"]), 1
        )
        self.assertGreaterEqual(
            committed[0]["timing"]["state_t_plus_1_receive_sec"],
            harness.terminal_receipt_sec,
        )
        self.assertEqual(ownership.active_interval_count, 0)

    def test_max_episode_time_closes_from_real_causal_pre_latch_state(self):
        harness = AsyncEpisodeHarness()
        truncation = {}

        def truncate_after_accept(step_index, accepted):
            if step_index != 0 or not accepted:
                return
            now = harness.clock()
            packet = harness._record_observation(now + 0.010)
            truncation["observation_receive_sec"] = (
                packet.state.provenance.observation_receive_sec
            )
            truncation["receipt_sec"] = now + 0.020
            harness.suppress_observations.set()
            harness.env.record_external_truncation(
                "max_episode_time",
                receive_sec=truncation["receipt_sec"],
            )
            # A repeated coordinator callback must not create another row.
            harness.env.record_external_truncation(
                "max_episode_time",
                receive_sec=truncation["receipt_sec"] + 0.001,
            )

        result, committed, ownership = self._run_with_actor_ownership(
            harness,
            acceptance_hook=truncate_after_accept,
        )

        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["metrics"]["actor_action_count"], 1)
        self.assertEqual(result["metrics"]["accepted_applied_ack_count"], 1)
        self.assertEqual(result["metrics"]["transition_count"], 1)
        self.assertTrue(result["metrics"]["identity_chain_strict_1_to_1"])
        self.assertEqual(
            result["metrics"]["maximum_active_action_interval_count"], 1
        )
        self.assertEqual(
            result["metrics"]["active_action_interval_count_at_closure"], 0
        )
        self.assertEqual(result["metrics"]["timeout_count"], 0)
        self.assertFalse(result["episode"]["terminated"])
        self.assertTrue(result["episode"]["truncated"])
        self.assertEqual(result["episode"]["terminal_reason"], "max_episode_time")
        self.assertEqual(len(committed), 1)
        final = committed[0]
        self.assertFalse(final["terminated"])
        self.assertTrue(final["truncated"])
        self.assertEqual(final["terminal_reason"], "max_episode_time")
        self.assertEqual(final["timing"]["close_trigger"], "external_truncation")
        self.assertEqual(
            final["timing"]["planner_diagnostic_status"],
            "not_expected",
        )
        self.assertGreaterEqual(
            final["timing"]["state_t_plus_1_receive_sec"],
            final["timing"]["applied_receive_ros_time_sec"],
        )
        self.assertLess(
            final["timing"]["state_t_plus_1_receive_sec"],
            truncation["receipt_sec"],
        )
        self.assertEqual(
            final["timing"]["state_t_plus_1_receive_sec"],
            truncation["observation_receive_sec"],
        )
        ownership.require_closed()

    def test_terminal_action_cycle_thread_orderings_are_deterministic(self):
        for iteration in range(5):
            with self.subTest(order="terminal_before_accept", iteration=iteration):
                harness = AsyncEpisodeHarness()

                def before_hook(step_index):
                    if step_index != 1:
                        return
                    worker = threading.Thread(
                        target=lambda: harness.env.record_mission_failure(
                            True, receive_sec=harness.clock()
                        )
                    )
                    worker.start()
                    worker.join(timeout=1.0)
                    self.assertFalse(worker.is_alive())

                result, committed, ownership = self._run_with_actor_ownership(
                    harness,
                    provider_hook=before_hook,
                )
                self.assertEqual(len(harness.published), 1)
                self.assertEqual(len(committed), 1)
                self.assertTrue(result["steps"][-1]["terminated"])
                ownership.require_closed()

            with self.subTest(order="terminal_after_accept", iteration=iteration):
                harness = AsyncEpisodeHarness()

                def after_hook(step_index, accepted):
                    if step_index != 0 or not accepted:
                        return
                    worker = threading.Thread(
                        target=lambda: harness.env.record_mission_failure(
                            True, receive_sec=harness.clock()
                        )
                    )
                    worker.start()
                    worker.join(timeout=1.0)
                    self.assertFalse(worker.is_alive())

                result, committed, ownership = self._run_with_actor_ownership(
                    harness,
                    acceptance_hook=after_hook,
                )
                self.assertEqual(len(harness.published), 1)
                self.assertEqual(len(committed), 1)
                self.assertTrue(result["steps"][-1]["terminated"])
                ownership.require_closed()

    def test_terminal_sources_use_one_actor_ownership_contract(self):
        for terminal_kind in (
            "success",
            "collision",
            "planner_failure",
            "stale",
        ):
            with self.subTest(terminal_kind=terminal_kind):
                harness = AsyncEpisodeHarness(
                    terminal_after=1,
                    terminal_kind=terminal_kind,
                )
                result, committed, ownership = self._run_with_actor_ownership(
                    harness
                )
                self.assertEqual(len(committed), 1)
                self.assertEqual(len(harness.published), 1)
                self.assertEqual(
                    sum(bool(step["terminated"]) for step in result["steps"]),
                    1,
                )
                ownership.require_closed()

    def test_repeated_terminal_callback_leaves_no_actor_ownership(self):
        harness = AsyncEpisodeHarness(
            terminal_after=1,
            terminal_kind="planner_failure",
            duplicate_terminal_callback=True,
        )
        result, committed, ownership = self._run_with_actor_ownership(harness)
        self.assertEqual(len(committed), 1)
        self.assertEqual(
            sum(bool(step["terminated"]) for step in result["steps"]), 1
        )
        ownership.require_closed()

    def test_trajectory_end_gate_prevents_active_interval_deadlock(self):
        harness = AsyncEpisodeHarness(
            trajectory_end_sec=10.16,
            terminal_after_trajectory_end=True,
        )
        result, committed, ownership = self._run_with_actor_ownership(
            harness, max_steps=20
        )

        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["metrics"]["actor_action_count"], 1)
        self.assertEqual(result["metrics"]["accepted_applied_ack_count"], 1)
        self.assertEqual(result["metrics"]["transition_count"], 1)
        self.assertEqual(
            result["metrics"]["maximum_active_action_interval_count"], 1
        )
        self.assertEqual(
            result["metrics"]["active_action_interval_count_at_closure"], 0
        )
        self.assertEqual(result["metrics"]["timeout_count"], 0)
        self.assertGreater(
            result["metrics"]["trajectory_lifetime_skip_count"], 0
        )
        self.assertEqual(
            result["metrics"]["policy_tick_miss_reasons"].get(
                "official_trajectory_lifetime_insufficient", 0
            ),
            result["metrics"]["trajectory_lifetime_skip_count"],
        )
        self.assertTrue(result["episode"]["terminated"])
        self.assertFalse(result["episode"]["truncated"])
        self.assertEqual(
            sum(bool(step["terminated"]) for step in result["steps"]), 1
        )
        self.assertEqual(len(committed), 1)
        self.assertEqual(ownership.candidate_count, 0)
        self.assertEqual(ownership.active_interval_count, 0)
        self.assertEqual(len(harness.published), 1)

    def test_trajectory_lifetime_skip_waits_for_new_official_without_catch_up(self):
        harness = AsyncEpisodeHarness(
            trajectory_end_sec=10.16,
            replacement_trajectory_at_sec=10.25,
        )
        result = harness.run(max_steps=2)

        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["metrics"]["actor_action_count"], 2)
        self.assertEqual(result["metrics"]["transition_count"], 2)
        self.assertGreater(
            result["metrics"]["trajectory_lifetime_skip_count"], 0
        )
        tick_indices = result["metrics"]["accepted_policy_tick_indices"]
        self.assertEqual(tick_indices[0], 0)
        self.assertGreaterEqual(tick_indices[1], 3)
        self.assertEqual(
            [step["step_index"] for step in result["steps"]], [0, 1]
        )
        self.assertEqual(
            [item[1].request_id for item in harness.published], [1, 2]
        )
        self.assertEqual(
            result["metrics"]["active_action_interval_count_at_closure"], 0
        )

    def test_acceptance_linearization_rechecks_lifetime_after_actor_delay(self):
        harness = AsyncEpisodeHarness(trajectory_end_sec=10.15)
        with harness.env._condition:
            state = harness.env._observations[-1]
        harness.clock.advance(0.08)

        with self.assertRaisesRegex(
            RuntimeError, "official_trajectory_lifetime_insufficient"
        ):
            harness.env._begin_active_action_interval(
                0.75,
                10.0,
                0,
                state,
                minimum_trajectory_lifetime_sec=0.1,
            )
        self.assertEqual(harness.published, [])

    def test_delayed_observation_misses_without_overlapping_active_interval(self):
        harness = AsyncEpisodeHarness(observation_period_sec=0.25)
        result = harness.run(max_steps=3)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["metrics"]["actor_action_count"], 3)
        self.assertEqual(result["metrics"]["transition_count"], 3)
        self.assertGreater(result["metrics"]["decision_tick_miss_count"], 0)
        self.assertGreater(result["metrics"]["observation_wait_count"], 0)
        self.assertEqual(
            result["metrics"]["maximum_active_action_interval_count"], 1
        )
        self.assertEqual(
            [step["step_index"] for step in result["steps"]], [0, 1, 2]
        )
        tick_indices = [step["policy_tick_index"] for step in result["steps"]]
        self.assertEqual(tick_indices[0], 0)
        self.assertTrue(
            all(later > earlier for earlier, later in zip(tick_indices, tick_indices[1:]))
        )
        self.assertNotEqual(tick_indices, [0, 1, 2])
        for step in result["steps"]:
            self.assertAlmostEqual(
                step["scheduled_ros_time_sec"],
                result["episode"]["episode_start_time"]
                + 0.1 * step["policy_tick_index"],
                places=6,
            )
        self.assertGreater(
            result["metrics"]["policy_tick_miss_reasons"].get(
                "causal_snapshot_unavailable", 0
            ),
            0,
        )

    def test_terminal_sources_share_exactly_once_sequential_closure(self):
        for terminal_kind, expected_reason in (
            ("collision", "collision_proxy"),
            ("planner_failure", "mission_failure"),
            ("stale", "mission_failure"),
            ("success", "mission_success"),
        ):
            with self.subTest(terminal_kind=terminal_kind):
                harness = AsyncEpisodeHarness(
                    terminal_after=3,
                    terminal_kind=terminal_kind,
                )
                result = harness.run(max_steps=20)
                self.assertEqual(result["status"], "completed")
                self.assertEqual(result["metrics"]["actor_action_count"], 3)
                self.assertEqual(result["metrics"]["transition_count"], 3)
                self.assertEqual(
                    sum(bool(step["terminated"]) for step in result["steps"]),
                    1,
                )
                self.assertIn(expected_reason, result["episode"]["terminal_reason"])
                self.assertEqual(
                    result["metrics"]["active_action_interval_count_at_closure"], 0
                )

    def test_repeated_terminal_callback_does_not_duplicate_terminal_row(self):
        harness = AsyncEpisodeHarness(
            terminal_after=3,
            terminal_kind="failure",
            duplicate_terminal_callback=True,
        )
        result = harness.run(max_steps=20)
        self.assertEqual(result["metrics"]["actor_action_count"], 3)
        self.assertEqual(
            sum(bool(step["terminated"]) for step in result["steps"]), 1
        )

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
            reward=LearningSpeedReward(),
            config=AstraDroneStepConfig(),
        )
        with self.assertRaisesRegex(NotImplementedError, "does not implement"):
            env.reset()

    def test_episode_requires_exact_0p1_policy_period(self):
        with self.assertRaisesRegex(ValueError, "must be 0.1"):
            AstraDroneEpisodeConfig(policy_period_sec=0.2)

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
        active_interval = harness.env._begin_active_action_interval(
            0.75, 10.0, 0, state
        )
        harness.env.record_action_stamped(
            stamp_sec=10.0,
            episode_id=active_interval.episode_id,
            step_index=0,
            request_id=99,
            source_mode="mock",
            requested_v_max=0.75,
            filtered_v_max=0.75,
            receive_sec=10.001,
        )
        with harness.env._condition:
            completed = harness.env._advance_active_action_interval_locked(
                active_interval
            )
        self.assertIsNone(completed)
        self.assertIsNone(active_interval.action_event)

    def test_duplicate_applied_id_cannot_close_twice(self):
        harness = AsyncEpisodeHarness(acknowledge=False)
        state = harness.env._observations[-1]
        active_interval = harness.env._begin_active_action_interval(
            0.75, 10.0, 0, state
        )
        harness.env.record_action_stamped(
            stamp_sec=10.0,
            episode_id=active_interval.episode_id,
            step_index=0,
            request_id=1,
            source_mode="mock",
            requested_v_max=0.75,
            filtered_v_max=0.75,
            receive_sec=10.001,
        )
        first = harness.env.record_applied_stamped(
            stamp_sec=10.0,
            episode_id=active_interval.episode_id,
            step_index=0,
            request_id=1,
            applied_v_max=0.75,
            receive_sec=10.002,
        )
        duplicate = harness.env.record_applied_stamped(
            stamp_sec=10.0,
            episode_id=active_interval.episode_id,
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
        active_interval = harness.env._begin_active_action_interval(
            0.75, 10.0, 0, state
        )
        harness.env.record_applied_stamped(
            stamp_sec=9.999,
            episode_id=active_interval.episode_id,
            step_index=0,
            request_id=1,
            applied_v_max=0.75,
            receive_sec=10.001,
        )
        harness.env.record_action_stamped(
            stamp_sec=10.0,
            episode_id=active_interval.episode_id,
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
                harness.env._advance_active_action_interval_locked(
                    active_interval
                )

    def test_callback_reordering_preserves_valid_source_order(self):
        harness = AsyncEpisodeHarness(acknowledge=False)
        state = harness.env._observations[-1]
        active_interval = harness.env._begin_active_action_interval(
            0.75, 10.0, 0, state
        )
        harness.env.record_applied_stamped(
            stamp_sec=10.0001,
            episode_id=active_interval.episode_id,
            step_index=0,
            request_id=1,
            applied_v_max=0.75,
            receive_sec=10.002,
        )
        harness.env.record_action_stamped(
            stamp_sec=10.0,
            episode_id=active_interval.episode_id,
            step_index=0,
            request_id=1,
            source_mode="mock",
            requested_v_max=0.75,
            filtered_v_max=0.75,
            receive_sec=10.003,
        )
        with harness.env._condition:
            completed = harness.env._advance_active_action_interval_locked(
                active_interval
            )
        self.assertIsNone(completed)
        self.assertIsNotNone(active_interval.action_event)
        self.assertIsNotNone(active_interval.applied_event)

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
