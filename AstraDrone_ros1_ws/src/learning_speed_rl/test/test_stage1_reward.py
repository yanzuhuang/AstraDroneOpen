"""Unit coverage for the reviewed Stage 1 reward boundary."""

from collections import Counter
from dataclasses import fields
import importlib.util
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
import yaml

from learning_speed_rl.training import (
    AppliedSpeedAction,
    OfficialTrajectoryIdentity,
    PolicyStateProvenance,
    PolicyStateV1,
    ProgressContextState,
    ProgressRewardContext,
    RunEpisodeProvenance,
    SacTransitionV1,
    Stage1Reward,
    Stage1RewardConfig,
    Stage1RewardInput,
    actual_speed_mps_from_body_velocity,
    lidar_clutter_metrics,
    reward_from_config,
    stage1_reward_input_from_signals,
)


class Stage1RewardTest(unittest.TestCase):
    def setUp(self):
        self.reward = Stage1Reward()

    @staticmethod
    def value(
        nearest=4.25,
        density=0.060,
        unknown=False,
        applied=1.25,
        previous=1.25,
        actual=1.0,
        dangerous=False,
        terminated=False,
        valid=True,
        same_episode=True,
        truncated=False,
    ):
        return Stage1RewardInput(
            nearest_obstacle_distance_m=nearest,
            known_obstacle_bin_fraction=density,
            unknown_majority=unknown,
            applied_v_max_mps=applied,
            previous_applied_v_max_mps=previous,
            actual_speed_mps=actual,
            dangerous_terminal=dangerous,
            terminated=terminated,
            observation_valid=valid,
            same_episode=same_episode,
            truncated=truncated,
        )

    def test_low_medium_high_unknown_typical_inputs(self):
        cases = (
            (self.value(nearest=8.0, density=0.02, applied=1.75), "Low", 1.75, 0.0),
            (self.value(), "Medium", 1.25, 0.5),
            (
                self.value(nearest=2.0, density=0.03, applied=0.75),
                "High",
                0.75,
                1.0,
            ),
            (
                self.value(nearest=None, density=0.0, unknown=True),
                "Unknown",
                1.25,
                0.5,
            ),
        )
        for value, label, phi_1, phi_2 in cases:
            result = self.reward.evaluate(value)
            self.assertTrue(result.reward_valid)
            self.assertEqual(result.complexity_context.label, label)
            self.assertAlmostEqual(result.phi_1, phi_1)
            self.assertAlmostEqual(result.phi_2, phi_2)

    def test_candidate_c_boundaries_are_inclusive(self):
        low = self.reward.evaluate(self.value(nearest=6.0, density=0.040))
        near_high = self.reward.evaluate(self.value(nearest=2.5, density=0.040))
        density_high = self.reward.evaluate(self.value(nearest=8.0, density=0.080))
        self.assertEqual(low.complexity_context.label, "Low")
        self.assertEqual(near_high.complexity_context.label, "High")
        self.assertEqual(density_high.complexity_context.label, "High")
        self.assertEqual((low.phi_2, near_high.phi_2, density_high.phi_2), (0.0, 1.0, 1.0))

    def test_nearest_na_unknown_and_non_unknown_are_distinct(self):
        unknown = self.reward.evaluate(
            self.value(nearest=None, density=0.0, unknown=True)
        )
        observed_open = self.reward.evaluate(
            self.value(nearest=None, density=0.0, unknown=False)
        )
        self.assertEqual(unknown.complexity_context.label, "Unknown")
        self.assertEqual(observed_open.complexity_context.label, "Low")
        self.assertNotEqual(unknown.phi_2, observed_open.phi_2)
        with self.assertRaisesRegex(ValueError, "inconsistent"):
            self.value(nearest=None, density=0.01, unknown=False)

    def test_speed_increase_and_decrease_have_symmetric_weak_smoothing(self):
        increase = self.reward.evaluate(self.value(applied=1.5, previous=1.0))
        decrease = self.reward.evaluate(self.value(applied=1.0, previous=1.5))
        self.assertAlmostEqual(increase.reward_smoothing, -0.025)
        self.assertAlmostEqual(decrease.reward_smoothing, -0.025)
        self.assertLess(abs(increase.reward_smoothing), abs(increase.reward_speed))

    def test_dangerous_terminal_uses_actual_speed_squared(self):
        slow = self.reward.evaluate(
            self.value(actual=0.5, dangerous=True, terminated=True)
        )
        fast = self.reward.evaluate(
            self.value(actual=2.0, dangerous=True, terminated=True)
        )
        self.assertAlmostEqual(slow.reward_danger, -0.5)
        self.assertAlmostEqual(fast.reward_danger, -8.0)
        self.assertLess(fast.reward_total, slow.reward_total)
        with self.assertRaisesRegex(ValueError, "must terminate"):
            self.value(dangerous=True, terminated=False)

    def test_non_dangerous_terminal_has_no_danger_penalty(self):
        ordinary_failure = self.reward.evaluate(
            self.value(terminated=True, dangerous=False)
        )
        self.assertTrue(ordinary_failure.reward_valid)
        self.assertEqual(ordinary_failure.reward_danger, 0.0)

    def test_invalid_observation_episode_boundary_and_truncation_are_rejected(self):
        invalid = self.reward.evaluate(self.value(valid=False))
        boundary = self.reward.evaluate(self.value(same_episode=False))
        truncated = self.reward.evaluate(self.value(truncated=True))
        self.assertEqual(invalid.invalid_reason, "invalid_observation")
        self.assertEqual(boundary.invalid_reason, "episode_boundary")
        self.assertEqual(truncated.invalid_reason, "truncated_transition")
        self.assertIsNone(invalid.reward_total)

    def test_reward_is_finite_over_calibrated_domain(self):
        for nearest in (None, 0.75, 2.5, 4.25, 6.0, 10.0):
            for density in (0.0, 0.04, 0.06, 0.08, 0.21):
                if nearest is None and density != 0.0:
                    continue
                for speed in (0.05, 0.75, 1.25, 1.75, 4.0):
                    result = self.reward.evaluate(
                        self.value(
                            nearest=nearest,
                            density=density,
                            unknown=nearest is None,
                            applied=speed,
                            previous=1.25,
                            actual=speed,
                        )
                    )
                    self.assertTrue(math.isfinite(result.reward_total))

    def test_online_and_offline_signal_conversion_produce_identical_reward(self):
        surrogate = np.full(3200, 13.0, dtype=np.float32)
        semantic = np.zeros(3200, dtype=np.uint8)
        semantic[:128] = 2
        surrogate[:128] = np.linspace(3.0, 5.0, 128)
        metrics = lidar_clutter_metrics(surrogate, semantic)
        common = dict(
            nearest_obstacle_distance_m=metrics["nearest_obstacle_distance_m"],
            known_obstacle_bin_fraction=metrics["known_obstacle_bin_fraction"],
            unknown_bin_count=metrics["unknown_bin_count"],
            lidar_bin_count=3200,
            applied_v_max_mps=1.5,
            previous_applied_v_max_mps=1.25,
            actual_speed_mps=actual_speed_mps_from_body_velocity(
                [1.2, -0.3, 0.4]
            ),
            dangerous_terminal=False,
            terminated=False,
            observation_valid=True,
            same_episode=True,
            truncated=False,
        )
        online = stage1_reward_input_from_signals(**common)
        replay = stage1_reward_input_from_signals(
            **dict(
                common,
                nearest_obstacle_distance_m=float(
                    "{:.17g}".format(metrics["nearest_obstacle_distance_m"])
                ),
                known_obstacle_bin_fraction=float(
                    "{:.17g}".format(metrics["known_obstacle_bin_fraction"])
                ),
                unknown_bin_count=int(str(metrics["unknown_bin_count"])),
            )
        )
        self.assertEqual(online, replay)
        self.assertEqual(
            self.reward.evaluate(online).to_record(),
            self.reward.evaluate(replay).to_record(),
        )

    def test_stage1_yaml_selects_the_same_reviewed_implementation(self):
        config_path = Path(__file__).resolve().parents[1] / "config/stage1_reward.yaml"
        values = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        configured = reward_from_config(values)
        self.assertIsInstance(configured, Stage1Reward)
        self.assertEqual(configured.config, Stage1RewardConfig())

    def test_reward_input_cannot_read_tracking_or_future_state(self):
        names = {field.name for field in fields(Stage1RewardInput)}
        self.assertNotIn("tracking_error", names)
        self.assertNotIn("tracking_error_body", names)
        self.assertNotIn("state_t_plus_1", names)
        self.assertNotIn("future_state", names)

    def test_eq10_blend_is_continuous_at_phi_thresholds(self):
        config = Stage1RewardConfig()
        for threshold in (config.lambda_phi_2, 0.5, config.lambda_phi_1):
            values = []
            for offset in (-1.0e-7, 1.0e-7):
                risk = threshold + offset
                density = config.density_safe + risk * (
                    config.density_dangerous - config.density_safe
                )
                values.append(
                    self.reward.evaluate(
                        self.value(nearest=10.0, density=density, applied=1.5)
                    ).reward_speed
                )
            self.assertLess(abs(values[1] - values[0]), 1.0e-5)

    def test_paper_lambda_relation_is_enforced(self):
        with self.assertRaisesRegex(ValueError, "lambda_speed_2"):
            Stage1RewardConfig(lambda_speed_2=0.25, lambda_speed_3=0.25)

    @staticmethod
    def _state(stamp, receive, trajectory, tracking, velocity=(1.0, 0.0, 0.0)):
        return PolicyStateV1(
            lidar_surrogate=np.ones(3200, dtype=np.float32),
            future_positions_body=np.zeros((20, 3), dtype=np.float32),
            actual_velocity_body=np.asarray(velocity),
            tracking_error_body=np.asarray(tracking, dtype=np.float32),
            previous_applied_v_max=1.25,
            provenance=PolicyStateProvenance(
                observation_stamp_sec=stamp,
                observation_receive_sec=receive,
                body_frame="uav1/body",
                observation_version="scheme_c_trajectory_fusion_v1.0",
                official_trajectory=trajectory,
            ),
        )

    def test_valid_reward_binds_to_contract_with_components(self):
        trajectory = OfficialTrajectoryIdentity(1, 9.0, "uav1/camera_init")
        state_t = self._state(10.0, 10.01, trajectory, [0.0, 0.0, 0.0])
        state_t_plus_1 = self._state(10.1, 10.11, trajectory, [9.0, 9.0, 9.0])
        context_t = ProgressContextState(0.1, 10.0, "NAVIGATING", 10.0, 10.01)
        context_next = ProgressContextState(0.2, 10.1, "NAVIGATING", 10.1, 10.11)
        transition = SacTransitionV1(
            state_t=state_t,
            action_t=AppliedSpeedAction(1.25, 10.02, 1.25, 10.03, 1.25, 10.04, trajectory),
            state_t_plus_1=state_t_plus_1,
            reward_context=ProgressRewardContext(
                RunEpisodeProvenance("run", "episode"), context_t, context_next
            ),
            reward=None,
            reward_defined=False,
            terminated=False,
            truncated=False,
        )
        evaluation = self.reward.evaluate(self.value())
        defined = transition.with_defined_reward(evaluation)
        self.assertTrue(defined.training_ready)
        self.assertEqual(defined.reward, evaluation.reward_total)
        self.assertIn("complexity_context", defined.reward_components)
        self.assertNotIn("reward_tracking", defined.reward_components)

    def test_recorder_writes_finite_reward_matching_offline_evaluation(self):
        package_root = Path(__file__).resolve().parents[1]
        recorder_path = package_root / "scripts/calibration_data_recorder.py"
        spec = importlib.util.spec_from_file_location(
            "online_reward_recorder_test_module", str(recorder_path)
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        trajectory = OfficialTrajectoryIdentity(1, 9.0, "uav1/camera_init")
        state_t = self._state(
            10.0,
            10.01,
            trajectory,
            [0.0, 0.0, 0.0],
            velocity=[1.2, -0.3, 0.4],
        )
        state_t_plus_1 = self._state(
            10.1, 10.11, trajectory, [0.0, 0.0, 0.0]
        )
        context_t = ProgressContextState(0.1, 10.0, "NAVIGATING", 10.0, 10.01)
        context_next = ProgressContextState(
            0.2, 10.1, "NAVIGATING", 10.1, 10.11
        )
        recorder = module.CalibrationDataRecorder.__new__(
            module.CalibrationDataRecorder
        )
        recorder._run_episode = RunEpisodeProvenance("run", "episode")
        recorder._reward = self.reward
        recorder._mission_done = False
        recorder._collision_terminal = True
        recorder._emergency_terminal = False
        recorder._tracking_mirror = SimpleNamespace(triggered=False)
        recorder._transition_stream = io.StringIO()
        recorder._transition_candidates = 0
        recorder._reward_defined_transitions = 0
        recorder._reward_invalid_reasons = Counter()
        recorder._transition_negative_delta_p = 0
        recorder._transition_terminal_written = False
        recorder._pending_transition = {
            "state": state_t,
            "reward_context": context_t,
            "reward_observation": {
                "nearest_obstacle_distance_m": 4.25,
                "known_obstacle_bin_fraction": 0.06,
                "unknown_bin_count": 400,
            },
            "latest_official_trajectory": trajectory,
            "requested": (10.02, 1.5),
            "filtered": (10.03, 1.5),
            "applied": (10.04, 1.5),
        }

        recorder._maybe_write_transition(state_t_plus_1, context_next)
        online = json.loads(recorder._transition_stream.getvalue())
        offline_input = stage1_reward_input_from_signals(
            nearest_obstacle_distance_m=4.25,
            known_obstacle_bin_fraction=0.06,
            unknown_bin_count=400,
            lidar_bin_count=3200,
            applied_v_max_mps=1.5,
            previous_applied_v_max_mps=state_t.previous_applied_v_max,
            actual_speed_mps=actual_speed_mps_from_body_velocity(
                state_t.actual_velocity_body
            ),
            dangerous_terminal=True,
            terminated=True,
        )
        offline = self.reward.evaluate(offline_input)

        self.assertTrue(online["reward_defined"])
        self.assertTrue(online["training_ready"])
        self.assertTrue(math.isfinite(online["reward"]))
        self.assertEqual(online["reward"], offline.reward_total)
        self.assertEqual(online["reward_components"], offline.to_record())
        self.assertAlmostEqual(online["reward_components"]["reward_smoothing"], -0.00625)
        expected_danger = -2.0 * (
            actual_speed_mps_from_body_velocity(state_t.actual_velocity_body) ** 2
        )
        self.assertEqual(
            online["reward_components"]["reward_danger"], expected_danger
        )
        self.assertEqual(recorder._reward_defined_transitions, 1)
        self.assertTrue(recorder._transition_terminal_written)


if __name__ == "__main__":
    unittest.main()
