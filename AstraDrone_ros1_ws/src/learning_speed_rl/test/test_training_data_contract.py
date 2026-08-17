"""Frozen policy-state/transition and calibration-metric contracts."""

import unittest

import numpy as np

from learning_speed_rl.training import (
    DIAGNOSTIC_ONLY_FIELDS,
    POLICY_INPUT_FIELDS,
    AppliedSpeedAction,
    OfficialTrajectoryIdentity,
    PlannerFailureEpisodeTracker,
    PolicyStateProvenance,
    PolicyStateV1,
    SacTransitionV1,
    TrackingSafetyMirror,
    causal_observation_receipt_time,
    lidar_clutter_metrics,
)


class TrainingDataContractTest(unittest.TestCase):
    def _state(self, stamp, receive, trajectory=None):
        trajectory = trajectory or OfficialTrajectoryIdentity(7, 9.0, "uav1/camera_init")
        return PolicyStateV1(
            lidar_surrogate=np.ones(3200),
            future_positions_body=np.zeros((20, 3)),
            actual_velocity_body=[0.1, 0.0, 0.0],
            tracking_error_body=[0.01, -0.02, 0.03],
            previous_applied_v_max=0.12,
            provenance=PolicyStateProvenance(
                stamp, receive, "uav1/body", "scheme_c_trajectory_fusion_v1.0", trajectory
            ),
        )

    def test_policy_input_contains_exactly_five_fields(self):
        state = self._state(10.0, 10.01)
        self.assertEqual(tuple(state.policy_input().keys()), POLICY_INPUT_FIELDS)
        self.assertTrue(set(DIAGNOSTIC_ONLY_FIELDS).isdisjoint(state.policy_input()))

    def test_causal_receipt_survives_per_process_sim_clock_lag(self):
        self.assertAlmostEqual(
            causal_observation_receipt_time(11962.528, 11962.532, 11962.530),
            11962.532,
        )
        with self.assertRaises(ValueError):
            causal_observation_receipt_time(float("nan"), 1.0, 1.0)

    def test_shapes_are_frozen(self):
        with self.assertRaises(ValueError):
            PolicyStateV1(
                lidar_surrogate=np.ones(3199),
                future_positions_body=np.zeros((20, 3)),
                actual_velocity_body=np.zeros(3),
                tracking_error_body=np.zeros(3),
                previous_applied_v_max=0.1,
                provenance=self._state(10.0, 10.01).provenance,
            )
        with self.assertRaises(ValueError):
            PolicyStateV1(
                lidar_surrogate=np.ones(3200),
                future_positions_body=np.zeros((21, 3)),
                actual_velocity_body=np.zeros(3),
                tracking_error_body=np.zeros(3),
                previous_applied_v_max=0.1,
                provenance=self._state(10.0, 10.01).provenance,
            )

    def test_transition_is_causal_and_reward_stays_undefined(self):
        trajectory = OfficialTrajectoryIdentity(7, 9.0, "uav1/camera_init")
        transition = SacTransitionV1(
            state_t=self._state(10.0, 10.01, trajectory),
            action_t=AppliedSpeedAction(
                0.18, 10.02, 0.17, 10.03, 0.17, 10.04, trajectory
            ),
            state_t_plus_1=self._state(10.1, 10.11, trajectory),
            reward=None,
            reward_defined=False,
            terminated=False,
            truncated=False,
        )
        self.assertFalse(transition.training_ready)
        self.assertIsNone(transition.to_record()["reward"])

    def test_rejects_state_built_from_non_latest_official_bspline(self):
        old = OfficialTrajectoryIdentity(7, 9.0, "uav1/camera_init")
        latest = OfficialTrajectoryIdentity(8, 9.5, "uav1/camera_init")
        with self.assertRaisesRegex(ValueError, "latest official B-spline"):
            SacTransitionV1(
                state_t=self._state(10.0, 10.01, old),
                action_t=AppliedSpeedAction(
                    0.18, 10.02, 0.17, 10.03, 0.17, 10.04, latest
                ),
                state_t_plus_1=self._state(10.1, 10.11, latest),
                reward=None,
                reward_defined=False,
                terminated=False,
                truncated=False,
            )

    def test_rejects_future_leakage_and_noncausal_action_order(self):
        trajectory = OfficialTrajectoryIdentity(7, 9.0, "uav1/camera_init")
        with self.assertRaisesRegex(ValueError, "action timestamps"):
            AppliedSpeedAction(0.1, 10.2, 0.1, 10.1, 0.1, 10.3, trajectory)
        with self.assertRaisesRegex(ValueError, "received after action"):
            SacTransitionV1(
                state_t=self._state(10.0, 10.2, trajectory),
                action_t=AppliedSpeedAction(
                    0.1, 10.1, 0.1, 10.2, 0.1, 10.3, trajectory
                ),
                state_t_plus_1=self._state(10.4, 10.5, trajectory),
                reward=None,
                reward_defined=False,
                terminated=False,
                truncated=False,
            )

    def test_lidar_clutter_metrics_do_not_change_clearance_semantics(self):
        surrogate = np.asarray([2.0, 10.0, 13.0, 0.8])
        semantic = np.asarray([2, 1, 0, 2])
        metrics = lidar_clutter_metrics(surrogate, semantic)
        self.assertEqual(metrics["known_obstacle_bin_count"], 2)
        self.assertAlmostEqual(metrics["known_obstacle_bin_fraction"], 0.5)
        self.assertAlmostEqual(metrics["nearest_obstacle_distance_m"], 0.8)

    def test_lidar_clutter_metrics_accepts_rospy_uint8_bytes(self):
        surrogate = np.asarray([2.0, 10.0, 13.0, 0.8])
        metrics = lidar_clutter_metrics(surrogate, bytes([2, 1, 0, 2]))
        self.assertEqual(metrics["known_obstacle_bin_count"], 2)
        self.assertAlmostEqual(metrics["known_obstacle_bin_fraction"], 0.5)

    def test_planner_failure_episode_records_recovery(self):
        tracker = PlannerFailureEpisodeTracker()
        tracker.update(10.0, True, "REPLAN_FAILED", 1)
        tracker.update(10.1, True, "NO_FEASIBLE_TRAJECTORY", 3)
        tracker.update(10.2, False, "NONE", 0)
        self.assertEqual(len(tracker.completed), 1)
        self.assertTrue(tracker.completed[0].recovered)
        self.assertEqual(tracker.completed[0].maximum_consecutive_failures, 3)
        self.assertEqual(
            tracker.completed[0].reasons,
            ["REPLAN_FAILED", "NO_FEASIBLE_TRAJECTORY"],
        )

    def test_tracking_gate_mirror_uses_existing_one_meter_one_second_rule(self):
        mirror = TrackingSafetyMirror(1.0, 1.0)
        self.assertFalse(mirror.update(10.0, 1.01, "TRACK_EGO"))
        self.assertFalse(mirror.update(10.9, 1.01, "TRACK_EGO"))
        self.assertTrue(mirror.update(11.0, 1.01, "TRACK_EGO"))


if __name__ == "__main__":
    unittest.main()
