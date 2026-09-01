#!/usr/bin/env python3

import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from hector_ego_training_backend.episode_reset_contract import (
    EpisodeBinding,
    EpisodeIdentityLedger,
    RandomResetConfig,
    RandomResetSampler,
    ResetCandidate,
    ResetSamplingError,
    StaticObstacleXY,
    action_matches,
    classify_observation_staleness,
    episode_count_stop_due,
    fixed_terminal_hold_matches,
    observation_matches,
    route_acceptance_required,
    sac_closure_matches,
    sac_closure_reset_allowed,
    trajectory_matches,
    validate_reset_candidate,
)


def random_config(seed=1001, attempts=8, obstacles=()):
    return RandomResetConfig(
        enabled=True,
        center_x=0.0,
        center_y=0.0,
        x_min_offset=-1.0,
        x_max_offset=1.0,
        y_min_offset=-1.0,
        y_max_offset=1.0,
        z=3.0,
        yaw=0.0,
        seed=seed,
        max_sampling_attempts=attempts,
        uav_collision_radius_xy=0.395567,
        ego_obstacles_inflation=0.30,
        additional_static_clearance=0.50,
        static_obstacles=tuple(obstacles),
    )


class EpisodeResetContractTest(unittest.TestCase):
    def test_stale_recheck_distinguishes_recovery_source_and_producer(self):
        common = {
            "raw_lidar_present": True,
            "raw_lidar_receipt_age_sec": 0.10,
            "observation_v2_present": True,
            "observation_v2_valid": True,
            "observation_v2_receipt_age_sec": 0.01,
            "observation_freshness_sec": 0.35,
            "raw_lidar_freshness_sec": 0.35,
        }
        self.assertIsNone(
            classify_observation_staleness(
                observation_present=True,
                observation_receipt_age_sec=0.02,
                **common
            )
        )
        self.assertEqual(
            classify_observation_staleness(
                observation_present=True,
                observation_receipt_age_sec=0.36,
                **common
            ),
            "infrastructure:observation_c_producer_stall",
        )
        source_stale = dict(common)
        source_stale["raw_lidar_receipt_age_sec"] = 0.40
        self.assertEqual(
            classify_observation_staleness(
                observation_present=True,
                observation_receipt_age_sec=0.36,
                **source_stale
            ),
            "invalid_observation:source_stale",
        )

    def test_identity_advances_exactly_once_per_reset(self):
        ledger = EpisodeIdentityLedger()
        self.assertEqual(ledger.current, EpisodeBinding(1, 0))
        self.assertEqual(ledger.current.episode_key, "training_episode_000001")
        self.assertEqual(ledger.advance_after_reset(1), EpisodeBinding(2, 1))
        with self.assertRaises(ValueError):
            ledger.advance_after_reset(3)

    def test_action_requires_exact_episode_identity(self):
        binding = EpisodeBinding(7, 6)
        self.assertTrue(action_matches(binding, binding.episode_key, 0, 7))
        self.assertFalse(action_matches(binding, binding.episode_key, 1, 7))
        self.assertFalse(action_matches(binding, "training_episode_000006", 0, 7))

    def test_trajectory_requires_event_and_source_barriers(self):
        binding = EpisodeBinding(2, 1)
        self.assertTrue(
            trajectory_matches(binding, 12, 20.1, 101, 100, 20.0, 19.0, 11)
        )
        self.assertFalse(
            trajectory_matches(binding, 12, 20.1, 99, 100, 20.0, 19.0, 11)
        )
        self.assertFalse(
            trajectory_matches(binding, 11, 20.1, 101, 100, 20.0, 19.0, 11)
        )

    def test_sac_closure_requires_exact_episode_generation_and_identity(self):
        binding = EpisodeBinding(4, 3)
        payload = {
            "episode_id": binding.episode_key,
            "reset_generation": 3,
            "terminal_transition_closed": True,
            "transition_count": 101,
            "last_step_index": 100,
            "last_request_id": 101,
            "status": "completed",
            "scheduler_state": "CLOSED",
            "active_action_interval_count": 0,
            "actor_action_outstanding_count": 0,
            "terminal_row_count": 1,
            "replay_step_sequence_contiguous": True,
            "ordered_replay_writer_submitted_count": 101,
            "ordered_replay_writer_persisted_count": 101,
            "ordered_replay_writer_pending_count": 0,
        }
        self.assertTrue(sac_closure_matches(binding, payload))
        payload["reset_generation"] = 2
        self.assertFalse(sac_closure_matches(binding, payload))
        payload["reset_generation"] = 3
        payload["last_request_id"] = 100
        self.assertFalse(sac_closure_matches(binding, payload))
        payload["last_request_id"] = 101
        payload["active_action_interval_count"] = 1
        self.assertFalse(sac_closure_matches(binding, payload))
        payload["active_action_interval_count"] = 0
        payload["ordered_replay_writer_pending_count"] = 1
        self.assertFalse(sac_closure_matches(binding, payload))
        payload["ordered_replay_writer_pending_count"] = 0
        payload["ordered_replay_writer_persisted_count"] = 100
        self.assertFalse(sac_closure_matches(binding, payload))
        payload["ordered_replay_writer_persisted_count"] = 101
        payload["open_transition_count"] = payload.pop(
            "active_action_interval_count"
        )
        self.assertTrue(sac_closure_matches(binding, payload))

    def test_infrastructure_closure_forbids_reset_after_exactly_once_close(self):
        binding = EpisodeBinding(12, 11)
        payload = {
            "episode_id": binding.episode_key,
            "reset_generation": 11,
            "terminal_transition_closed": True,
            "transition_count": 1,
            "last_step_index": 0,
            "last_request_id": 1,
            "status": "completed",
            "scheduler_state": "CLOSED",
            "active_action_interval_count": 0,
            "actor_action_outstanding_count": 0,
            "terminal_row_count": 1,
            "replay_step_sequence_contiguous": True,
            "ordered_replay_writer_submitted_count": 1,
            "ordered_replay_writer_persisted_count": 1,
            "ordered_replay_writer_pending_count": 0,
            "fail_closed_after_closure": True,
            "fail_closed_reason": (
                "infrastructure:observation_c_producer_stall"
            ),
        }
        self.assertFalse(
            sac_closure_reset_allowed(
                binding,
                payload,
                "infrastructure:observation_c_producer_stall",
            )
        )
        payload["fail_closed_after_closure"] = False
        with self.assertRaisesRegex(ValueError, "exact fail-closed reason"):
            sac_closure_reset_allowed(
                binding,
                payload,
                "infrastructure:observation_c_producer_stall",
            )

    def test_ordinary_terminals_continue_after_valid_closure(self):
        binding = EpisodeBinding(4, 3)
        base = {
            "episode_id": binding.episode_key,
            "reset_generation": 3,
            "terminal_transition_closed": True,
            "transition_count": 101,
            "last_step_index": 100,
            "last_request_id": 101,
            "status": "completed",
            "scheduler_state": "CLOSED",
            "active_action_interval_count": 0,
            "actor_action_outstanding_count": 0,
            "terminal_row_count": 1,
            "replay_step_sequence_contiguous": True,
            "ordered_replay_writer_submitted_count": 101,
            "ordered_replay_writer_persisted_count": 101,
            "ordered_replay_writer_pending_count": 0,
            "fail_closed_after_closure": False,
            "fail_closed_reason": "",
        }
        for reason in (
            "collision",
            "planner_failure:NO_FEASIBLE_TRAJECTORY",
            "max_episode_time",
        ):
            with self.subTest(reason=reason):
                self.assertTrue(
                    sac_closure_reset_allowed(binding, base, reason)
                )

    def test_training_stops_exactly_at_completed_episode_10000(self):
        self.assertFalse(episode_count_stop_due(9999, 10000))
        self.assertTrue(episode_count_stop_due(10000, 10000))
        with self.assertRaises(ValueError):
            episode_count_stop_due(10001, 10000)

    def test_evaluation_still_uses_its_independent_episode_count(self):
        self.assertFalse(episode_count_stop_due(2, 3))
        self.assertTrue(episode_count_stop_due(3, 3))

    def test_map_switch_preflight_excludes_route_only_acceptance(self):
        self.assertFalse(route_acceptance_required("map_switch_preflight"))
        self.assertTrue(route_acceptance_required("training"))
        self.assertTrue(route_acceptance_required("qualification"))

        launch_text = (
            Path(__file__).resolve().parents[1]
            / "launch/hector_forest_sac_training.launch"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "arg('runner_mode') == 'map_switch_preflight'",
            launch_text,
        )

    def test_sac_launch_has_one_episode_stop_owner_and_no_transition_target(self):
        package = Path(__file__).resolve().parents[1]
        sac_launch = ET.parse(
            package / "launch/hector_worksite_sac_training.launch"
        ).getroot()
        arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in sac_launch.findall("arg")
        }
        self.assertEqual(arguments["total_training_episodes"], "10000")
        self.assertEqual(arguments["evaluation_episodes"], "100")
        self.assertEqual(arguments["v_max_min"], "0.30")
        self.assertEqual(arguments["v_max_max"], "1.75")
        self.assertEqual(arguments["max_episode_time"], "55.0")
        self.assertNotIn("target_valid_transitions", arguments)
        self.assertNotIn("max_training_episodes", arguments)

        nested = ET.parse(
            package / "launch/hector_training_episode_reset.launch"
        ).getroot()
        nested_arguments = {
            element.attrib["name"] for element in nested.findall("arg")
        }
        self.assertNotIn("target_valid_transitions", nested_arguments)
        self.assertNotIn("max_training_episodes", nested_arguments)

    def test_formal_vmax_bounds_reach_ego_and_speed_filter_launches(self):
        package = Path(__file__).resolve().parents[1]
        backend_arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in ET.parse(
                package / "launch/hector_ego_training_backend.launch"
            ).getroot().findall("arg")
        }
        self.assertEqual(backend_arguments["max_vel"], "4.00")
        self.assertEqual(backend_arguments["max_acc"], "3.00")

        episode_arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in ET.parse(
                package / "launch/hector_training_episode_reset.launch"
            ).getroot().findall("arg")
        }
        self.assertEqual(episode_arguments["static_max_vel"], "4.00")
        self.assertEqual(episode_arguments["static_max_acc"], "3.00")

        observation_launch = (
            package / "launch/hector_training_observation_c.launch"
        ).read_text(encoding="utf-8")
        speed_launch = (
            package.parents[1]
            / "learning_speed_rl/launch/speed_adapter.launch"
        ).read_text(encoding="utf-8")
        self.assertIn(
            '<arg name="max_vel" value="$(arg static_max_vel)"/>',
            observation_launch,
        )
        self.assertIn(
            '<arg name="max_acc" value="$(arg static_max_acc)"/>',
            observation_launch,
        )
        self.assertIn(
            '<arg name="dynamic_speed_limit_minimum" value="$(arg v_max_min)"/>',
            observation_launch,
        )
        self.assertIn(
            '<arg name="static_min_vel" value="$(arg v_max_min)"/>',
            observation_launch,
        )
        self.assertIn(
            '<param name="safety/v_max_min" value="$(arg static_min_vel)"/>',
            speed_launch,
        )
        self.assertIn(
            '<param name="safety/v_max_max" value="$(arg static_max_vel)"/>',
            speed_launch,
        )

        for launch_name in (
            "hector_worksite_sac_training.launch",
            "hector_forest_sac_training.launch",
        ):
            text = (package / "launch" / launch_name).read_text(encoding="utf-8")
            self.assertIn('<arg name="v_max_max" default="1.75"/>', text)
            self.assertIn('<arg name="static_max_vel" value="4.00"/>', text)
            self.assertIn('<arg name="static_max_acc" value="3.00"/>', text)

    def test_observation_requires_current_generation_and_trajectory(self):
        binding = EpisodeBinding(3, 2)
        self.assertTrue(observation_matches(binding, True, 2, 2, 30.0, 29.0, 8, 8))
        self.assertFalse(observation_matches(binding, True, 1, 2, 30.0, 29.0, 8, 8))
        self.assertFalse(observation_matches(binding, True, 2, 2, 28.0, 29.0, 8, 8))
        self.assertFalse(observation_matches(binding, True, 2, 2, 30.0, 29.0, 7, 8))

    def test_fixed_terminal_hold_requires_ended_matching_goal_command(self):
        values = {
            "observation_valid": False,
            "observation_diagnostics": ["trajectory_unavailable"],
            "trajectory_lookup_result": "no_active_trajectory_at_stamp",
            "observation_stamp_sec": 12.825,
            "observation_latest_trajectory_id": 11,
            "trajectory_id": 11,
            "trajectory_end_sec": 12.795,
            "position_command_trajectory_id": 11,
            "position_command_flag": 1,
            "position_command_age_sec": 0.005,
            "position_command_distance_to_goal": 0.079,
            "position_command_speed": 0.0,
            "actual_distance_to_goal": 0.077,
            "goal_position_tolerance": 0.25,
            "goal_speed_tolerance": 0.20,
            "freshness_limit_sec": 0.35,
        }
        self.assertTrue(fixed_terminal_hold_matches(**values))
        for key, invalid in (
            ("observation_diagnostics", ["trajectory_unavailable", "other"]),
            ("trajectory_lookup_result", "history_empty"),
            ("observation_stamp_sec", 12.700),
            ("observation_latest_trajectory_id", 10),
            ("position_command_trajectory_id", 10),
            ("position_command_flag", 3),
            ("position_command_age_sec", 0.36),
            ("position_command_distance_to_goal", 0.251),
            ("position_command_speed", 0.201),
            ("actual_distance_to_goal", 0.251),
        ):
            candidate = dict(values)
            candidate[key] = invalid
            self.assertFalse(
                fixed_terminal_hold_matches(**candidate), key
            )

    def test_random_reset_seed_reproduces_sequence_and_bounds(self):
        left = RandomResetSampler(random_config(seed=1001))
        right = RandomResetSampler(random_config(seed=1001))
        left_sequence = [left.sample() for _ in range(20)]
        right_sequence = [right.sample() for _ in range(20)]
        for left_sample, right_sample in zip(left_sequence, right_sequence):
            self.assertEqual(left_sample["candidate"], right_sample["candidate"])
            candidate = left_sample["candidate"]
            self.assertGreaterEqual(candidate.x, -1.0)
            self.assertLessEqual(candidate.x, 1.0)
            self.assertGreaterEqual(candidate.y, -1.0)
            self.assertLessEqual(candidate.y, 1.0)
            self.assertEqual(candidate.z, 3.0)
            self.assertEqual(candidate.yaw, 0.0)

    def test_unsafe_candidate_is_rejected_then_resampled(self):
        sampler = RandomResetSampler(random_config(attempts=4))
        calls = []

        def reject_first(candidate, _config):
            calls.append(candidate)
            return {
                "valid": len(calls) > 1,
                "reasons": [] if len(calls) > 1 else ["test_unsafe"],
            }

        sample = sampler.sample(validator=reject_first)
        self.assertEqual(sample["attempt_count"], 2)
        self.assertEqual(len(sample["sampling_attempts"]), 2)
        self.assertFalse(
            sample["sampling_attempts"][0]["validation"]["valid"]
        )

    def test_max_attempts_fail_closed(self):
        sampler = RandomResetSampler(random_config(attempts=3))

        def reject_all(_candidate, _config):
            return {"valid": False, "reasons": ["test_unsafe"]}

        with self.assertRaises(ResetSamplingError) as context:
            sampler.sample(validator=reject_all)
        self.assertEqual(len(context.exception.attempts), 3)

    def test_worksite_clearance_rejects_unsafe_static_candidate(self):
        obstacle = StaticObstacleXY("test_tree", 0.0, 0.0, 10.0)
        sampler = RandomResetSampler(
            random_config(attempts=2, obstacles=(obstacle,))
        )
        with self.assertRaises(ResetSamplingError):
            sampler.sample()

    def test_random_sampling_does_not_change_episode_generation(self):
        ledger = EpisodeIdentityLedger()
        sampler = RandomResetSampler(random_config())
        sampler.sample()
        self.assertEqual(ledger.current, EpisodeBinding(1, 0))
        self.assertEqual(ledger.advance_after_reset(1), EpisodeBinding(2, 1))

    def test_worksite_profile_has_safe_complete_square(self):
        path = Path(__file__).resolve().parents[1] / "config/worksite_training_reset.yaml"
        values = yaml.safe_load(path.read_text(encoding="utf-8"))["random_start"]
        obstacles = tuple(
            StaticObstacleXY(
                item["name"], item["x"], item["y"], item["radius_at_hover_z"]
            )
            for item in values["static_obstacles"]
        )
        config = RandomResetConfig(
            enabled=True,
            center_x=values["center_x"],
            center_y=values["center_y"],
            x_min_offset=values["x_min_offset"],
            x_max_offset=values["x_max_offset"],
            y_min_offset=values["y_min_offset"],
            y_max_offset=values["y_max_offset"],
            z=values["z"],
            yaw=values["yaw"],
            seed=values["seed"],
            max_sampling_attempts=values["max_sampling_attempts"],
            uav_collision_radius_xy=values["uav_collision_radius_xy"],
            ego_obstacles_inflation=values["ego_obstacles_inflation"],
            additional_static_clearance=values["additional_static_clearance"],
            static_obstacles=obstacles,
        )
        minimum = float("inf")
        for x in (-1.0, 1.0):
            for y in (-1.0, 1.0):
                result = validate_reset_candidate(
                    ResetCandidate(x, y, 3.0, 0.0), config
                )
                self.assertTrue(result["valid"], result)
                minimum = min(minimum, result["minimum_static_clearance_m"])
        self.assertGreater(minimum, 1.33)


if __name__ == "__main__":
    unittest.main()
