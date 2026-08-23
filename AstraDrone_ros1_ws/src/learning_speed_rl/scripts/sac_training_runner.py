#!/usr/bin/env python3
"""Training-only SAC owner layered on AstraDroneEnv and reset coordinator."""

import json
import math
import os
from pathlib import Path
import statistics
import threading
import time

import numpy as np
import rospy
from std_msgs.msg import Float64, String

from learning_speed_rl.training import AstraDroneEnv, AstraDroneEpisodeConfig
from learning_speed_rl.training.sac import LearnerWorker, SacAgent, SacConfig
from learning_speed_rl.training.sac_replay import (
    ActionMapping,
    SacReplayBuffer,
    flatten_policy_input,
)


SCHEMA_VERSION = "astradrone_sac_training_runtime_v1.0"


def _artifact_directory(value):
    path = Path(str(value)).expanduser().resolve()
    repository_root = next(
        (
            parent
            for parent in Path(__file__).resolve().parents
            if (parent / "runtime_artifacts").is_dir()
            and (parent / "AstraDrone_ros1_ws").is_dir()
        ),
        None,
    )
    if repository_root is None:
        raise rospy.ROSInitException("AstraDroneOpen root was not found")
    artifact_root = (repository_root / "runtime_artifacts").resolve()
    if path != artifact_root and artifact_root not in path.parents:
        raise rospy.ROSInitException(
            "SAC output must be below {}".format(artifact_root)
        )
    path.mkdir(parents=True, exist_ok=True)
    return path


def _write_json(path, value):
    temporary = str(path) + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    os.replace(temporary, str(path))


def _finite_metrics(metrics, names):
    for metric in metrics:
        for name in names:
            value = metric.get(name)
            if value is not None and not math.isfinite(float(value)):
                return False
    return True


class SacTrainingRunner:
    def __init__(self):
        self.output_dir = _artifact_directory(rospy.get_param("~output_dir"))
        self.run_id = str(rospy.get_param("~run_id", "sac_runtime_smoke"))
        self.mode = str(
            rospy.get_param("~training/mode", "qualification")
        ).strip().lower()
        if self.mode not in ("qualification", "pilot", "evaluation"):
            raise rospy.ROSInitException(
                "training/mode must be qualification, pilot or evaluation"
            )
        self.target_valid_transitions = int(
            rospy.get_param("~training/target_valid_transitions", 10000)
        )
        self.learning_starts = int(
            rospy.get_param("~training/learning_starts", 1000)
        )
        self.warmup_action_sampling = str(
            rospy.get_param(
                "~training/warmup_action_sampling",
                "deterministic_actor_mean",
            )
        ).strip()
        self.checkpoint_steps = sorted(
            set(
                int(value)
                for value in rospy.get_param(
                    "~training/checkpoint_steps",
                    [0, 2000, 4000, 6000, 8000, 10000],
                )
            )
        )
        if self.target_valid_transitions <= 0 or self.learning_starts <= 0:
            raise rospy.ROSInitException(
                "pilot target and learning_starts must be positive"
            )
        if self.learning_starts >= self.target_valid_transitions:
            raise rospy.ROSInitException(
                "learning_starts must be below the pilot target"
            )
        if self.warmup_action_sampling != "deterministic_actor_mean":
            raise rospy.ROSInitException(
                "formal v1 only supports deterministic_actor_mean warm-up"
            )
        self.expected_episode_count = int(
            rospy.get_param("~qualification/episode_count", 5)
        )
        self.phase_a_min_transitions = int(
            rospy.get_param("~qualification/phase_a_min_transitions", 1000)
        )
        self.phase_a_deterministic_actor = bool(
            rospy.get_param(
                "~qualification/phase_a_deterministic_actor", True
            )
        )
        self.phase_b_min_updates = int(
            rospy.get_param("~qualification/phase_b_min_updates", 100)
        )
        self.phase_b_update_timeout_wall = float(
            rospy.get_param("~qualification/phase_b_update_timeout_wall", 90.0)
        )
        self.stochastic_min_transitions = int(
            rospy.get_param(
                "~qualification/stochastic_min_transitions", 500
            )
        )
        self.stochastic_action_std_max = float(
            rospy.get_param(
                "~qualification/stochastic_action_std_max", 0.45
            )
        )
        self.stochastic_boundary_fraction_max = float(
            rospy.get_param(
                "~qualification/stochastic_boundary_fraction_max", 0.01
            )
        )
        self.stochastic_force_replan_delta_max = int(
            rospy.get_param(
                "~qualification/stochastic_force_replan_delta_max", 0
            )
        )
        if self.expected_episode_count <= 0 or (
            self.mode == "qualification"
            and min(self.phase_a_min_transitions, self.phase_b_min_updates) <= 0
        ):
            raise rospy.ROSInitException("qualification counts must be positive")
        if (
            self.stochastic_min_transitions <= 0
            or self.stochastic_action_std_max <= 0.0
            or not 0.0 <= self.stochastic_boundary_fraction_max <= 1.0
            or self.stochastic_force_replan_delta_max < 0
        ):
            raise rospy.ROSInitException(
                "qualification action-stability thresholds are invalid"
            )

        self.evaluation_checkpoint_path = str(
            rospy.get_param("~evaluation/checkpoint_path", "")
        ).strip()
        self.evaluation_checkpoint_step = int(
            rospy.get_param("~evaluation/checkpoint_step", 0)
        )
        self.q_abs_limit = float(
            rospy.get_param("~numerical_safety/q_abs_limit", 1000.0)
        )
        self.q_consecutive_limit = int(
            rospy.get_param("~numerical_safety/q_consecutive_limit", 5)
        )
        self.gradient_abs_limit = float(
            rospy.get_param("~numerical_safety/gradient_abs_limit", 100000.0)
        )
        self.loss_abs_limit = float(
            rospy.get_param("~numerical_safety/loss_abs_limit", 100000000.0)
        )
        if min(
            self.q_abs_limit,
            self.q_consecutive_limit,
            self.gradient_abs_limit,
            self.loss_abs_limit,
        ) <= 0:
            raise rospy.ROSInitException("numerical safety thresholds are invalid")
        self._checked_learner_metrics = 0
        self._consecutive_q_limit = 0
        self._checkpoint_manifest = []
        self._pending_checkpoint_steps = set()
        self._pilot_learner_enabled = False
        self._evaluation_transition_count = 0
        self._current_episode_audits = []

        self._condition = threading.Condition(threading.RLock())
        self._identity = None
        self._backend = {}
        self._current_env_episode = ""
        self._processed_episodes = set()
        self._shutdown = False

        expected_min = float(rospy.get_param("~action/expected_v_max_min"))
        expected_max = float(rospy.get_param("~action/expected_v_max_max"))
        parameter_deadline = time.monotonic() + 30.0
        while (
            not rospy.has_param("/uav1/speed_adapter/safety/v_max_min")
            or not rospy.has_param("/uav1/speed_adapter/safety/v_max_max")
        ) and time.monotonic() < parameter_deadline and not rospy.is_shutdown():
            time.sleep(0.05)
        if (
            not rospy.has_param("/uav1/speed_adapter/safety/v_max_min")
            or not rospy.has_param("/uav1/speed_adapter/safety/v_max_max")
        ):
            raise rospy.ROSInitException("live SpeedSafetyFilter bounds unavailable")
        live_min = float(rospy.get_param("/uav1/speed_adapter/safety/v_max_min"))
        live_max = float(rospy.get_param("/uav1/speed_adapter/safety/v_max_max"))
        if abs(live_min - expected_min) > 1.0e-12 or abs(
            live_max - expected_max
        ) > 1.0e-12:
            raise rospy.ROSInitException(
                "live SpeedSafetyFilter bounds disagree with SAC config: "
                "[{:.6f}, {:.6f}] != [{:.6f}, {:.6f}]".format(
                    live_min, live_max, expected_min, expected_max
                )
            )
        self.mapping = ActionMapping(live_min, live_max)

        config = SacConfig(
            observation_dim=int(rospy.get_param("~sac/observation_dim", 3267)),
            action_dim=int(rospy.get_param("~sac/action_dim", 1)),
            hidden_dim=int(rospy.get_param("~sac/hidden_dim", 256)),
            gamma=float(rospy.get_param("~sac/gamma", 0.99)),
            tau=float(rospy.get_param("~sac/tau", 0.005)),
            batch_size=int(rospy.get_param("~sac/batch_size", 64)),
            policy_learning_rate=float(
                rospy.get_param("~sac/policy_learning_rate", 3.0e-4)
            ),
            critic_learning_rate=float(
                rospy.get_param("~sac/critic_learning_rate", 1.0e-3)
            ),
            alpha_learning_rate=float(
                rospy.get_param("~sac/alpha_learning_rate", 1.0e-3)
            ),
            policy_frequency=int(
                rospy.get_param("~sac/policy_frequency", 2)
            ),
            critic_warmup_updates=int(
                rospy.get_param("~sac/critic_warmup_updates", 100)
            ),
            target_network_frequency=int(
                rospy.get_param("~sac/target_network_frequency", 1)
            ),
            log_std_min=float(rospy.get_param("~sac/log_std_min", -5.0)),
            log_std_max=float(rospy.get_param("~sac/log_std_max", 2.0)),
            automatic_entropy_tuning=bool(
                rospy.get_param("~sac/automatic_entropy_tuning", True)
            ),
            initial_alpha=float(rospy.get_param("~sac/initial_alpha", 0.2)),
            target_entropy=float(rospy.get_param("~sac/target_entropy", -1.0)),
            device=str(rospy.get_param("~sac/device", "cpu")),
            torch_num_threads=int(
                rospy.get_param("~sac/torch_num_threads", 1)
            ),
            seed=int(rospy.get_param("~sac/seed", 1)),
            learner_side_normalization=bool(
                rospy.get_param("~sac/learner_side_normalization", False)
            ),
        )
        self.agent = SacAgent(config)
        self.replay = None
        self.learner = None
        if self.mode != "evaluation":
            self.replay = SacReplayBuffer(
                int(rospy.get_param("~replay/capacity", 5000)),
                self.mapping,
                intervention_tolerance_mps=float(
                    rospy.get_param(
                        "~replay/intervention_tolerance_mps", 0.005
                    )
                ),
                initial_allocation=int(
                    rospy.get_param("~replay/initial_allocation", 5000)
                ),
            )
            self.learner = LearnerWorker(
                self.agent,
                self.replay,
                float(rospy.get_param("~sac/updates_per_second", 5.0)),
                seed=config.seed + 1,
            )
            self.learner.start()
        elif self.evaluation_checkpoint_path:
            checkpoint = Path(self.evaluation_checkpoint_path).resolve()
            if not checkpoint.is_file():
                raise rospy.ROSInitException(
                    "evaluation checkpoint does not exist: {}".format(checkpoint)
                )
            payload = self.agent.load_checkpoint(checkpoint)
            if self.agent.global_environment_step != self.evaluation_checkpoint_step:
                raise rospy.ROSInitException(
                    "evaluation checkpoint step does not match requested step"
                )

        max_steps = int(rospy.get_param("~episode/max_steps", 0))
        self.episode_config = AstraDroneEpisodeConfig(
            policy_period_sec=float(
                rospy.get_param("~episode/policy_period_sec", 0.1)
            ),
            max_steps=(None if max_steps <= 0 else max_steps),
            max_duration_sec=float(
                rospy.get_param("~episode/max_duration_sec", 80.0)
            ),
            start_timeout_sec=float(
                rospy.get_param("~episode/start_timeout_sec", 30.0)
            ),
            completion_timeout_sec=float(
                rospy.get_param("~episode/completion_timeout_sec", 30.0)
            ),
            deadline_tolerance_sec=float(
                rospy.get_param("~episode/deadline_tolerance_sec", 0.02)
            ),
            maximum_in_flight=int(
                rospy.get_param("~episode/maximum_in_flight", 16)
            ),
            minimum_active_speed_mps=float(
                rospy.get_param("~episode/minimum_active_speed_mps", 0.05)
            ),
        )
        self.env = AstraDroneEnv.from_ros_params()
        self._closure_pub = rospy.Publisher(
            "/uav1/learning_speed/sac_episode_closed",
            String,
            queue_size=10,
        )
        self._identity_sub = rospy.Subscriber(
            "/uav1/training/episode_identity",
            String,
            self._identity_callback,
            queue_size=20,
        )
        self._backend_sub = rospy.Subscriber(
            "/uav1/position_command_to_hector/backend_state",
            String,
            self._backend_callback,
            queue_size=20,
        )
        self._tracking_sub = rospy.Subscriber(
            "/uav1/position_command_to_hector/position_tracking_error",
            Float64,
            lambda message: self.env.record_tracking_error(message.data),
            queue_size=50,
        )

        self.episodes = []
        self.phase_a = None
        self.phase_b = None
        self.failure = ""
        self.transition_file = open(
            str(self.output_dir / "sac_transition_audit.jsonl"),
            "w",
            encoding="utf-8",
        )
        self.summary = {
            "schema_version": SCHEMA_VERSION,
            "status": "running",
            "run_id": self.run_id,
            "mode": self.mode,
            "started_wall_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "live_action_bounds": {
                "v_max_min": live_min,
                "v_max_max": live_max,
                "source": "live /uav1/speed_adapter/safety params",
            },
            "sac_config": config.__dict__,
            "phase_a_action_sampling": (
                "deterministic_actor_mean"
                if self.phase_a_deterministic_actor
                else "stochastic_actor_sample"
            ),
            "training_contract": {
                "target_valid_transitions": self.target_valid_transitions,
                "learning_starts": self.learning_starts,
                "warmup_action_sampling": self.warmup_action_sampling,
                "checkpoint_steps": self.checkpoint_steps,
                "replay_capacity": (
                    None if self.replay is None else self.replay.capacity
                ),
                "updates_per_second": float(
                    rospy.get_param("~sac/updates_per_second", 5.0)
                ),
                "evaluation_checkpoint_path": self.evaluation_checkpoint_path,
                "evaluation_checkpoint_step": self.evaluation_checkpoint_step,
            },
        }
        if self.mode == "pilot":
            self._save_checkpoint(0)
        _write_json(self.output_dir / "sac_runtime_summary.json", self.summary)

    @staticmethod
    def _statistics(values):
        finite = [float(value) for value in values if math.isfinite(float(value))]
        if not finite:
            return {
                "count": 0, "mean": None, "min": None, "max": None,
                "p95": None,
            }
        ordered = sorted(finite)
        position = 0.95 * (len(ordered) - 1)
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        p95 = ordered[lower] + (position - lower) * (
            ordered[upper] - ordered[lower]
        )
        return {
            "count": len(finite),
            "mean": statistics.mean(finite),
            "min": min(finite),
            "max": max(finite),
            "p95": p95,
        }

    def _checkpoint_extra_config(self, step):
        return {
            "run_id": self.run_id,
            "mode": self.mode,
            "checkpoint_environment_step": int(step),
            "target_valid_transitions": self.target_valid_transitions,
            "learning_starts": self.learning_starts,
            "replay_capacity": None if self.replay is None else self.replay.capacity,
            "action_bounds": {
                "v_max_min": self.mapping.v_max_min,
                "v_max_max": self.mapping.v_max_max,
            },
            "random_seed": self.agent.config.seed,
        }

    def _save_checkpoint(self, step):
        step = int(step)
        if any(item["environment_step"] == step for item in self._checkpoint_manifest):
            return
        if self.agent.global_environment_step != step:
            raise RuntimeError(
                "checkpoint step does not equal global environment step"
            )
        path = self.output_dir / "sac_checkpoint_step_{:05d}.pt".format(step)
        if path.exists():
            raise RuntimeError("refusing to overwrite checkpoint: {}".format(path))
        self.agent.save_checkpoint(
            path, extra_config=self._checkpoint_extra_config(step)
        )
        entry = {
            "environment_step": step,
            "gradient_update_step": int(self.agent.update_step),
            "path": str(path),
            "size_bytes": path.stat().st_size,
        }
        self._checkpoint_manifest.append(entry)
        _write_json(
            self.output_dir / "sac_checkpoint_manifest.json",
            self._checkpoint_manifest,
        )

    def _check_learner_health(self):
        if self.learner is None:
            return
        if self.learner.failure:
            raise RuntimeError("learner failed: " + self.learner.failure)
        metrics = self.learner.metrics
        while self._checked_learner_metrics < len(metrics):
            metric = metrics[self._checked_learner_metrics]
            self._checked_learner_metrics += 1
            names = (
                "critic1_loss", "critic2_loss", "actor_loss", "alpha_loss",
                "alpha", "entropy", "q1_mean", "q1_min", "q1_max",
                "q2_mean", "q2_min", "q2_max", "target_q_mean",
                "target_q_min", "target_q_max", "critic_gradient_max_abs",
                "actor_gradient_max_abs", "actor_action_mean",
                "actor_action_std", "actor_action_min", "actor_action_max",
            )
            if not _finite_metrics([metric], names):
                raise RuntimeError("non-finite SAC learner metric")
            gradients = [
                abs(float(metric[name]))
                for name in (
                    "critic_gradient_max_abs", "actor_gradient_max_abs"
                )
                if metric.get(name) is not None
            ]
            if gradients and max(gradients) > self.gradient_abs_limit:
                raise RuntimeError(
                    "SAC gradient explosion threshold exceeded: {:.6g}".format(
                        max(gradients)
                    )
                )
            losses = [
                abs(float(metric[name]))
                for name in ("critic1_loss", "critic2_loss", "actor_loss")
                if metric.get(name) is not None
            ]
            if losses and max(losses) > self.loss_abs_limit:
                raise RuntimeError(
                    "SAC loss explosion threshold exceeded: {:.6g}".format(
                        max(losses)
                    )
                )
            q_values = [
                abs(float(metric[name]))
                for name in (
                    "q1_min", "q1_max", "q2_min", "q2_max",
                    "target_q_min", "target_q_max",
                )
            ]
            if max(q_values) > self.q_abs_limit:
                self._consecutive_q_limit += 1
            else:
                self._consecutive_q_limit = 0
            if self._consecutive_q_limit >= self.q_consecutive_limit:
                raise RuntimeError(
                    "SAC Q values exceeded {:.6g} for {} consecutive updates".format(
                        self.q_abs_limit, self._consecutive_q_limit
                    )
                )

    def _identity_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._condition:
            self._identity = payload
            current = self._current_env_episode
            self._condition.notify_all()
        if current != str(payload.get("episode_key", "")):
            return
        now = rospy.Time.now().to_sec()
        state = str(payload.get("state", ""))
        if state == "EPISODE_ACTIVE":
            self.env.record_mission_state("TRAINING_EPISODE_ACTIVE", now)
            self.env.record_mission_progress(0.0, now)
        elif state == "TERMINAL_LATCHED":
            outcome = str(payload.get("terminal_outcome", ""))
            reason = str(payload.get("terminal_reason", "")) or outcome.lower()
            if outcome == "SUCCESS":
                self.env.record_mission_progress(1.0, now)
                self.env.record_mission_success(True, now)
            elif outcome == "FAILURE":
                self.env.record_mission_failure(True, now)
            elif outcome == "TRUNCATED":
                self.env.record_external_truncation(reason, now)

    def _backend_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._condition:
            self._backend = payload
        if self._current_env_episode:
            self.env.record_bridge_state(
                "TRACK_EGO" if payload.get("mode") == "TRACK" else str(
                    payload.get("mode", "")
                )
            )

    def _wait_identity(self, predicate, timeout, description):
        deadline = time.monotonic() + float(timeout)
        with self._condition:
            while not rospy.is_shutdown():
                payload = self._identity
                if payload is not None and predicate(payload):
                    return dict(payload)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError("timeout waiting for " + description)
                self._condition.wait(min(remaining, 0.1))
        raise RuntimeError("ROS shutdown while waiting for " + description)

    def _record_transition(self, record, normalized_actions, generation):
        step = int(record["step_index"])
        if step not in normalized_actions:
            raise RuntimeError("closed transition has no Actor action")
        normalized_action = float(normalized_actions.pop(step))
        replay_index = None
        if self.replay is not None:
            replay_index = self.replay.add(
                record, normalized_action, generation
            )
        transition = record["transition"]
        state = transition["state_t"]
        velocity = np.asarray(state["actual_velocity_body"], dtype=np.float64)
        tracking = np.asarray(state["tracking_error_body"], dtype=np.float64)
        audit = {
            "episode_id": record["episode_id"],
            "step_index": step,
            "request_id": int(record["request_id"]),
            "reset_generation": generation,
            "observation_valid": True,
            "observation_dimension": 3267,
            "normalized_policy_action": normalized_action,
            "requested_v_max": transition["action_t"]["requested_v_max"],
            "applied_v_max": transition["action_t"]["applied_v_max"],
            "actual_speed_mps": float(np.linalg.norm(velocity)),
            "tracking_error_m": float(np.linalg.norm(tracking)),
            "reward": transition["reward"],
            "reward_components": transition["reward_components"],
            "terminated": transition["terminated"],
            "truncated": bool(record.get("truncated", False)),
            "terminal_reason": record.get("terminal_reason", ""),
        }
        self.transition_file.write(json.dumps(audit, sort_keys=True) + "\n")
        self._current_episode_audits.append(audit)
        if self.mode == "evaluation":
            self._evaluation_transition_count += 1
            return
        self.agent.global_environment_step += 1
        if self.mode != "pilot":
            return
        environment_step = int(self.agent.global_environment_step)
        if (
            not self._pilot_learner_enabled
            and environment_step >= self.learning_starts
        ):
            self.learner.enable()
            self._pilot_learner_enabled = True
            rospy.logwarn(
                "SAC pilot reached learning_starts=%d; stochastic training enabled",
                self.learning_starts,
            )
        self._check_learner_health()
        if environment_step == self.target_valid_transitions:
            # Environment interaction ends at this exact transition. Freeze the
            # learner before the final checkpoint so reload represents the true
            # terminal training state rather than a later background update.
            self.learner.stop()
            self._check_learner_health()
        if environment_step in self.checkpoint_steps:
            self._pending_checkpoint_steps.add(environment_step)
        if environment_step > self.target_valid_transitions:
            raise RuntimeError("pilot exceeded target valid transitions")

    def _phase_a_audit(self):
        replay = self.replay.audit()
        failures = list(replay["failures"])
        if replay["size"] < self.phase_a_min_transitions:
            failures.append("less_than_{}_transitions".format(
                self.phase_a_min_transitions
            ))
        for episode in self.episodes:
            metrics = episode["metrics"]
            if not metrics["identity_chain_strict_1_to_1"]:
                failures.append("identity_chain")
            if not metrics["closed_request_ids_strictly_monotonic"]:
                failures.append("request_monotonicity")
            if metrics["timeout_count"] or metrics["causal_mismatch_count"]:
                failures.append("causal_timeout_or_mismatch")
            if (
                metrics["accepted_action_count"]
                != metrics["scheduled_action_count"]
                or metrics["accepted_applied_ack_count"]
                != metrics["scheduled_action_count"]
                or metrics["transition_count"]
                + metrics["cancelled_in_flight_after_terminal"]
                != metrics["scheduled_action_count"]
            ):
                failures.append("request_action_applied_identity_closure")
            if metrics["dropped_deadline_count"]:
                failures.append("dropped_deadline")
            if metrics["generation_mismatch_observation_count"]:
                failures.append("old_generation_contamination")
            if not 9.5 <= float(metrics["action_rate_hz"]) <= 10.5:
                failures.append("action_rate")
            if metrics["reward_finite_rate"] != 1.0:
                failures.append("non_finite_reward")
        if self.learner.metrics:
            failures.append("gradient_update_during_phase_a")
        return {
            "passed": not failures,
            "failures": sorted(set(failures)),
            "replay": replay,
            "episode_count": len(self.episodes),
            "learner_update_count": len(self.learner.metrics),
        }

    def _episode_summary(self, result, identity):
        audits = list(self._current_episode_audits)
        requested = [item["requested_v_max"] for item in audits]
        applied = [item["applied_v_max"] for item in audits]
        actual_speed = [item["actual_speed_mps"] for item in audits]
        tracking_error = [item["tracking_error_m"] for item in audits]
        rewards = [item["reward"] for item in audits]
        normalized = [item["normalized_policy_action"] for item in audits]
        reason = str(result["episode"].get("terminal_reason", ""))
        return {
            "episode": result["episode"],
            "metrics": result["metrics"],
            "reset_generation": int(identity["reset_generation"]),
            "coordinator_terminal_outcome": identity.get("terminal_outcome"),
            "coordinator_terminal_reason": identity.get("terminal_reason"),
            "transition_count": len(audits),
            "episode_return": sum(float(value) for value in rewards),
            "mean_reward": (
                statistics.mean(rewards) if rewards else None
            ),
            "normalized_action": self._statistics(normalized),
            "requested_v_max": self._statistics(requested),
            "applied_v_max": self._statistics(applied),
            "actual_speed_mps": self._statistics(actual_speed),
            "tracking_error_m": self._statistics(tracking_error),
            "planner_failure": "planner_failure" in reason,
            "collision": "collision" in reason,
        }

    def _publish_closure(self, result, identity):
        steps = result["steps"]
        if not steps:
            raise RuntimeError("Episode closed without transitions")
        last = steps[-1]
        terminal_closed = bool(
            result["episode"]["terminated"]
            or result["episode"]["truncated"]
        )
        payload = {
            "version": "astradrone_sac_episode_closure_v1.0",
            "status": result["status"],
            "episode_id": identity["episode_key"],
            "reset_generation": int(identity["reset_generation"]),
            "transition_count": len(steps),
            "last_step_index": int(last["step_index"]),
            "last_request_id": int(last["request_id"]),
            "terminal_transition_closed": terminal_closed,
            "terminated": bool(result["episode"]["terminated"]),
            "truncated": bool(result["episode"]["truncated"]),
        }
        if not terminal_closed:
            raise RuntimeError("terminal transition was not closed before reset")
        self._closure_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        return payload

    def _run_one_episode(self, identity):
        episode_key = str(identity["episode_key"])
        generation = int(identity["reset_generation"])
        self.env.begin_external_episode(self.run_id, episode_key, generation)
        with self._condition:
            self._current_env_episode = episode_key
            backend = dict(self._backend)
        now = rospy.Time.now().to_sec()
        self.env.record_mission_state("TRAINING_EPISODE_ACTIVE", now)
        self.env.record_mission_progress(0.0, now)
        self.env.record_bridge_state(
            "TRACK_EGO" if backend.get("mode") == "TRACK" else str(
                backend.get("mode", "")
            )
        )
        normalized_actions = {}
        self._current_episode_audits = []

        def action_provider(state, step_index):
            vector = flatten_policy_input(state)
            if self.mode == "evaluation":
                deterministic = True
            elif self.mode == "pilot":
                deterministic = (
                    self.agent.global_environment_step < self.learning_starts
                )
            else:
                deterministic = (
                    self.phase_a is None
                    and self.phase_a_deterministic_actor
                )
            normalized = self.agent.sample_action(
                vector,
                deterministic=deterministic,
            )
            normalized_actions[int(step_index)] = normalized
            return self.mapping.to_v_max(normalized)

        transition_consumer = None
        if self.mode in ("pilot", "evaluation"):
            transition_consumer = lambda record: self._record_transition(
                record, normalized_actions, generation
            )

        result = self.env.run_episode(
            action_provider,
            self.episode_config,
            transition_consumer=transition_consumer,
        )
        if self.mode == "pilot" and self._pending_checkpoint_steps:
            for checkpoint_step in sorted(self._pending_checkpoint_steps):
                self._save_checkpoint(checkpoint_step)
            self._pending_checkpoint_steps.clear()
        _write_json(
            self.output_dir / "sac_last_environment_result.json",
            {
                "version": result["version"],
                "status": result["status"],
                "error": result["error"],
                "episode": result["episode"],
                "metrics": result["metrics"],
                "last_step": result["steps"][-1] if result["steps"] else None,
            },
        )
        if result["status"] != "completed":
            raise RuntimeError(
                "AstraDroneEnv Episode failed before coordinator terminal: "
                "{} {}".format(result["status"], result["error"])
            )
        terminal_identity = self._wait_identity(
            lambda payload: payload.get("episode_key") == episode_key
            and payload.get("state") == "TERMINAL_LATCHED",
            10.0,
            "coordinator terminal identity",
        )
        if self.mode == "qualification":
            for record in result["steps"]:
                self._record_transition(record, normalized_actions, generation)
        cancelled_actions = len(normalized_actions)
        if cancelled_actions != int(
            result["metrics"]["cancelled_in_flight_after_terminal"]
        ):
            raise RuntimeError(
                "unclosed Actor actions disagree with terminal cancellation audit"
            )
        normalized_actions.clear()
        if self.replay is not None:
            self.replay.mark_episode_boundary(
                episode_key,
                result["episode"]["truncated"],
                result["episode"]["terminal_reason"],
            )
        summary = self._episode_summary(result, terminal_identity)
        self.episodes.append(summary)
        self.transition_file.flush()
        closure = self._publish_closure(result, terminal_identity)
        summary["closure"] = closure
        with self._condition:
            self._processed_episodes.add(episode_key)
            self._current_env_episode = ""
        _write_json(self.output_dir / "sac_episode_summaries.json", self.episodes)

        if (
            self.mode == "qualification"
            and self.phase_a is None
            and len(self.replay) >= self.phase_a_min_transitions
        ):
            self.phase_a = self._phase_a_audit()
            _write_json(self.output_dir / "sac_phase_a.json", self.phase_a)
            if not self.phase_a["passed"]:
                raise RuntimeError(
                    "Phase A failed: {}".format(self.phase_a["failures"])
                )
            self.learner.enable()
            rospy.logwarn(
                "SAC Phase A PASS at %d transitions; Phase B updates enabled",
                len(self.replay),
            )

    def _finalize_phase_b(self):
        if self.phase_a is None or not self.phase_a["passed"]:
            raise RuntimeError("Phase A did not pass")
        deadline = time.monotonic() + self.phase_b_update_timeout_wall
        while (
            len(self.learner.metrics) < self.phase_b_min_updates
            and not self.learner.failure
            and time.monotonic() < deadline
            and not rospy.is_shutdown()
        ):
            time.sleep(0.05)
        if self.learner.failure:
            raise RuntimeError("learner failed: " + self.learner.failure)
        if len(self.learner.metrics) < self.phase_b_min_updates:
            raise RuntimeError("Phase B update target was not reached")
        self.learner.stop()
        metrics = list(self.learner.metrics)
        metric_names = (
            "critic1_loss", "critic2_loss", "actor_loss", "alpha_loss",
            "alpha", "entropy", "q_mean", "q_min", "q_max",
            "target_q_mean", "target_q_min", "target_q_max",
        )
        failures = self._episode_contract_failures()
        if not _finite_metrics(metrics, metric_names):
            failures.append("non_finite_learner_metric")
        if not all(item["gradient_finite"] for item in metrics):
            failures.append("non_finite_gradient")
        action_stability = self._qualification_action_stability(metrics)
        failures.extend(action_stability["failures"])
        parameter_audit = self.agent.parameter_update_audit()
        if min(parameter_audit.values()) <= 0.0:
            failures.append("parameters_not_updated")
        update_durations = [item["update_duration_wall_sec"] for item in metrics]
        checkpoint = self.output_dir / "sac_smoke_checkpoint.pt"
        self.agent.save_checkpoint(
            checkpoint,
            extra_config={
                "action_bounds": {
                    "v_max_min": self.mapping.v_max_min,
                    "v_max_max": self.mapping.v_max_max,
                },
                "run_id": self.run_id,
            },
        )
        restored = SacAgent(self.agent.config)
        restored_payload = restored.load_checkpoint(checkpoint)
        checkpoint_equal = self.agent.state_equal(restored)
        if not checkpoint_equal:
            failures.append("checkpoint_restore_mismatch")
        self.phase_b = {
            "passed": not failures,
            "failures": failures,
            "update_count": len(metrics),
            "finite_metric_count": sum(
                1 for item in metrics if _finite_metrics([item], metric_names)
            ),
            "episode_contract_failures": self._episode_contract_failures(),
            "action_stability": action_stability,
            "parameter_update": parameter_audit,
            "alpha_final": self.agent.alpha,
            "update_duration_wall_sec": {
                "min": min(update_durations),
                "mean": sum(update_durations) / len(update_durations),
                "max": max(update_durations),
            },
            "learner_update_hz": (
                len(metrics) / max(
                    1.0e-9,
                    self.learner.finished_wall - self.learner.started_wall,
                )
            ),
            "checkpoint": str(checkpoint),
            "checkpoint_parameter_restore_equal": checkpoint_equal,
            "checkpoint_optimizer_state_entries": {
                "actor": len(restored_payload["actor_optimizer"]["state"]),
                "critic": len(restored_payload["critic_optimizer"]["state"]),
                "alpha": len(restored_payload["alpha_optimizer"]["state"]),
            },
            "metrics": metrics,
        }
        _write_json(self.output_dir / "sac_phase_b.json", self.phase_b)
        with open(
            str(self.output_dir / "sac_learner_metrics.jsonl"),
            "w",
            encoding="utf-8",
        ) as stream:
            for metric in metrics:
                stream.write(json.dumps(metric, sort_keys=True) + "\n")
        if failures:
            raise RuntimeError("Phase B failed: {}".format(failures))

    def _qualification_action_stability(self, metrics):
        snapshot = self.replay.snapshot()
        start = min(self.phase_a_min_transitions, len(snapshot["policy_actions"]))
        actions = snapshot["policy_actions"][start:, 0]
        requested = snapshot["requested_v_max"][start:]
        episode_ids = snapshot["episode_ids"][start:]
        deltas = [
            float(requested[index] - requested[index - 1])
            for index in range(1, len(requested))
            if episode_ids[index] == episode_ids[index - 1]
        ]
        boundary_fraction = (
            None
            if len(actions) == 0
            else float(np.mean(np.abs(actions) >= 0.99))
        )
        action_std = None if len(actions) == 0 else float(np.std(actions))
        force_replan_delta_count = sum(
            value < -0.3 or value > 0.5 for value in deltas
        )
        actor_boundary_metric_count = sum(
            1
            for item in metrics
            if item.get("actor_update_applied")
            and (
                float(item["actor_deterministic_action_min"]) <= -0.99
                or float(item["actor_deterministic_action_max"]) >= 0.99
            )
        )
        failures = []
        if len(actions) < self.stochastic_min_transitions:
            failures.append("insufficient_stochastic_transitions")
        if action_std is not None and action_std > self.stochastic_action_std_max:
            failures.append("stochastic_action_std")
        if (
            boundary_fraction is not None
            and boundary_fraction > self.stochastic_boundary_fraction_max
        ):
            failures.append("stochastic_action_boundary_saturation")
        if force_replan_delta_count > self.stochastic_force_replan_delta_max:
            failures.append("stochastic_force_replan_delta")
        if actor_boundary_metric_count:
            failures.append("actor_mean_boundary_saturation")
        return {
            "passed": not failures,
            "failures": failures,
            "transition_count": int(len(actions)),
            "normalized_action_std": action_std,
            "normalized_action_min": (
                None if len(actions) == 0 else float(np.min(actions))
            ),
            "normalized_action_max": (
                None if len(actions) == 0 else float(np.max(actions))
            ),
            "boundary_fraction_abs_ge_0_99": boundary_fraction,
            "force_replan_delta_count": int(force_replan_delta_count),
            "signed_delta_v_max_min": (
                None if not deltas else float(min(deltas))
            ),
            "signed_delta_v_max_max": (
                None if not deltas else float(max(deltas))
            ),
            "actor_boundary_metric_count": int(actor_boundary_metric_count),
            "thresholds": {
                "minimum_transitions": self.stochastic_min_transitions,
                "maximum_normalized_action_std": self.stochastic_action_std_max,
                "maximum_boundary_fraction": (
                    self.stochastic_boundary_fraction_max
                ),
                "maximum_force_replan_delta_count": (
                    self.stochastic_force_replan_delta_max
                ),
            },
        }

    def _write_learner_metrics(self):
        metrics = [] if self.learner is None else list(self.learner.metrics)
        scale = 0.5 * (
            self.mapping.v_max_max - self.mapping.v_max_min
        )
        bias = 0.5 * (
            self.mapping.v_max_max + self.mapping.v_max_min
        )
        with open(
            str(self.output_dir / "sac_learner_metrics.jsonl"),
            "w",
            encoding="utf-8",
        ) as stream:
            for original in metrics:
                metric = dict(original)
                for suffix in ("mean", "min", "max"):
                    action = metric.get("actor_action_" + suffix)
                    metric["requested_v_max_" + suffix] = (
                        None
                        if action is None
                        else bias + scale * float(action)
                    )
                action_std = metric.get("actor_action_std")
                metric["requested_v_max_std"] = (
                    None if action_std is None else scale * float(action_std)
                )
                stream.write(json.dumps(metric, sort_keys=True) + "\n")
        return metrics

    def _episode_contract_failures(self):
        failures = []
        for episode in self.episodes:
            metrics = episode["metrics"]
            if not metrics["identity_chain_strict_1_to_1"]:
                failures.append("identity_chain")
            if not metrics["closed_request_ids_strictly_monotonic"]:
                failures.append("request_monotonicity")
            if metrics["timeout_count"] or metrics["causal_mismatch_count"]:
                failures.append("causal_timeout_or_mismatch")
            if metrics["dropped_deadline_count"]:
                failures.append("dropped_deadline")
            if metrics["generation_mismatch_observation_count"]:
                failures.append("old_generation_contamination")
            if not 9.5 <= float(metrics["action_rate_hz"]) <= 10.5:
                failures.append("action_rate")
            if metrics["reward_finite_rate"] != 1.0:
                failures.append("non_finite_reward")
            reason = " ".join(
                (
                    str(episode["episode"].get("terminal_reason", "")),
                    str(episode.get("coordinator_terminal_reason", "")),
                )
            )
            if "planner_failure" in reason:
                failures.append("planner_failure")
            if "collision" in reason:
                failures.append("collision")
            if "controller_failure" in reason:
                failures.append("controller_failure")
            if "invalid_observation" in reason:
                failures.append("invalid_observation")
        return sorted(set(failures))

    def _finalize_pilot(self):
        self._check_learner_health()
        self.learner.stop()
        self._check_learner_health()
        metrics = self._write_learner_metrics()
        failures = self._episode_contract_failures()
        if self.agent.global_environment_step != self.target_valid_transitions:
            failures.append(
                "valid_transition_target:{}_of_{}".format(
                    self.agent.global_environment_step,
                    self.target_valid_transitions,
                )
            )
        if self.agent.update_step <= 0:
            failures.append("no_gradient_updates")
        replay_audit = self.replay.audit()
        failures.extend(replay_audit["failures"])
        missing_checkpoints = sorted(
            set(self.checkpoint_steps)
            - set(item["environment_step"] for item in self._checkpoint_manifest)
        )
        if missing_checkpoints:
            failures.append("missing_checkpoints:{}".format(missing_checkpoints))

        final_checkpoint = self.output_dir / "sac_checkpoint_step_{:05d}.pt".format(
            self.target_valid_transitions
        )
        reload_audit = {
            "checkpoint": str(final_checkpoint),
            "checkpoint_exists": final_checkpoint.is_file(),
            "parameter_restore_equal": False,
            "deterministic_action_finite": False,
            "deterministic_action": None,
            "requested_v_max": None,
        }
        if final_checkpoint.is_file() and len(self.replay) > 0:
            restored = SacAgent(self.agent.config)
            restored.load_checkpoint(final_checkpoint)
            reload_audit["parameter_restore_equal"] = self.agent.state_equal(
                restored
            )
            observation = self.replay.observations[len(self.replay) - 1].copy()
            action = restored.sample_action(observation, deterministic=True)
            requested = self.mapping.to_v_max(action)
            reload_audit.update(
                {
                    "deterministic_action_finite": bool(
                        math.isfinite(action) and math.isfinite(requested)
                    ),
                    "deterministic_action": action,
                    "requested_v_max": requested,
                }
            )
        if not reload_audit["parameter_restore_equal"]:
            failures.append("checkpoint_restore_mismatch")
        if not reload_audit["deterministic_action_finite"]:
            failures.append("checkpoint_inference_invalid")

        snapshot = self.replay.snapshot()
        np.savez_compressed(
            str(self.output_dir / "sac_replay_snapshot.npz"), **snapshot
        )
        learner_hz = None
        if (
            self.learner.started_wall is not None
            and self.learner.finished_wall is not None
        ):
            learner_hz = len(metrics) / max(
                1.0e-9,
                self.learner.finished_wall - self.learner.started_wall,
            )
        self.summary.update(
            {
                "status": "completed" if not failures else "failed",
                "verdict": "PILOT TRAINING COMPLETE" if not failures else "NO-GO",
                "failures": sorted(set(failures)),
                "episode_count": len(self.episodes),
                "global_environment_step": self.agent.global_environment_step,
                "gradient_update_step": self.agent.update_step,
                "learner_update_count": len(metrics),
                "learner_update_hz": learner_hz,
                "replay_audit": replay_audit,
                "checkpoint_manifest": self._checkpoint_manifest,
                "checkpoint_reload_audit": reload_audit,
                "episode_contract_failures": self._episode_contract_failures(),
            }
        )
        if failures:
            raise RuntimeError("pilot final audit failed: {}".format(failures))

    def _finalize_evaluation(self):
        failures = self._episode_contract_failures()
        if len(self.episodes) != self.expected_episode_count:
            failures.append("evaluation_episode_count")
        self.summary.update(
            {
                "status": "completed" if not failures else "failed",
                "verdict": "DETERMINISTIC EVALUATION PASS" if not failures else "NO-GO",
                "failures": sorted(set(failures)),
                "episode_count": len(self.episodes),
                "evaluation_transition_count": self._evaluation_transition_count,
                "evaluation_checkpoint_step": self.evaluation_checkpoint_step,
                "evaluation_checkpoint_path": self.evaluation_checkpoint_path,
                "episode_contract_failures": self._episode_contract_failures(),
            }
        )
        if failures:
            raise RuntimeError(
                "deterministic evaluation failed: {}".format(failures)
            )

    def run(self):
        try:
            while not rospy.is_shutdown():
                identity = self._wait_identity(
                    lambda payload: (
                        payload.get("state") == "EPISODE_ACTIVE"
                        and payload.get("episode_key")
                        not in self._processed_episodes
                    )
                    or payload.get("state")
                    in ("QUALIFICATION_COMPLETE_HOVER", "QUALIFICATION_FAILED"),
                    900.0,
                    "active Episode or coordinator completion",
                )
                state = identity.get("state")
                if state == "QUALIFICATION_FAILED":
                    raise RuntimeError(
                        "coordinator failed: {}".format(
                            identity.get("terminal_reason", "")
                        )
                    )
                if state == "QUALIFICATION_COMPLETE_HOVER":
                    break
                self._run_one_episode(identity)
            if self.mode == "qualification":
                self._finalize_phase_b()
                snapshot = self.replay.snapshot()
                np.savez_compressed(
                    str(self.output_dir / "sac_replay_snapshot.npz"), **snapshot
                )
                self.summary.update(
                    {
                        "status": "completed",
                        "verdict": "GO FOR SAC TRAINING",
                        "phase_a": self.phase_a,
                        "phase_b": {
                            key: value
                            for key, value in self.phase_b.items()
                            if key != "metrics"
                        },
                        "episode_count": len(self.episodes),
                        "replay_audit": self.replay.audit(),
                    }
                )
            elif self.mode == "pilot":
                self._finalize_pilot()
            else:
                self._finalize_evaluation()
        except Exception as error:
            self.failure = "{}: {}".format(type(error).__name__, error)
            try:
                if self.learner is not None:
                    self.learner.stop()
            except Exception as stop_error:
                self.failure += "; learner_stop={}".format(stop_error)
            self.summary.update(
                {
                    "status": "failed",
                    "verdict": "NO-GO",
                    "failure": self.failure,
                    "phase_a": self.phase_a,
                    "phase_b": self.phase_b,
                    "episode_count": len(self.episodes),
                    "global_environment_step": self.agent.global_environment_step,
                    "gradient_update_step": self.agent.update_step,
                    "replay_audit": (
                        None if self.replay is None else self.replay.audit()
                    ),
                    "checkpoint_manifest": self._checkpoint_manifest,
                }
            )
            try:
                self._write_learner_metrics()
            except Exception as metric_error:
                self.failure += "; metric_write={}".format(metric_error)
            rospy.logerr("SAC %s failed: %s", self.mode, self.failure)
        finally:
            self.transition_file.close()
            self.summary["finished_wall_iso"] = time.strftime(
                "%Y-%m-%dT%H:%M:%S%z"
            )
            _write_json(
                self.output_dir / "sac_runtime_summary.json", self.summary
            )
        return self.summary


def main():
    rospy.init_node("sac_training_runner", anonymous=False)
    deadline = time.monotonic() + 60.0
    while (
        rospy.Time.now().to_sec() <= 0.0
        and time.monotonic() < deadline
        and not rospy.is_shutdown()
    ):
        time.sleep(0.05)
    if rospy.Time.now().to_sec() <= 0.0:
        raise rospy.ROSInitException("positive ROS/simulation time unavailable")
    summary = SacTrainingRunner().run()
    if summary.get("status") != "completed":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
