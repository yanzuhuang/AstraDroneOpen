#!/usr/bin/env python3
"""Training-only SAC owner layered on AstraDroneEnv and reset coordinator."""

import json
import hashlib
import math
import os
from pathlib import Path
import resource
import statistics
import threading
import time

import numpy as np
import rospy
from std_msgs.msg import Float64, String

from learning_speed_rl.training import (
    ActorActionOwnership,
    SUPPORTED_RUNNER_MODES,
    AstraDroneEnv,
    AstraDroneEpisodeConfig,
    BalancedForestMapScheduler,
    FormalTrainingSchedule,
    OrderedTransitionWriter,
    checkpoint_filename,
    infrastructure_terminal_reason,
    learning_started,
    mode_uses_training_replay,
    validate_training_episode_target,
)
from learning_speed_rl.training.sac import LearnerWorker, SacAgent, SacConfig
from learning_speed_rl.training.sac_replay import (
    ActionMapping,
    SacReplayBuffer,
    flatten_policy_input,
    validate_action_range_with_capability,
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
        self.run_id = str(
            rospy.get_param("~run_id", "worksite_sac_training_10000ep")
        )
        self.mode = str(
            rospy.get_param("~training/mode", "qualification")
        ).strip().lower()
        if self.mode not in SUPPORTED_RUNNER_MODES:
            raise rospy.ROSInitException(
                "training/mode must be qualification, training or evaluation"
            )
        self.smoke_test = bool(rospy.get_param("~training/smoke_test", False))
        self.timing_qualification = bool(
            rospy.get_param("~training/timing_qualification", False)
        )
        self.early_learning_observation = bool(
            rospy.get_param("~training/early_learning_observation", False)
        )
        self.startup_contract_only = bool(
            rospy.get_param("~training/startup_contract_only", False)
        )
        if self.mode == "training":
            marker = self.output_dir / ".sac_training_run_started"
            try:
                descriptor = os.open(
                    str(marker), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644
                )
            except FileExistsError:
                raise rospy.ROSInitException(
                    "refusing to reuse a prior training output directory"
                )
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                stream.write(self.run_id + "\n")
        self.total_training_episodes = int(
            rospy.get_param("~training/total_training_episodes", 10000)
        )
        if self.mode == "training":
            try:
                self.training_scope = validate_training_episode_target(
                    self.total_training_episodes,
                    smoke_test=self.smoke_test,
                    timing_qualification=self.timing_qualification,
                    early_learning_observation=(
                        self.early_learning_observation
                    ),
                )
            except ValueError as error:
                raise rospy.ROSInitException(
                    str(error)
                ) from error
        else:
            self.training_scope = self.mode
        self.learning_starts = int(
            rospy.get_param("~training/learning_starts", 1000)
        )
        self.warmup_action_sampling = str(
            rospy.get_param(
                "~training/warmup_action_sampling",
                "deterministic_actor_mean",
            )
        ).strip()
        configured_checkpoint_episodes = sorted(
            set(
                int(value)
                for value in rospy.get_param(
                    "~training/checkpoint_episodes",
                    list(range(500, 10001, 500)),
                )
            )
        )
        self.checkpoint_episodes = (
            [self.total_training_episodes]
            if self.mode == "training"
            and (
                self.smoke_test
                or self.timing_qualification
                or self.early_learning_observation
            )
            else configured_checkpoint_episodes
        )
        self.training_schedule = FormalTrainingSchedule(
            total_training_episodes=self.total_training_episodes,
            checkpoint_episodes=tuple(self.checkpoint_episodes),
        )
        try:
            self.training_schedule.validate()
        except ValueError as error:
            raise rospy.ROSInitException(str(error))
        if self.total_training_episodes <= 0 or self.learning_starts <= 0:
            raise rospy.ROSInitException(
                "training Episode target and transition learning_starts must be positive"
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
        self.evaluation_checkpoint_episode = int(
            rospy.get_param("~evaluation/checkpoint_episode", 0)
        )
        self.evaluation_episode_count = int(
            rospy.get_param("~evaluation/episode_count", 100)
        )
        reset_randomization_enabled = bool(
            rospy.get_param(
                "~environment/reset_randomization_enabled", False
            )
        )
        if self.mode == "training" and not reset_randomization_enabled:
            raise rospy.ROSInitException(
                "formal training requires random reset enabled"
            )
        if self.mode == "evaluation" and reset_randomization_enabled:
            raise rospy.ROSInitException(
                "evaluation must use fixed nominal reset"
            )
        if self.mode == "evaluation" and self.evaluation_checkpoint_episode <= 0:
            raise rospy.ROSInitException(
                "evaluation/checkpoint_episode must be positive"
            )
        if self.evaluation_episode_count <= 0:
            raise rospy.ROSInitException(
                "evaluation/episode_count must be positive"
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
        self._training_learner_enabled = False
        self._evaluation_transition_count = 0
        self._current_episode_audits = []
        self._active_actor_timing = None

        self._condition = threading.Condition(threading.RLock())
        self._identity = None
        self._backend = {}
        self._current_env_episode = ""
        self._processed_episodes = set()
        self._shutdown = False
        self._stale_snapshots_by_episode = {}
        self._stale_snapshot_count = 0
        self.forest_enabled = bool(rospy.get_param("~forest/enabled", False))
        self._forest_state = {}
        self._current_forest_assignment = None
        self._pending_map_switch_continuity = None
        self._forest_map_switch_audits = []
        self.forest_scheduler = None
        self.evaluation_forest_seed = int(
            rospy.get_param("~forest/evaluation_logical_seed", -1)
        )
        self.forest_switch_timeout = float(
            rospy.get_param("~forest/switch_timeout_wall", 45.0)
        )
        self.forest_config_path = str(
            rospy.get_param("~forest/config_path", "")
        ).strip()
        if self.forest_enabled:
            if self.mode not in ("training", "evaluation"):
                raise rospy.ROSInitException(
                    "Forest scheduling is training/evaluation only"
                )
            config_path = Path(self.forest_config_path).expanduser().resolve()
            if not config_path.is_file():
                raise rospy.ROSInitException(
                    "qualified Forest config does not exist: {}".format(config_path)
                )
            forest_config = json.loads(config_path.read_text(encoding="utf-8"))
            scheduler_config = forest_config["scheduler"]
            logical_to_raw = {
                int(item["logical_seed"]): int(item["raw_seed"])
                for item in forest_config["seed_pool"]
            }
            self.forest_scheduler = BalancedForestMapScheduler(
                scheduler_config["training_logical_seeds"],
                scheduler_config["evaluation_logical_seeds"],
                logical_to_raw,
                episodes_per_map_block=int(
                    rospy.get_param(
                        "~forest/episodes_per_map_block",
                        scheduler_config["episodes_per_map_block"],
                    )
                ),
                rng_seed=int(
                    rospy.get_param(
                        "~forest/scheduler_rng_seed",
                        scheduler_config["rng_seed"],
                    )
                ),
                smoke_test=(self.smoke_test or self.timing_qualification),
            )
            if self.smoke_test and self.mode != "training":
                raise rospy.ROSInitException(
                    "bounded Forest smoke is training-mode only"
                )
            if self.mode == "evaluation":
                self.forest_scheduler.evaluation_assignment(
                    self.evaluation_forest_seed
                )

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
        try:
            validate_action_range_with_capability(
                expected_min,
                expected_max,
                live_min,
                live_max,
            )
        except ValueError as error:
            raise rospy.ROSInitException(
                "SAC action/capability contract failed: {}".format(error)
            )
        self.mapping = ActionMapping(expected_min, expected_max)
        self.evaluation_fixed_vmax = float(rospy.get_param("~evaluation/fixed_vmax", 0.0))
        if self.evaluation_fixed_vmax != 0.0:
            if self.mode != "evaluation" or self.evaluation_fixed_vmax not in (0.60, 1.00, 1.40):
                raise rospy.ROSInitException("fixed comparison policy requires evaluation and 0.60/1.00/1.40")

        config = SacConfig(
            observation_dim=int(rospy.get_param("~sac/observation_dim", 3267)),
            action_dim=int(rospy.get_param("~sac/action_dim", 1)),
            hidden_dim=int(rospy.get_param("~sac/hidden_dim", 256)),
            gamma=float(rospy.get_param("~sac/gamma", 0.99)),
            tau=float(rospy.get_param("~sac/tau", 0.005)),
            batch_size=int(rospy.get_param("~sac/batch_size", 64)),
            policy_learning_rate=float(
                rospy.get_param("~sac/policy_learning_rate", 1.0e-5)
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
            log_std_min=float(rospy.get_param("~sac/log_std_min", -3.0)),
            log_std_max=float(rospy.get_param("~sac/log_std_max", -1.0)),
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
        if mode_uses_training_replay(self.mode):
            self.replay = SacReplayBuffer(
                int(rospy.get_param("~replay/capacity", 100000)),
                self.mapping,
                intervention_tolerance_mps=float(
                    rospy.get_param(
                        "~replay/intervention_tolerance_mps", 0.005
                    )
                ),
                initial_allocation=int(
                    rospy.get_param("~replay/initial_allocation", 4096)
                ),
            )
            self.learner = LearnerWorker(
                self.agent,
                self.replay,
                float(rospy.get_param("~sac/updates_per_second", 5.0)),
                seed=config.seed + 1,
            )
            self.learner.start()
        else:
            if not self.evaluation_checkpoint_path:
                raise rospy.ROSInitException(
                    "evaluation/checkpoint_path is required"
                )
            checkpoint = Path(self.evaluation_checkpoint_path).resolve()
            if not checkpoint.is_file():
                raise rospy.ROSInitException(
                    "evaluation checkpoint does not exist: {}".format(checkpoint)
                )
            payload = self.agent.load_checkpoint(checkpoint)
            checkpoint_episode = int(
                payload.get("extra_config", {}).get(
                    "checkpoint_completed_episode", -1
                )
            )
            if checkpoint_episode != self.evaluation_checkpoint_episode:
                raise rospy.ROSInitException(
                    "evaluation checkpoint Episode does not match requested Episode"
                )
        self._evaluation_network_hash_at_load = self._network_hash() if self.mode == "evaluation" else None
        self._evaluation_agent_counters_at_load = (
            int(self.agent.global_environment_step), int(self.agent.update_step)
        )

        max_steps = int(rospy.get_param("~episode/max_steps", 500))
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
        self._stale_snapshot_sub = rospy.Subscriber(
            "/uav1/learning_speed/stale_observation_snapshot",
            String,
            self._stale_snapshot_callback,
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
        self._forest_request_pub = None
        self._forest_state_sub = None
        if self.forest_enabled:
            self._forest_request_pub = rospy.Publisher(
                "/uav1/learning_speed/forest_map_request",
                String, queue_size=1, latch=True,
            )
            self._forest_state_sub = rospy.Subscriber(
                "/uav1/learning_speed/forest_map_state",
                String, self._forest_state_callback, queue_size=10,
            )

        self.episodes = []
        self.phase_a = None
        self.phase_b = None
        self.failure = ""
        self.transition_file = open(
            str(self.output_dir / "sac_transition_audit.jsonl"),
            "w",
            encoding="utf-8",
            buffering=1,
        )
        self.stale_snapshot_file = open(
            str(self.output_dir / "sac_stale_observation_snapshots.jsonl"),
            "w", encoding="utf-8", buffering=1,
        )
        self.summary = {
            "schema_version": SCHEMA_VERSION,
            "status": "running",
            "run_id": self.run_id,
            "mode": self.mode,
            "bounded_smoke_test": self.smoke_test,
            "timing_qualification": self.timing_qualification,
            "early_learning_observation": self.early_learning_observation,
            "training_scope": self.training_scope,
            "startup_contract_only": self.startup_contract_only,
            "started_wall_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "live_action_bounds": {
                "v_max_min": live_min,
                "v_max_max": live_max,
                "source": "live /uav1/speed_adapter/safety params",
            },
            "sac_action_bounds": {
                "v_max_min": self.mapping.v_max_min,
                "v_max_max": self.mapping.v_max_max,
                "source": "SAC action config",
            },
            "downstream_capability_bounds": {
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
                "total_training_episodes": self.total_training_episodes,
                "learning_starts": self.learning_starts,
                "warmup_action_sampling": self.warmup_action_sampling,
                "checkpoint_episodes": self.checkpoint_episodes,
                "replay_capacity": (
                    None if self.replay is None else self.replay.capacity
                ),
                "updates_per_second": float(
                    rospy.get_param("~sac/updates_per_second", 5.0)
                ),
                "evaluation_checkpoint_path": self.evaluation_checkpoint_path,
                "evaluation_checkpoint_episode": (
                    self.evaluation_checkpoint_episode
                ),
                "evaluation_episode_count": self.evaluation_episode_count,
                "reset_randomization_enabled": reset_randomization_enabled,
            },
            "forest_contract": {
                "enabled": self.forest_enabled,
                "config_path": self.forest_config_path,
                "training_logical_seeds": (
                    None if self.forest_scheduler is None
                    else list(self.forest_scheduler.training_logical_seeds)
                ),
                "evaluation_logical_seeds": (
                    None if self.forest_scheduler is None
                    else list(self.forest_scheduler.evaluation_logical_seeds)
                ),
                "episodes_per_map_block": (
                    None if self.forest_scheduler is None
                    else self.forest_scheduler.episodes_per_map_block
                ),
                "episodes_per_round": (
                    None if self.forest_scheduler is None
                    else self.forest_scheduler.episodes_per_round
                ),
                "scheduler_rng_seed": (
                    None if self.forest_scheduler is None
                    else self.forest_scheduler.rng_seed
                ),
                "evaluation_logical_seed": self.evaluation_forest_seed,
            },
        }
        _write_json(self.output_dir / "sac_runtime_summary.json", self.summary)

    @staticmethod
    def _statistics(values):
        finite = [float(value) for value in values if math.isfinite(float(value))]
        if not finite:
            return {
                "count": 0, "mean": None, "min": None, "max": None,
                "median": None, "p95": None,
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
            "median": statistics.median(finite),
            "min": min(finite),
            "max": max(finite),
            "p95": p95,
        }

    def _forest_state_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._condition:
            self._forest_state = payload
            self._condition.notify_all()

    def _stale_snapshot_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        episode_key = str(payload.get("episode_key", ""))
        if not episode_key:
            return
        event = str(payload.get("event", ""))
        with self._condition:
            record = self._stale_snapshots_by_episode.setdefault(
                episode_key, {}
            )
            if event == "STALE_CANDIDATE":
                record["candidate"] = payload
            elif event == "STALE_TERMINAL_LATCH":
                record["terminal_latch"] = payload
            else:
                record.setdefault("other", []).append(payload)
            self._stale_snapshot_count += 1
            self.stale_snapshot_file.write(
                json.dumps(payload, sort_keys=True) + "\n"
            )
            self.stale_snapshot_file.flush()
            self._condition.notify_all()

    def _training_owner_snapshot(self):
        return {
            "actor_object_id": id(self.agent.actor),
            "critic_object_ids": [id(self.agent.critic1), id(self.agent.critic2)],
            "target_critic_object_ids": [
                id(self.agent.target_critic1), id(self.agent.target_critic2)
            ],
            "optimizer_object_ids": [
                id(self.agent.actor_optimizer),
                id(self.agent.critic_optimizer),
                id(self.agent.alpha_optimizer),
            ],
            "replay_object_id": None if self.replay is None else id(self.replay),
            "replay_size": None if self.replay is None else len(self.replay),
            "global_environment_step": int(self.agent.global_environment_step),
            "gradient_update_step": int(self.agent.update_step),
            "completed_training_episodes": len(self.episodes),
        }

    def _forest_request_payload(self, assignment, request_id):
        return {
            "request_id": str(request_id),
            "logical_seed": int(assignment.logical_seed),
            "raw_seed": int(assignment.raw_seed),
            "mode": str(assignment.mode),
            "map_block_id": int(assignment.map_block_id),
            "map_round_id": int(assignment.map_round_id),
            "scheduler_order": list(assignment.scheduler_order),
        }

    def _wait_for_forest_assignment(self, assignment, request_id=None):
        deadline = time.monotonic() + self.forest_switch_timeout
        with self._condition:
            while not rospy.is_shutdown():
                state = dict(self._forest_state)
                request_matches = (
                    request_id is None
                    or state.get("request_id") == str(request_id)
                )
                seed_matches = bool(
                    int(state.get("logical_seed", -1)) == assignment.logical_seed
                    and int(state.get("raw_seed", -1)) == assignment.raw_seed
                )
                if request_matches and seed_matches and state.get("status") == "failed":
                    raise RuntimeError(
                        "Forest map manager failed: " + str(state.get("failure", ""))
                    )
                if bool(
                    request_matches and seed_matches
                    and state.get("status") == "ready"
                    and state.get("map_ready", False)
                    and state.get("passed", False)
                    and int(state.get("loaded_obstacle_count", -1)) == 18
                    and int(state.get("residual_obstacle_count", -1)) == 0
                    and state.get("old_obstacles_absent_verified", False)
                ):
                    self._current_forest_assignment = assignment
                    return state
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError(
                        "timeout waiting for Forest map logical seed{}".format(
                            assignment.logical_seed
                        )
                    )
                self._condition.wait(min(remaining, 0.1))
        raise RuntimeError("ROS shutdown while waiting for Forest map")

    def _request_initial_forest(self):
        if not self.forest_enabled:
            return
        assignment = (
            self.forest_scheduler.training_assignment(0)
            if self.mode == "training"
            else self.forest_scheduler.evaluation_assignment(
                self.evaluation_forest_seed, 0
            )
        )
        deadline = time.monotonic() + self.forest_switch_timeout
        while (
            self._forest_request_pub.get_num_connections() <= 0
            and time.monotonic() < deadline
            and not rospy.is_shutdown()
        ):
            time.sleep(0.05)
        if self._forest_request_pub.get_num_connections() <= 0:
            raise RuntimeError("Forest map manager request subscriber unavailable")
        request_id = "{}:{}:initial".format(self.run_id, self.mode)
        request = self._forest_request_payload(assignment, request_id)
        with self._condition:
            self._forest_state = {}
        before = self._training_owner_snapshot()
        self._forest_request_pub.publish(String(data=json.dumps(request, sort_keys=True)))
        ready = self._wait_for_forest_assignment(assignment, request_id=request_id)
        after = self._training_owner_snapshot()
        self._forest_map_switch_audits.append(
            {
                "kind": "initial_load",
                "request": request,
                "ready_state": ready,
                "training_owner_before": before,
                "training_owner_after": after,
                "training_owner_continuous": before == after,
            }
        )

    def _ensure_episode_forest(self, episode_index):
        if not self.forest_enabled:
            return None
        assignment = (
            self.forest_scheduler.training_assignment(episode_index)
            if self.mode == "training"
            else self.forest_scheduler.evaluation_assignment(
                self.evaluation_forest_seed, episode_index
            )
        )
        state = self._wait_for_forest_assignment(assignment)
        if self._pending_map_switch_continuity is not None:
            before = self._pending_map_switch_continuity.pop("training_owner_before")
            after = self._training_owner_snapshot()
            continuous = bool(
                before["actor_object_id"] == after["actor_object_id"]
                and before["critic_object_ids"] == after["critic_object_ids"]
                and before["target_critic_object_ids"] == after["target_critic_object_ids"]
                and before["optimizer_object_ids"] == after["optimizer_object_ids"]
                and before["replay_object_id"] == after["replay_object_id"]
                and before["replay_size"] == after["replay_size"]
                and before["global_environment_step"] == after["global_environment_step"]
                and before["completed_training_episodes"] == after["completed_training_episodes"]
                and after["gradient_update_step"] >= before["gradient_update_step"]
            )
            audit = {
                **self._pending_map_switch_continuity,
                "ready_state": state,
                "training_owner_before": before,
                "training_owner_after": after,
                "training_owner_continuous": continuous,
            }
            self._forest_map_switch_audits.append(audit)
            self._pending_map_switch_continuity = None
            if not continuous:
                raise RuntimeError("SAC/replay state changed discontinuously across map switch")
        return assignment

    def _checkpoint_extra_config(self, completed_episode):
        result = {
            "run_id": self.run_id,
            "mode": self.mode,
            "checkpoint_completed_episode": int(completed_episode),
            "total_training_episodes": self.total_training_episodes,
            "checkpoint_environment_step": int(
                self.agent.global_environment_step
            ),
            "learning_starts": self.learning_starts,
            "replay_capacity": None if self.replay is None else self.replay.capacity,
            "replay_size": None if self.replay is None else len(self.replay),
            "gradient_update_step": int(self.agent.update_step),
            "action_bounds": {
                "v_max_min": self.mapping.v_max_min,
                "v_max_max": self.mapping.v_max_max,
            },
            "random_seed": self.agent.config.seed,
        }
        if self.forest_enabled and self.mode == "training":
            result["forest_scheduler_state"] = self.forest_scheduler.snapshot(
                completed_episode
            )
            result["forest_map_at_checkpoint"] = (
                None if self._current_forest_assignment is None
                else self._current_forest_assignment.as_dict()
            )
            result["training_resume_supported"] = False
            result["training_resume_blocker"] = (
                "Replay Buffer restore is not implemented; scheduler state is "
                "preserved for audit but formal resume remains fail-closed"
            )
        return result

    def _save_checkpoint(self, completed_episode):
        completed_episode = int(completed_episode)
        if any(
            item["completed_episode"] == completed_episode
            for item in self._checkpoint_manifest
        ):
            return
        if len(self.episodes) != completed_episode:
            raise RuntimeError(
                "checkpoint Episode does not equal formally closed Episode count"
            )
        path = self.output_dir / checkpoint_filename(completed_episode)
        if path.exists():
            raise RuntimeError("refusing to overwrite checkpoint: {}".format(path))
        self.agent.save_checkpoint(
            path,
            extra_config=self._checkpoint_extra_config(completed_episode),
        )
        entry = {
            "completed_episode": completed_episode,
            "environment_step": int(self.agent.global_environment_step),
            "replay_size": len(self.replay),
            "gradient_update_step": int(self.agent.update_step),
            "path": str(path),
            "size_bytes": path.stat().st_size,
        }
        self._checkpoint_manifest.append(entry)
        _write_json(
            self.output_dir / "sac_checkpoint_manifest.json",
            self._checkpoint_manifest,
        )
        rospy.logwarn(
            "[SAC TRAINING] checkpoint episode=%d transition=%d "
            "replay=%d updates=%d path=%s",
            completed_episode,
            int(self.agent.global_environment_step),
            len(self.replay),
            int(self.agent.update_step),
            path,
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

    def _record_transition(self, record, normalized_action, generation):
        step = int(record["step_index"])
        normalized_action = float(normalized_action)
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
            "timing": record.get("timing", {}),
            "state_t_provenance": state.get("provenance", {}),
            "state_t_plus_1_provenance": transition["state_t_plus_1"].get(
                "provenance", {}
            ),
        }
        if self._current_forest_assignment is not None:
            audit.update(self._current_forest_assignment.as_dict())
        self.transition_file.write(json.dumps(audit, sort_keys=True) + "\n")
        self.transition_file.flush()
        self._current_episode_audits.append(audit)
        if self.mode == "evaluation":
            self._evaluation_transition_count += 1
            return
        self.agent.global_environment_step += 1
        if self.mode != "training":
            return
        environment_step = int(self.agent.global_environment_step)
        if self.replay is None or len(self.replay) != environment_step:
            raise RuntimeError(
                "1 valid environment step must equal 1 Replay experience"
            )
        if (
            not self._training_learner_enabled
            and learning_started(environment_step, self.learning_starts)
        ):
            self.learner.enable()
            self._training_learner_enabled = True
            rospy.logwarn(
                "SAC training reached learning_starts=%d; stochastic training enabled",
                self.learning_starts,
            )
        self._check_learner_health()
        if environment_step % 100 == 0:
            latest = (
                self.learner.metrics[-1] if self.learner.metrics else {}
            )
            rospy.logwarn(
                "[SAC TRAINING] transition=%d episode=%s completed_episode=%d/%d "
                "reward=%.6f "
                "replay=%d updates=%d critic1_loss=%s actor_loss=%s "
                "alpha=%s action=%.6f v_max=%.6f",
                environment_step,
                record["episode_id"],
                len(self.episodes),
                self.total_training_episodes,
                float(transition["reward"]),
                len(self.replay),
                int(self.agent.update_step),
                latest.get("critic1_loss"),
                latest.get("actor_loss"),
                latest.get("alpha"),
                normalized_action,
                float(transition["action_t"]["requested_v_max"]),
            )

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
                != metrics["actor_action_count"]
                or metrics["accepted_applied_ack_count"]
                != metrics["actor_action_count"]
                or metrics["transition_count"]
                != metrics["actor_action_count"]
            ):
                failures.append("request_action_applied_identity_closure")
            if (
                metrics["maximum_active_action_interval_count"] > 1
                or metrics["active_action_interval_count_at_closure"] != 0
            ):
                failures.append("single_active_action_interval")
            if metrics["generation_mismatch_observation_count"]:
                failures.append("old_generation_contamination")
            if (
                metrics["observation_wait_count"] == 0
                and not 9.5
                <= float(metrics["effective_policy_rate_hz"])
                <= 10.5
            ):
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
        phase_names = sorted(
            {
                name
                for item in audits
                for name in item.get("timing", {})
                .get("phase_wall_sec", {})
            }
        )
        phase_latency = {}
        for name in phase_names:
            values = [
                float(item["timing"]["phase_wall_sec"][name])
                for item in audits
                if name in item.get("timing", {}).get("phase_wall_sec", {})
            ]
            phase_latency[name] = {
                **self._statistics(values),
                "sum": sum(values),
            }
        observation_build_ms = [
            float(
                item["timing"]["state_t_plus_1_telemetry"][
                    "lidar_build_duration_ms"
                ]
            )
            for item in audits
            if item.get("timing", {})
            .get("state_t_plus_1_telemetry", {})
            .get("lidar_build_duration_ms") is not None
        ]
        snapshot_stamps = [
            float(
                item["state_t_plus_1_provenance"]["observation_stamp_sec"]
            )
            for item in audits
            if item.get("state_t_plus_1_provenance", {}).get(
                "observation_stamp_sec"
            ) is not None
        ]
        unique_snapshot_stamps = list(dict.fromkeys(snapshot_stamps))
        snapshot_intervals = [
            later - earlier
            for earlier, later in zip(
                unique_snapshot_stamps, unique_snapshot_stamps[1:]
            )
            if later > earlier
        ]
        producer_queue_lag = [
            float(
                item["timing"]["state_t_plus_1_telemetry"]
                ["producer_input_queue_lag_wall_sec"]
            )
            for item in audits
            if item.get("timing", {})
            .get("state_t_plus_1_telemetry", {})
            .get("producer_input_queue_lag_wall_sec") is not None
        ]
        producer_fusion_ms = [
            float(
                item["timing"]["state_t_plus_1_telemetry"]
                ["producer_fusion_duration_wall_ms"]
            )
            for item in audits
            if item.get("timing", {})
            .get("state_t_plus_1_telemetry", {})
            .get("producer_fusion_duration_wall_ms") is not None
        ]
        coalesced_counts = [
            int(
                item["timing"]["state_t_plus_1_telemetry"]
                ["producer_coalesced_drop_count"]
            )
            for item in audits
            if item.get("timing", {})
            .get("state_t_plus_1_telemetry", {})
            .get("producer_coalesced_drop_count") is not None
        ]
        reason = str(result["episode"].get("terminal_reason", ""))
        summary = {
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
            "phase_latency_wall_sec": phase_latency,
            "observation_v2_build_duration_ms": self._statistics(
                observation_build_ms
            ),
            "observation_unique_snapshot_count": len(unique_snapshot_stamps),
            "observation_unique_snapshot_rate_hz": (
                float(len(unique_snapshot_stamps) - 1)
                / (unique_snapshot_stamps[-1] - unique_snapshot_stamps[0])
                if len(unique_snapshot_stamps) >= 2
                and unique_snapshot_stamps[-1] > unique_snapshot_stamps[0]
                else 0.0
            ),
            "observation_snapshot_interarrival_sec": self._statistics(
                snapshot_intervals
            ),
            "observation_c_producer_queue_lag_wall_sec": self._statistics(
                producer_queue_lag
            ),
            "observation_c_fusion_duration_wall_ms": self._statistics(
                producer_fusion_ms
            ),
            "observation_c_coalesced_drop_count_delta": (
                0
                if not coalesced_counts
                else max(coalesced_counts) - min(coalesced_counts)
            ),
            "terminal_provenance": {
                "coordinator_identity": dict(identity),
                "last_transition_timing": (
                    None if not audits else audits[-1].get("timing", {})
                ),
                "stale_snapshot": dict(
                    self._stale_snapshots_by_episode.get(
                        str(result["episode"].get("episode_id", "")), {}
                    )
                ),
            },
        }
        if self._current_forest_assignment is not None:
            summary.update(self._current_forest_assignment.as_dict())
            summary["episode_id"] = result["episode"].get("episode_id")
            summary["global_step"] = int(self.agent.global_environment_step)
            summary["episode_reward"] = summary["episode_return"]
            summary["success"] = bool(
                summary.get("coordinator_terminal_outcome") == "SUCCESS"
            )
            summary["termination_reason"] = reason
        return summary

    def _complete_episode_closure(self, result, identity):
        """Validate and materialize closure before checkpoint/reset handoff."""

        steps = result["steps"]
        if not steps:
            raise RuntimeError("Episode closed without transitions")
        last = steps[-1]
        expected_steps = list(range(len(steps)))
        actual_steps = [int(step["step_index"]) for step in steps]
        actual_requests = [int(step["request_id"]) for step in steps]
        contiguous = bool(
            actual_steps == expected_steps
            and actual_requests == [step + 1 for step in expected_steps]
        )
        terminal_row_count = sum(
            1
            for step in steps
            if bool(step.get("terminated", False))
            or bool(step.get("truncated", False))
        )
        terminal_closed = bool(
            result["episode"]["terminated"]
            or result["episode"]["truncated"]
        )
        fail_closed_reason = infrastructure_terminal_reason(
            identity.get("terminal_reason", ""),
            result["episode"].get("terminal_reason", ""),
        )
        payload = {
            "version": "astradrone_sac_episode_closure_v1.2",
            "status": result["status"],
            "episode_id": identity["episode_key"],
            "reset_generation": int(identity["reset_generation"]),
            "transition_count": len(steps),
            "last_step_index": int(last["step_index"]),
            "last_request_id": int(last["request_id"]),
            "terminal_transition_closed": terminal_closed,
            "terminated": bool(result["episode"]["terminated"]),
            "truncated": bool(result["episode"]["truncated"]),
            "scheduler_state": result["metrics"]["scheduler_state"],
            "active_action_interval_count": result["metrics"][
                "active_action_interval_count_at_closure"
            ],
            "actor_action_outstanding_count": 0,
            "terminal_row_count": terminal_row_count,
            "replay_step_sequence_contiguous": contiguous,
            "ordered_replay_writer_submitted_count": result["metrics"].get(
                "ordered_replay_writer_submitted_count", -1
            ),
            "ordered_replay_writer_persisted_count": result["metrics"].get(
                "ordered_replay_writer_persisted_count", -1
            ),
            "ordered_replay_writer_pending_count": result["metrics"].get(
                "ordered_replay_writer_pending_at_closure", -1
            ),
            "fail_closed_after_closure": bool(fail_closed_reason),
            "fail_closed_reason": fail_closed_reason,
        }
        # Deprecated telemetry alias retained only for readers of frozen
        # Phase 1 runtime artifacts. Production validation uses the new field.
        payload["open_transition_count"] = payload[
            "active_action_interval_count"
        ]
        if not terminal_closed:
            raise RuntimeError("terminal transition was not closed before reset")
        if not contiguous:
            raise RuntimeError("Episode transition sequence is not contiguous")
        if terminal_row_count != 1:
            raise RuntimeError("Episode must contain exactly one terminal row")
        if payload["scheduler_state"] != "CLOSED":
            raise RuntimeError("Episode scheduler did not reach CLOSED")
        if payload["active_action_interval_count"] != 0:
            raise RuntimeError(
                "Episode closure still has an active action interval"
            )
        if (
            payload["ordered_replay_writer_submitted_count"] != len(steps)
            or payload["ordered_replay_writer_persisted_count"] != len(steps)
            or payload["ordered_replay_writer_pending_count"] != 0
        ):
            raise RuntimeError(
                "Episode closure reached terminal before ordered Replay drain"
            )
        return payload

    def _publish_closure(self, payload):
        self._closure_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _run_one_episode(self, identity):
        self._ensure_episode_forest(len(self.episodes))
        episode_key = str(identity["episode_key"])
        generation = int(identity["reset_generation"])
        self.env.begin_external_episode(
            self.run_id,
            episode_key,
            generation,
            official_trajectory_id=identity.get("trajectory_id"),
            official_trajectory_start_time=identity.get(
                "trajectory_start_time"
            ),
            official_trajectory_start_secs=identity.get(
                "trajectory_start_time_secs"
            ),
            official_trajectory_start_nsecs=identity.get(
                "trajectory_start_time_nsecs"
            ),
        )
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
        actor_ownership = ActorActionOwnership()
        self._current_episode_audits = []
        self._active_actor_timing = None

        def persist_formed_transition(record, payload):
            normalized = float(payload["normalized_action"])
            actor_timing = dict(payload.get("actor_timing", {}))
            transition_wall = float(payload["transition_formed_wall_time_sec"])
            learner_metrics = (
                [] if self.learner is None else list(self.learner.metrics)
            )
            actor_start_wall = float(
                actor_timing.get("actor_start_wall_time_sec", transition_wall)
            )
            overlapping_updates = [
                metric
                for metric in learner_metrics
                if actor_start_wall
                <= float(metric.get("wall_time", -math.inf))
                <= transition_wall
            ]
            learner_durations = [
                float(metric["update_duration_wall_sec"])
                for metric in overlapping_updates
                if metric.get("update_duration_wall_sec") is not None
            ]
            timing = record.setdefault("timing", {})
            timing["actor"] = actor_timing
            timing["transition_formed_wall_time_sec"] = transition_wall
            timing["replay_persist_start_wall_time_sec"] = time.time()
            timing["learner_overlap"] = {
                "update_count": len(overlapping_updates),
                "update_duration_sum_wall_sec": sum(learner_durations),
                "update_duration_max_wall_sec": (
                    max(learner_durations) if learner_durations else 0.0
                ),
                "update_steps": [
                    int(metric.get("update_step", -1))
                    for metric in overlapping_updates
                ],
            }
            phase = timing.setdefault("phase_wall_sec", {})
            phase["agent_lock_wait"] = float(
                actor_timing.get("agent_lock_wait_wall_sec", 0.0)
            )
            phase["actor_inference"] = float(
                actor_timing.get("actor_inference_wall_sec", 0.0)
            )
            phase["learner_update_overlap"] = sum(learner_durations)
            self._record_transition(record, normalized, generation)

        replay_writer = OrderedTransitionWriter(persist_formed_transition)

        def action_provider(state, step_index):
            actor_start_monotonic = time.monotonic()
            actor_start_wall = time.time()
            writer_pending_at_actor_start = replay_writer.pending_count
            writer_submitted_at_actor_start = replay_writer.submitted_count
            writer_persisted_at_actor_start = replay_writer.persisted_count
            vector = flatten_policy_input(state)
            if self.mode == "evaluation":
                deterministic = True
            elif self.mode == "training":
                deterministic = (
                    not learning_started(
                        self.agent.global_environment_step,
                        self.learning_starts,
                    )
                )
            else:
                deterministic = (
                    self.phase_a is None
                    and self.phase_a_deterministic_actor
                )
            lock_wait_started = time.monotonic()
            with self.agent.lock:
                lock_acquired = time.monotonic()
                normalized = (
                    self.mapping.to_normalized(self.evaluation_fixed_vmax)
                    if self.mode == "evaluation" and self.evaluation_fixed_vmax
                    else self.agent.sample_action(vector, deterministic=deterministic)
                )
                inference_finished = time.monotonic()
            usage = resource.getrusage(resource.RUSAGE_SELF)
            try:
                load_average = list(os.getloadavg())
            except OSError:
                load_average = None
            if self._active_actor_timing is not None:
                raise RuntimeError(
                    "Actor timing exists while another action interval is active"
                )
            self._active_actor_timing = (int(step_index), {
                "actor_start_monotonic_sec": actor_start_monotonic,
                "actor_start_wall_time_sec": actor_start_wall,
                "agent_lock_wait_wall_sec": max(
                    0.0, lock_acquired - lock_wait_started
                ),
                "actor_inference_wall_sec": max(
                    0.0, inference_finished - lock_acquired
                ),
                "actor_end_monotonic_sec": inference_finished,
                "actor_end_wall_time_sec": time.time(),
                "ordered_replay_writer_pending_at_actor_start": int(
                    writer_pending_at_actor_start
                ),
                "ordered_replay_writer_submitted_at_actor_start": int(
                    writer_submitted_at_actor_start
                ),
                "ordered_replay_writer_persisted_at_actor_start": int(
                    writer_persisted_at_actor_start
                ),
                "actor_started_while_replay_writer_pending": bool(
                    writer_pending_at_actor_start > 0
                ),
                "runner_user_cpu_sec": float(usage.ru_utime),
                "runner_system_cpu_sec": float(usage.ru_stime),
                "runner_max_rss_mib": float(usage.ru_maxrss) / 1024.0,
                "system_load_average_1_5_15": load_average,
            })
            actor_ownership.propose(step_index, normalized)
            return self.mapping.to_v_max(normalized)

        def action_acceptance_recorder(step_index, accepted):
            actor_ownership.resolve_acceptance(step_index, accepted)
            if not accepted:
                self._active_actor_timing = None

        def transition_submitter(record):
            normalized = actor_ownership.consume_transition(
                record["step_index"]
            )
            timing_item = self._active_actor_timing
            if (
                timing_item is None
                or timing_item[0] != int(record["step_index"])
            ):
                raise RuntimeError(
                    "formed transition has no matching active Actor timing"
                )
            actor_timing = timing_item[1]
            self._active_actor_timing = None
            replay_writer.submit(
                record,
                {
                    "reset_generation": generation,
                    "normalized_action": normalized,
                    "actor_timing": actor_timing,
                    "transition_formed_wall_time_sec": time.time(),
                },
            )

        try:
            result = self.env.run_episode(
                action_provider,
                self.episode_config,
                transition_submitter=transition_submitter,
                action_acceptance_recorder=action_acceptance_recorder,
            )
            replay_writer.close(
                timeout_sec=self.episode_config.completion_timeout_sec
            )
        except BaseException:
            try:
                replay_writer.close(
                    timeout_sec=self.episode_config.completion_timeout_sec
                )
            except BaseException:
                pass
            raise
        result["metrics"].update(
            {
                "ordered_replay_writer_submitted_count": (
                    replay_writer.submitted_count
                ),
                "ordered_replay_writer_persisted_count": (
                    replay_writer.persisted_count
                ),
                "ordered_replay_writer_maximum_pending_count": (
                    replay_writer.maximum_pending_count
                ),
                "ordered_replay_writer_pending_at_closure": (
                    replay_writer.pending_count
                ),
                "replay_writer_blocks_policy_clock": False,
            }
        )
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
        actor_ownership.require_closed()
        if self.replay is not None:
            self.replay.mark_episode_boundary(
                episode_key,
                terminated=result["episode"]["terminated"],
                truncated=result["episode"]["truncated"],
                terminal_reason=result["episode"]["terminal_reason"],
            )
        summary = self._episode_summary(result, terminal_identity)
        self.transition_file.flush()
        closure = self._complete_episode_closure(result, terminal_identity)
        summary["closure"] = closure
        fail_closed_reason = closure["fail_closed_reason"]
        expected_completed_episodes = None
        if self.mode == "training":
            expected_completed_episodes = self.training_schedule.complete_episode(
                len(self.episodes),
                closure["terminal_transition_closed"],
            )
        self.episodes.append(summary)
        if (
            expected_completed_episodes is not None
            and len(self.episodes) != expected_completed_episodes
        ):
            raise RuntimeError("completed training Episode count advanced unexpectedly")
        if self.forest_enabled:
            closure["forest_map"] = self._current_forest_assignment.as_dict()
            closure["next_forest_map_request"] = None
            if (
                self.mode == "training"
                and not fail_closed_reason
                and self.forest_scheduler.map_switch_due_after(
                    len(self.episodes), self.total_training_episodes
                )
            ):
                next_assignment = self.forest_scheduler.training_assignment(
                    len(self.episodes)
                )
                request = self._forest_request_payload(
                    next_assignment,
                    "{}:training:block:{}".format(
                        self.run_id, next_assignment.map_block_id
                    ),
                )
                closure["next_forest_map_request"] = request
                self._pending_map_switch_continuity = {
                    "kind": "map_block_switch",
                    "from_forest": self._current_forest_assignment.as_dict(),
                    "to_forest": next_assignment.as_dict(),
                    "request": request,
                    "training_owner_before": self._training_owner_snapshot(),
                }
        with self._condition:
            self._processed_episodes.add(episode_key)
            self._current_env_episode = ""
        _write_json(self.output_dir / "sac_episode_summaries.json", self.episodes)

        if self.mode == "training" and not fail_closed_reason:
            completed_episodes = len(self.episodes)
            if self.training_schedule.stop_due(completed_episodes):
                # Freeze the final model only after the Episode's last transition
                # is in Replay and its formal closure has completed.
                self.learner.stop()
                self._check_learner_health()
            if self.training_schedule.checkpoint_due(completed_episodes):
                self._save_checkpoint(completed_episodes)
        # The coordinator may reset only after all due checkpoint work completes.
        self._publish_closure(closure)
        if fail_closed_reason:
            self._wait_identity(
                lambda payload: (
                    payload.get("episode_key") == episode_key
                    and payload.get("state")
                    in ("CLOSED", "QUALIFICATION_FAILED")
                ),
                10.0,
                "coordinator infrastructure fail-closed acknowledgement",
            )
            raise RuntimeError(
                "infrastructure terminal fail-closed after exactly-once "
                "Episode closure: {}".format(fail_closed_reason)
            )

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
        checkpoint = self.output_dir / "sac_qualification_checkpoint.pt"
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
            if (
                metrics["maximum_active_action_interval_count"] > 1
                or metrics["active_action_interval_count_at_closure"] != 0
            ):
                failures.append("single_active_action_interval")
            if metrics["generation_mismatch_observation_count"]:
                failures.append("old_generation_contamination")
            if (
                metrics["observation_wait_count"] == 0
                and not 9.5
                <= float(metrics["effective_policy_rate_hz"])
                <= 10.5
            ):
                failures.append("action_rate")
            if metrics["reward_finite_rate"] != 1.0:
                failures.append("non_finite_reward")
            if not metrics.get("policy_tick_indices_strictly_monotonic", False):
                failures.append("policy_tick_identity")
            reason = " ".join(
                (
                    str(episode["episode"].get("terminal_reason", "")),
                    str(episode.get("coordinator_terminal_reason", "")),
                )
            )
            # Training success/collision/planner/truncation are real environment
            # outcomes and all close one completed Episode. Qualification and
            # evaluation still reject unsafe outcomes.
            if self.mode != "training":
                if "planner_failure" in reason:
                    failures.append("planner_failure")
                if "collision" in reason:
                    failures.append("collision")
            if "controller_failure" in reason:
                failures.append("controller_failure")
            if "invalid_observation" in reason:
                failures.append("invalid_observation")
            if "infrastructure:observation_c_producer_stall" in reason:
                failures.append("infrastructure_observation_producer_stall")
        return sorted(set(failures))

    def _finalize_training(self):
        self._check_learner_health()
        self.learner.stop()
        self._check_learner_health()
        metrics = self._write_learner_metrics()
        failures = self._episode_contract_failures()
        if len(self.episodes) != self.total_training_episodes:
            failures.append(
                "completed_episode_target:{}_of_{}".format(
                    len(self.episodes),
                    self.total_training_episodes,
                )
            )
        if (
            self.agent.global_environment_step >= self.learning_starts
            and self.agent.update_step <= 0
        ):
            failures.append("no_gradient_updates")
        replay_audit = self.replay.audit()
        failures.extend(replay_audit["failures"])
        missing_checkpoints = sorted(
            set(self.checkpoint_episodes)
            - set(item["completed_episode"] for item in self._checkpoint_manifest)
        )
        if missing_checkpoints:
            failures.append("missing_checkpoints:{}".format(missing_checkpoints))

        final_checkpoint = self.output_dir / checkpoint_filename(
            self.total_training_episodes
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
                "verdict": (
                    "BOUNDED FOREST MAP-SWITCH SMOKE COMPLETE"
                    if self.smoke_test and not failures
                    else "RL TIMING QUALIFICATION COMPLETE"
                    if self.timing_qualification and not failures
                    else "SAC 100-EPISODE EARLY-LEARNING OBSERVATION COMPLETE"
                    if self.early_learning_observation and not failures
                    else "FORMAL {}-EPISODE TRAINING COMPLETE".format(
                        self.total_training_episodes
                    )
                    if not failures
                    else "NO-GO"
                ),
                "failures": sorted(set(failures)),
                "episode_count": len(self.episodes),
                "total_valid_transitions": self.agent.global_environment_step,
                "global_environment_step": self.agent.global_environment_step,
                "replay_size": len(self.replay),
                "gradient_update_step": self.agent.update_step,
                "learner_update_count": len(metrics),
                "learner_update_hz": learner_hz,
                "replay_audit": replay_audit,
                "checkpoint_manifest": self._checkpoint_manifest,
                "checkpoint_reload_audit": reload_audit,
                "episode_contract_failures": self._episode_contract_failures(),
                "completion_reason": (
                    "bounded_forest_smoke_episode_count_reached"
                    if self.smoke_test
                    else "rl_timing_qualification_episode_count_reached"
                    if self.timing_qualification
                    else "early_learning_100episode_count_reached"
                    if self.early_learning_observation
                    else "training_episode_count_reached"
                ),
            }
        )
        if failures:
            raise RuntimeError("training final audit failed: {}".format(failures))

    def _network_hash(self):
        digest = hashlib.sha256()
        for module in (self.agent.actor, self.agent.critic1, self.agent.critic2,
                       self.agent.target_critic1, self.agent.target_critic2):
            for name, value in sorted(module.state_dict().items()):
                digest.update(name.encode())
                digest.update(value.detach().cpu().numpy().tobytes())
        digest.update(self.agent.log_alpha.detach().cpu().numpy().tobytes())
        return digest.hexdigest()

    def _finalize_evaluation(self):
        failures = self._episode_contract_failures()
        if len(self.episodes) != self.evaluation_episode_count:
            failures.append("evaluation_episode_count")
        evaluation_counters_unchanged = (
            self._evaluation_agent_counters_at_load
            == (
                int(self.agent.global_environment_step),
                int(self.agent.update_step),
            )
        )
        if not evaluation_counters_unchanged:
            failures.append("evaluation_changed_training_counters")
        networks_unchanged = self._network_hash() == self._evaluation_network_hash_at_load
        if not networks_unchanged:
            failures.append("evaluation_changed_network_parameters")
        self.summary.update(
            {
                "status": "completed" if not failures else "failed",
                "verdict": "DETERMINISTIC EVALUATION PASS" if not failures else "NO-GO",
                "failures": sorted(set(failures)),
                "episode_count": len(self.episodes),
                "configured_evaluation_episode_count": (
                    self.evaluation_episode_count
                ),
                "evaluation_transition_count": self._evaluation_transition_count,
                "evaluation_checkpoint_episode": (
                    self.evaluation_checkpoint_episode
                ),
                "evaluation_checkpoint_path": self.evaluation_checkpoint_path,
                "episode_contract_failures": self._episode_contract_failures(),
                "training_counters_unchanged": evaluation_counters_unchanged,
                "evaluation_fixed_vmax": self.evaluation_fixed_vmax,
                "network_parameters_unchanged": networks_unchanged,
                "network_sha256_at_load": self._evaluation_network_hash_at_load,
                "replay_buffer_created": self.replay is not None,
                "learner_created": self.learner is not None,
            }
        )
        if failures:
            raise RuntimeError(
                "deterministic evaluation failed: {}".format(failures)
            )

    def run(self):
        try:
            if self.startup_contract_only:
                if self.learner is not None:
                    self.learner.stop()
                self.summary.update(
                    {
                        "status": "completed",
                        "verdict": "STARTUP ACTION/CAPABILITY CONTRACT PASS",
                        "episode_count": 0,
                        "global_environment_step": 0,
                        "gradient_update_step": 0,
                        "map_request_count": 0,
                    }
                )
                return self.summary
            self._request_initial_forest()
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
                if self.mode == "training":
                    completed_episodes = len(self.episodes)
                    if self.training_schedule.stop_due(completed_episodes):
                        raise RuntimeError(
                            "coordinator attempted to start Episode after the "
                            "formal training Episode target"
                        )
                    self.training_schedule.next_episode_number(
                        completed_episodes
                    )
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
            elif self.mode == "training":
                self._finalize_training()
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
            self.stale_snapshot_file.close()
            self.summary["stale_observation_snapshot_event_count"] = (
                self._stale_snapshot_count
            )
            self.summary["stale_observation_snapshot_episode_count"] = len(
                self._stale_snapshots_by_episode
            )
            self.summary["stale_observation_snapshot_file"] = str(
                self.output_dir / "sac_stale_observation_snapshots.jsonl"
            )
            self.summary["forest_map_switch_audits"] = self._forest_map_switch_audits
            self.summary["pending_forest_map_switch_continuity"] = (
                self._pending_map_switch_continuity
            )
            if self.forest_enabled and self.mode == "training":
                self.summary["forest_scheduler_state"] = (
                    self.forest_scheduler.snapshot(len(self.episodes))
                )
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
