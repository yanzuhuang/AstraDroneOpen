"""Pure identity and generation gates for training Episode/reset integration."""

from dataclasses import dataclass
import math
import random
from typing import Callable, Optional, Tuple


@dataclass(frozen=True)
class StaticObstacleXY:
    name: str
    x: float
    y: float
    radius_at_hover_z: float

    def validate(self):
        values = (self.x, self.y, self.radius_at_hover_z)
        if not self.name or not all(math.isfinite(value) for value in values):
            raise ValueError("static reset obstacle must be named and finite")
        if self.radius_at_hover_z < 0.0:
            raise ValueError("static reset obstacle radius must be non-negative")


@dataclass(frozen=True)
class ResetCandidate:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class RandomResetConfig:
    enabled: bool
    center_x: float
    center_y: float
    x_min_offset: float
    x_max_offset: float
    y_min_offset: float
    y_max_offset: float
    z: float
    yaw: float
    seed: int
    max_sampling_attempts: int
    uav_collision_radius_xy: float
    ego_obstacles_inflation: float
    additional_static_clearance: float
    static_obstacles: Tuple[StaticObstacleXY, ...] = ()

    def validate(self):
        numeric = (
            self.center_x,
            self.center_y,
            self.x_min_offset,
            self.x_max_offset,
            self.y_min_offset,
            self.y_max_offset,
            self.z,
            self.yaw,
            self.uav_collision_radius_xy,
            self.ego_obstacles_inflation,
            self.additional_static_clearance,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("random reset configuration must be finite")
        if self.x_min_offset > self.x_max_offset:
            raise ValueError("random reset x offsets are reversed")
        if self.y_min_offset > self.y_max_offset:
            raise ValueError("random reset y offsets are reversed")
        if self.z <= 0.0 or self.max_sampling_attempts <= 0:
            raise ValueError("random reset z/attempt count is invalid")
        if min(
            self.uav_collision_radius_xy,
            self.ego_obstacles_inflation,
            self.additional_static_clearance,
        ) < 0.0:
            raise ValueError("random reset clearances must be non-negative")
        for obstacle in self.static_obstacles:
            obstacle.validate()

    @property
    def nominal(self):
        return ResetCandidate(self.center_x, self.center_y, self.z, self.yaw)


class ResetSamplingError(RuntimeError):
    def __init__(self, attempts):
        self.attempts = tuple(attempts)
        super().__init__(
            "no safe random reset candidate after {} attempts".format(
                len(self.attempts)
            )
        )


def validate_reset_candidate(candidate, config):
    config.validate()
    reasons = []
    values = (candidate.x, candidate.y, candidate.z, candidate.yaw)
    if not all(math.isfinite(value) for value in values):
        reasons.append("non_finite_candidate")
    x_offset = candidate.x - config.center_x
    y_offset = candidate.y - config.center_y
    if not config.x_min_offset <= x_offset <= config.x_max_offset:
        reasons.append("x_out_of_bounds")
    if not config.y_min_offset <= y_offset <= config.y_max_offset:
        reasons.append("y_out_of_bounds")
    if abs(candidate.z - config.z) > 1.0e-12:
        reasons.append("z_not_fixed")
    if abs(candidate.yaw - config.yaw) > 1.0e-12:
        reasons.append("yaw_not_fixed")

    minimum_clearance = None
    limiting_obstacle = None
    required_padding = (
        config.uav_collision_radius_xy
        + config.ego_obstacles_inflation
        + config.additional_static_clearance
    )
    if not reasons:
        for obstacle in config.static_obstacles:
            center_distance = math.hypot(
                candidate.x - obstacle.x, candidate.y - obstacle.y
            )
            clearance = (
                center_distance
                - obstacle.radius_at_hover_z
                - required_padding
            )
            if minimum_clearance is None or clearance < minimum_clearance:
                minimum_clearance = clearance
                limiting_obstacle = obstacle.name
            if clearance < 0.0:
                reasons.append("static_obstacle_clearance:" + obstacle.name)
    return {
        "valid": not reasons,
        "reasons": reasons,
        "minimum_static_clearance_m": minimum_clearance,
        "limiting_static_obstacle": limiting_obstacle,
        "required_padding_m": required_padding,
    }


class RandomResetSampler:
    """Deterministic per-run reset sampler with fail-closed validation."""

    def __init__(self, config):
        config.validate()
        self.config = config
        self._rng = random.Random(int(config.seed))
        self._sample_index = 0

    def sample(
        self,
        validator: Optional[
            Callable[[ResetCandidate, RandomResetConfig], dict]
        ] = None,
    ):
        check = validate_reset_candidate if validator is None else validator
        self._sample_index += 1
        attempts = []
        attempt_limit = self.config.max_sampling_attempts if self.config.enabled else 1
        for attempt_count in range(1, attempt_limit + 1):
            if self.config.enabled:
                candidate = ResetCandidate(
                    x=self.config.center_x
                    + self._rng.uniform(
                        self.config.x_min_offset,
                        self.config.x_max_offset,
                    ),
                    y=self.config.center_y
                    + self._rng.uniform(
                        self.config.y_min_offset,
                        self.config.y_max_offset,
                    ),
                    z=self.config.z,
                    yaw=self.config.yaw,
                )
            else:
                candidate = self.config.nominal
            validation = dict(check(candidate, self.config))
            attempt = {
                "attempt_count": attempt_count,
                "candidate": {
                    "x": candidate.x,
                    "y": candidate.y,
                    "z": candidate.z,
                    "yaw": candidate.yaw,
                },
                "validation": validation,
            }
            attempts.append(attempt)
            if validation.get("valid") is True:
                return {
                    "candidate": candidate,
                    "sample_index": self._sample_index,
                    "attempt_count": attempt_count,
                    "candidate_validation_result": validation,
                    "sampling_attempts": attempts,
                }
        raise ResetSamplingError(attempts)


@dataclass(frozen=True)
class EpisodeBinding:
    episode_id: int
    reset_generation: int

    @property
    def episode_key(self):
        return "training_episode_{:06d}".format(self.episode_id)

    def validate(self):
        if self.episode_id <= 0:
            raise ValueError("episode_id must be positive")
        if self.reset_generation < 0:
            raise ValueError("reset_generation must be non-negative")


class EpisodeIdentityLedger:
    """Own the exact one-reset/one-generation/one-next-episode relation."""

    def __init__(self):
        self._current = EpisodeBinding(episode_id=1, reset_generation=0)

    @property
    def current(self):
        return self._current

    def advance_after_reset(self, observed_generation):
        expected = self._current.reset_generation + 1
        if int(observed_generation) != expected:
            raise ValueError(
                "reset generation must advance exactly once: expected {}, got {}"
                .format(expected, observed_generation)
            )
        self._current = EpisodeBinding(
            episode_id=self._current.episode_id + 1,
            reset_generation=expected,
        )
        return self._current


def action_matches(binding, episode_key, step_index, request_id):
    binding.validate()
    return bool(
        str(episode_key) == binding.episode_key
        and int(step_index) == 0
        and int(request_id) == binding.episode_id
    )


def sac_closure_matches(binding, payload):
    """Require exact terminal-transition closure before coordinator reset."""

    binding.validate()
    if not isinstance(payload, dict):
        return False
    try:
        return bool(
            str(payload.get("episode_id", "")) == binding.episode_key
            and int(payload.get("reset_generation", -1))
            == binding.reset_generation
            and bool(payload.get("terminal_transition_closed", False))
            and int(payload.get("transition_count", 0)) > 0
            and int(payload.get("last_step_index", -1)) >= 0
            and int(payload.get("last_request_id", 0))
            == int(payload.get("last_step_index", -1)) + 1
            and str(payload.get("status", "")) == "completed"
        )
    except (TypeError, ValueError):
        return False


def episode_count_stop_due(completed_episodes, episode_count):
    """Return true only at the configured completed-Episode boundary."""

    completed = int(completed_episodes)
    configured = int(episode_count)
    if completed < 0:
        raise ValueError("completed Episode count is invalid")
    if configured <= 0 or completed > configured:
        raise ValueError("fixed Episode count is invalid or exceeded")
    return completed == configured


def trajectory_matches(
    binding,
    trajectory_id,
    trajectory_start_sec,
    callback_sequence,
    goal_sequence,
    goal_stamp_sec,
    reset_barrier_sec,
    previous_trajectory_id,
):
    binding.validate()
    return bool(
        int(trajectory_id) > int(previous_trajectory_id)
        and int(callback_sequence) > int(goal_sequence)
        and float(trajectory_start_sec) > float(reset_barrier_sec)
        and float(trajectory_start_sec) + 1.0e-9 >= float(goal_stamp_sec)
    )


def observation_matches(
    binding,
    valid,
    temporal_generation,
    lidar_temporal_generation,
    source_stamp_sec,
    reset_barrier_sec,
    trajectory_id,
    accepted_trajectory_id,
):
    binding.validate()
    return bool(
        valid
        and int(temporal_generation) == binding.reset_generation
        and int(lidar_temporal_generation) == binding.reset_generation
        and float(source_stamp_sec) > float(reset_barrier_sec)
        and int(trajectory_id) >= int(accepted_trajectory_id)
    )


def fixed_terminal_hold_matches(
    *,
    observation_valid,
    observation_diagnostics,
    trajectory_lookup_result,
    observation_stamp_sec,
    observation_latest_trajectory_id,
    trajectory_id,
    trajectory_end_sec,
    position_command_trajectory_id,
    position_command_flag,
    position_command_age_sec,
    position_command_distance_to_goal,
    position_command_speed,
    actual_distance_to_goal,
    goal_position_tolerance,
    goal_speed_tolerance,
    freshness_limit_sec,
    ready_flag=1,
):
    """Recognize only the fixed-qualification post-B-spline goal hold.

    Observation C correctly has no future trajectory after the formal B-spline
    ends, while traj_server keeps publishing the final zero-velocity
    PositionCommand so the real vehicle can settle.  This gate must not accept
    an ordinary in-flight trajectory loss.
    """

    try:
        numeric = tuple(
            float(value)
            for value in (
                observation_stamp_sec,
                trajectory_end_sec,
                position_command_age_sec,
                position_command_distance_to_goal,
                position_command_speed,
                actual_distance_to_goal,
                goal_position_tolerance,
                goal_speed_tolerance,
                freshness_limit_sec,
            )
        )
        if not all(math.isfinite(value) for value in numeric):
            return False
        diagnostics = tuple(str(value) for value in observation_diagnostics)
        return bool(
            not bool(observation_valid)
            and diagnostics == ("trajectory_unavailable",)
            and str(trajectory_lookup_result)
            == "no_active_trajectory_at_stamp"
            and float(observation_stamp_sec) > float(trajectory_end_sec)
            and int(trajectory_id) > 0
            and int(observation_latest_trajectory_id) == int(trajectory_id)
            and int(position_command_trajectory_id) == int(trajectory_id)
            and int(position_command_flag) == int(ready_flag)
            and 0.0 <= float(position_command_age_sec)
            <= float(freshness_limit_sec)
            and float(position_command_distance_to_goal)
            <= float(goal_position_tolerance)
            and float(position_command_speed) <= float(goal_speed_tolerance)
            and float(actual_distance_to_goal)
            <= float(goal_position_tolerance)
            and float(goal_position_tolerance) > 0.0
            and float(goal_speed_tolerance) >= 0.0
            and float(freshness_limit_sec) > 0.0
        )
    except (TypeError, ValueError):
        return False
