"""Versioned SAC transition contract with an optional reviewed reward.

This module is deliberately framework-neutral.  It freezes the data boundary
used by offline calibration and a future reviewed SAC implementation.  Reward
calculation remains a separate strategy module; this contract only validates
and binds its audit record.  It never instantiates a policy or publishes a
speed command.
"""

from dataclasses import dataclass, replace
import copy
import math
from typing import Dict, Iterable, Mapping, Optional, Tuple

import numpy as np


SAC_TRANSITION_VERSION = "learning_speed_sac_transition_v1.3"
POLICY_STATE_VERSION = "learning_speed_policy_state_v1.0"
PROGRESS_REWARD_CONTEXT_VERSION = "learning_speed_progress_reward_context_v1.0"
LIDAR_BINS = 3200
FUTURE_POSITION_SAMPLES = 20

POLICY_INPUT_FIELDS: Tuple[str, ...] = (
    "lidar_surrogate",
    "future_positions_body",
    "actual_velocity_body",
    "tracking_error_body",
    "previous_applied_v_max",
)

DIAGNOSTIC_ONLY_FIELDS: Tuple[str, ...] = (
    "mission_state",
    "mission_progress",
    "progress_receipt_ros_time",
    "reward_context",
    "run_episode_provenance",
    "planner_state",
    "clearance",
    "lidar_valid_mask",
    "lidar_unknown_mask",
    "lidar_semantic",
    "diagnostics",
)


def _finite_array(values, shape, name):
    result = np.asarray(values, dtype=np.float32)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError("{} must be finite with shape {}".format(name, shape))
    return result


def causal_observation_receipt_time(
    callback_ros_time_sec, producer_receive_sec, source_stamp_sec,
):
    """Return a causal receipt lower bound across independently delivered /clock."""
    values = (
        float(callback_ros_time_sec),
        float(producer_receive_sec),
        float(source_stamp_sec),
    )
    if not all(math.isfinite(value) and value >= 0.0 for value in values):
        raise ValueError("observation receipt times must be finite and non-negative")
    return max(values)


def official_trajectory_identity_from_bspline(message):
    """Convert the official ROS B-spline identity without importing ROS here."""

    return OfficialTrajectoryIdentity(
        trajectory_id=int(message.traj_id),
        start_time_sec=message.start_time.to_sec(),
        source_frame=message.frame_id.lstrip("/"),
    )


def official_trajectory_identity_from_observation(message):
    """Return the official trajectory identity embedded in Observation C."""

    return OfficialTrajectoryIdentity(
        trajectory_id=int(message.trajectory_id),
        start_time_sec=message.trajectory_start_time.to_sec(),
        source_frame=message.trajectory_source_frame.lstrip("/"),
    )


@dataclass(frozen=True)
class OfficialTrajectoryIdentity:
    trajectory_id: int
    start_time_sec: float
    source_frame: str

    def __post_init__(self):
        if self.trajectory_id < 0:
            raise ValueError("trajectory_id must be non-negative")
        if not math.isfinite(self.start_time_sec) or self.start_time_sec <= 0.0:
            raise ValueError("trajectory start time must be positive and finite")
        if not self.source_frame:
            raise ValueError("trajectory source frame is empty")


@dataclass(frozen=True)
class PolicyStateProvenance:
    """Metadata used to prove causality; none of it is a policy input."""

    observation_stamp_sec: float
    observation_receive_sec: float
    body_frame: str
    observation_version: str
    official_trajectory: OfficialTrajectoryIdentity

    def __post_init__(self):
        if (
            not math.isfinite(self.observation_stamp_sec)
            or self.observation_stamp_sec <= 0.0
            or not math.isfinite(self.observation_receive_sec)
            or self.observation_receive_sec < self.observation_stamp_sec
        ):
            raise ValueError("observation timestamps are invalid")
        if not self.body_frame or not self.observation_version:
            raise ValueError("observation frame/version is empty")


@dataclass(frozen=True)
class PolicyStateV1:
    lidar_surrogate: np.ndarray
    future_positions_body: np.ndarray
    actual_velocity_body: np.ndarray
    tracking_error_body: np.ndarray
    previous_applied_v_max: float
    provenance: PolicyStateProvenance
    version: str = POLICY_STATE_VERSION

    def __post_init__(self):
        object.__setattr__(
            self,
            "lidar_surrogate",
            _finite_array(self.lidar_surrogate, (LIDAR_BINS,), "lidar_surrogate"),
        )
        object.__setattr__(
            self,
            "future_positions_body",
            _finite_array(
                self.future_positions_body,
                (FUTURE_POSITION_SAMPLES, 3),
                "future_positions_body",
            ),
        )
        object.__setattr__(
            self,
            "actual_velocity_body",
            _finite_array(self.actual_velocity_body, (3,), "actual_velocity_body"),
        )
        object.__setattr__(
            self,
            "tracking_error_body",
            _finite_array(self.tracking_error_body, (3,), "tracking_error_body"),
        )
        if (
            not math.isfinite(self.previous_applied_v_max)
            or self.previous_applied_v_max <= 0.0
        ):
            raise ValueError("previous_applied_v_max must be positive and finite")
        if self.version != POLICY_STATE_VERSION:
            raise ValueError("unsupported policy-state version")

    def policy_input(self) -> Dict[str, object]:
        """Return exactly the five frozen policy inputs, without diagnostics."""
        return {
            "lidar_surrogate": self.lidar_surrogate.tolist(),
            "future_positions_body": self.future_positions_body.tolist(),
            "actual_velocity_body": self.actual_velocity_body.tolist(),
            "tracking_error_body": self.tracking_error_body.tolist(),
            "previous_applied_v_max": float(self.previous_applied_v_max),
        }

    def to_record(self) -> Dict[str, object]:
        result = self.policy_input()
        trajectory = self.provenance.official_trajectory
        result["provenance"] = {
            "observation_stamp_sec": self.provenance.observation_stamp_sec,
            "observation_receive_sec": self.provenance.observation_receive_sec,
            "body_frame": self.provenance.body_frame,
            "observation_version": self.provenance.observation_version,
            "official_trajectory": {
                "trajectory_id": trajectory.trajectory_id,
                "start_time_sec": trajectory.start_time_sec,
                "source_frame": trajectory.source_frame,
            },
        }
        result["version"] = self.version
        return result


def policy_state_from_observation_c(message, receive_sec):
    """Build the frozen policy state from one valid atomic Observation C."""

    if not message.valid:
        return None
    positions = np.asarray(
        [[point.x, point.y, point.z] for point in message.future_positions_body],
        dtype=np.float32,
    )
    velocity = message.actual_velocity_body
    tracking = message.tracking_error_body
    return PolicyStateV1(
        lidar_surrogate=np.asarray(message.lidar_surrogate, dtype=np.float32),
        future_positions_body=positions,
        actual_velocity_body=np.asarray(
            [velocity.x, velocity.y, velocity.z], dtype=np.float32
        ),
        tracking_error_body=np.asarray(
            [tracking.x, tracking.y, tracking.z], dtype=np.float32
        ),
        previous_applied_v_max=float(message.previous_v_max),
        provenance=PolicyStateProvenance(
            observation_stamp_sec=message.header.stamp.to_sec(),
            observation_receive_sec=float(receive_sec),
            body_frame=message.header.frame_id.lstrip("/"),
            observation_version=message.version,
            official_trajectory=official_trajectory_identity_from_observation(
                message
            ),
        ),
    )


@dataclass(frozen=True)
class AppliedSpeedAction:
    requested_v_max: float
    requested_stamp_sec: float
    filtered_v_max: float
    filtered_stamp_sec: float
    applied_v_max: float
    applied_stamp_sec: float
    # Action-time trajectory provenance is diagnostic. It is intentionally
    # independent of the valid trajectory snapshot embedded in state_t.
    latest_official_trajectory: OfficialTrajectoryIdentity

    def __post_init__(self):
        values = (self.requested_v_max, self.filtered_v_max, self.applied_v_max)
        stamps = (
            self.requested_stamp_sec,
            self.filtered_stamp_sec,
            self.applied_stamp_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("all v_max action values must be positive and finite")
        if (
            not all(math.isfinite(stamp) and stamp > 0.0 for stamp in stamps)
            or not self.requested_stamp_sec
            <= self.filtered_stamp_sec
            <= self.applied_stamp_sec
        ):
            raise ValueError("action timestamps must be causal")


def latest_official_trajectory_before(
    trajectory_history: Iterable[Tuple[float, OfficialTrajectoryIdentity]],
    action_stamp_sec: float,
):
    """Return the newest official trajectory received no later than action_t."""

    stamp = float(action_stamp_sec)
    if not math.isfinite(stamp) or stamp <= 0.0:
        raise ValueError("action stamp must be positive and finite")
    latest = None
    for receive_sec, identity in trajectory_history:
        if float(receive_sec) <= stamp + 1.0e-9:
            latest = identity
    return latest


def is_causal_next_policy_state(
    state_t: PolicyStateV1,
    state_t_plus_1: PolicyStateV1,
    applied_receive_sec: float,
    not_before_receive_sec: Optional[float] = None,
) -> bool:
    """Check recorder freshness plus an optional post-hold receive barrier."""

    boundary = float(applied_receive_sec)
    if not math.isfinite(boundary) or boundary < 0.0:
        return False
    if not_before_receive_sec is not None:
        hold_boundary = float(not_before_receive_sec)
        if not math.isfinite(hold_boundary) or hold_boundary < boundary:
            return False
        boundary = hold_boundary
    return bool(
        state_t_plus_1.provenance.observation_stamp_sec
        > state_t.provenance.observation_stamp_sec
        and state_t_plus_1.provenance.observation_receive_sec >= boundary
    )


@dataclass(frozen=True)
class RunEpisodeProvenance:
    """Stable identity for reward context; never part of the policy input."""

    run_id: str
    episode_id: str

    def __post_init__(self):
        if not self.run_id or not self.episode_id:
            raise ValueError("run and episode provenance must be nonempty")

    def to_record(self) -> Dict[str, str]:
        return {"run_id": self.run_id, "episode_id": self.episode_id}


@dataclass(frozen=True)
class ProgressContextState:
    """Headerless mission progress/state received no later than one observation."""

    mission_progress: float
    progress_receipt_ros_time_sec: float
    mission_state: str
    mission_state_receipt_ros_time_sec: float
    observation_receive_sec: float

    def __post_init__(self):
        times = (
            self.progress_receipt_ros_time_sec,
            self.mission_state_receipt_ros_time_sec,
            self.observation_receive_sec,
        )
        if (
            not math.isfinite(self.mission_progress)
            or self.mission_progress < 0.0
            or self.mission_progress > 1.0
        ):
            raise ValueError("mission progress must be finite and in [0, 1]")
        if not all(math.isfinite(value) and value >= 0.0 for value in times):
            raise ValueError("reward-context receipt times must be finite and non-negative")
        if self.progress_receipt_ros_time_sec > self.observation_receive_sec:
            raise ValueError("progress was received after its observation")
        if self.mission_state_receipt_ros_time_sec > self.observation_receive_sec:
            raise ValueError("mission state was received after its observation")
        if not self.mission_state:
            raise ValueError("mission state is empty")

    def to_record(self) -> Dict[str, object]:
        return {
            "mission_progress": float(self.mission_progress),
            "progress_receipt_ros_time_sec": self.progress_receipt_ros_time_sec,
            "mission_state": self.mission_state,
            "mission_state_receipt_ros_time_sec": self.mission_state_receipt_ros_time_sec,
            "observation_receive_sec": self.observation_receive_sec,
        }


@dataclass(frozen=True)
class ProgressRewardContext:
    """Causal task-progress pair recorded for a future reward implementation."""

    provenance: RunEpisodeProvenance
    state_t: ProgressContextState
    state_t_plus_1: ProgressContextState
    version: str = PROGRESS_REWARD_CONTEXT_VERSION

    def __post_init__(self):
        if self.version != PROGRESS_REWARD_CONTEXT_VERSION:
            raise ValueError("unsupported progress reward-context version")
        if self.state_t_plus_1.observation_receive_sec <= self.state_t.observation_receive_sec:
            raise ValueError("progress context state_t+1 must be newer than state_t")

    @property
    def delta_p(self) -> float:
        return self.state_t_plus_1.mission_progress - self.state_t.mission_progress

    @property
    def monotonic(self) -> bool:
        return self.delta_p >= -1.0e-12

    def to_record(self) -> Dict[str, object]:
        return {
            "version": self.version,
            "provenance": self.provenance.to_record(),
            "state_t": self.state_t.to_record(),
            "state_t_plus_1": self.state_t_plus_1.to_record(),
            "P_t": float(self.state_t.mission_progress),
            "P_t_plus_1": float(self.state_t_plus_1.mission_progress),
            "Delta_P": float(self.delta_p),
            "monotonic": self.monotonic,
        }


@dataclass(frozen=True)
class SacTransitionV1:
    state_t: PolicyStateV1
    action_t: AppliedSpeedAction
    state_t_plus_1: PolicyStateV1
    reward_context: ProgressRewardContext
    reward: Optional[float]
    reward_defined: bool
    terminated: bool
    truncated: bool
    terminal_reason: str = ""
    dangerous_terminal: bool = False
    reward_version: str = ""
    reward_components: Optional[Dict[str, object]] = None
    version: str = SAC_TRANSITION_VERSION

    def __post_init__(self):
        if self.version != SAC_TRANSITION_VERSION:
            raise ValueError("unsupported transition version")
        if self.terminated and self.truncated:
            raise ValueError("a transition cannot be both terminated and truncated")
        if self.dangerous_terminal and not self.terminated:
            raise ValueError("dangerous terminal must terminate the transition")
        if self.reward_defined:
            if self.reward is None or not math.isfinite(self.reward):
                raise ValueError("defined reward must be finite")
            self._validate_reward_components()
        elif self.reward is not None:
            raise ValueError("calibration transition reward must remain null")
        elif self.reward_version or self.reward_components is not None:
            raise ValueError("undefined reward cannot carry reward metadata")

        state_provenance = self.state_t.provenance
        next_provenance = self.state_t_plus_1.provenance
        if state_provenance.observation_receive_sec > self.action_t.requested_stamp_sec:
            raise ValueError("state_t was received after action_t was requested")
        if next_provenance.observation_stamp_sec <= state_provenance.observation_stamp_sec:
            raise ValueError("state_t+1 must be newer than state_t")
        if next_provenance.observation_receive_sec < self.action_t.applied_stamp_sec:
            raise ValueError("state_t+1 was received before action_t was applied")
        if (
            abs(
                self.reward_context.state_t.observation_receive_sec
                - state_provenance.observation_receive_sec
            )
            > 1.0e-9
            or abs(
                self.reward_context.state_t_plus_1.observation_receive_sec
                - next_provenance.observation_receive_sec
            )
            > 1.0e-9
        ):
            raise ValueError("reward context is not bound to transition observations")

    def _validate_reward_components(self) -> None:
        if not self.reward_version or not isinstance(self.reward_components, Mapping):
            raise ValueError("defined reward requires versioned reward components")
        required = {
            "reward_total",
            "reward_speed",
            "reward_smoothing",
            "reward_error",
            "reward_danger",
            "phi_1",
            "phi_2",
            "complexity_context",
        }
        missing = required.difference(self.reward_components)
        if missing:
            raise ValueError(
                "defined reward components missing {}".format(sorted(missing))
            )
        forbidden = {"reward_tracking", "r_tracking", "reward_progress", "r_progress"}
        present_forbidden = forbidden.intersection(self.reward_components)
        if present_forbidden:
            raise ValueError(
                "Learning Speed reward contains forbidden terms {}".format(
                    sorted(present_forbidden)
                )
            )
        for name in (
            "reward_total",
            "reward_speed",
            "reward_smoothing",
            "reward_error",
            "reward_danger",
            "phi_1",
            "phi_2",
        ):
            value = self.reward_components[name]
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("reward component {} must be finite".format(name))
        if abs(float(self.reward_components["reward_total"]) - self.reward) > 1.0e-12:
            raise ValueError("reward total does not match transition reward")

    def with_defined_reward(self, evaluation):
        """Return a new transition bound to one valid reward evaluation."""
        if self.reward_defined:
            raise ValueError("transition reward is already defined")
        record = evaluation.to_record()
        if not record.get("reward_valid", False):
            raise ValueError("invalid reward evaluation cannot create a transition")
        return replace(
            self,
            reward=float(record["reward_total"]),
            reward_defined=True,
            reward_version=str(record["version"]),
            reward_components=copy.deepcopy(record),
        )

    @property
    def training_ready(self) -> bool:
        return self.reward_defined

    def to_record(self) -> Dict[str, object]:
        action = self.action_t
        return {
            "version": self.version,
            "state_t": self.state_t.to_record(),
            "action_t": {
                "requested_v_max": action.requested_v_max,
                "requested_stamp_sec": action.requested_stamp_sec,
                "filtered_v_max": action.filtered_v_max,
                "filtered_stamp_sec": action.filtered_stamp_sec,
                "applied_v_max": action.applied_v_max,
                "applied_stamp_sec": action.applied_stamp_sec,
                "latest_official_trajectory": {
                    "trajectory_id": action.latest_official_trajectory.trajectory_id,
                    "start_time_sec": action.latest_official_trajectory.start_time_sec,
                    "source_frame": action.latest_official_trajectory.source_frame,
                },
            },
            "state_t_plus_1": self.state_t_plus_1.to_record(),
            "reward_context": self.reward_context.to_record(),
            "reward": self.reward,
            "reward_defined": self.reward_defined,
            "reward_version": self.reward_version or None,
            "reward_components": self.reward_components,
            "terminated": self.terminated,
            "truncated": self.truncated,
            "terminal_reason": self.terminal_reason,
            "dangerous_terminal": self.dangerous_terminal,
            "training_ready": self.training_ready,
        }
