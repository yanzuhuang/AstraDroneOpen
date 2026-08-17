"""Versioned SAC transition contract without a reward implementation.

This module is deliberately framework-neutral.  It freezes the data boundary
used by offline calibration and a future reviewed SAC implementation; it does
not calculate rewards, instantiate a policy, or publish a speed command.
"""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Tuple

import numpy as np


SAC_TRANSITION_VERSION = "learning_speed_sac_transition_v1.0"
POLICY_STATE_VERSION = "learning_speed_policy_state_v1.0"
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


@dataclass(frozen=True)
class AppliedSpeedAction:
    requested_v_max: float
    requested_stamp_sec: float
    filtered_v_max: float
    filtered_stamp_sec: float
    applied_v_max: float
    applied_stamp_sec: float
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


@dataclass(frozen=True)
class SacTransitionV1:
    state_t: PolicyStateV1
    action_t: AppliedSpeedAction
    state_t_plus_1: PolicyStateV1
    reward: Optional[float]
    reward_defined: bool
    terminated: bool
    truncated: bool
    terminal_reason: str = ""
    version: str = SAC_TRANSITION_VERSION

    def __post_init__(self):
        if self.version != SAC_TRANSITION_VERSION:
            raise ValueError("unsupported transition version")
        if self.terminated and self.truncated:
            raise ValueError("a transition cannot be both terminated and truncated")
        if self.reward_defined:
            if self.reward is None or not math.isfinite(self.reward):
                raise ValueError("defined reward must be finite")
        elif self.reward is not None:
            raise ValueError("calibration transition reward must remain null")

        state_provenance = self.state_t.provenance
        next_provenance = self.state_t_plus_1.provenance
        if state_provenance.official_trajectory != self.action_t.latest_official_trajectory:
            raise ValueError(
                "state_t did not use the latest official B-spline available before action_t"
            )
        if state_provenance.observation_receive_sec > self.action_t.requested_stamp_sec:
            raise ValueError("state_t was received after action_t was requested")
        if next_provenance.observation_stamp_sec <= state_provenance.observation_stamp_sec:
            raise ValueError("state_t+1 must be newer than state_t")
        if next_provenance.observation_receive_sec < self.action_t.applied_stamp_sec:
            raise ValueError("state_t+1 was received before action_t was applied")

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
            "reward": self.reward,
            "reward_defined": self.reward_defined,
            "terminated": self.terminated,
            "truncated": self.truncated,
            "terminal_reason": self.terminal_reason,
            "training_ready": self.training_ready,
        }
