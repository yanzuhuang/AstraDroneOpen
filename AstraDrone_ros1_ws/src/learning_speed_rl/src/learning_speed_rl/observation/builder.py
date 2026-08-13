"""Low-dimensional vehicle/planning-state observation builder."""

from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from .types import LOW_DIM_FIELDS, MapTensorSpec, PolicyObservation, vector3


@dataclass
class VehiclePlanningState:
    stamp_sec: float
    frame_id: str
    position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    desired_position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    desired_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    desired_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    local_goal: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    have_command: bool = False
    have_goal: bool = False


class LowDimObservationBuilder:
    def __init__(self, normalization_scales: Optional[Sequence[float]] = None):
        if normalization_scales is None:
            normalization_scales = np.ones(len(LOW_DIM_FIELDS), dtype=np.float32)
        self._scales = np.asarray(normalization_scales, dtype=np.float32)
        if self._scales.shape != (len(LOW_DIM_FIELDS),):
            raise ValueError("normalization_scales must match LOW_DIM_FIELDS")
        if not np.all(np.isfinite(self._scales)) or np.any(self._scales <= 0.0):
            raise ValueError("normalization scales must be finite and positive")

    def build(
        self,
        state: VehiclePlanningState,
        previous_v_max: float,
        map_spec: MapTensorSpec,
        map_tensor: Optional[np.ndarray] = None,
        map_semantics_complete: bool = False,
        trajectory_context_complete: bool = False,
        metadata=None,
    ) -> PolicyObservation:
        position = vector3(state.position, "position")
        velocity = vector3(state.velocity, "velocity")
        acceleration = vector3(state.acceleration, "acceleration")
        desired_position = vector3(state.desired_position, "desired_position")
        desired_velocity = vector3(state.desired_velocity, "desired_velocity")
        desired_acceleration = vector3(state.desired_acceleration, "desired_acceleration")
        local_goal = vector3(state.local_goal, "local_goal")
        if not np.isfinite(previous_v_max):
            raise ValueError("previous_v_max must be finite")

        tracking_error = position - desired_position if state.have_command else np.zeros(3)
        local_goal_delta = local_goal - position if state.have_goal else np.zeros(3)
        low_dim = np.concatenate(
            (
                position,
                velocity,
                acceleration,
                tracking_error,
                local_goal_delta,
                desired_velocity if state.have_command else np.zeros(3),
                desired_acceleration if state.have_command else np.zeros(3),
                np.asarray([previous_v_max], dtype=np.float32),
            )
        ).astype(np.float32)

        return PolicyObservation(
            stamp_sec=float(state.stamp_sec),
            frame_id=state.frame_id,
            low_dim=low_dim,
            normalized_low_dim=low_dim / self._scales,
            map_tensor=map_tensor,
            map_spec=map_spec,
            map_semantics_complete=map_semantics_complete,
            trajectory_context_complete=trajectory_context_complete,
            metadata=dict(metadata or {}),
        )
