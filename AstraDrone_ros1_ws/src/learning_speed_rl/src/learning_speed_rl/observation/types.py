"""Stable observation contracts shared by mock, training and inference code."""

from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

import numpy as np


MAP_CHANNELS: Tuple[str, ...] = (
    "free",
    "occupied",
    "unknown",
    "on_trajectory",
)

LOW_DIM_FIELDS: Tuple[str, ...] = (
    "position_x",
    "position_y",
    "position_z",
    "velocity_x",
    "velocity_y",
    "velocity_z",
    "acceleration_x",
    "acceleration_y",
    "acceleration_z",
    "tracking_error_x",
    "tracking_error_y",
    "tracking_error_z",
    "local_goal_delta_x",
    "local_goal_delta_y",
    "local_goal_delta_z",
    "desired_velocity_x",
    "desired_velocity_y",
    "desired_velocity_z",
    "desired_acceleration_x",
    "desired_acceleration_y",
    "desired_acceleration_z",
    "previous_v_max",
)


@dataclass(frozen=True)
class MapTensorSpec:
    """Dense tensor contract: channels x z x y x x, centered on the UAV."""

    size_xyz_m: Tuple[float, float, float]
    resolution_m: float
    channels: Tuple[str, ...] = MAP_CHANNELS

    def __post_init__(self) -> None:
        if len(self.size_xyz_m) != 3 or any(value <= 0.0 for value in self.size_xyz_m):
            raise ValueError("size_xyz_m must contain three positive values")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if self.channels != MAP_CHANNELS:
            raise ValueError("the v1 categorical channel order is fixed")

    @property
    def shape_zyx(self) -> Tuple[int, int, int]:
        xyz = tuple(int(round(value / self.resolution_m)) for value in self.size_xyz_m)
        if any(abs(count * self.resolution_m - value) > 1.0e-6
               for count, value in zip(xyz, self.size_xyz_m)):
            raise ValueError("map sizes must be integer multiples of resolution")
        return xyz[2], xyz[1], xyz[0]

    @property
    def tensor_shape(self) -> Tuple[int, int, int, int]:
        return (len(self.channels),) + self.shape_zyx


@dataclass
class PolicyObservation:
    stamp_sec: float
    frame_id: str
    low_dim: np.ndarray
    normalized_low_dim: np.ndarray
    map_tensor: Optional[np.ndarray]
    map_spec: MapTensorSpec
    map_semantics_complete: bool = False
    trajectory_context_complete: bool = False
    metadata: Dict[str, object] = field(default_factory=dict)

    @property
    def ready_for_rl(self) -> bool:
        return (
            self.map_tensor is not None
            and self.map_tensor.shape == self.map_spec.tensor_shape
            and self.low_dim.shape == (len(LOW_DIM_FIELDS),)
            and self.map_semantics_complete
            and self.trajectory_context_complete
        )


def vector3(values: Sequence[float], name: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float32)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError("{} must be a finite xyz vector".format(name))
    return result
