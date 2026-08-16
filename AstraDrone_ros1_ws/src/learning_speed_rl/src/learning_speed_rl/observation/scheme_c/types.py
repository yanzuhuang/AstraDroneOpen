"""Versioned, network-agnostic data contract for Scheme C observation."""

from dataclasses import dataclass, field
from typing import Dict, Tuple

import numpy as np


OBSERVATION_C_VERSION = "scheme_c_trajectory_fusion_v1.0"


def _array(value, shape, name, dtype=np.float32):
    result = np.asarray(value, dtype=dtype)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError("{} must have shape {} and be finite".format(name, shape))
    return result


@dataclass(frozen=True)
class LidarSurrogateFeature:
    stamp_sec: float
    frame_id: str
    surrogate: np.ndarray
    valid_mask: np.ndarray
    unknown_mask: np.ndarray
    semantic: np.ndarray

    def __post_init__(self) -> None:
        surrogate = np.asarray(self.surrogate, dtype=np.float32)
        valid_mask = np.asarray(self.valid_mask, dtype=np.float32)
        unknown_mask = np.asarray(self.unknown_mask, dtype=np.float32)
        semantic = np.asarray(self.semantic, dtype=np.uint8)
        if surrogate.ndim != 1 or surrogate.size == 0:
            raise ValueError("lidar surrogate must be a nonempty vector")
        if any(value.shape != surrogate.shape for value in (valid_mask, unknown_mask, semantic)):
            raise ValueError("lidar feature arrays must have one equal shape")
        if not np.all(np.isfinite(surrogate)) or not np.all(np.isfinite(valid_mask)):
            raise ValueError("lidar feature contains non-finite values")
        if not np.all(np.isfinite(unknown_mask)) or not np.all(np.isin(semantic, [0, 1, 2])):
            raise ValueError("lidar masks or semantics are invalid")
        if not np.isfinite(self.stamp_sec) or self.stamp_sec <= 0.0 or not self.frame_id:
            raise ValueError("lidar feature metadata is invalid")
        object.__setattr__(self, "surrogate", surrogate)
        object.__setattr__(self, "valid_mask", valid_mask)
        object.__setattr__(self, "unknown_mask", unknown_mask)
        object.__setattr__(self, "semantic", semantic)


@dataclass(frozen=True)
class FutureTrajectoryFeature:
    positions_body: np.ndarray
    sample_offsets: np.ndarray
    sampling_mode: str
    sample_spacing: float
    max_distance: float
    trajectory_id: int
    trajectory_start_time_sec: float
    source_frame_id: str
    metadata: Dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        points = np.asarray(self.positions_body, dtype=np.float32)
        offsets = np.asarray(self.sample_offsets, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
            raise ValueError("future positions must have shape [N,3]")
        if offsets.shape != (points.shape[0],):
            raise ValueError("future sample offsets must match future positions")
        if not np.all(np.isfinite(points)) or not np.all(np.isfinite(offsets)):
            raise ValueError("future trajectory contains non-finite values")
        if np.any(np.diff(offsets) < -1.0e-6):
            raise ValueError("future sample offsets must be nondecreasing")
        if self.sampling_mode not in ("distance", "time"):
            raise ValueError("unsupported trajectory sampling mode")
        if self.sample_spacing <= 0.0 or self.max_distance <= 0.0:
            raise ValueError("trajectory sampling metadata is invalid")
        if not np.isfinite(self.trajectory_start_time_sec) or not self.source_frame_id:
            raise ValueError("trajectory source metadata is invalid")
        object.__setattr__(self, "positions_body", points)
        object.__setattr__(self, "sample_offsets", offsets)


@dataclass(frozen=True)
class SystemStateFeature:
    actual_velocity_body: np.ndarray
    tracking_error_body: np.ndarray
    previous_v_max: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "actual_velocity_body",
            _array(self.actual_velocity_body, (3,), "actual_velocity_body"),
        )
        object.__setattr__(
            self, "tracking_error_body",
            _array(self.tracking_error_body, (3,), "tracking_error_body"),
        )
        if not np.isfinite(self.previous_v_max) or self.previous_v_max <= 0.0:
            raise ValueError("previous_v_max must be positive and finite")

    @property
    def tracking_error_norm(self) -> float:
        return float(np.linalg.norm(self.tracking_error_body))


@dataclass(frozen=True)
class ObservationC:
    stamp_sec: float
    frame_id: str
    lidar_surrogate: LidarSurrogateFeature
    future_trajectory: FutureTrajectoryFeature
    system_state: SystemStateFeature
    valid: bool = True
    diagnostics: Tuple[str, ...] = ()
    metadata: Dict[str, object] = field(default_factory=dict)
    version: str = OBSERVATION_C_VERSION

    def __post_init__(self) -> None:
        if not np.isfinite(self.stamp_sec) or self.stamp_sec <= 0.0 or not self.frame_id:
            raise ValueError("Observation C metadata is invalid")
        if abs(self.stamp_sec - self.lidar_surrogate.stamp_sec) > 1.0e-9:
            raise ValueError("lidar and Observation C timestamps differ")
        if self.frame_id != self.lidar_surrogate.frame_id:
            raise ValueError("lidar and Observation C frames differ")
        if not self.valid:
            raise ValueError("invalid inputs must not construct a formal Observation C")

    @property
    def ready_for_policy(self) -> bool:
        return self.valid and not self.diagnostics
