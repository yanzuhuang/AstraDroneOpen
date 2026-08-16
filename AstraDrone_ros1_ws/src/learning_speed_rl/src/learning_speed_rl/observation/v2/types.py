"""Versioned, policy-agnostic data contract for Observation v2."""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict

import numpy as np


OBSERVATION_V2_VERSION = "lidar_surrogate_v2.0"


class BinSemantic(IntEnum):
    UNKNOWN = 0
    OBSERVED_FREE = 1
    KNOWN_OBSTACLE = 2


BIN_SEMANTIC_NAMES = (
    "unknown",
    "observed_free",
    "known_obstacle",
)


@dataclass(frozen=True)
class ObservationV2:
    """Stable v2 contract; future trajectory/state fusion stays outside it."""

    stamp_sec: float
    frame_id: str
    lidar_surrogate: np.ndarray
    lidar_valid_mask: np.ndarray
    unknown_mask: np.ndarray
    semantic: np.ndarray
    nearest_obstacle_distance: np.ndarray
    observed_free_range: np.ndarray
    metadata: Dict[str, object] = field(default_factory=dict)
    version: str = OBSERVATION_V2_VERSION

    def __post_init__(self) -> None:
        arrays = (
            self.lidar_surrogate,
            self.lidar_valid_mask,
            self.unknown_mask,
            self.semantic,
            self.nearest_obstacle_distance,
            self.observed_free_range,
        )
        shape = self.lidar_surrogate.shape
        if len(shape) != 1 or any(value.shape != shape for value in arrays):
            raise ValueError("all ObservationV2 bin arrays must have one equal shape")
        if not self.frame_id or not np.isfinite(self.stamp_sec):
            raise ValueError("ObservationV2 requires a finite stamp and frame_id")
        if not np.all(np.isfinite(self.lidar_surrogate)):
            raise ValueError("lidar_surrogate must be finite")
        if not np.all(np.isin(self.semantic, list(BinSemantic))):
            raise ValueError("semantic contains an unsupported value")

    @property
    def number_of_bins(self) -> int:
        return int(self.lidar_surrogate.size)

    @property
    def ready(self) -> bool:
        return self.number_of_bins > 0 and bool(np.all(np.isfinite(self.lidar_surrogate)))
