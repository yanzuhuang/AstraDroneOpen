"""Full-sphere equal-angle partition in an FLU body frame."""

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class AngularPartitionSpec:
    angular_resolution_deg: float
    azimuth_min_deg: float = -180.0
    azimuth_max_deg: float = 180.0
    elevation_min_deg: float = -90.0
    elevation_max_deg: float = 90.0
    expected_number_of_bins: int = 0

    def __post_init__(self) -> None:
        spans = (
            self.azimuth_max_deg - self.azimuth_min_deg,
            self.elevation_max_deg - self.elevation_min_deg,
        )
        if self.angular_resolution_deg <= 0.0 or any(span <= 0.0 for span in spans):
            raise ValueError("invalid angular partition ranges")
        counts = [span / self.angular_resolution_deg for span in spans]
        if any(abs(value - round(value)) > 1.0e-8 for value in counts):
            raise ValueError("angular spans must be integer multiples of resolution")
        if self.expected_number_of_bins and self.number_of_bins != self.expected_number_of_bins:
            raise ValueError("number_of_bins does not match angular ranges/resolution")

    @property
    def azimuth_bins(self) -> int:
        return int(round((self.azimuth_max_deg - self.azimuth_min_deg) / self.angular_resolution_deg))

    @property
    def elevation_bins(self) -> int:
        return int(round((self.elevation_max_deg - self.elevation_min_deg) / self.angular_resolution_deg))

    @property
    def number_of_bins(self) -> int:
        return self.azimuth_bins * self.elevation_bins


class AngularPartition:
    """Azimuth-fast order: flat=elevation_index*N_azimuth+azimuth_index.

    Body axes follow ROS FLU: +X forward, +Y left, +Z up. Azimuth is
    atan2(y,x); elevation is atan2(z,hypot(x,y)). Intervals are half-open,
    with the numerical upper boundary clamped into the final bin.
    """

    def __init__(self, spec: AngularPartitionSpec):
        self.spec = spec

    def indices(self, points_body: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        points = np.asarray(points_body, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape [N,3]")
        with np.errstate(invalid="ignore", over="ignore"):
            ranges = np.linalg.norm(points, axis=1)
        positive_range = np.zeros(ranges.shape, dtype=bool)
        np.greater(ranges, 0.0, out=positive_range, where=np.isfinite(ranges))
        finite = np.all(np.isfinite(points), axis=1) & np.isfinite(ranges) & positive_range
        result = np.full(points.shape[0], -1, dtype=np.int64)
        if not np.any(finite):
            return result, ranges
        selected = points[finite]
        azimuth = np.degrees(np.arctan2(selected[:, 1], selected[:, 0]))
        elevation = np.degrees(
            np.arctan2(selected[:, 2], np.hypot(selected[:, 0], selected[:, 1]))
        )
        azimuth = np.where(azimuth >= self.spec.azimuth_max_deg, self.spec.azimuth_min_deg, azimuth)
        inside = (
            (azimuth >= self.spec.azimuth_min_deg)
            & (azimuth < self.spec.azimuth_max_deg)
            & (elevation >= self.spec.elevation_min_deg)
            & (elevation <= self.spec.elevation_max_deg)
        )
        az_index = np.floor(
            (azimuth - self.spec.azimuth_min_deg) / self.spec.angular_resolution_deg
        ).astype(np.int64)
        el_index = np.floor(
            (elevation - self.spec.elevation_min_deg) / self.spec.angular_resolution_deg
        ).astype(np.int64)
        el_index = np.clip(el_index, 0, self.spec.elevation_bins - 1)
        flat = el_index * self.spec.azimuth_bins + az_index
        finite_indices = np.flatnonzero(finite)
        result[finite_indices[inside]] = flat[inside]
        return result, ranges

    def direction_centers(self) -> np.ndarray:
        azimuth = self.spec.azimuth_min_deg + (
            np.arange(self.spec.azimuth_bins, dtype=np.float64) + 0.5
        ) * self.spec.angular_resolution_deg
        elevation = self.spec.elevation_min_deg + (
            np.arange(self.spec.elevation_bins, dtype=np.float64) + 0.5
        ) * self.spec.angular_resolution_deg
        el_grid, az_grid = np.meshgrid(elevation, azimuth, indexing="ij")
        az_rad = np.radians(az_grid.ravel())
        el_rad = np.radians(el_grid.ravel())
        horizontal = np.cos(el_rad)
        return np.column_stack(
            (horizontal * np.cos(az_rad), horizontal * np.sin(az_rad), np.sin(el_rad))
        )
