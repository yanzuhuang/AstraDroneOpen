"""Auditable historical-FoV approximation for observed-free versus unknown."""

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .frame_transform import Pose3D, quaternion_to_matrix
from .history_buffer import CloudFrame


@dataclass(frozen=True)
class FovCoverageConfig:
    sensor_min_range_m: float
    distance_clip_m: float
    radial_sample_step_m: float
    azimuth_min_deg: float
    azimuth_max_deg: float
    elevation_min_deg: float
    elevation_max_deg: float
    occlusion_angular_resolution_deg: float
    occlusion_margin_m: float
    sensor_translation_body_m: Sequence[float]
    sensor_rotation_body_xyzw: Sequence[float]

    def __post_init__(self) -> None:
        if not (0.0 <= self.sensor_min_range_m < self.distance_clip_m):
            raise ValueError("invalid sensor range")
        if self.radial_sample_step_m <= 0.0 or self.occlusion_angular_resolution_deg <= 0.0:
            raise ValueError("radial_sample_step_m must be positive")
        if self.occlusion_margin_m < 0.0:
            raise ValueError("occlusion_margin_m must be non-negative")
        translation = np.asarray(self.sensor_translation_body_m, dtype=np.float64)
        if translation.shape != (3,) or not np.all(np.isfinite(translation)):
            raise ValueError("sensor translation must be finite xyz")
        rotation = quaternion_to_matrix(self.sensor_rotation_body_xyzw)
        object.__setattr__(self, "sensor_translation_body_m", translation)
        object.__setattr__(self, "sensor_rotation_body_xyzw", np.asarray(self.sensor_rotation_body_xyzw, dtype=np.float64))
        object.__setattr__(self, "_rotation_body_sensor", rotation)


class HistoricalFovEstimator:
    """Estimate continuous observed distance on current-body bin-center rays.

    This is deliberately labelled an approximation, not the unpublished
    Flying-on-Point-Clouds `d_unknown` implementation. A radial sample is
    observed when it lies inside at least one historical calibrated Mid360
    frustum after pose transformation. The result is the first uncovered
    sample along the current ray (or distance_clip when fully covered).
    """

    def __init__(self, config: FovCoverageConfig):
        self.config = config
        self._radial = np.arange(
            config.sensor_min_range_m + config.radial_sample_step_m,
            config.distance_clip_m,
            config.radial_sample_step_m,
            dtype=np.float64,
        )
        self._radial = np.unique(
            np.append(self._radial, config.distance_clip_m)
        )
        self._full_azimuth = bool(
            config.azimuth_max_deg - config.azimuth_min_deg >= 360.0 - 1.0e-6
        )
        self._occlusion_azimuth_bins = int(
            np.ceil(
                (config.azimuth_max_deg - config.azimuth_min_deg)
                / config.occlusion_angular_resolution_deg
            )
        )
        self._occlusion_elevation_bins = int(
            np.ceil(
                (config.elevation_max_deg - config.elevation_min_deg)
                / config.occlusion_angular_resolution_deg
            )
        )
        self._nearest_surface_cache = {}

    def _nearest_surface(self, frame):
        key = (float(frame.stamp_sec), id(frame.points_body))
        cached = self._nearest_surface_cache.get(key)
        if cached is not None:
            return cached
        rotation_body_sensor = self.config._rotation_body_sensor
        translation_body_sensor = self.config.sensor_translation_body_m
        source_points_sensor = (
            frame.points_body.astype(np.float64) - translation_body_sensor
        ) @ rotation_body_sensor
        source_ranges = np.linalg.norm(source_points_sensor, axis=1)
        source_azimuth = np.degrees(
            np.arctan2(source_points_sensor[:, 1], source_points_sensor[:, 0])
        )
        source_elevation = np.degrees(
            np.arctan2(
                source_points_sensor[:, 2],
                np.hypot(source_points_sensor[:, 0], source_points_sensor[:, 1]),
            )
        )
        source_valid = (
            np.all(np.isfinite(source_points_sensor), axis=1)
            & (source_ranges > self.config.sensor_min_range_m)
            & (source_ranges <= self.config.distance_clip_m)
            & (source_elevation >= self.config.elevation_min_deg)
            & (source_elevation <= self.config.elevation_max_deg)
        )
        if not self._full_azimuth:
            source_valid &= (
                (source_azimuth >= self.config.azimuth_min_deg)
                & (source_azimuth <= self.config.azimuth_max_deg)
            )
        nearest_surface = np.full(
            self._occlusion_azimuth_bins * self._occlusion_elevation_bins,
            np.inf,
            dtype=np.float64,
        )
        if np.any(source_valid):
            normalized_source_azimuth = source_azimuth[source_valid]
            if self._full_azimuth:
                normalized_source_azimuth = (
                    (
                        normalized_source_azimuth
                        - self.config.azimuth_min_deg
                    )
                    % 360.0
                ) + self.config.azimuth_min_deg
            source_az_index = np.floor(
                (normalized_source_azimuth - self.config.azimuth_min_deg)
                / self.config.occlusion_angular_resolution_deg
            ).astype(np.int64)
            source_el_index = np.floor(
                (source_elevation[source_valid] - self.config.elevation_min_deg)
                / self.config.occlusion_angular_resolution_deg
            ).astype(np.int64)
            source_az_index = np.clip(
                source_az_index, 0, self._occlusion_azimuth_bins - 1
            )
            source_el_index = np.clip(
                source_el_index, 0, self._occlusion_elevation_bins - 1
            )
            source_flat = (
                source_el_index * self._occlusion_azimuth_bins + source_az_index
            )
            np.minimum.at(
                nearest_surface, source_flat, source_ranges[source_valid]
            )
        self._nearest_surface_cache[key] = nearest_surface
        return nearest_surface

    def estimate(
        self,
        direction_centers_body: np.ndarray,
        historical_frames: Sequence[CloudFrame],
        current_pose_world_body: Pose3D,
    ) -> np.ndarray:
        directions = np.asarray(direction_centers_body, dtype=np.float64)
        if directions.ndim != 2 or directions.shape[1] != 3:
            raise ValueError("direction centers must have shape [bins,3]")
        radial = self._radial
        points_current = directions[:, None, :] * radial[None, :, None]
        flattened_current = points_current.reshape(-1, 3)
        points_world = current_pose_world_body.body_to_world(flattened_current)
        observed = np.zeros((directions.shape[0], radial.size), dtype=bool)
        rotation_body_sensor = self.config._rotation_body_sensor
        translation_body_sensor = self.config.sensor_translation_body_m

        active_cache_keys = {
            (float(frame.stamp_sec), id(frame.points_body))
            for frame in historical_frames
        }
        self._nearest_surface_cache = {
            key: value
            for key, value in self._nearest_surface_cache.items()
            if key in active_cache_keys
        }
        observed_flat = observed.reshape(-1)
        for frame in historical_frames:
            unresolved = np.flatnonzero(~observed_flat)
            if unresolved.size == 0:
                break
            pose = frame.pose_world_body
            points_source_body = pose.world_to_body(points_world[unresolved])
            points_sensor = (
                points_source_body - translation_body_sensor
            ) @ rotation_body_sensor
            ranges = np.linalg.norm(points_sensor, axis=1)
            azimuth = np.degrees(np.arctan2(points_sensor[:, 1], points_sensor[:, 0]))
            elevation = np.degrees(
                np.arctan2(points_sensor[:, 2], np.hypot(points_sensor[:, 0], points_sensor[:, 1]))
            )
            in_azimuth = np.ones_like(ranges, dtype=bool) if self._full_azimuth else (
                (azimuth >= self.config.azimuth_min_deg)
                & (azimuth <= self.config.azimuth_max_deg)
            )
            visible = (
                np.isfinite(ranges)
                & (ranges >= self.config.sensor_min_range_m)
                & (ranges <= self.config.distance_clip_m)
                & in_azimuth
                & (elevation >= self.config.elevation_min_deg)
                & (elevation <= self.config.elevation_max_deg)
            )

            # A return makes the corresponding angular cell line-of-sight
            # observed only up to its nearest surface. Empty cells inside the
            # calibrated FoV remain observed to the configured range. This is
            # an explicit engineering approximation because filtered
            # PointCloud2 does not retain per-ray miss records.
            nearest_surface = self._nearest_surface(frame)

            candidate_azimuth = azimuth.copy()
            if self._full_azimuth:
                candidate_azimuth = (
                    (candidate_azimuth - self.config.azimuth_min_deg) % 360.0
                ) + self.config.azimuth_min_deg
            candidate_az_index = np.floor(
                (candidate_azimuth - self.config.azimuth_min_deg)
                / self.config.occlusion_angular_resolution_deg
            ).astype(np.int64)
            candidate_el_index = np.floor(
                (elevation - self.config.elevation_min_deg)
                / self.config.occlusion_angular_resolution_deg
            ).astype(np.int64)
            candidate_az_index = np.clip(
                candidate_az_index, 0, self._occlusion_azimuth_bins - 1
            )
            candidate_el_index = np.clip(
                candidate_el_index, 0, self._occlusion_elevation_bins - 1
            )
            candidate_flat = (
                candidate_el_index * self._occlusion_azimuth_bins
                + candidate_az_index
            )
            surface_limit = nearest_surface[candidate_flat] + self.config.occlusion_margin_m
            visible &= (ranges <= surface_limit) | ~np.isfinite(surface_limit)
            observed_flat[unresolved[visible]] = True

        # Require coverage from the first evaluated range outward. A hole marks
        # the unknown boundary even if a farther historical frustum overlaps.
        contiguous = np.logical_and.accumulate(observed, axis=1)
        counts = np.sum(contiguous, axis=1)
        free_range = np.full(directions.shape[0], self.config.sensor_min_range_m, dtype=np.float64)
        covered = counts > 0
        free_range[covered] = radial[np.minimum(counts[covered] - 1, radial.size - 1)]
        return np.minimum(free_range, self.config.distance_clip_m).astype(np.float32)
