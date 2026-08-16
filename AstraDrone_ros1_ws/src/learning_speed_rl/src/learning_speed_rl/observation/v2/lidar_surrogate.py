"""History alignment, voxel filtering, angular binning and v2 encoding."""

from dataclasses import dataclass
from time import perf_counter
from typing import Dict, Sequence, Tuple

import numpy as np

from .angular_partition import AngularPartition
from .history_buffer import CloudFrame
from .types import BinSemantic, ObservationV2
from .unknown_estimator import HistoricalFovEstimator


def voxel_downsample(points_xyz: np.ndarray, voxel_size_m: float) -> np.ndarray:
    points = np.asarray(points_xyz, dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or voxel_size_m <= 0.0:
        raise ValueError("invalid voxel downsample input")
    keys = np.floor(points / voxel_size_m).astype(np.int64)
    packed = np.ascontiguousarray(keys).view(
        np.dtype((np.void, keys.dtype.itemsize * keys.shape[1]))
    ).reshape(-1)
    _, indices = np.unique(packed, return_index=True)
    return points[np.sort(indices)].astype(np.float32)


@dataclass(frozen=True)
class LidarSurrogateConfig:
    voxel_downsample_m: float
    sensor_min_range_m: float
    distance_clip_m: float
    unknown_encoding_offset_m: float
    minimum_history_frames: int

    def __post_init__(self) -> None:
        if self.voxel_downsample_m <= 0.0:
            raise ValueError("voxel_downsample_m must be positive")
        if not (0.0 <= self.sensor_min_range_m < self.distance_clip_m):
            raise ValueError("invalid lidar distance range")
        if self.unknown_encoding_offset_m <= self.distance_clip_m:
            raise ValueError("unknown encoding must be disjoint from distances")
        if self.minimum_history_frames <= 0:
            raise ValueError("minimum_history_frames must be positive")


class LidarSurrogateBuilder:
    def __init__(
        self,
        config: LidarSurrogateConfig,
        partition: AngularPartition,
        unknown_estimator: HistoricalFovEstimator,
    ):
        self.config = config
        self.partition = partition
        self.unknown_estimator = unknown_estimator

    def build(self, frames: Sequence[CloudFrame], frame_id: str) -> Tuple[ObservationV2, np.ndarray, Dict[str, float]]:
        if len(frames) < self.config.minimum_history_frames:
            raise RuntimeError("insufficient_cloud_history")
        current = frames[-1]
        start = perf_counter()
        aligned = []
        for frame in frames:
            points_world = frame.pose_world_body.body_to_world(frame.points_body)
            aligned.append(current.pose_world_body.world_to_body(points_world))
        merged = np.concatenate(aligned, axis=0) if aligned else np.empty((0, 3))
        alignment_ms = (perf_counter() - start) * 1000.0

        start = perf_counter()
        ranges = np.linalg.norm(merged, axis=1)
        valid = (
            np.all(np.isfinite(merged), axis=1)
            & np.isfinite(ranges)
            & (ranges > self.config.sensor_min_range_m)
            & (ranges <= self.config.distance_clip_m)
        )
        merged = voxel_downsample(merged[valid], self.config.voxel_downsample_m)
        fusion_downsample_ms = (perf_counter() - start) * 1000.0

        start = perf_counter()
        bin_indices, ranges = self.partition.indices(merged)
        in_partition = bin_indices >= 0
        nearest = np.full(self.partition.spec.number_of_bins, np.inf, dtype=np.float32)
        if np.any(in_partition):
            np.minimum.at(nearest, bin_indices[in_partition], ranges[in_partition].astype(np.float32))
        obstacle = np.isfinite(nearest)
        binning_ms = (perf_counter() - start) * 1000.0

        start = perf_counter()
        observed_free_range = np.full(
            self.partition.spec.number_of_bins,
            self.config.distance_clip_m,
            dtype=np.float32,
        )
        empty_indices = np.flatnonzero(~obstacle)
        if empty_indices.size:
            observed_free_range[empty_indices] = self.unknown_estimator.estimate(
                self.partition.direction_centers()[empty_indices],
                frames,
                current.pose_world_body,
            )
        free = (~obstacle) & (observed_free_range >= self.config.distance_clip_m - 1.0e-6)
        unknown = (~obstacle) & (~free)
        unknown_ms = (perf_counter() - start) * 1000.0

        semantic = np.full(nearest.shape, int(BinSemantic.UNKNOWN), dtype=np.uint8)
        semantic[free] = int(BinSemantic.OBSERVED_FREE)
        semantic[obstacle] = int(BinSemantic.KNOWN_OBSTACLE)
        surrogate = np.empty(nearest.shape, dtype=np.float32)
        surrogate[obstacle] = nearest[obstacle]
        surrogate[free] = observed_free_range[free]
        surrogate[unknown] = self.config.unknown_encoding_offset_m - observed_free_range[unknown]
        valid_mask = (~unknown).astype(np.float32)
        unknown_mask = unknown.astype(np.float32)
        nearest_output = nearest.copy()
        nearest_output[~obstacle] = self.config.distance_clip_m

        timings = {
            "history_alignment_ms": alignment_ms,
            "fusion_downsample_ms": fusion_downsample_ms,
            "angular_binning_ms": binning_ms,
            "unknown_estimation_ms": unknown_ms,
            "total_build_ms": (alignment_ms + fusion_downsample_ms + binning_ms + unknown_ms),
        }
        metadata = {
            "history_frames": len(frames),
            "aligned_points": int(sum(frame.points_body.shape[0] for frame in frames)),
            "fused_points": int(merged.shape[0]),
            "known_obstacle_bins": int(np.count_nonzero(obstacle)),
            "observed_free_bins": int(np.count_nonzero(free)),
            "unknown_bins": int(np.count_nonzero(unknown)),
            "azimuth_bins": self.partition.spec.azimuth_bins,
            "elevation_bins": self.partition.spec.elevation_bins,
            "bin_order": "elevation_major_azimuth_fast",
            "body_axes": "FLU:+X_forward,+Y_left,+Z_up",
            **timings,
        }
        observation = ObservationV2(
            stamp_sec=current.stamp_sec,
            frame_id=frame_id,
            lidar_surrogate=surrogate,
            lidar_valid_mask=valid_mask,
            unknown_mask=unknown_mask,
            semantic=semantic,
            nearest_obstacle_distance=nearest_output,
            observed_free_range=observed_free_range,
            metadata=metadata,
        )
        return observation, merged.astype(np.float32), timings
