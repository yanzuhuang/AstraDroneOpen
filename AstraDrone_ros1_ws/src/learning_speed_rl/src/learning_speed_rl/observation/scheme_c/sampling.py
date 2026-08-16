"""Configurable future sampling of EGO's active local B-spline."""

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .bspline import EgoBsplineTrajectory


@dataclass(frozen=True)
class TrajectorySamplingConfig:
    sample_count: int = 20
    sample_spacing: float = 0.25
    max_distance: float = 5.0
    sampling_mode: str = "distance"
    arc_length_resolution_m: float = 0.02
    arc_length_max_depth: int = 16

    def __post_init__(self) -> None:
        mode = self.sampling_mode.strip().lower()
        if self.sample_count <= 0:
            raise ValueError("trajectory_sample_count must be positive")
        if self.sample_spacing <= 0.0 or not np.isfinite(self.sample_spacing):
            raise ValueError("trajectory_sample_spacing must be positive and finite")
        if self.max_distance <= 0.0 or not np.isfinite(self.max_distance):
            raise ValueError("trajectory_max_distance must be positive and finite")
        if mode not in ("distance", "time"):
            raise ValueError("trajectory_sampling_mode must be distance or time")
        if self.arc_length_resolution_m <= 0.0 or self.arc_length_max_depth <= 0:
            raise ValueError("arc-length approximation configuration is invalid")
        object.__setattr__(self, "sampling_mode", mode)


class TrajectorySampler:
    def __init__(self, config: TrajectorySamplingConfig):
        self.config = config

    def sample(
        self, trajectory: EgoBsplineTrajectory, observation_stamp_sec: float
    ) -> Tuple[np.ndarray, np.ndarray, dict]:
        elapsed = trajectory.elapsed_at(observation_stamp_sec)
        if trajectory.duration_sec - elapsed <= 1.0e-6:
            raise ValueError("trajectory has no future segment")
        if self.config.sampling_mode == "distance":
            return self._sample_distance(trajectory, elapsed)
        return self._sample_time(trajectory, elapsed)

    def _adaptive_polyline(self, trajectory, elapsed):
        times = []
        points = []

        def append_segment(t0, p0, t1, p1, depth):
            midpoint_time = 0.5 * (t0 + t1)
            midpoint = trajectory.evaluate_elapsed(midpoint_time)
            chord_midpoint = 0.5 * (p0 + p1)
            chord_length = float(np.linalg.norm(p1 - p0))
            curve_error = float(np.linalg.norm(midpoint - chord_midpoint))
            needs_split = (
                chord_length > self.config.arc_length_resolution_m
                or curve_error > 0.25 * self.config.arc_length_resolution_m
            )
            if needs_split and depth < self.config.arc_length_max_depth:
                append_segment(t0, p0, midpoint_time, midpoint, depth + 1)
                append_segment(midpoint_time, midpoint, t1, p1, depth + 1)
                return
            times.append(t1)
            points.append(p1)

        boundaries = trajectory.future_knot_times(elapsed)
        first = trajectory.evaluate_elapsed(boundaries[0])
        times.append(float(boundaries[0]))
        points.append(first)
        for start, end in zip(boundaries[:-1], boundaries[1:]):
            if end - start <= 1.0e-12:
                continue
            p0 = trajectory.evaluate_elapsed(float(start))
            p1 = trajectory.evaluate_elapsed(float(end))
            append_segment(float(start), p0, float(end), p1, 0)
        return np.asarray(times), np.asarray(points)

    def _sample_distance(self, trajectory, elapsed):
        dense_times, dense_points = self._adaptive_polyline(trajectory, elapsed)
        segment_lengths = np.linalg.norm(np.diff(dense_points, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        remaining_length = float(cumulative[-1])
        if remaining_length <= 1.0e-8:
            raise ValueError("trajectory future arc length is zero")
        requested = self.config.sample_spacing * np.arange(
            1, self.config.sample_count + 1, dtype=np.float64
        )
        offsets = np.minimum(requested, self.config.max_distance)
        query = np.minimum(offsets, remaining_length)
        sampled = np.empty((self.config.sample_count, 3), dtype=np.float64)
        sampled_times = np.empty(self.config.sample_count, dtype=np.float64)
        for index, distance in enumerate(query):
            upper = int(np.searchsorted(cumulative, distance, side="left"))
            if upper <= 0:
                sampled[index] = dense_points[0]
                sampled_times[index] = dense_times[0]
                continue
            if upper >= len(cumulative):
                sampled[index] = dense_points[-1]
                sampled_times[index] = dense_times[-1]
                continue
            lower = upper - 1
            span = cumulative[upper] - cumulative[lower]
            ratio = 0.0 if span <= 1.0e-12 else (distance - cumulative[lower]) / span
            sampled[index] = (
                (1.0 - ratio) * dense_points[lower] + ratio * dense_points[upper]
            )
            sampled_times[index] = (
                (1.0 - ratio) * dense_times[lower] + ratio * dense_times[upper]
            )
        return sampled, query, {
            "remaining_arc_length_m": remaining_length,
            "sample_elapsed_times_sec": sampled_times,
            "arc_length_polyline_points": int(len(dense_points)),
        }

    def _sample_time(self, trajectory, elapsed):
        requested = elapsed + self.config.sample_spacing * np.arange(
            1, self.config.sample_count + 1, dtype=np.float64
        )
        dense_times, dense_points = self._adaptive_polyline(trajectory, elapsed)
        cumulative = np.concatenate(
            ([0.0], np.cumsum(np.linalg.norm(np.diff(dense_points, axis=0), axis=1)))
        )
        remaining_length = float(cumulative[-1])
        maximum_query_distance = min(self.config.max_distance, remaining_length)
        upper = int(np.searchsorted(cumulative, maximum_query_distance, side="left"))
        if upper <= 0:
            maximum_time = float(dense_times[0])
        elif upper >= len(cumulative):
            maximum_time = float(dense_times[-1])
        else:
            lower = upper - 1
            span = cumulative[upper] - cumulative[lower]
            ratio = 0.0 if span <= 1.0e-12 else (
                maximum_query_distance - cumulative[lower]
            ) / span
            maximum_time = float(
                (1.0 - ratio) * dense_times[lower] + ratio * dense_times[upper]
            )
        query_times = np.minimum(requested, maximum_time)
        points = np.asarray(
            [trajectory.evaluate_elapsed(value) for value in query_times],
            dtype=np.float64,
        )
        offsets = query_times - elapsed
        return points, offsets, {
            "remaining_arc_length_m": remaining_length,
            "sample_elapsed_times_sec": query_times,
            "arc_length_polyline_points": int(len(dense_points)),
        }
