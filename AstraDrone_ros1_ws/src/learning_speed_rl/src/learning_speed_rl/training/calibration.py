"""Pure helpers for read-only Learning Speed calibration recording."""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional

import numpy as np


KNOWN_OBSTACLE = 2


def lidar_clutter_metrics(lidar_surrogate, semantic) -> Dict[str, object]:
    surrogate = np.asarray(lidar_surrogate, dtype=np.float64)
    labels = np.asarray(semantic, dtype=np.uint8)
    if surrogate.shape != labels.shape or surrogate.ndim != 1 or surrogate.size == 0:
        raise ValueError("surrogate and semantic must be equal non-empty vectors")
    if not np.all(np.isfinite(surrogate)):
        raise ValueError("surrogate must be finite")
    obstacle = labels == KNOWN_OBSTACLE
    obstacle_distances = surrogate[obstacle]
    if obstacle_distances.size and np.any(obstacle_distances <= 0.0):
        raise ValueError("known-obstacle distances must be positive")
    count = int(np.count_nonzero(obstacle))
    return {
        "nearest_obstacle_distance_m": (
            None if count == 0 else float(np.min(obstacle_distances))
        ),
        "known_obstacle_bin_count": count,
        "known_obstacle_bin_fraction": float(count) / float(surrogate.size),
        "observed_free_bin_count": int(np.count_nonzero(labels == 1)),
        "unknown_bin_count": int(np.count_nonzero(labels == 0)),
    }


@dataclass
class PlannerFailureEpisode:
    episode_id: int
    start_stamp_sec: float
    end_stamp_sec: Optional[float]
    recovered: bool
    reasons: List[str]
    maximum_consecutive_failures: int


class PlannerFailureEpisodeTracker:
    def __init__(self):
        self.completed: List[PlannerFailureEpisode] = []
        self.active: Optional[PlannerFailureEpisode] = None

    def update(
        self,
        stamp_sec: float,
        failed: bool,
        reason: str,
        consecutive_failures: int,
    ) -> None:
        if not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
            raise ValueError("planner status stamp must be positive and finite")
        if consecutive_failures < 0:
            raise ValueError("consecutive_failures must be non-negative")
        if failed:
            if self.active is None:
                self.active = PlannerFailureEpisode(
                    episode_id=len(self.completed) + 1,
                    start_stamp_sec=stamp_sec,
                    end_stamp_sec=None,
                    recovered=False,
                    reasons=[],
                    maximum_consecutive_failures=consecutive_failures,
                )
            if reason and reason != "NONE" and reason not in self.active.reasons:
                self.active.reasons.append(reason)
            self.active.maximum_consecutive_failures = max(
                self.active.maximum_consecutive_failures, consecutive_failures
            )
        elif self.active is not None:
            self.active.end_stamp_sec = stamp_sec
            self.active.recovered = True
            self.completed.append(self.active)
            self.active = None

    def close_unrecovered(self, stamp_sec: float) -> None:
        if self.active is None:
            return
        self.active.end_stamp_sec = stamp_sec
        self.active.recovered = False
        self.completed.append(self.active)
        self.active = None


class TrackingSafetyMirror:
    """Read-only mirror of the existing bridge threshold/duration gate."""

    def __init__(self, limit_m: float, duration_sec: float):
        if limit_m <= 0.0 or duration_sec <= 0.0:
            raise ValueError("tracking gate mirror parameters must be positive")
        self.limit_m = float(limit_m)
        self.duration_sec = float(duration_sec)
        self.above_since_sec: Optional[float] = None
        self.triggered = False
        self.trigger_stamp_sec: Optional[float] = None

    def update(self, stamp_sec: float, error_m: float, bridge_state: str) -> bool:
        if not math.isfinite(stamp_sec) or not math.isfinite(error_m):
            raise ValueError("tracking sample must be finite")
        if bridge_state != "TRACK_EGO" or error_m <= self.limit_m:
            self.above_since_sec = None
            return self.triggered
        if self.above_since_sec is None or stamp_sec < self.above_since_sec:
            self.above_since_sec = stamp_sec
        if stamp_sec - self.above_since_sec >= self.duration_sec:
            self.triggered = True
            if self.trigger_stamp_sec is None:
                self.trigger_stamp_sec = stamp_sec
        return self.triggered
