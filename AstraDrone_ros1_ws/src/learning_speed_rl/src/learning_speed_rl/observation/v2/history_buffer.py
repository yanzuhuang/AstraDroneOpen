"""Bounded body-frame cloud history."""

from dataclasses import dataclass
from typing import List

import numpy as np

from .frame_transform import Pose3D


@dataclass(frozen=True)
class CloudFrame:
    stamp_sec: float
    source_frame_id: str
    points_body: np.ndarray
    pose_world_body: Pose3D
    input_points: int = 0
    valid_points: int = 0
    downsample_ms: float = 0.0
    pose_lookup_mode: str = ""

    def __post_init__(self) -> None:
        points = np.asarray(self.points_body, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
            raise ValueError("points_body must have shape [N,3] and be finite")
        object.__setattr__(self, "points_body", points)


class CloudHistoryBuffer:
    def __init__(self, history_frames: int):
        if history_frames <= 0:
            raise ValueError("history_frames must be positive")
        self.history_frames = int(history_frames)
        self._frames: List[CloudFrame] = []

    def clear(self) -> None:
        self._frames = []

    def append(self, frame: CloudFrame) -> bool:
        reset = bool(self._frames and frame.stamp_sec < self._frames[-1].stamp_sec - 1.0e-9)
        if reset:
            self.clear()
        if self._frames and abs(frame.stamp_sec - self._frames[-1].stamp_sec) <= 1.0e-9:
            self._frames[-1] = frame
        else:
            self._frames.append(frame)
        self._frames = self._frames[-self.history_frames :]
        return reset

    @property
    def frames(self) -> List[CloudFrame]:
        return list(self._frames)

    @property
    def size(self) -> int:
        return len(self._frames)
