"""Timestamped rigid poses and interpolation used by point-cloud history."""

from bisect import bisect_left
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


def _unit_quaternion_xyzw(value) -> np.ndarray:
    quaternion = np.asarray(value, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion must contain four finite xyzw values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-9:
        raise ValueError("quaternion norm is zero")
    return quaternion / norm


def quaternion_to_matrix(quaternion_xyzw) -> np.ndarray:
    x, y, z, w = _unit_quaternion_xyzw(quaternion_xyzw)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def slerp_xyzw(first, second, ratio: float) -> np.ndarray:
    q0 = _unit_quaternion_xyzw(first)
    q1 = _unit_quaternion_xyzw(second)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return _unit_quaternion_xyzw(q0 + ratio * (q1 - q0))
    theta = np.arccos(dot)
    return (
        np.sin((1.0 - ratio) * theta) / np.sin(theta) * q0
        + np.sin(ratio * theta) / np.sin(theta) * q1
    )


@dataclass(frozen=True)
class Pose3D:
    stamp_sec: float
    translation: np.ndarray
    quaternion_xyzw: np.ndarray

    def __post_init__(self) -> None:
        translation = np.asarray(self.translation, dtype=np.float64)
        if translation.shape != (3,) or not np.all(np.isfinite(translation)):
            raise ValueError("translation must contain three finite values")
        if not np.isfinite(self.stamp_sec):
            raise ValueError("pose stamp must be finite")
        object.__setattr__(self, "translation", translation)
        object.__setattr__(self, "quaternion_xyzw", _unit_quaternion_xyzw(self.quaternion_xyzw))

    @property
    def rotation(self) -> np.ndarray:
        return quaternion_to_matrix(self.quaternion_xyzw)

    def body_to_world(self, points_body: np.ndarray) -> np.ndarray:
        points = np.asarray(points_body, dtype=np.float64)
        return points @ self.rotation.T + self.translation

    def world_to_body(self, points_world: np.ndarray) -> np.ndarray:
        points = np.asarray(points_world, dtype=np.float64)
        return (points - self.translation) @ self.rotation


class PoseBuffer:
    """Monotonic pose history with interpolation and fail-closed lookup."""

    def __init__(self, capacity: int, maximum_interpolation_gap_sec: float):
        if capacity < 2 or maximum_interpolation_gap_sec <= 0.0:
            raise ValueError("invalid pose buffer configuration")
        self.capacity = int(capacity)
        self.maximum_interpolation_gap_sec = float(maximum_interpolation_gap_sec)
        self._poses: List[Pose3D] = []

    def clear(self) -> None:
        self._poses = []

    @property
    def size(self) -> int:
        return len(self._poses)

    @property
    def newest_stamp(self) -> Optional[float]:
        return None if not self._poses else self._poses[-1].stamp_sec

    def add(self, pose: Pose3D) -> bool:
        """Add a pose. Return True when a backward ROS-time reset was detected."""
        reset = bool(self._poses and pose.stamp_sec < self._poses[-1].stamp_sec - 1.0e-9)
        if reset:
            self.clear()
        if self._poses and abs(pose.stamp_sec - self._poses[-1].stamp_sec) <= 1.0e-9:
            self._poses[-1] = pose
        else:
            self._poses.append(pose)
        if len(self._poses) > self.capacity:
            self._poses = self._poses[-self.capacity :]
        return reset

    def lookup(self, stamp_sec: float) -> Tuple[Optional[Pose3D], str]:
        if not self._poses:
            return None, "pose_buffer_empty"
        stamps = [pose.stamp_sec for pose in self._poses]
        index = bisect_left(stamps, stamp_sec)
        if index < len(stamps) and abs(stamps[index] - stamp_sec) <= 1.0e-9:
            return self._poses[index], "exact"
        if index == 0:
            return None, "cloud_precedes_pose_history"
        if index == len(stamps):
            return None, "cloud_newer_than_pose_history"
        before = self._poses[index - 1]
        after = self._poses[index]
        gap = after.stamp_sec - before.stamp_sec
        if gap <= 0.0 or gap > self.maximum_interpolation_gap_sec:
            return None, "pose_interpolation_gap_too_large"
        ratio = (stamp_sec - before.stamp_sec) / gap
        pose = Pose3D(
            stamp_sec=float(stamp_sec),
            translation=(1.0 - ratio) * before.translation + ratio * after.translation,
            quaternion_xyzw=slerp_xyzw(before.quaternion_xyzw, after.quaternion_xyzw, ratio),
        )
        return pose, "interpolated"
