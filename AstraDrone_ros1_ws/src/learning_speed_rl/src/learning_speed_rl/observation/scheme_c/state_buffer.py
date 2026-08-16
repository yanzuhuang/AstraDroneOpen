"""Timestamped FAST-LIO kinematic state with fail-closed interpolation."""

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from ..v2.frame_transform import Pose3D, slerp_xyzw


@dataclass(frozen=True)
class KinematicState:
    pose: Pose3D
    velocity_world: np.ndarray
    velocity_valid: bool

    def __post_init__(self) -> None:
        velocity = np.asarray(self.velocity_world, dtype=np.float64)
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            raise ValueError("velocity_world must be a finite xyz vector")
        object.__setattr__(self, "velocity_world", velocity)


class KinematicStateBuffer:
    def __init__(
        self, capacity: int, maximum_interpolation_gap_sec: float,
        velocity_filter_alpha: float = 0.30,
        maximum_velocity_dt_sec: float = 0.50,
    ):
        if capacity < 2 or maximum_interpolation_gap_sec <= 0.0:
            raise ValueError("invalid kinematic buffer configuration")
        if not 0.0 < velocity_filter_alpha <= 1.0:
            raise ValueError("velocity_filter_alpha must be in (0,1]")
        if maximum_velocity_dt_sec <= 0.0:
            raise ValueError("maximum_velocity_dt_sec must be positive")
        self.capacity = int(capacity)
        self.maximum_interpolation_gap_sec = float(maximum_interpolation_gap_sec)
        self.velocity_filter_alpha = float(velocity_filter_alpha)
        self.maximum_velocity_dt_sec = float(maximum_velocity_dt_sec)
        self._states: List[KinematicState] = []

    def clear(self) -> None:
        self._states = []

    @property
    def newest_stamp(self) -> Optional[float]:
        return None if not self._states else self._states[-1].pose.stamp_sec

    def add_pose(self, pose: Pose3D) -> bool:
        reset = bool(
            self._states
            and pose.stamp_sec < self._states[-1].pose.stamp_sec - 1.0e-9
        )
        if reset:
            self.clear()
        velocity = np.zeros(3, dtype=np.float64)
        valid = False
        if self._states:
            previous = self._states[-1]
            dt = pose.stamp_sec - previous.pose.stamp_sec
            if abs(dt) <= 1.0e-9:
                velocity = previous.velocity_world
                valid = previous.velocity_valid
            elif 1.0e-4 <= dt <= self.maximum_velocity_dt_sec:
                measured = (pose.translation - previous.pose.translation) / dt
                velocity = measured
                if previous.velocity_valid:
                    alpha = self.velocity_filter_alpha
                    velocity = alpha * measured + (1.0 - alpha) * previous.velocity_world
                valid = bool(np.all(np.isfinite(velocity)))
        state = KinematicState(pose, velocity, valid)
        if self._states and abs(pose.stamp_sec - self._states[-1].pose.stamp_sec) <= 1.0e-9:
            self._states[-1] = state
        else:
            self._states.append(state)
        self._states = self._states[-self.capacity :]
        return reset

    def lookup(self, stamp_sec: float) -> Tuple[Optional[KinematicState], str]:
        if not self._states:
            return None, "kinematic_buffer_empty"
        stamps = [state.pose.stamp_sec for state in self._states]
        index = bisect_left(stamps, stamp_sec)
        if index < len(stamps) and abs(stamps[index] - stamp_sec) <= 1.0e-9:
            state = self._states[index]
            return (state, "exact") if state.velocity_valid else (None, "velocity_unavailable")
        if index == 0:
            return None, "observation_precedes_kinematic_history"
        if index == len(stamps):
            return None, "observation_newer_than_kinematic_history"
        before = self._states[index - 1]
        after = self._states[index]
        gap = after.pose.stamp_sec - before.pose.stamp_sec
        if gap <= 0.0 or gap > self.maximum_interpolation_gap_sec:
            return None, "kinematic_interpolation_gap_too_large"
        if not before.velocity_valid or not after.velocity_valid:
            return None, "velocity_unavailable"
        ratio = (stamp_sec - before.pose.stamp_sec) / gap
        pose = Pose3D(
            stamp_sec=stamp_sec,
            translation=(1.0 - ratio) * before.pose.translation + ratio * after.pose.translation,
            quaternion_xyzw=slerp_xyzw(
                before.pose.quaternion_xyzw, after.pose.quaternion_xyzw, ratio
            ),
        )
        velocity = (
            (1.0 - ratio) * before.velocity_world + ratio * after.velocity_world
        )
        return KinematicState(pose, velocity, True), "interpolated"


class TimestampedScalarBuffer:
    """Zero-order-held scalar state queried at, never after, observation time."""

    def __init__(self, capacity: int = 100):
        if capacity <= 0:
            raise ValueError("scalar buffer capacity must be positive")
        self.capacity = int(capacity)
        self._samples = []

    def clear(self) -> None:
        self._samples = []

    @property
    def latest(self):
        return None if not self._samples else self._samples[-1]

    def add(self, stamp_sec: float, value: float) -> bool:
        if not np.isfinite(stamp_sec) or stamp_sec <= 0.0 or not np.isfinite(value):
            raise ValueError("timestamped scalar must be finite with positive stamp")
        reset = bool(self._samples and stamp_sec < self._samples[-1][0] - 1.0e-9)
        if reset:
            self.clear()
        sample = (float(stamp_sec), float(value))
        if self._samples and abs(stamp_sec - self._samples[-1][0]) <= 1.0e-9:
            self._samples[-1] = sample
        else:
            self._samples.append(sample)
        self._samples = self._samples[-self.capacity :]
        return reset

    def lookup(self, stamp_sec: float):
        if not self._samples:
            return None, "scalar_state_unavailable"
        stamps = [sample[0] for sample in self._samples]
        index = bisect_right(stamps, stamp_sec + 1.0e-9) - 1
        if index < 0:
            return None, "scalar_state_newer_than_observation"
        sample_stamp, value = self._samples[index]
        return value, "zero_order_hold:{:.9f}".format(sample_stamp)
