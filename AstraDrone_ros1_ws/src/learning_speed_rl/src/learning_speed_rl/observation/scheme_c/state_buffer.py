"""Timestamped backend-neutral kinematic state with fail-closed interpolation."""

from bisect import bisect_left, bisect_right
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

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


@dataclass(frozen=True)
class KinematicLookupDiagnostics:
    target_stamp_sec: float
    receipt_stamp_sec: Optional[float]
    before_stamp_sec: Optional[float]
    after_stamp_sec: Optional[float]
    dt_before_sec: Optional[float]
    dt_after_sec: Optional[float]
    bracket_span_sec: Optional[float]
    nearest_state_dt_sec: Optional[float]
    oldest_stamp_sec: Optional[float]
    newest_stamp_sec: Optional[float]
    buffer_size: int
    buffer_coverage_sec: float
    latest_state_age_at_lookup_sec: Optional[float]
    source_rate_hz: float
    insert_rate_hz: float
    insert_count: int
    out_of_order_state_count: int
    duplicate_state_stamp_count: int
    eviction_count: int
    result: str
    failure_reason: str


class KinematicStateBuffer:
    def __init__(
        self, capacity: int, maximum_interpolation_gap_sec: float,
        velocity_filter_alpha: float = 0.30,
        maximum_velocity_dt_sec: float = 0.50,
        lookup_policy: str = "interpolate",
    ):
        if capacity < 2 or maximum_interpolation_gap_sec <= 0.0:
            raise ValueError("invalid kinematic buffer configuration")
        if not 0.0 < velocity_filter_alpha <= 1.0:
            raise ValueError("velocity_filter_alpha must be in (0,1]")
        if maximum_velocity_dt_sec <= 0.0:
            raise ValueError("maximum_velocity_dt_sec must be positive")
        if lookup_policy not in ("interpolate", "causal_at_or_before"):
            raise ValueError("unsupported kinematic lookup policy")
        self.capacity = int(capacity)
        self.maximum_interpolation_gap_sec = float(maximum_interpolation_gap_sec)
        self.velocity_filter_alpha = float(velocity_filter_alpha)
        self.maximum_velocity_dt_sec = float(maximum_velocity_dt_sec)
        self.lookup_policy = str(lookup_policy)
        self._states: List[KinematicState] = []
        self._source_intervals: Deque[float] = deque(maxlen=100)
        self._receipt_intervals: Deque[float] = deque(maxlen=100)
        self._last_receipt_sec: Optional[float] = None
        self._insert_count = 0
        self._out_of_order_state_count = 0
        self._duplicate_state_stamp_count = 0
        self._eviction_count = 0

    def clear(self) -> None:
        self._states = []
        self._source_intervals.clear()
        self._receipt_intervals.clear()
        self._last_receipt_sec = None

    @property
    def size(self) -> int:
        return len(self._states)

    @property
    def oldest_stamp(self) -> Optional[float]:
        return None if not self._states else self._states[0].pose.stamp_sec

    @property
    def newest_stamp(self) -> Optional[float]:
        return None if not self._states else self._states[-1].pose.stamp_sec

    @staticmethod
    def _rate(intervals: Deque[float]) -> float:
        return 0.0 if not intervals else 1.0 / (sum(intervals) / len(intervals))

    def add_pose(self, pose: Pose3D, receipt_sec: Optional[float] = None) -> bool:
        reset = bool(
            self._states
            and pose.stamp_sec < self._states[-1].pose.stamp_sec - 1.0e-9
        )
        if reset:
            self._out_of_order_state_count += 1
            self.clear()
        self._insert_count += 1
        if self._states:
            source_dt = pose.stamp_sec - self._states[-1].pose.stamp_sec
            if source_dt > 1.0e-9:
                self._source_intervals.append(source_dt)
            elif abs(source_dt) <= 1.0e-9:
                self._duplicate_state_stamp_count += 1
        if receipt_sec is not None and np.isfinite(receipt_sec):
            if self._last_receipt_sec is not None:
                receipt_dt = float(receipt_sec) - self._last_receipt_sec
                if receipt_dt > 1.0e-9:
                    self._receipt_intervals.append(receipt_dt)
            self._last_receipt_sec = float(receipt_sec)
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
        if len(self._states) > self.capacity:
            self._eviction_count += len(self._states) - self.capacity
        self._states = self._states[-self.capacity :]
        return reset

    def lookup(self, stamp_sec: float) -> Tuple[Optional[KinematicState], str]:
        state, result, _ = self.lookup_with_diagnostics(stamp_sec)
        return state, result

    def lookup_with_diagnostics(
        self, stamp_sec: float, receipt_sec: Optional[float] = None,
    ) -> Tuple[Optional[KinematicState], str, KinematicLookupDiagnostics]:
        stamps = [state.pose.stamp_sec for state in self._states]
        oldest = None if not stamps else stamps[0]
        newest = None if not stamps else stamps[-1]
        coverage = 0.0 if oldest is None else max(0.0, newest - oldest)
        latest_age = (
            None
            if receipt_sec is None or newest is None
            else float(receipt_sec) - newest
        )

        def finish(
            state, result, failure, before=None, after=None,
        ):
            dt_before = None if before is None else stamp_sec - before
            dt_after = None if after is None else after - stamp_sec
            bracket = (
                None if before is None or after is None else after - before
            )
            candidates = [
                value for value in (dt_before, dt_after)
                if value is not None and value >= -1.0e-9
            ]
            nearest = None if not candidates else min(abs(value) for value in candidates)
            diagnostic = KinematicLookupDiagnostics(
                target_stamp_sec=float(stamp_sec),
                receipt_stamp_sec=(
                    None if receipt_sec is None else float(receipt_sec)
                ),
                before_stamp_sec=before,
                after_stamp_sec=after,
                dt_before_sec=dt_before,
                dt_after_sec=dt_after,
                bracket_span_sec=bracket,
                nearest_state_dt_sec=nearest,
                oldest_stamp_sec=oldest,
                newest_stamp_sec=newest,
                buffer_size=len(stamps),
                buffer_coverage_sec=coverage,
                latest_state_age_at_lookup_sec=latest_age,
                source_rate_hz=self._rate(self._source_intervals),
                insert_rate_hz=self._rate(self._receipt_intervals),
                insert_count=self._insert_count,
                out_of_order_state_count=self._out_of_order_state_count,
                duplicate_state_stamp_count=self._duplicate_state_stamp_count,
                eviction_count=self._eviction_count,
                result=result,
                failure_reason=failure,
            )
            return state, result, diagnostic

        if not self._states:
            return finish(None, "kinematic_buffer_empty", "kinematic_buffer_empty")
        index = bisect_left(stamps, stamp_sec)
        if index < len(stamps) and abs(stamps[index] - stamp_sec) <= 1.0e-9:
            state = self._states[index]
            if state.velocity_valid:
                return finish(state, "exact", "", stamps[index], stamps[index])
            return finish(
                None, "velocity_unavailable", "velocity_unavailable",
                stamps[index], stamps[index],
            )
        if index == 0:
            return finish(
                None,
                "observation_precedes_kinematic_history",
                "observation_precedes_kinematic_history",
                after=stamps[0],
            )
        if self.lookup_policy == "causal_at_or_before":
            before = self._states[index - 1]
            age = stamp_sec - before.pose.stamp_sec
            if age > self.maximum_interpolation_gap_sec + 1.0e-9:
                return finish(
                    None,
                    "causal_state_too_old",
                    "causal_state_too_old",
                    before=before.pose.stamp_sec,
                )
            if not before.velocity_valid:
                return finish(
                    None,
                    "velocity_unavailable",
                    "velocity_unavailable",
                    before=before.pose.stamp_sec,
                )
            return finish(
                before,
                "causal_previous",
                "",
                before=before.pose.stamp_sec,
            )
        if index == len(stamps):
            return finish(
                None,
                "observation_newer_than_kinematic_history",
                "observation_newer_than_kinematic_history",
                before=stamps[-1],
            )
        before = self._states[index - 1]
        after = self._states[index]
        gap = after.pose.stamp_sec - before.pose.stamp_sec
        if gap <= 0.0 or gap > self.maximum_interpolation_gap_sec + 1.0e-9:
            return finish(
                None,
                "kinematic_interpolation_gap_too_large",
                "kinematic_interpolation_gap_too_large",
                before.pose.stamp_sec,
                after.pose.stamp_sec,
            )
        if not before.velocity_valid or not after.velocity_valid:
            return finish(
                None, "velocity_unavailable", "velocity_unavailable",
                before.pose.stamp_sec, after.pose.stamp_sec,
            )
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
        return finish(
            KinematicState(pose, velocity, True), "interpolated", "",
            before.pose.stamp_sec, after.pose.stamp_sec,
        )


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
