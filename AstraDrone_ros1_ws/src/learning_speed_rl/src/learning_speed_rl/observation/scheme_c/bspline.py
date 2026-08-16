"""Read-only evaluator for EGO's formally published ``traj_utils/Bspline``."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class EgoBsplineTrajectory:
    """A faithful NumPy form of EGO ``UniformBspline`` message state."""

    degree: int
    control_points: np.ndarray
    knots: np.ndarray
    start_time_sec: float
    trajectory_id: int
    frame_id: str

    def __post_init__(self) -> None:
        points = np.asarray(self.control_points, dtype=np.float64)
        knots = np.asarray(self.knots, dtype=np.float64)
        if self.degree < 1:
            raise ValueError("B-spline degree must be positive")
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("control_points must have shape [N,3]")
        expected_knots = points.shape[0] + self.degree + 1
        if points.shape[0] < self.degree + 1 or knots.shape != (expected_knots,):
            raise ValueError("control-point/knot count does not define one B-spline")
        if not np.all(np.isfinite(points)) or not np.all(np.isfinite(knots)):
            raise ValueError("B-spline values must be finite")
        if np.any(np.diff(knots) < 0.0):
            raise ValueError("B-spline knots must be nondecreasing")
        if not np.isfinite(self.start_time_sec) or self.start_time_sec <= 0.0:
            raise ValueError("trajectory start time must be positive and finite")
        if not self.frame_id:
            raise ValueError("trajectory frame is empty")
        object.__setattr__(self, "control_points", points)
        object.__setattr__(self, "knots", knots)
        if self.duration_sec <= 0.0:
            raise ValueError("B-spline duration must be positive")

    @property
    def parameter_start(self) -> float:
        return float(self.knots[self.degree])

    @property
    def parameter_end(self) -> float:
        # EGO: m=n+p+1=len(knots)-1, valid end is u_(m-p).
        return float(self.knots[len(self.knots) - 1 - self.degree])

    @property
    def duration_sec(self) -> float:
        return self.parameter_end - self.parameter_start

    @property
    def end_time_sec(self) -> float:
        return self.start_time_sec + self.duration_sec

    def is_active_at(self, stamp_sec: float, tolerance_sec: float = 1.0e-9) -> bool:
        return (
            np.isfinite(stamp_sec)
            and stamp_sec >= self.start_time_sec - tolerance_sec
            and stamp_sec <= self.end_time_sec + tolerance_sec
        )

    def elapsed_at(self, stamp_sec: float) -> float:
        if not self.is_active_at(stamp_sec):
            raise ValueError("observation timestamp lies outside active trajectory")
        return min(self.duration_sec, max(0.0, stamp_sec - self.start_time_sec))

    def evaluate_elapsed(self, elapsed_sec: float) -> np.ndarray:
        if not np.isfinite(elapsed_sec):
            raise ValueError("elapsed time must be finite")
        parameter = self.parameter_start + min(
            self.duration_sec, max(0.0, float(elapsed_sec))
        )
        return self._evaluate_parameter(parameter)

    def evaluate_stamp(self, stamp_sec: float) -> np.ndarray:
        return self.evaluate_elapsed(self.elapsed_at(stamp_sec))

    def derivative(self) -> "EgoBsplineTrajectory":
        denominators = (
            self.knots[np.arange(self.control_points.shape[0] - 1) + self.degree + 1]
            - self.knots[np.arange(self.control_points.shape[0] - 1) + 1]
        )
        if np.any(denominators <= 0.0):
            raise ValueError("B-spline derivative contains a zero knot span")
        points = (
            self.degree
            * np.diff(self.control_points, axis=0)
            / denominators[:, np.newaxis]
        )
        return EgoBsplineTrajectory(
            degree=self.degree - 1,
            control_points=points,
            knots=self.knots[1:-1],
            start_time_sec=self.start_time_sec,
            trajectory_id=self.trajectory_id,
            frame_id=self.frame_id,
        )

    def velocity_elapsed(self, elapsed_sec: float) -> np.ndarray:
        return self.derivative().evaluate_elapsed(elapsed_sec)

    def acceleration_elapsed(self, elapsed_sec: float) -> np.ndarray:
        if self.degree < 2:
            return np.zeros(3, dtype=np.float64)
        return self.derivative().derivative().evaluate_elapsed(elapsed_sec)

    def future_knot_times(self, elapsed_sec: float) -> np.ndarray:
        """Return unique elapsed knot boundaries from now through trajectory end."""
        now = min(self.duration_sec, max(0.0, float(elapsed_sec)))
        values = self.knots[self.degree : len(self.knots) - self.degree]
        elapsed = np.clip(values - self.parameter_start, now, self.duration_sec)
        return np.unique(np.concatenate(([now], elapsed, [self.duration_sec])))

    def _evaluate_parameter(self, parameter: float) -> np.ndarray:
        lower = self.parameter_start
        upper = self.parameter_end
        value = min(upper, max(lower, float(parameter)))
        k = self.degree
        # Match EGO's first interval whose right knot is >= the query.
        while k + 1 < len(self.knots) and self.knots[k + 1] < value:
            k += 1
        working = [
            self.control_points[k - self.degree + index].copy()
            for index in range(self.degree + 1)
        ]
        for recursion in range(1, self.degree + 1):
            for index in range(self.degree, recursion - 1, -1):
                left_index = index + k - self.degree
                right_index = index + 1 + k - recursion
                denominator = self.knots[right_index] - self.knots[left_index]
                if denominator <= 0.0:
                    raise ValueError("B-spline evaluation encountered a zero knot span")
                alpha = (value - self.knots[left_index]) / denominator
                working[index] = (
                    (1.0 - alpha) * working[index - 1] + alpha * working[index]
                )
        result = np.asarray(working[self.degree], dtype=np.float64)
        if result.shape != (3,) or not np.all(np.isfinite(result)):
            raise ValueError("B-spline evaluation produced an invalid point")
        return result
