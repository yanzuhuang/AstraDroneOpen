"""Monotonic replacement policy for EGO replanning messages."""

from typing import Optional

from .bspline import EgoBsplineTrajectory


class ActiveTrajectoryStore:
    def __init__(self):
        self._trajectory: Optional[EgoBsplineTrajectory] = None

    @property
    def current(self) -> Optional[EgoBsplineTrajectory]:
        return self._trajectory

    def clear(self) -> None:
        self._trajectory = None

    def update(self, candidate: EgoBsplineTrajectory) -> bool:
        current = self._trajectory
        if current is not None:
            if candidate.start_time_sec < current.start_time_sec - 1.0e-9:
                return False
            if (
                abs(candidate.start_time_sec - current.start_time_sec) <= 1.0e-9
                and candidate.trajectory_id <= current.trajectory_id
            ):
                return False
        self._trajectory = candidate
        return True
