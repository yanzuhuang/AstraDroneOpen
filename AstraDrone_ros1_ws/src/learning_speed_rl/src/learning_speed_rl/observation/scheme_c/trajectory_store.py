"""Bounded causal history for EGO replanning messages."""

from typing import List, Optional, Tuple
from dataclasses import dataclass

from .bspline import EgoBsplineTrajectory


@dataclass(frozen=True)
class TrajectoryLookupDiagnostics:
    target_stamp_sec: float
    history_size: int
    result: str
    selected_trajectory_id: Optional[int]
    selected_start_stamp_sec: Optional[float]
    selected_end_stamp_sec: Optional[float]
    latest_trajectory_id: Optional[int]
    latest_start_stamp_sec: Optional[float]


class ActiveTrajectoryStore:
    def __init__(self, capacity: int = 100):
        if capacity <= 0:
            raise ValueError("trajectory history capacity must be positive")
        self.capacity = int(capacity)
        self._trajectories: List[EgoBsplineTrajectory] = []

    @property
    def current(self) -> Optional[EgoBsplineTrajectory]:
        return None if not self._trajectories else self._trajectories[-1]

    def clear(self) -> None:
        self._trajectories = []

    def update(self, candidate: EgoBsplineTrajectory) -> bool:
        current = self.current
        if current is not None:
            if candidate.start_time_sec < current.start_time_sec - 1.0e-9:
                return False
            if (
                abs(candidate.start_time_sec - current.start_time_sec) <= 1.0e-9
                and candidate.trajectory_id <= current.trajectory_id
            ):
                return False
        self._trajectories.append(candidate)
        self._trajectories = self._trajectories[-self.capacity :]
        return True

    def lookup(self, stamp_sec: float) -> Optional[EgoBsplineTrajectory]:
        trajectory, _ = self.lookup_with_diagnostics(stamp_sec)
        return trajectory

    def lookup_with_diagnostics(
        self, stamp_sec: float,
    ) -> Tuple[Optional[EgoBsplineTrajectory], TrajectoryLookupDiagnostics]:
        """Return the latest received formal trajectory active at ``stamp_sec``.

        Replanning can replace ``current`` before a delayed lidar packet reaches
        Observation C.  Looking up by the packet's source stamp retains causal
        matching without ever selecting a trajectory that starts in its future.
        """
        for trajectory in reversed(self._trajectories):
            if trajectory.is_active_at(stamp_sec):
                return trajectory, TrajectoryLookupDiagnostics(
                    target_stamp_sec=float(stamp_sec),
                    history_size=len(self._trajectories),
                    result="selected",
                    selected_trajectory_id=trajectory.trajectory_id,
                    selected_start_stamp_sec=trajectory.start_time_sec,
                    selected_end_stamp_sec=trajectory.end_time_sec,
                    latest_trajectory_id=self.current.trajectory_id,
                    latest_start_stamp_sec=self.current.start_time_sec,
                )
        if not self._trajectories:
            result = "history_empty"
        elif stamp_sec < self._trajectories[0].start_time_sec:
            result = "before_first_trajectory"
        else:
            result = "no_active_trajectory_at_stamp"
        return None, TrajectoryLookupDiagnostics(
            target_stamp_sec=float(stamp_sec),
            history_size=len(self._trajectories),
            result=result,
            selected_trajectory_id=None,
            selected_start_stamp_sec=None,
            selected_end_stamp_sec=None,
            latest_trajectory_id=(
                None if self.current is None else self.current.trajectory_id
            ),
            latest_start_stamp_sec=(
                None if self.current is None else self.current.start_time_sec
            ),
        )
