"""Scheme C: timestamped EGO trajectory and system-state fusion contracts."""

from .bspline import EgoBsplineTrajectory
from .builder import ObservationCBuilder
from .sampling import TrajectorySampler, TrajectorySamplingConfig
from .state_buffer import KinematicState, KinematicStateBuffer, TimestampedScalarBuffer
from .trajectory_store import ActiveTrajectoryStore
from .types import (
    OBSERVATION_C_VERSION,
    FutureTrajectoryFeature,
    LidarSurrogateFeature,
    ObservationC,
    SystemStateFeature,
)

__all__ = [
    "OBSERVATION_C_VERSION",
    "ActiveTrajectoryStore",
    "EgoBsplineTrajectory",
    "FutureTrajectoryFeature",
    "KinematicState",
    "KinematicStateBuffer",
    "LidarSurrogateFeature",
    "ObservationC",
    "ObservationCBuilder",
    "SystemStateFeature",
    "TrajectorySampler",
    "TrajectorySamplingConfig",
    "TimestampedScalarBuffer",
]
