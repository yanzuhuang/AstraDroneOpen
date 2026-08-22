"""Pure validation and geometry used by the ROS adapter and unit tests."""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


@dataclass(frozen=True)
class CommandSample:
    stamp_sec: float
    frame_id: str
    trajectory_id: int
    trajectory_ready: bool
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    acceleration: Tuple[float, float, float]
    yaw: float
    yaw_dot: float


@dataclass(frozen=True)
class StateSample:
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]


def command_is_valid(
    command: CommandSample,
    expected_frame: str,
    now_sec: float,
    freshness_sec: float,
    future_tolerance_sec: float,
) -> Tuple[bool, str]:
    if not command.trajectory_ready:
        return False, "trajectory_not_ready"
    if not command.frame_id or command.frame_id != expected_frame:
        return False, "frame_mismatch"
    if command.trajectory_id <= 0:
        return False, "trajectory_id_invalid"
    if not _finite(
        command.position
        + command.velocity
        + command.acceleration
        + (command.stamp_sec, command.yaw, command.yaw_dot, now_sec)
    ):
        return False, "non_finite"
    if command.stamp_sec <= 0.0:
        return False, "zero_stamp"
    age = now_sec - command.stamp_sec
    if age < -future_tolerance_sec:
        return False, "future_stamp"
    if age > freshness_sec:
        return False, "stale_stamp"
    return True, ""


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    if not math.isfinite(yaw):
        raise ValueError("yaw must be finite")
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def trajectory_id_is_new(trajectory_id: int, required_greater_than: int) -> bool:
    """Return true only for a strictly newer EGO trajectory generation."""
    return int(trajectory_id) > 0 and int(trajectory_id) > int(required_greater_than)


def tracking_errors(
    command: CommandSample, state: StateSample
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], float, float]:
    if not _finite(command.position + command.velocity + state.position + state.velocity):
        raise ValueError("command and state must be finite")
    position_error = tuple(
        desired - actual
        for desired, actual in zip(command.position, state.position)
    )
    velocity_error = tuple(
        desired - actual
        for desired, actual in zip(command.velocity, state.velocity)
    )
    position_norm = math.sqrt(sum(value * value for value in position_error))
    velocity_norm = math.sqrt(sum(value * value for value in velocity_error))
    return position_error, velocity_error, position_norm, velocity_norm
