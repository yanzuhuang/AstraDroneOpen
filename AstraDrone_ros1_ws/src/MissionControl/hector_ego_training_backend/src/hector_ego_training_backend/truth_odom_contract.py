"""Pure contract helpers for the training-only Gazebo truth odometry path."""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple


CONTRACT_VERSION = "astradrone_training_odometry_v1.0"
BACKEND_MODE = "gazebo_truth_training"
SOURCE_TYPE = "gazebo_ros_p3d_ground_truth"
VELOCITY_FRAME = "world"


def normalize_frame(frame_id: str) -> str:
    return str(frame_id).lstrip("/")


def finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def quaternion_norm(quaternion: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in quaternion))


def yaw_from_xyzw(quaternion: Sequence[float]) -> float:
    x, y, z, w = (float(value) for value in quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def wrapped_angle_difference(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


@dataclass(frozen=True)
class TruthOdometrySample:
    stamp_sec: float
    frame_id: str
    child_frame_id: str
    position: Tuple[float, float, float]
    orientation_xyzw: Tuple[float, float, float, float]
    linear_velocity_world: Tuple[float, float, float]
    angular_velocity_world: Tuple[float, float, float]

    def values(self) -> Tuple[float, ...]:
        return (
            self.stamp_sec,
            *self.position,
            *self.orientation_xyzw,
            *self.linear_velocity_world,
            *self.angular_velocity_world,
        )


def validate_truth_sample(
    sample: TruthOdometrySample,
    now_sec: float,
    expected_frame: str,
    expected_child_frame: str,
    maximum_age_sec: float,
    future_tolerance_sec: float,
    quaternion_norm_tolerance: float,
):
    if not finite(sample.values()) or not math.isfinite(float(now_sec)):
        return False, "non_finite"
    if sample.stamp_sec <= 0.0:
        return False, "zero_stamp"
    if normalize_frame(sample.frame_id) != normalize_frame(expected_frame):
        return False, "frame_mismatch"
    if normalize_frame(sample.child_frame_id) != normalize_frame(
        expected_child_frame
    ):
        return False, "child_frame_mismatch"
    if abs(quaternion_norm(sample.orientation_xyzw) - 1.0) > float(
        quaternion_norm_tolerance
    ):
        return False, "invalid_quaternion_norm"
    age = float(now_sec) - sample.stamp_sec
    if age < -float(future_tolerance_sec):
        return False, "future_stamp"
    if age > float(maximum_age_sec):
        return False, "stale_stamp"
    return True, ""


def vector_difference_norm(first, second) -> float:
    return math.sqrt(
        sum((float(a) - float(b)) ** 2 for a, b in zip(first, second))
    )
