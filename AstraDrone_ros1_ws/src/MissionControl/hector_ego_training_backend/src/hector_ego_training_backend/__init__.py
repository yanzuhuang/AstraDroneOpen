"""Training-only Hector execution backend helpers."""

from .adapter_contract import (
    CommandSample,
    StateSample,
    command_is_valid,
    quaternion_from_yaw,
    trajectory_id_is_new,
    tracking_errors,
)

__all__ = [
    "CommandSample",
    "StateSample",
    "command_is_valid",
    "quaternion_from_yaw",
    "trajectory_id_is_new",
    "tracking_errors",
]
