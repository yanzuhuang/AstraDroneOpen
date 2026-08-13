"""Offline/Gazebo training contracts; never imported by the flight node."""

from .environment_interface import SpeedTrainingEnvironment, validate_artifact_root

__all__ = ["SpeedTrainingEnvironment", "validate_artifact_root"]
