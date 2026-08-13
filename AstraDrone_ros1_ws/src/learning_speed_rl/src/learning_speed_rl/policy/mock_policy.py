"""Operator-commanded policy for interface and flight validation only."""

import math
import threading

from .base import SpeedPolicy


class MockSpeedPolicy(SpeedPolicy):
    def __init__(self, initial_v_max: float):
        if not math.isfinite(initial_v_max):
            raise ValueError("initial_v_max must be finite")
        self._command = float(initial_v_max)
        self._lock = threading.Lock()

    def set_command(self, requested_v_max: float) -> None:
        if not math.isfinite(requested_v_max):
            raise ValueError("mock command must be finite")
        with self._lock:
            self._command = float(requested_v_max)

    def predict(self, observation) -> float:
        del observation  # The mock keeps the future policy signature intact.
        with self._lock:
            return self._command
