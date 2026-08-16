"""Immutable fixed-speed source for repeatable baseline experiments."""

import math

from .base import SpeedPolicy


class FixedSpeedPolicy(SpeedPolicy):
    def __init__(self, fixed_v_max: float):
        if not math.isfinite(fixed_v_max) or fixed_v_max <= 0.0:
            raise ValueError("fixed_v_max must be finite and positive")
        self._fixed_v_max = float(fixed_v_max)

    @property
    def fixed_v_max(self) -> float:
        return self._fixed_v_max

    def predict(self, observation) -> float:
        del observation
        return self._fixed_v_max
