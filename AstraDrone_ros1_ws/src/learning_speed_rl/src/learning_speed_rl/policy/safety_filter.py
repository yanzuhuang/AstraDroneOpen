"""Deterministic safety envelope applied after every policy backend."""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class SafetyFilterConfig:
    v_max_min: float
    v_max_max: float
    initial_v_max: float
    rise_rate_mps2: float
    fall_rate_mps2: float
    maximum_step_mps: float
    low_pass_alpha: float
    hysteresis_mps: float

    def validate(self) -> None:
        values = (
            self.v_max_min,
            self.v_max_max,
            self.initial_v_max,
            self.rise_rate_mps2,
            self.fall_rate_mps2,
            self.maximum_step_mps,
            self.low_pass_alpha,
            self.hysteresis_mps,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("all safety filter parameters must be finite")
        if self.v_max_min <= 0.0 or self.v_max_max < self.v_max_min:
            raise ValueError("invalid v_max range")
        if not self.v_max_min <= self.initial_v_max <= self.v_max_max:
            raise ValueError("initial_v_max is outside the configured range")
        if self.rise_rate_mps2 <= 0.0 or self.fall_rate_mps2 <= 0.0:
            raise ValueError("rise/fall rates must be positive")
        if self.maximum_step_mps <= 0.0:
            raise ValueError("maximum_step_mps must be positive")
        if not 0.0 < self.low_pass_alpha <= 1.0:
            raise ValueError("low_pass_alpha must be in (0, 1]")
        if self.hysteresis_mps < 0.0:
            raise ValueError("hysteresis_mps must be non-negative")


class SpeedSafetyFilter:
    def __init__(self, config: SafetyFilterConfig):
        config.validate()
        self.config = config
        self.output = config.initial_v_max
        self.target = config.initial_v_max
        self._last_time: Optional[float] = None

    def update(self, raw_v_max: float, stamp_sec: float) -> float:
        if not math.isfinite(raw_v_max) or not math.isfinite(stamp_sec):
            raise ValueError("policy output and time must be finite")

        clamped = min(self.config.v_max_max, max(self.config.v_max_min, raw_v_max))
        if abs(clamped - self.target) > self.config.hysteresis_mps:
            self.target = clamped

        if self._last_time is None or stamp_sec <= self._last_time:
            self._last_time = stamp_sec
            return self.output

        dt = stamp_sec - self._last_time
        self._last_time = stamp_sec
        rate = self.config.rise_rate_mps2 if self.target > self.output else self.config.fall_rate_mps2
        allowed = min(self.config.maximum_step_mps, rate * dt)
        delta = min(allowed, max(-allowed, self.target - self.output))
        slewed = self.output + delta
        self.output += self.config.low_pass_alpha * (slewed - self.output)
        self.output = min(self.config.v_max_max, max(self.config.v_max_min, self.output))
        return self.output

    def reset_time(self) -> None:
        self._last_time = None
