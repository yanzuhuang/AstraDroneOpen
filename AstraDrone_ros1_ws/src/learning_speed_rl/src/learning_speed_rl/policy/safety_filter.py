"""Stateless reviewed-range protection applied after every policy backend."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SafetyFilterConfig:
    v_max_min: float
    v_max_max: float
    initial_v_max: float

    def validate(self) -> None:
        values = (
            self.v_max_min,
            self.v_max_max,
            self.initial_v_max,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("all safety filter parameters must be finite")
        if self.v_max_min <= 0.0 or self.v_max_max < self.v_max_min:
            raise ValueError("invalid v_max range")
        if not self.v_max_min <= self.initial_v_max <= self.v_max_max:
            raise ValueError("initial_v_max is outside the configured range")


class SpeedSafetyFilter:
    def __init__(self, config: SafetyFilterConfig):
        config.validate()
        self.config = config

    def filter(self, raw_v_max: float) -> float:
        if not math.isfinite(raw_v_max):
            raise ValueError("policy output must be finite")
        return min(
            self.config.v_max_max,
            max(self.config.v_max_min, raw_v_max),
        )
