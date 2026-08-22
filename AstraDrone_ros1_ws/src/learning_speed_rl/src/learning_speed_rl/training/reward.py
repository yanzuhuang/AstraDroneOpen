"""Paper-guided, framework-neutral reward terms for Learning Speed.

The module consumes only causal state/action diagnostics supplied by the
training-data boundary.  It does not publish commands, import a policy, or
touch planner, bridge, PX4, and Observation C semantics.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
import math
from typing import Dict, Mapping, Optional, Tuple


STAGE1_REWARD_VERSION = "astradrone_stage1_reward_v1.0"
STAGE1_REWARD_MODE = "stage1"


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("{} must be finite".format(name))
    return result


def _clip01(value: float) -> float:
    return min(1.0, max(0.0, value))


def _smoothstep01(value: float) -> float:
    value = _clip01(value)
    return value * value * (3.0 - 2.0 * value)


@dataclass(frozen=True)
class Stage1RewardConfig:
    """All numeric values are AstraDroneOpen calibrated parameters."""

    nearest_safe_m: float = 6.0
    nearest_dangerous_m: float = 2.5
    density_safe: float = 0.040
    density_dangerous: float = 0.080
    low_anchor_mps: float = 1.75
    medium_anchor_mps: float = 1.25
    high_anchor_mps: float = 0.75
    unknown_anchor_mps: float = 1.25
    lambda_phi_1: float = 0.65
    lambda_phi_2: float = 0.35
    lambda_speed_1: float = 1.00
    lambda_speed_2: float = 0.80
    lambda_speed_3: float = 0.25
    lambda_smoothing: float = 0.10
    lambda_danger: float = 2.00
    mode: str = STAGE1_REWARD_MODE
    version: str = STAGE1_REWARD_VERSION

    def __post_init__(self):
        numeric = {
            name: _finite(getattr(self, name), name)
            for name in (
                "nearest_safe_m",
                "nearest_dangerous_m",
                "density_safe",
                "density_dangerous",
                "low_anchor_mps",
                "medium_anchor_mps",
                "high_anchor_mps",
                "unknown_anchor_mps",
                "lambda_phi_1",
                "lambda_phi_2",
                "lambda_speed_1",
                "lambda_speed_2",
                "lambda_speed_3",
                "lambda_smoothing",
                "lambda_danger",
            )
        }
        if self.mode != STAGE1_REWARD_MODE or self.version != STAGE1_REWARD_VERSION:
            raise ValueError("unsupported reward mode/version")
        if numeric["nearest_safe_m"] <= numeric["nearest_dangerous_m"]:
            raise ValueError("nearest safe boundary must exceed dangerous boundary")
        if numeric["density_safe"] >= numeric["density_dangerous"]:
            raise ValueError("density safe boundary must be below dangerous boundary")
        if not 0.0 < self.lambda_phi_2 < 0.5 < self.lambda_phi_1 < 1.0:
            raise ValueError("lambda_phi_2 < 0.5 < lambda_phi_1 is required")
        if self.lambda_speed_2 <= self.lambda_speed_3:
            raise ValueError("paper relation lambda_speed_2 > lambda_speed_3 failed")
        if min(
            self.lambda_speed_1,
            self.lambda_speed_2,
            self.lambda_speed_3,
            self.lambda_smoothing,
            self.lambda_danger,
        ) <= 0.0:
            raise ValueError("reward lambdas must be positive")
        if not self.low_anchor_mps > self.medium_anchor_mps > self.high_anchor_mps:
            raise ValueError("human anchors must be ordered low > medium > high")
        midpoint = 0.5 * (self.low_anchor_mps + self.high_anchor_mps)
        if abs(self.medium_anchor_mps - midpoint) > 1.0e-12:
            raise ValueError("medium anchor must be the low/high midpoint")
        if abs(self.unknown_anchor_mps - self.medium_anchor_mps) > 1.0e-12:
            raise ValueError("unknown anchor must equal the conservative medium anchor")

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]):
        reward = values.get("reward", values)
        if not isinstance(reward, Mapping):
            raise ValueError("reward configuration must be a mapping")
        complexity = reward.get("complexity", {})
        anchors = reward.get("anchors_mps", {})
        lambdas = reward.get("lambdas", {})
        return cls(
            mode=str(reward.get("mode", STAGE1_REWARD_MODE)),
            version=str(reward.get("version", STAGE1_REWARD_VERSION)),
            nearest_safe_m=float(complexity.get("nearest_safe_m", 6.0)),
            nearest_dangerous_m=float(
                complexity.get("nearest_dangerous_m", 2.5)
            ),
            density_safe=float(complexity.get("density_safe", 0.040)),
            density_dangerous=float(
                complexity.get("density_dangerous", 0.080)
            ),
            low_anchor_mps=float(anchors.get("low", 1.75)),
            medium_anchor_mps=float(anchors.get("medium", 1.25)),
            high_anchor_mps=float(anchors.get("high", 0.75)),
            unknown_anchor_mps=float(anchors.get("unknown", 1.25)),
            lambda_phi_1=float(lambdas.get("phi_1", 0.65)),
            lambda_phi_2=float(lambdas.get("phi_2", 0.35)),
            lambda_speed_1=float(lambdas.get("speed_1", 1.00)),
            lambda_speed_2=float(lambdas.get("speed_2", 0.80)),
            lambda_speed_3=float(lambdas.get("speed_3", 0.25)),
            lambda_smoothing=float(lambdas.get("smoothing", 0.10)),
            lambda_danger=float(lambdas.get("danger", 2.00)),
        )


@dataclass(frozen=True)
class Stage1RewardInput:
    """Causal inputs for one transition; no state_t+1 payload is accepted."""

    nearest_obstacle_distance_m: Optional[float]
    known_obstacle_bin_fraction: float
    unknown_majority: bool
    applied_v_max_mps: float
    previous_applied_v_max_mps: float
    actual_speed_mps: float
    dangerous_terminal: bool
    terminated: bool
    observation_valid: bool = True
    same_episode: bool = True
    truncated: bool = False

    def __post_init__(self):
        density = _finite(
            self.known_obstacle_bin_fraction, "known_obstacle_bin_fraction"
        )
        applied = _finite(self.applied_v_max_mps, "applied_v_max_mps")
        previous = _finite(
            self.previous_applied_v_max_mps, "previous_applied_v_max_mps"
        )
        actual = _finite(self.actual_speed_mps, "actual_speed_mps")
        if density < 0.0 or density > 1.0:
            raise ValueError("known_obstacle_bin_fraction must be in [0, 1]")
        if min(applied, previous) <= 0.0 or actual < 0.0:
            raise ValueError("speed inputs are outside their valid ranges")
        if self.nearest_obstacle_distance_m is not None:
            nearest = _finite(
                self.nearest_obstacle_distance_m,
                "nearest_obstacle_distance_m",
            )
            if nearest <= 0.0:
                raise ValueError("numeric nearest distance must be positive")
            if self.unknown_majority:
                raise ValueError("numeric nearest cannot be unknown-majority")
        elif density > 1.0e-12:
            raise ValueError("nearest NA is inconsistent with nonzero obstacle density")
        if self.unknown_majority and self.nearest_obstacle_distance_m is not None:
            raise ValueError("unknown-majority requires nearest NA")
        if self.dangerous_terminal and not self.terminated:
            raise ValueError("dangerous terminal must terminate the episode")


def stage1_reward_input_from_signals(
    *,
    nearest_obstacle_distance_m: Optional[float],
    known_obstacle_bin_fraction: float,
    unknown_bin_count: int,
    lidar_bin_count: int,
    applied_v_max_mps: float,
    previous_applied_v_max_mps: float,
    actual_speed_mps: float,
    dangerous_terminal: bool,
    terminated: bool,
    observation_valid: bool = True,
    same_episode: bool = True,
    truncated: bool = False,
) -> Stage1RewardInput:
    """Build the reviewed input from online or replay transition signals.

    This helper contains only the frozen unknown-majority conversion shared by
    the recorder and offline replay.  Reward terms remain exclusively in
    :meth:`Stage1Reward.evaluate`.
    """

    unknown_count = int(unknown_bin_count)
    total_count = int(lidar_bin_count)
    if total_count <= 0 or unknown_count < 0 or unknown_count > total_count:
        raise ValueError("lidar bin counts are invalid")
    density = float(known_obstacle_bin_fraction)
    unknown_majority = bool(
        nearest_obstacle_distance_m is None
        and math.isfinite(density)
        and abs(density) <= 1.0e-12
        and unknown_count > total_count / 2.0
    )
    return Stage1RewardInput(
        nearest_obstacle_distance_m=nearest_obstacle_distance_m,
        known_obstacle_bin_fraction=density,
        unknown_majority=unknown_majority,
        applied_v_max_mps=applied_v_max_mps,
        previous_applied_v_max_mps=previous_applied_v_max_mps,
        actual_speed_mps=actual_speed_mps,
        dangerous_terminal=dangerous_terminal,
        terminated=terminated,
        observation_valid=observation_valid,
        same_episode=same_episode,
        truncated=truncated,
    )


def actual_speed_mps_from_body_velocity(actual_velocity_body) -> float:
    """Return the shared recorder/replay norm of the three body components."""

    components = tuple(float(component) for component in actual_velocity_body)
    if len(components) != 3 or not all(math.isfinite(value) for value in components):
        raise ValueError("actual_velocity_body must contain three finite components")
    return math.sqrt(sum(component * component for component in components))


@dataclass(frozen=True)
class ComplexityContext:
    label: str
    nearest_obstacle_distance_m: Optional[float]
    known_obstacle_bin_fraction: float
    unknown_majority: bool
    nearest_risk: float
    density_risk: float
    branch_weights: Tuple[float, float, float]

    def to_record(self) -> Dict[str, object]:
        safe, middle, dangerous = self.branch_weights
        return {
            "label": self.label,
            "nearest_obstacle_distance_m": self.nearest_obstacle_distance_m,
            "known_obstacle_bin_fraction": self.known_obstacle_bin_fraction,
            "unknown_majority": self.unknown_majority,
            "nearest_risk": self.nearest_risk,
            "density_risk": self.density_risk,
            "safe_weight": safe,
            "middle_weight": middle,
            "dangerous_weight": dangerous,
        }


@dataclass(frozen=True)
class RewardEvaluation:
    reward_total: Optional[float]
    reward_speed: Optional[float]
    reward_smoothing: Optional[float]
    reward_danger: Optional[float]
    phi_1: Optional[float]
    phi_2: Optional[float]
    complexity_context: Optional[ComplexityContext]
    reward_valid: bool
    invalid_reason: str = ""
    version: str = STAGE1_REWARD_VERSION

    def __post_init__(self):
        values = (
            self.reward_total,
            self.reward_speed,
            self.reward_smoothing,
            self.reward_danger,
            self.phi_1,
            self.phi_2,
        )
        if self.reward_valid:
            if any(value is None or not math.isfinite(value) for value in values):
                raise ValueError("valid reward terms must all be finite")
            if self.complexity_context is None or self.invalid_reason:
                raise ValueError("valid reward has invalid metadata")
        elif any(value is not None for value in values) or not self.invalid_reason:
            raise ValueError("invalid reward must contain no numeric reward terms")

    def to_record(self) -> Dict[str, object]:
        return {
            "version": self.version,
            "reward_valid": self.reward_valid,
            "invalid_reason": self.invalid_reason,
            "reward_total": self.reward_total,
            "reward_speed": self.reward_speed,
            "reward_smoothing": self.reward_smoothing,
            "reward_danger": self.reward_danger,
            "phi_1": self.phi_1,
            "phi_2": self.phi_2,
            "complexity_context": (
                None
                if self.complexity_context is None
                else self.complexity_context.to_record()
            ),
        }


class SpeedRewardTerm(ABC):
    """Replace this strategy to implement Stage 2 without changing other terms."""

    @abstractmethod
    def evaluate(self, phi_1: float, phi_2: float, applied_v_max_mps: float):
        """Return (reward, safe/middle/dangerous weights)."""


class PaperGuidedStage1SpeedReward(SpeedRewardTerm):
    """Continuous blend of the three paper Eq. (10) branch expressions."""

    def __init__(self, config: Stage1RewardConfig):
        self.config = config

    def _weights(self, phi_2: float) -> Tuple[float, float, float]:
        lower = self.config.lambda_phi_2
        upper = self.config.lambda_phi_1
        midpoint = 0.5
        if phi_2 <= lower:
            return (1.0, 0.0, 0.0)
        if phi_2 < midpoint:
            safe = 1.0 - _smoothstep01((phi_2 - lower) / (midpoint - lower))
            return (safe, 1.0 - safe, 0.0)
        if phi_2 < upper:
            dangerous = _smoothstep01(
                (phi_2 - midpoint) / (upper - midpoint)
            )
            return (0.0, 1.0 - dangerous, dangerous)
        return (0.0, 0.0, 1.0)

    def evaluate(self, phi_1: float, phi_2: float, applied_v_max_mps: float):
        safe_weight, middle_weight, dangerous_weight = self._weights(phi_2)
        dangerous_branch = self.config.lambda_speed_1 * (
            phi_1 - applied_v_max_mps
        )
        safe_branch = self.config.lambda_speed_2 * (
            applied_v_max_mps - phi_1
        )
        middle_branch = self.config.lambda_speed_3 * applied_v_max_mps
        reward = (
            dangerous_weight * dangerous_branch
            + safe_weight * safe_branch
            + middle_weight * middle_branch
        )
        return reward, (safe_weight, middle_weight, dangerous_weight)


class Stage1Reward:
    """Eq. (6)/(7)/(9)/(10) paper-guided AstraDroneOpen Stage 1 reward."""

    def __init__(
        self,
        config: Optional[Stage1RewardConfig] = None,
        speed_reward: Optional[SpeedRewardTerm] = None,
    ):
        self.config = config or Stage1RewardConfig()
        self.speed_reward = speed_reward or PaperGuidedStage1SpeedReward(self.config)

    def _complexity(self, value: Stage1RewardInput):
        config = self.config
        density = value.known_obstacle_bin_fraction
        nearest = value.nearest_obstacle_distance_m
        if value.unknown_majority:
            phi_2 = 0.5
            phi_1 = config.unknown_anchor_mps
            nearest_risk = 0.5
            density_risk = 0.5
            label = "Unknown"
        else:
            nearest_risk = (
                0.0
                if nearest is None
                else _clip01(
                    (config.nearest_safe_m - nearest)
                    / (config.nearest_safe_m - config.nearest_dangerous_m)
                )
            )
            density_risk = _clip01(
                (density - config.density_safe)
                / (config.density_dangerous - config.density_safe)
            )
            phi_2 = max(nearest_risk, density_risk)
            phi_1 = config.low_anchor_mps + phi_2 * (
                config.high_anchor_mps - config.low_anchor_mps
            )
            if (
                (nearest is None or nearest >= config.nearest_safe_m)
                and density <= config.density_safe
            ):
                label = "Low"
            elif (
                (nearest is not None and nearest <= config.nearest_dangerous_m)
                or density >= config.density_dangerous
            ):
                label = "High"
            else:
                label = "Medium"
        reward_speed, weights = self.speed_reward.evaluate(
            phi_1, phi_2, value.applied_v_max_mps
        )
        context = ComplexityContext(
            label=label,
            nearest_obstacle_distance_m=nearest,
            known_obstacle_bin_fraction=density,
            unknown_majority=value.unknown_majority,
            nearest_risk=nearest_risk,
            density_risk=density_risk,
            branch_weights=weights,
        )
        return phi_1, phi_2, reward_speed, context

    def evaluate(self, value: Stage1RewardInput) -> RewardEvaluation:
        if not value.observation_valid:
            return self._invalid("invalid_observation")
        if not value.same_episode:
            return self._invalid("episode_boundary")
        if value.truncated:
            return self._invalid("truncated_transition")
        phi_1, phi_2, reward_speed, context = self._complexity(value)
        delta_action = value.applied_v_max_mps - value.previous_applied_v_max_mps
        reward_smoothing = -self.config.lambda_smoothing * delta_action * delta_action
        reward_danger = (
            -self.config.lambda_danger * value.actual_speed_mps ** 2
            if value.dangerous_terminal
            else 0.0
        )
        reward_total = reward_speed + reward_smoothing + reward_danger
        return RewardEvaluation(
            reward_total=reward_total,
            reward_speed=reward_speed,
            reward_smoothing=reward_smoothing,
            reward_danger=reward_danger,
            phi_1=phi_1,
            phi_2=phi_2,
            complexity_context=context,
            reward_valid=True,
        )

    @staticmethod
    def _invalid(reason: str) -> RewardEvaluation:
        return RewardEvaluation(
            reward_total=None,
            reward_speed=None,
            reward_smoothing=None,
            reward_danger=None,
            phi_1=None,
            phi_2=None,
            complexity_context=None,
            reward_valid=False,
            invalid_reason=reason,
        )


def reward_from_config(values: Mapping[str, object]) -> Stage1Reward:
    config = Stage1RewardConfig.from_mapping(values)
    if config.mode != STAGE1_REWARD_MODE:
        raise ValueError("only the implemented stage1 reward may be selected")
    return Stage1Reward(config=config)
