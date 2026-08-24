"""Paper-guided, framework-neutral reward for Learning Speed.

The implementation consumes only causal state/action diagnostics supplied by
the training-data boundary. It does not publish commands, import a policy, or
touch planner, bridge, PX4, Hector, and Observation C semantics.
"""

from dataclasses import dataclass
import math
from typing import Dict, Mapping, Optional, Tuple


REWARD_VERSION = "astradrone_paper_guided_reward_v3.0"
STAGE_1_REWARD_MODE = "stage_1"
STAGE_2_REWARD_MODE = "stage_2"
SUPPORTED_REWARD_MODES = (STAGE_1_REWARD_MODE, STAGE_2_REWARD_MODE)


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("{} must be finite".format(name))
    return result


def _clip01(value: float) -> float:
    return min(1.0, max(0.0, value))


@dataclass(frozen=True)
class LearningSpeedRewardConfig:
    """AstraDroneOpen parameters; no numeric value claims paper reproduction."""

    nearest_safe_m: float = 6.0
    nearest_dangerous_m: float = 2.5
    density_safe: float = 0.040
    density_dangerous: float = 0.080
    nearest_weight: float = 0.46
    density_weight: float = 0.54
    low_anchor_mps: float = 1.75
    medium_anchor_mps: float = 1.25
    high_anchor_mps: float = 0.75
    unknown_anchor_mps: float = 1.25
    lambda_speed_1: float = 1.00
    lambda_speed_2: float = 0.80
    lambda_speed_3: float = 0.25
    lambda_smoothing: float = 0.10
    lambda_error: float = 2.00
    error_clip_max_m: float = 0.40
    lambda_danger: float = 2.00
    mode: str = STAGE_1_REWARD_MODE
    version: str = REWARD_VERSION

    def __post_init__(self):
        numeric = {
            name: _finite(getattr(self, name), name)
            for name in (
                "nearest_safe_m",
                "nearest_dangerous_m",
                "density_safe",
                "density_dangerous",
                "nearest_weight",
                "density_weight",
                "low_anchor_mps",
                "medium_anchor_mps",
                "high_anchor_mps",
                "unknown_anchor_mps",
                "lambda_speed_1",
                "lambda_speed_2",
                "lambda_speed_3",
                "lambda_smoothing",
                "lambda_error",
                "error_clip_max_m",
                "lambda_danger",
            )
        }
        if self.mode not in SUPPORTED_REWARD_MODES or self.version != REWARD_VERSION:
            raise ValueError("unsupported reward mode/version")
        if numeric["nearest_safe_m"] <= numeric["nearest_dangerous_m"]:
            raise ValueError("nearest safe boundary must exceed dangerous boundary")
        if numeric["density_safe"] >= numeric["density_dangerous"]:
            raise ValueError("density safe boundary must be below dangerous boundary")
        if min(numeric["nearest_weight"], numeric["density_weight"]) <= 0.0:
            raise ValueError("complexity fusion weights must both be positive")
        if abs(self.nearest_weight + self.density_weight - 1.0) > 1.0e-12:
            raise ValueError("complexity fusion weights must sum to one")
        if self.lambda_speed_2 <= self.lambda_speed_3:
            raise ValueError("paper relation lambda_speed_2 > lambda_speed_3 failed")
        if min(
            self.lambda_speed_1,
            self.lambda_speed_2,
            self.lambda_speed_3,
            self.lambda_smoothing,
            self.lambda_error,
            self.error_clip_max_m,
            self.lambda_danger,
        ) <= 0.0:
            raise ValueError("reward weights and error clip must be positive")
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
        fusion = reward.get("fusion", {})
        anchors = reward.get("anchors_mps", {})
        lambdas = reward.get("lambdas", {})
        tracking_error = reward.get("tracking_error", {})
        return cls(
            mode=str(reward.get("mode", STAGE_1_REWARD_MODE)),
            version=str(reward.get("version", REWARD_VERSION)),
            nearest_safe_m=float(complexity.get("nearest_safe_m", 6.0)),
            nearest_dangerous_m=float(complexity.get("nearest_dangerous_m", 2.5)),
            density_safe=float(complexity.get("density_safe", 0.040)),
            density_dangerous=float(complexity.get("density_dangerous", 0.080)),
            nearest_weight=float(fusion.get("nearest_weight", 0.46)),
            density_weight=float(fusion.get("density_weight", 0.54)),
            low_anchor_mps=float(anchors.get("low", 1.75)),
            medium_anchor_mps=float(anchors.get("medium", 1.25)),
            high_anchor_mps=float(anchors.get("high", 0.75)),
            unknown_anchor_mps=float(anchors.get("unknown", 1.25)),
            lambda_speed_1=float(lambdas.get("speed_1", 1.00)),
            lambda_speed_2=float(lambdas.get("speed_2", 0.80)),
            lambda_speed_3=float(lambdas.get("speed_3", 0.25)),
            lambda_smoothing=float(lambdas.get("smoothing", 0.10)),
            lambda_error=float(lambdas.get("error", 2.00)),
            error_clip_max_m=float(tracking_error.get("clip_max_m", 0.40)),
            lambda_danger=float(lambdas.get("danger", 2.00)),
        )


@dataclass(frozen=True)
class LearningSpeedRewardInput:
    """Causal state_t/action inputs; state_t+1 is deliberately unavailable."""

    nearest_obstacle_distance_m: Optional[float]
    known_obstacle_bin_fraction: float
    unknown_majority: bool
    applied_v_max_mps: float
    previous_applied_v_max_mps: float
    actual_speed_mps: float
    tracking_error_m: float
    dangerous_terminal: bool
    terminated: bool
    observation_valid: bool = True
    same_episode: bool = True
    truncated: bool = False

    def __post_init__(self):
        density = _finite(self.known_obstacle_bin_fraction, "known_obstacle_bin_fraction")
        applied = _finite(self.applied_v_max_mps, "applied_v_max_mps")
        previous = _finite(self.previous_applied_v_max_mps, "previous_applied_v_max_mps")
        actual = _finite(self.actual_speed_mps, "actual_speed_mps")
        tracking = _finite(self.tracking_error_m, "tracking_error_m")
        if density < 0.0 or density > 1.0:
            raise ValueError("known_obstacle_bin_fraction must be in [0, 1]")
        if min(applied, previous) <= 0.0 or min(actual, tracking) < 0.0:
            raise ValueError("speed/tracking inputs are outside their valid ranges")
        if self.nearest_obstacle_distance_m is not None:
            nearest = _finite(self.nearest_obstacle_distance_m, "nearest_obstacle_distance_m")
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


def reward_input_from_signals(
    *,
    nearest_obstacle_distance_m: Optional[float],
    known_obstacle_bin_fraction: float,
    unknown_bin_count: int,
    lidar_bin_count: int,
    applied_v_max_mps: float,
    previous_applied_v_max_mps: float,
    actual_speed_mps: float,
    tracking_error_m: float,
    dangerous_terminal: bool,
    terminated: bool,
    observation_valid: bool = True,
    same_episode: bool = True,
    truncated: bool = False,
) -> LearningSpeedRewardInput:
    """Build the reviewed input identically for online and offline evaluation."""

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
    return LearningSpeedRewardInput(
        nearest_obstacle_distance_m=nearest_obstacle_distance_m,
        known_obstacle_bin_fraction=density,
        unknown_majority=unknown_majority,
        applied_v_max_mps=applied_v_max_mps,
        previous_applied_v_max_mps=previous_applied_v_max_mps,
        actual_speed_mps=actual_speed_mps,
        tracking_error_m=tracking_error_m,
        dangerous_terminal=dangerous_terminal,
        terminated=terminated,
        observation_valid=observation_valid,
        same_episode=same_episode,
        truncated=truncated,
    )


def _vector_norm(values, name: str) -> float:
    components = tuple(float(component) for component in values)
    if len(components) != 3 or not all(math.isfinite(value) for value in components):
        raise ValueError("{} must contain three finite components".format(name))
    return math.sqrt(sum(component * component for component in components))


def actual_speed_mps_from_body_velocity(actual_velocity_body) -> float:
    """Shared norm for the existing Observation C actual-velocity source."""

    return _vector_norm(actual_velocity_body, "actual_velocity_body")


def tracking_error_m_from_body_error(tracking_error_body) -> float:
    """Shared norm for the existing Observation C tracking-error source."""

    return _vector_norm(tracking_error_body, "tracking_error_body")


@dataclass(frozen=True)
class ComplexityContext:
    label: str
    nearest_obstacle_distance_m: Optional[float]
    known_obstacle_bin_fraction: float
    unknown_majority: bool
    nearest_risk: float
    density_risk: float
    fusion_method: str
    nearest_weight: float
    density_weight: float
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
            "fusion_method": self.fusion_method,
            "nearest_weight": self.nearest_weight,
            "density_weight": self.density_weight,
            "safe_weight": safe,
            "middle_weight": middle,
            "dangerous_weight": dangerous,
        }


@dataclass(frozen=True)
class RewardEvaluation:
    reward_total: Optional[float]
    reward_speed: Optional[float]
    reward_smoothing: Optional[float]
    reward_error: Optional[float]
    reward_danger: Optional[float]
    phi_1: Optional[float]
    phi_2: Optional[float]
    complexity_context: Optional[ComplexityContext]
    reward_valid: bool
    invalid_reason: str = ""
    mode: str = STAGE_1_REWARD_MODE
    version: str = REWARD_VERSION

    def __post_init__(self):
        values = (
            self.reward_total,
            self.reward_speed,
            self.reward_smoothing,
            self.reward_error,
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
            "mode": self.mode,
            "reward_valid": self.reward_valid,
            "invalid_reason": self.invalid_reason,
            "reward_total": self.reward_total,
            "reward_speed": self.reward_speed,
            "reward_smoothing": self.reward_smoothing,
            "reward_error": self.reward_error,
            "reward_danger": self.reward_danger,
            "phi_1": self.phi_1,
            "phi_2": self.phi_2,
            "complexity_context": (
                None if self.complexity_context is None else self.complexity_context.to_record()
            ),
        }


class LearningSpeedReward:
    """Single owner for paper-guided Stage 1 and candidate Stage 2 rewards."""

    def __init__(self, config: Optional[LearningSpeedRewardConfig] = None):
        self.config = config or LearningSpeedRewardConfig()

    def _branch_weights(self, phi_2: float) -> Tuple[float, float, float]:
        """Quadratic Bernstein blend with continuous weights and speed slope."""

        risk = _clip01(phi_2)
        safe = (1.0 - risk) ** 2
        middle = 2.0 * risk * (1.0 - risk)
        dangerous = risk ** 2
        return (safe, middle, dangerous)

    def _complexity(self, value: LearningSpeedRewardInput):
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
            phi_2 = 1.0 - (
                (1.0 - nearest_risk) ** config.nearest_weight
                * (1.0 - density_risk) ** config.density_weight
            )
            phi_1 = config.low_anchor_mps + phi_2 * (
                config.high_anchor_mps - config.low_anchor_mps
            )
            if (nearest is None or nearest >= config.nearest_safe_m) and density <= config.density_safe:
                label = "Low"
            elif (nearest is not None and nearest <= config.nearest_dangerous_m) or density >= config.density_dangerous:
                label = "High"
            else:
                label = "Medium"
        weights = self._branch_weights(phi_2)
        context = ComplexityContext(
            label=label,
            nearest_obstacle_distance_m=nearest,
            known_obstacle_bin_fraction=density,
            unknown_majority=value.unknown_majority,
            nearest_risk=nearest_risk,
            density_risk=density_risk,
            fusion_method="weighted_geometric_survival",
            nearest_weight=config.nearest_weight,
            density_weight=config.density_weight,
            branch_weights=weights,
        )
        return phi_1, phi_2, context

    def _speed_reward(
        self,
        phi_1: float,
        weights: Tuple[float, float, float],
        actual_speed_mps: float,
    ) -> float:
        if self.config.mode == STAGE_2_REWARD_MODE:
            return self.config.lambda_speed_3 * actual_speed_mps
        safe_weight, middle_weight, dangerous_weight = weights
        dangerous_branch = self.config.lambda_speed_1 * (
            phi_1 - actual_speed_mps
        )
        safe_branch = self.config.lambda_speed_2 * (
            actual_speed_mps - phi_1
        )
        middle_branch = self.config.lambda_speed_3 * actual_speed_mps
        return (
            dangerous_weight * dangerous_branch
            + safe_weight * safe_branch
            + middle_weight * middle_branch
        )

    def evaluate(self, value: LearningSpeedRewardInput) -> RewardEvaluation:
        if not value.observation_valid:
            return self._invalid("invalid_observation")
        if not value.same_episode:
            return self._invalid("episode_boundary")
        if value.truncated:
            return self._invalid("truncated_transition")
        phi_1, phi_2, context = self._complexity(value)
        reward_speed = self._speed_reward(
            phi_1, context.branch_weights, value.actual_speed_mps
        )
        delta_constraint = value.applied_v_max_mps - value.previous_applied_v_max_mps
        reward_smoothing = -self.config.lambda_smoothing * delta_constraint ** 2
        clipped_error = min(value.tracking_error_m, self.config.error_clip_max_m)
        reward_error = -self.config.lambda_error * clipped_error ** 2
        reward_danger = (
            -self.config.lambda_danger * value.actual_speed_mps ** 2
            if value.dangerous_terminal
            else 0.0
        )
        reward_total = reward_speed + reward_smoothing + reward_error + reward_danger
        return RewardEvaluation(
            reward_total=reward_total,
            reward_speed=reward_speed,
            reward_smoothing=reward_smoothing,
            reward_error=reward_error,
            reward_danger=reward_danger,
            phi_1=phi_1,
            phi_2=phi_2,
            complexity_context=context,
            reward_valid=True,
            mode=self.config.mode,
            version=self.config.version,
        )

    def _invalid(self, reason: str) -> RewardEvaluation:
        return RewardEvaluation(
            reward_total=None,
            reward_speed=None,
            reward_smoothing=None,
            reward_error=None,
            reward_danger=None,
            phi_1=None,
            phi_2=None,
            complexity_context=None,
            reward_valid=False,
            invalid_reason=reason,
            mode=self.config.mode,
            version=self.config.version,
        )


def reward_from_config(values: Mapping[str, object]) -> LearningSpeedReward:
    return LearningSpeedReward(config=LearningSpeedRewardConfig.from_mapping(values))
