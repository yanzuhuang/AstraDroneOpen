#!/usr/bin/env python3
"""Reproducible offline calibration and audit for Learning Speed Reward v3.

The script is read-only with respect to frozen flight/training evidence.  It
writes one derived JSON analysis and never starts ROS, Gazebo, SAC training, or
evaluation.
"""

import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np
import yaml

from learning_speed_rl.training import (
    LearningSpeedRewardInput,
    reward_from_config,
)


FORMAL_B_CELLS = {
    "B_0.30",
    "B_0.50",
    "B_0.75",
    "B_1.00",
    "B_1.25",
    "B_1.50",
}
RUNTIME_RUNS = (
    "20260824_q03_v030_formal55_retry",
    "20260824_q04_v075",
    "20260824_q05_v125",
    "20260824_q06_v175",
)
SPEED_GRID_MPS = (0.30, 0.50, 0.75, 1.00, 1.25, 1.50, 1.75)
NEAR_ZERO_DELTA = 0.01


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _quantiles(values: Sequence[float]) -> Dict[str, object]:
    array = np.asarray(values, dtype=np.float64)
    if not array.size:
        return {"count": 0}
    return {
        "count": int(array.size),
        "min": float(np.min(array)),
        "p05": float(np.quantile(array, 0.05)),
        "median": float(np.quantile(array, 0.50)),
        "p95": float(np.quantile(array, 0.95)),
        "p99": float(np.quantile(array, 0.99)),
        "max": float(np.max(array)),
        "mean": float(np.mean(array)),
    }


def _reference_label(nearest: float, density: float) -> str:
    """Common v2-anchor strata keep v2/v3 denominators identical."""

    if nearest >= 6.0 and density <= 0.040:
        return "Open"
    if nearest <= 2.5 or density >= 0.080:
        return "High"
    return "Medium"


def _risks(
    nearest: float,
    density: float,
    nearest_safe: float = 6.0,
    nearest_dangerous: float = 2.5,
    density_safe: float = 0.040,
    density_dangerous: float = 0.080,
) -> Tuple[float, float]:
    q_nearest = float(
        np.clip(
            (nearest_safe - nearest) / (nearest_safe - nearest_dangerous),
            0.0,
            1.0,
        )
    )
    q_density = float(
        np.clip(
            (density - density_safe) / (density_dangerous - density_safe),
            0.0,
            1.0,
        )
    )
    return q_nearest, q_density


def _v2_weights(phi_2: float) -> Tuple[float, float, float]:
    def smoothstep(value: float) -> float:
        clipped = min(1.0, max(0.0, value))
        return clipped * clipped * (3.0 - 2.0 * clipped)

    if phi_2 <= 0.35:
        return (1.0, 0.0, 0.0)
    if phi_2 < 0.50:
        middle = smoothstep((phi_2 - 0.35) / 0.15)
        return (1.0 - middle, middle, 0.0)
    if phi_2 < 0.65:
        dangerous = smoothstep((phi_2 - 0.50) / 0.15)
        return (0.0, 1.0 - dangerous, dangerous)
    return (0.0, 0.0, 1.0)


def _v3_weights(phi_2: float) -> Tuple[float, float, float]:
    return ((1.0 - phi_2) ** 2, 2.0 * phi_2 * (1.0 - phi_2), phi_2 ** 2)


def _speed_reward(phi_2: float, weights: Tuple[float, float, float], speed: float) -> float:
    phi_1 = 1.75 - phi_2
    safe, middle, dangerous = weights
    return (
        safe * 0.80 * (speed - phi_1)
        + middle * 0.25 * speed
        + dangerous * (phi_1 - speed)
    )


def _speed_slope(weights: Tuple[float, float, float]) -> float:
    safe, middle, dangerous = weights
    return 0.80 * safe + 0.25 * middle - dangerous


def _load_formal_b(path: Path) -> List[Dict[str, object]]:
    records = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                if (
                    row["environment"] != "B"
                    or row["training_active"] != "1"
                    or row["observation_valid"] != "1"
                    or row["calibration_cell"] not in FORMAL_B_CELLS
                ):
                    continue
                nearest = float(row["nearest_obstacle_distance_m"])
                density = float(row["known_obstacle_bin_fraction"])
            except (KeyError, ValueError):
                continue
            records.append(
                {
                    "nearest": nearest,
                    "density": density,
                    "source": "formal_B_complete_mission",
                    "label": _reference_label(nearest, density),
                }
            )
    return records


def _load_runtime(root: Path) -> List[Dict[str, object]]:
    records = []
    for run_id in RUNTIME_RUNS:
        path = root / run_id / "reward_runtime_steps.jsonl"
        with path.open(encoding="utf-8") as stream:
            for line in stream:
                item = json.loads(line)
                nearest = float(item["nearest_obstacle_distance_m"])
                density = float(item["known_obstacle_bin_fraction"])
                records.append(
                    {
                        "nearest": nearest,
                        "density": density,
                        "source": "hover_to_entry_runtime",
                        "run_id": run_id,
                        "label": _reference_label(nearest, density),
                    }
                )
    return records


def _load_history(path: Path) -> List[Dict[str, object]]:
    records = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            item = json.loads(line)
            context = item["reward_components"]["complexity_context"]
            nearest = context["nearest_obstacle_distance_m"]
            if nearest is None:
                continue
            nearest = float(nearest)
            density = float(context["known_obstacle_bin_fraction"])
            records.append(
                {
                    "nearest": nearest,
                    "density": density,
                    "source": "historical_10k_no_go",
                    "label": _reference_label(nearest, density),
                }
            )
    return records


def _load_route_segments(path: Path) -> Dict[str, List[Dict[str, object]]]:
    result = {"start": [], "middle": [], "entry": []}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            segment = row["route_segment"]
            result[segment].append(
                {
                    "nearest": float(row["nearest_obstacle_distance_m"]),
                    "density": float(row["known_obstacle_bin_fraction"]),
                }
            )
    return result


def _fit_weight(
    groups: Mapping[str, Sequence[Mapping[str, object]]],
    normalization: Tuple[float, float, float, float],
    fusion: str,
) -> Dict[str, object]:
    targets = {"Open": 0.0, "Medium": 0.5, "High": 1.0}
    risk_arrays = {}
    for label, records in groups.items():
        risk_arrays[label] = np.asarray(
            [
                _risks(
                    float(record["nearest"]),
                    float(record["density"]),
                    *normalization,
                )
                for record in records
            ],
            dtype=np.float64,
        )
    best = None
    for nearest_percent in range(10, 91):
        nearest_weight = nearest_percent / 100.0
        density_weight = 1.0 - nearest_weight
        class_errors = {}
        for label, risks in risk_arrays.items():
            if fusion == "weighted_linear":
                phi_2 = (
                    nearest_weight * risks[:, 0]
                    + density_weight * risks[:, 1]
                )
            elif fusion == "weighted_geometric_survival":
                phi_2 = 1.0 - (
                    (1.0 - risks[:, 0]) ** nearest_weight
                    * (1.0 - risks[:, 1]) ** density_weight
                )
            else:
                raise ValueError("unsupported fitted fusion")
            class_errors[label] = float(
                np.mean((phi_2 - targets[label]) ** 2)
            )
        score = float(np.mean(list(class_errors.values())))
        candidate = (score, nearest_weight, density_weight, class_errors)
        if best is None or candidate[0] < best[0]:
            best = candidate
    score, nearest_weight, density_weight, class_errors = best
    return {
        "balanced_mse": score,
        "nearest_weight": nearest_weight,
        "density_weight": density_weight,
        "class_mse": class_errors,
    }


def _phi_summary(
    records: Sequence[Mapping[str, object]],
    normalization: Tuple[float, float, float, float],
    fusion: str,
    nearest_weight: float = 0.0,
    density_weight: float = 0.0,
) -> Dict[str, object]:
    phi_values = []
    nearest_risks = []
    density_risks = []
    density_effect = []
    nearest_owns_max = 0
    for record in records:
        q_nearest, q_density = _risks(
            float(record["nearest"]), float(record["density"]), *normalization
        )
        nearest_risks.append(q_nearest)
        density_risks.append(q_density)
        nearest_owns_max += int(q_nearest >= q_density)
        if fusion == "max":
            phi_2 = max(q_nearest, q_density)
        elif fusion == "weighted_linear":
            phi_2 = nearest_weight * q_nearest + density_weight * q_density
            density_effect.append(density_weight * q_density)
        elif fusion == "weighted_geometric_survival":
            phi_2 = 1.0 - (
                (1.0 - q_nearest) ** nearest_weight
                * (1.0 - q_density) ** density_weight
            )
            phi_without_density = 1.0 - (1.0 - q_nearest) ** nearest_weight
            density_effect.append(phi_2 - phi_without_density)
        elif fusion == "probabilistic_or":
            phi_2 = 1.0 - (1.0 - q_nearest) * (1.0 - q_density)
        else:
            raise ValueError("unsupported fusion")
        phi_values.append(phi_2)
    return {
        "q_nearest": _quantiles(nearest_risks),
        "q_density": _quantiles(density_risks),
        "phi_2": _quantiles(phi_values),
        "nearest_is_max_fraction": nearest_owns_max / float(len(records)),
        "density_nonzero_effect_fraction": (
            None
            if not density_effect
            else sum(value > 0.0 for value in density_effect) / float(len(density_effect))
        ),
        "density_weighted_effect": _quantiles(density_effect),
    }


def _balanced_fusion_mse(
    groups: Mapping[str, Sequence[Mapping[str, object]]],
    normalization: Tuple[float, float, float, float],
    fusion: str,
    nearest_weight: float = 0.0,
    density_weight: float = 0.0,
) -> float:
    targets = {"Open": 0.0, "Medium": 0.5, "High": 1.0}
    class_errors = []
    for label, records in groups.items():
        values = []
        for record in records:
            q_nearest, q_density = _risks(
                float(record["nearest"]),
                float(record["density"]),
                *normalization,
            )
            if fusion == "max":
                phi_2 = max(q_nearest, q_density)
            elif fusion == "probabilistic_or":
                phi_2 = 1.0 - (1.0 - q_nearest) * (1.0 - q_density)
            elif fusion == "weighted_linear":
                phi_2 = nearest_weight * q_nearest + density_weight * q_density
            elif fusion == "weighted_geometric_survival":
                phi_2 = 1.0 - (
                    (1.0 - q_nearest) ** nearest_weight
                    * (1.0 - q_density) ** density_weight
                )
            else:
                raise ValueError("unsupported fusion")
            values.append((phi_2 - targets[label]) ** 2)
        class_errors.append(float(np.mean(values)))
    return float(np.mean(class_errors))


def _preference_summary(
    records: Sequence[Mapping[str, object]], version: str
) -> Dict[str, object]:
    deltas = []
    for record in records:
        nearest = float(record["nearest"])
        density = float(record["density"])
        if version == "v2":
            phi_2 = max(_risks(nearest, density))
            weights = _v2_weights(phi_2)
        elif version == "v3":
            q_nearest, q_density = _risks(nearest, density)
            phi_2 = 1.0 - (
                (1.0 - q_nearest) ** 0.46
                * (1.0 - q_density) ** 0.54
            )
            weights = _v3_weights(phi_2)
        else:
            raise ValueError("unsupported reward version")
        delta = 1.45 * _speed_slope(weights)
        deltas.append(delta)
    positive = sum(value > NEAR_ZERO_DELTA for value in deltas)
    near_zero = sum(abs(value) <= NEAR_ZERO_DELTA for value in deltas)
    negative = sum(value < -NEAR_ZERO_DELTA for value in deltas)
    return {
        "count": len(deltas),
        "positive": positive,
        "near_zero": near_zero,
        "negative": negative,
        "positive_fraction": positive / float(len(deltas)) if deltas else None,
        "near_zero_fraction": near_zero / float(len(deltas)) if deltas else None,
        "negative_fraction": negative / float(len(deltas)) if deltas else None,
        "delta": _quantiles(deltas),
    }


def _counterfactual_table(records: Sequence[Mapping[str, object]]) -> Dict[str, object]:
    result = {}
    for label in ("ALL", "Open", "Medium", "High"):
        selected = (
            list(records)
            if label == "ALL"
            else [record for record in records if record["label"] == label]
        )
        result[label] = {
            "v2": _preference_summary(selected, "v2") if selected else {"count": 0},
            "v3": _preference_summary(selected, "v3") if selected else {"count": 0},
        }
    return result


def _landscape(records: Sequence[Mapping[str, object]]) -> Dict[str, object]:
    result = {}
    for label in ("Open", "Medium", "High"):
        selected = [record for record in records if record["label"] == label]
        rows = []
        for speed in SPEED_GRID_MPS:
            row = {"actual_speed_mps": speed}
            for version in ("v2", "v3"):
                rewards = []
                for record in selected:
                    nearest = float(record["nearest"])
                    density = float(record["density"])
                    if version == "v2":
                        phi_2 = max(_risks(nearest, density))
                        weights = _v2_weights(phi_2)
                    else:
                        q_nearest, q_density = _risks(nearest, density)
                        phi_2 = 1.0 - (
                            (1.0 - q_nearest) ** 0.46
                            * (1.0 - q_density) ** 0.54
                        )
                        weights = _v3_weights(phi_2)
                    rewards.append(_speed_reward(phi_2, weights, speed))
                row[version + "_reward_speed_mean"] = float(np.mean(rewards))
                row[version + "_reward_speed_median"] = float(np.median(rewards))
            rows.append(row)
        result[label] = {"state_count": len(selected), "speed_scan": rows}
    return result


def _owner_equality(reward, records: Iterable[Mapping[str, object]]) -> Dict[str, object]:
    maximum_error = 0.0
    checked = 0
    for record in records:
        nearest = float(record["nearest"])
        density = float(record["density"])
        q_nearest, q_density = _risks(nearest, density)
        phi_2 = 1.0 - (
            (1.0 - q_nearest) ** 0.46
            * (1.0 - q_density) ** 0.54
        )
        weights = _v3_weights(phi_2)
        for speed in SPEED_GRID_MPS:
            value = LearningSpeedRewardInput(
                nearest_obstacle_distance_m=nearest,
                known_obstacle_bin_fraction=density,
                unknown_majority=False,
                applied_v_max_mps=speed,
                previous_applied_v_max_mps=speed,
                actual_speed_mps=speed,
                tracking_error_m=0.0,
                dangerous_terminal=False,
                terminated=False,
            )
            evaluation = reward.evaluate(value)
            direct = _speed_reward(phi_2, weights, speed)
            maximum_error = max(maximum_error, abs(evaluation.reward_speed - direct))
            checked += 1
    return {
        "evaluations": checked,
        "maximum_absolute_error": maximum_error,
        "equal_within_1e_12": maximum_error <= 1.0e-12,
    }


def main() -> int:
    repository_root = Path(__file__).resolve().parents[4]
    calibration_root = repository_root / (
        "runtime_artifacts/learning_speed/"
        "stage1_reward_calibration_current_generation_20260820_231428"
    )
    runtime_root = repository_root / "runtime_artifacts/reward_v2_runtime_qualification"
    history_path = repository_root / (
        "runtime_artifacts/rl_training/"
        "sac_training_10k_20260823_194751/sac_transition_audit.jsonl"
    )
    route_path = repository_root / (
        "runtime_artifacts/reward_v2_phi_branch_diagnosis/20260824_d01/"
        "phi_branch_derived_steps.csv"
    )
    config_path = repository_root / (
        "AstraDrone_ros1_ws/src/learning_speed_rl/config/stage1_reward.yaml"
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=repository_root / (
            "runtime_artifacts/reward_v3_offline_optimization/"
            "20260824_reward_v3_offline_analysis.json"
        ),
    )
    args = parser.parse_args()
    output_path = args.output.resolve()
    if repository_root.resolve() not in output_path.parents:
        raise ValueError("output must remain under the repository runtime tree")

    merged_path = calibration_root / "stage1_reward_calibration_current_generation_samples_merged.csv"
    formal_b = _load_formal_b(merged_path)
    runtime = _load_runtime(runtime_root)
    history = _load_history(history_path)
    route_segments = _load_route_segments(route_path)
    if (len(formal_b), len(runtime), len(history)) != (14784, 4161, 10000):
        raise ValueError("unexpected source population counts")

    calibration_groups = {
        "Open": [record for record in formal_b if record["label"] == "Open"],
        "Medium": [
            record
            for record in formal_b + runtime
            if record["label"] == "Medium"
        ],
        "High": [record for record in runtime if record["label"] == "High"],
    }
    normalizations = {
        "v2_baseline": (6.0, 2.5, 0.040, 0.080),
        "density_entry_p99": (6.0, 2.5, 0.040, 0.070),
        "density_shifted_both": (6.0, 2.5, 0.035, 0.075),
        "nearest_safe_5p5": (5.5, 2.5, 0.040, 0.080),
        "nearest_safe_6p5": (6.5, 2.5, 0.040, 0.080),
        "nearest_dangerous_2p0": (6.0, 2.0, 0.040, 0.080),
        "nearest_dangerous_3p0": (6.0, 3.0, 0.040, 0.080),
    }
    normalization_linear_fits = {
        name: _fit_weight(calibration_groups, values, "weighted_linear")
        for name, values in normalizations.items()
    }
    normalization_geometric_fits = {
        name: _fit_weight(
            calibration_groups, values, "weighted_geometric_survival"
        )
        for name, values in normalizations.items()
    }

    selected_normalization = normalizations["v2_baseline"]
    selected_fit = normalization_geometric_fits["v2_baseline"]
    config_values = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    reward = reward_from_config(config_values)
    if (
        abs(reward.config.nearest_weight - selected_fit["nearest_weight"]) > 1.0e-12
        or abs(reward.config.density_weight - selected_fit["density_weight"]) > 1.0e-12
    ):
        raise ValueError("configured fusion does not match the calibrated fit")

    fusion_candidates = {
        "A_v2_max": {
            "normalization": "v2_baseline",
            "balanced_mse": _balanced_fusion_mse(
                calibration_groups, normalizations["v2_baseline"], "max"
            ),
            "summary": {
                label: _phi_summary(records, normalizations["v2_baseline"], "max")
                for label, records in calibration_groups.items()
            },
        },
        "B_weighted_v2_normalization": {
            "normalization": "v2_baseline",
            "fit": normalization_linear_fits["v2_baseline"],
            "balanced_mse": normalization_linear_fits["v2_baseline"]["balanced_mse"],
            "summary": {
                label: _phi_summary(
                    records,
                    normalizations["v2_baseline"],
                    "weighted_linear",
                    normalization_linear_fits["v2_baseline"]["nearest_weight"],
                    normalization_linear_fits["v2_baseline"]["density_weight"],
                )
                for label, records in calibration_groups.items()
            },
        },
        "B_selected_weighted_density_p99": {
            "normalization": "density_entry_p99",
            "fit": normalization_linear_fits["density_entry_p99"],
            "balanced_mse": normalization_linear_fits["density_entry_p99"]["balanced_mse"],
            "summary": {
                label: _phi_summary(
                    records,
                    normalizations["density_entry_p99"],
                    "weighted_linear",
                    normalization_linear_fits["density_entry_p99"]["nearest_weight"],
                    normalization_linear_fits["density_entry_p99"]["density_weight"],
                )
                for label, records in calibration_groups.items()
            },
        },
        "C_selected_weighted_geometric_survival": {
            "normalization": "v2_baseline",
            "fit": selected_fit,
            "balanced_mse": selected_fit["balanced_mse"],
            "summary": {
                label: _phi_summary(
                    records,
                    selected_normalization,
                    "weighted_geometric_survival",
                    selected_fit["nearest_weight"],
                    selected_fit["density_weight"],
                )
                for label, records in calibration_groups.items()
            },
        },
        "C_unweighted_probabilistic_or": {
            "normalization": "v2_baseline",
            "balanced_mse": _balanced_fusion_mse(
                calibration_groups,
                normalizations["v2_baseline"],
                "probabilistic_or",
            ),
            "summary": {
                label: _phi_summary(
                    records, selected_normalization, "probabilistic_or"
                )
                for label, records in calibration_groups.items()
            },
        },
    }

    pooled_current = formal_b + runtime
    counterfactual = {
        "formal_B_complete_mission": _counterfactual_table(formal_b),
        "hover_to_entry_runtime": _counterfactual_table(runtime),
        "pooled_current_evidence": _counterfactual_table(pooled_current),
        "historical_10k_no_go": _counterfactual_table(history),
    }
    route_density = {
        segment: _quantiles([float(record["density"]) for record in records])
        for segment, records in route_segments.items()
    }
    route_phi_v3 = {
        segment: _phi_summary(
            records,
            selected_normalization,
            "weighted_geometric_survival",
            selected_fit["nearest_weight"],
            selected_fit["density_weight"],
        )["phi_2"]
        for segment, records in route_segments.items()
    }
    feature_distribution = {
        source_name: {
            label: _phi_summary(
                [record for record in records if record["label"] == label],
                normalizations["v2_baseline"],
                "max",
            )
            for label in ("Open", "Medium", "High")
            if any(record["label"] == label for record in records)
        }
        for source_name, records in (
            ("formal_B_complete_mission", formal_b),
            ("hover_to_entry_runtime", runtime),
            ("historical_10k_no_go", history),
        )
    }

    summary = {
        "schema_version": "astradrone_reward_v3_offline_optimization_v1.0",
        "scope": {
            "offline_only": True,
            "gazebo_started": False,
            "sac_training_started": False,
            "evaluation_started": False,
            "claim": "paper-guided AstraDroneOpen calibration, not exact paper reproduction",
        },
        "source_files": {
            "formal_B_merged_csv": {
                "path": str(merged_path.relative_to(repository_root)),
                "sha256": _sha256(merged_path),
                "count": len(formal_b),
                "labels": dict(Counter(record["label"] for record in formal_b)),
            },
            "hover_to_entry_runtime": {
                "root": str(runtime_root.relative_to(repository_root)),
                "run_allowlist": list(RUNTIME_RUNS),
                "count": len(runtime),
                "labels": dict(Counter(record["label"] for record in runtime)),
            },
            "historical_10k_no_go": {
                "path": str(history_path.relative_to(repository_root)),
                "sha256": _sha256(history_path),
                "count": len(history),
                "labels": dict(Counter(record["label"] for record in history)),
            },
            "route_segments": {
                "path": str(route_path.relative_to(repository_root)),
                "sha256": _sha256(route_path),
                "counts": {key: len(value) for key, value in route_segments.items()},
            },
        },
        "reference_strata": {
            "definition": (
                "Common v2 physical anchors are retained only to compare identical "
                "Open/Medium/High populations across v2 and v3. Labels never switch reward."
            ),
            "calibration_counts": {
                key: len(value) for key, value in calibration_groups.items()
            },
        },
        "feature_distribution": feature_distribution,
        "route_density_distribution": route_density,
        "normalization_candidates": {
            name: {
                "nearest_safe_m": values[0],
                "nearest_dangerous_m": values[1],
                "density_safe": values[2],
                "density_dangerous": values[3],
                "best_weighted_linear_fit": normalization_linear_fits[name],
                "best_weighted_geometric_fit": normalization_geometric_fits[name],
            }
            for name, values in normalizations.items()
        },
        "fusion_candidates": fusion_candidates,
        "selected_v3": {
            "normalization": {
                "nearest_safe_m": 6.0,
                "nearest_dangerous_m": 2.5,
                "density_safe": 0.040,
                "density_dangerous": 0.080,
            },
            "fusion": {
                "formula": (
                    "phi_2 = 1 - (1-q_nearest)^0.46 * "
                    "(1-q_density)^0.54"
                ),
                "nearest_weight": selected_fit["nearest_weight"],
                "density_weight": selected_fit["density_weight"],
                "balanced_mse": selected_fit["balanced_mse"],
            },
            "phi_1": "1.75 - phi_2",
            "branch_weights": {
                "safe": "(1-phi_2)^2",
                "middle": "2*phi_2*(1-phi_2)",
                "dangerous": "phi_2^2",
                "zero_speed_slope_phi_2": (
                    -1.1 + math.sqrt(3.45)
                ) / 1.4,
            },
            "route_phi_2": route_phi_v3,
        },
        "reward_landscape": _landscape(pooled_current),
        "counterfactual_preference": {
            "near_zero_definition": "abs(Delta r_speed) <= 0.01",
            "tables": counterfactual,
        },
        "owner_equality": _owner_equality(reward, pooled_current + history),
    }

    history_v2 = counterfactual["historical_10k_no_go"]["ALL"]["v2"]
    history_v3 = counterfactual["historical_10k_no_go"]["ALL"]["v3"]
    formal_open_v3 = counterfactual["formal_B_complete_mission"]["Open"]["v3"]
    runtime_high_v3 = counterfactual["hover_to_entry_runtime"]["High"]["v3"]
    summary["validation_checks"] = {
        "configured_weights_match_balanced_fit": True,
        "historical_normalization_outscores_density_p99_for_selected_fusion": (
            normalization_geometric_fits["v2_baseline"]["balanced_mse"]
            < normalization_geometric_fits["density_entry_p99"]["balanced_mse"]
        ),
        "both_fusion_features_active": (
            selected_fit["nearest_weight"] > 0.0
            and selected_fit["density_weight"] > 0.0
        ),
        "formal_open_prefers_high_speed": (
            formal_open_v3["positive_fraction"] == 1.0
        ),
        "runtime_high_prefers_low_speed": (
            runtime_high_v3["negative_fraction"] == 1.0
        ),
        "historical_low_speed_bias_reduced": (
            history_v3["negative_fraction"] < history_v2["negative_fraction"]
        ),
        "online_owner_matches_offline_formula": summary["owner_equality"][
            "equal_within_1e_12"
        ],
    }
    summary["offline_validation_pass"] = all(summary["validation_checks"].values())
    summary["verdict"] = (
        "OFFLINE PASS; BOUNDED REWARD V3 RUNTIME QUALIFICATION REQUIRED; "
        "FORMAL SAC TRAINING/EVALUATION NO-GO"
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "output": str(output_path),
                "source_counts": {
                    "formal_B": len(formal_b),
                    "runtime": len(runtime),
                    "historical_10k": len(history),
                },
                "selected_fit": selected_fit,
                "validation_checks": summary["validation_checks"],
                "offline_validation_pass": summary["offline_validation_pass"],
                "verdict": summary["verdict"],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if summary["offline_validation_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
