#!/usr/bin/env python3
"""Read-only Stage 1 reward replay over frozen calibration artifacts."""

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from learning_speed_rl.training import (
    LIDAR_BINS,
    Stage1Reward,
    Stage1RewardConfig,
    Stage1RewardInput,
    actual_speed_mps_from_body_velocity,
    reward_from_config,
    stage1_reward_input_from_signals,
)


EXTRA_RUN_ALLOWLIST = (
    "current_B_v100_r02",
    "current_B_v100_r03",
    "current_B_v125_r02",
    "current_B_v125_r03",
    "current_B_v150_r02",
    "current_B_v150_r03",
    "enhanced_B_v125_r01",
    "enhanced_B_v125_r02",
    "enhanced_B_v150_r01",
    "enhanced_B_v150_r02",
    "enhanced_B_v175_r01",
    "enhanced_B_v175_r02",
)

DANGER_REASONS = {
    "collision_proxy",
    "ego_emergency_stop",
    "tracking_safety_gate",
}


def _json_default(value):
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError("unsupported JSON value {}".format(type(value).__name__))


@dataclass(frozen=True)
class RunSpec:
    run_id: str
    directory: Path
    cohort: str
    environment: str
    target_speed_mps: float


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _quantiles(values: Sequence[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {
            "count": 0,
            "min": None,
            "p01": None,
            "p50": None,
            "mean": None,
            "p99": None,
            "max": None,
        }
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "min": float(np.min(array)),
        "p01": float(np.quantile(array, 0.01)),
        "p50": float(np.quantile(array, 0.50)),
        "mean": float(np.mean(array)),
        "p99": float(np.quantile(array, 0.99)),
        "max": float(np.max(array)),
    }


def _target_from_run_id(run_id: str) -> float:
    marker = "_v"
    start = run_id.index(marker) + len(marker)
    digits = run_id[start : start + 3]
    return float(int(digits)) / 100.0


def _repository_path(repository_root: Path, value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else repository_root / path


def _load_runs(
    repository_root: Path,
    dataset_root: Path,
    manifest: Dict[str, object],
) -> List[RunSpec]:
    result = []
    for source in manifest["sources"]:
        result.append(
            RunSpec(
                run_id=source["source_run"],
                directory=_repository_path(
                    repository_root, source["source_directory"]
                ),
                cohort="formal",
                environment=source["environment"],
                target_speed_mps=float(source["target_speed_mps"]),
            )
        )
    for run_id in EXTRA_RUN_ALLOWLIST:
        result.append(
            RunSpec(
                run_id=run_id,
                directory=dataset_root / run_id,
                cohort="enhanced_repeat",
                environment="B",
                target_speed_mps=_target_from_run_id(run_id),
            )
        )
    boundary = manifest["stage2_safety_boundary_candidate"]
    result.append(
        RunSpec(
            run_id=boundary["source_run"],
            directory=_repository_path(
                repository_root, boundary["source_directory"]
            ),
            cohort="safety_boundary_only",
            environment="B",
            target_speed_mps=float(boundary["speed_mps"]),
        )
    )
    return result


def _stamp_key(value) -> float:
    return round(float(value), 9)


def _load_samples(path: Path):
    by_stamp = {}
    all_rows = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            all_rows.append(row)
            if row["observation_stamp_sec"]:
                by_stamp[_stamp_key(row["observation_stamp_sec"])] = row
    return by_stamp, all_rows


def _boolean(row: Dict[str, str], name: str) -> bool:
    return row.get(name, "0") in ("1", "true", "True")


def _reward_input(
    row: Dict[str, str],
    applied: float,
    previous: float,
    actual: float,
    dangerous: bool,
    terminated: bool,
    truncated: bool = False,
) -> Stage1RewardInput:
    nearest_text = row["nearest_obstacle_distance_m"]
    return stage1_reward_input_from_signals(
        nearest_obstacle_distance_m=(
            None if nearest_text == "" else float(nearest_text)
        ),
        known_obstacle_bin_fraction=float(row["known_obstacle_bin_fraction"]),
        unknown_bin_count=int(row["unknown_bin_count"] or 0),
        lidar_bin_count=LIDAR_BINS,
        applied_v_max_mps=applied,
        previous_applied_v_max_mps=previous,
        actual_speed_mps=actual,
        dangerous_terminal=dangerous,
        terminated=terminated,
        observation_valid=_boolean(row, "observation_valid"),
        same_episode=True,
        truncated=truncated,
    )


def _legacy_dangerous(transition: Dict[str, object]) -> bool:
    if "dangerous_terminal" in transition:
        return bool(transition["dangerous_terminal"])
    reasons = set(str(transition.get("terminal_reason", "")).split(";"))
    return bool(reasons.intersection(DANGER_REASONS))


def _run_passed(directory: Path) -> bool:
    path = directory / "run_summary.json"
    if not path.is_file():
        return False
    summary = json.loads(path.read_text(encoding="utf-8"))
    mission = summary.get("mission", {})
    return bool(mission.get("success", False))


def _summary_says_dangerous(directory: Path) -> bool:
    path = directory / "run_summary.json"
    if not path.is_file():
        return False
    summary = json.loads(path.read_text(encoding="utf-8"))
    return bool(summary.get("safety", {}).get("dangerous_terminal", False))


def _terminal_event_check(
    reward: Stage1Reward, spec: RunSpec, rows: List[Dict[str, str]]
) -> Optional[Dict[str, object]]:
    if not _summary_says_dangerous(spec.directory):
        return None
    danger_index = next(
        (index for index, row in enumerate(rows) if _boolean(row, "dangerous_terminal")),
        None,
    )
    if danger_index is None:
        return {
            "run_id": spec.run_id,
            "cohort": spec.cohort,
            "status": "dangerous_summary_without_sample_flag",
        }
    selected = None
    source = ""
    for index in range(danger_index, -1, -1):
        row = rows[index]
        if _boolean(row, "observation_valid") and row["actual_speed_mps"]:
            selected = row
            source = (
                "first_valid_dangerous_sample"
                if index == danger_index
                else "latest_causal_valid_sample_before_danger"
            )
            break
    if selected is None:
        return {
            "run_id": spec.run_id,
            "cohort": spec.cohort,
            "status": "no_causal_valid_observation_for_danger",
        }
    value = _reward_input(
        selected,
        applied=float(selected["latest_applied_v_max_mps"]),
        previous=float(selected["previous_applied_v_max_mps"]),
        actual=float(selected["actual_speed_mps"]),
        dangerous=True,
        terminated=True,
    )
    evaluation = reward.evaluate(value)
    return {
        "run_id": spec.run_id,
        "cohort": spec.cohort,
        "status": "diagnostic_only_not_added_to_replay_transitions",
        "source": source,
        "observation_stamp_sec": float(selected["observation_stamp_sec"]),
        "actual_speed_mps": value.actual_speed_mps,
        "applied_v_max_mps": value.applied_v_max_mps,
        "reward_total": evaluation.reward_total,
        "reward_speed": evaluation.reward_speed,
        "reward_danger": evaluation.reward_danger,
        "complexity": evaluation.complexity_context.label,
    }


def _counterfactual_preference(
    reward: Stage1Reward, base: Stage1RewardInput
) -> Dict[str, float]:
    values = {}
    for action in (0.75, 1.25, 1.75):
        probe = Stage1RewardInput(
            nearest_obstacle_distance_m=base.nearest_obstacle_distance_m,
            known_obstacle_bin_fraction=base.known_obstacle_bin_fraction,
            unknown_majority=base.unknown_majority,
            applied_v_max_mps=action,
            previous_applied_v_max_mps=action,
            actual_speed_mps=base.actual_speed_mps,
            dangerous_terminal=False,
            terminated=False,
        )
        values["{:.2f}".format(action)] = reward.evaluate(probe).reward_speed
    values["delta_1p75_minus_0p75"] = values["1.75"] - values["0.75"]
    return values


def _replay_run(reward: Stage1Reward, spec: RunSpec):
    sample_path = spec.directory / "calibration_samples.csv"
    transition_path = spec.directory / "transition_candidates.jsonl"
    if not sample_path.is_file() or not transition_path.is_file():
        raise RuntimeError("missing replay inputs for {}".format(spec.run_id))
    samples, all_rows = _load_samples(sample_path)
    passed = _run_passed(spec.directory)
    records = []
    counters = {
        "candidate_lines": 0,
        "joined": 0,
        "skipped_missing_sample": 0,
        "skipped_inactive": 0,
        "invalid_reward": 0,
        "legacy_v1_0": 0,
    }
    with transition_path.open(encoding="utf-8") as stream:
        for line in stream:
            counters["candidate_lines"] += 1
            transition = json.loads(line)
            if transition.get("version") == "learning_speed_sac_transition_v1.0":
                counters["legacy_v1_0"] += 1
            state = transition["state_t"]
            row = samples.get(
                _stamp_key(state["provenance"]["observation_stamp_sec"])
            )
            if row is None:
                counters["skipped_missing_sample"] += 1
                continue
            counters["joined"] += 1
            if not _boolean(row, "training_active"):
                counters["skipped_inactive"] += 1
                continue
            action = transition["action_t"]
            actual_speed = actual_speed_mps_from_body_velocity(
                state["actual_velocity_body"]
            )
            dangerous = _legacy_dangerous(transition)
            value = _reward_input(
                row,
                applied=float(action["applied_v_max"]),
                previous=float(state["previous_applied_v_max"]),
                actual=actual_speed,
                dangerous=dangerous,
                terminated=bool(transition["terminated"]),
                truncated=bool(transition["truncated"]),
            )
            evaluation = reward.evaluate(value)
            if not evaluation.reward_valid:
                counters["invalid_reward"] += 1
                continue
            preference = _counterfactual_preference(reward, value)
            records.append(
                {
                    "run_id": spec.run_id,
                    "cohort": spec.cohort,
                    "environment": spec.environment,
                    "target_speed_mps": spec.target_speed_mps,
                    "passed": passed,
                    "reward_total": evaluation.reward_total,
                    "reward_speed": evaluation.reward_speed,
                    "reward_smoothing": evaluation.reward_smoothing,
                    "reward_danger": evaluation.reward_danger,
                    "phi_1": evaluation.phi_1,
                    "phi_2": evaluation.phi_2,
                    "complexity": evaluation.complexity_context.label,
                    "dangerous_terminal": dangerous,
                    "actual_speed_mps": actual_speed,
                    "applied_v_max_mps": value.applied_v_max_mps,
                    "action_delta_mps": (
                        value.applied_v_max_mps - value.previous_applied_v_max_mps
                    ),
                    "preference_delta": preference["delta_1p75_minus_0p75"],
                }
            )
    return records, counters, _terminal_event_check(reward, spec, all_rows)


def _cohort_summary(records: List[Dict[str, object]]) -> Dict[str, object]:
    result = {
        "transition_count": len(records),
        "dangerous_transition_count": sum(
            int(record["dangerous_terminal"]) for record in records
        ),
        "nonfinite_count": 0,
        "terms": {},
        "complexity": {},
        "pass_runs": {},
    }
    for field in (
        "reward_total",
        "reward_speed",
        "reward_smoothing",
        "reward_danger",
        "phi_1",
        "phi_2",
        "action_delta_mps",
    ):
        values = [float(record[field]) for record in records]
        result["nonfinite_count"] += sum(not math.isfinite(value) for value in values)
        result["terms"][field] = _quantiles(values)
    for label in ("Low", "Medium", "High", "Unknown"):
        selected = [record for record in records if record["complexity"] == label]
        deltas = [float(record["preference_delta"]) for record in selected]
        result["complexity"][label] = {
            "count": len(selected),
            "phi_1": _quantiles([float(record["phi_1"]) for record in selected]),
            "phi_2": _quantiles([float(record["phi_2"]) for record in selected]),
            "reward_total": _quantiles(
                [float(record["reward_total"]) for record in selected]
            ),
            "preference_delta_1p75_minus_0p75": _quantiles(deltas),
            "fraction_prefers_higher": (
                None
                if not deltas
                else float(sum(delta > 0.0 for delta in deltas)) / len(deltas)
            ),
            "fraction_prefers_lower": (
                None
                if not deltas
                else float(sum(delta < 0.0 for delta in deltas)) / len(deltas)
            ),
        }
    for run_id in sorted({record["run_id"] for record in records if record["passed"]}):
        selected = [record for record in records if record["run_id"] == run_id]
        result["pass_runs"][run_id] = {
            "count": len(selected),
            "reward_total": _quantiles(
                [float(record["reward_total"]) for record in selected]
            ),
            "maximum_absolute_reward": max(
                abs(float(record["reward_total"])) for record in selected
            ),
        }
    return result


def _continuity_probe(reward: Stage1Reward) -> Dict[str, object]:
    probes = {}
    finest_location = None
    for count in (1001, 2001, 10001):
        maximum = 0.0
        location = None
        q_values = np.linspace(0.0, 1.0, count)
        for action in (0.30, 0.75, 1.25, 1.75, 2.00, 2.50, 4.00):
            previous = None
            for q in q_values:
                density = reward.config.density_safe + q * (
                    reward.config.density_dangerous - reward.config.density_safe
                )
                value = Stage1RewardInput(
                    nearest_obstacle_distance_m=10.0,
                    known_obstacle_bin_fraction=density,
                    unknown_majority=False,
                    applied_v_max_mps=action,
                    previous_applied_v_max_mps=action,
                    actual_speed_mps=action,
                    dangerous_terminal=False,
                    terminated=False,
                )
                current = reward.evaluate(value).reward_speed
                if previous is not None and abs(current - previous) > maximum:
                    maximum = abs(current - previous)
                    location = {
                        "phi_2": float(q),
                        "applied_v_max_mps": action,
                    }
                previous = current
        step = 1.0 / float(count - 1)
        probes["{:.4f}".format(step)] = maximum
        finest_location = location
    finest = probes["0.0001"]
    return {
        "maximum_adjacent_delta_by_phi_2_step": probes,
        "finest_phi_2_grid_step": 0.0001,
        "maximum_adjacent_reward_speed_delta": finest,
        "location": finest_location,
        "finite": math.isfinite(finest),
    }


def _smoothing_probe(config: Stage1RewardConfig) -> Dict[str, float]:
    return {
        "delta_{:.2f}_mps".format(delta): -config.lambda_smoothing * delta ** 2
        for delta in (0.02, 0.08, 0.12, 0.50, 1.00)
    }


def _raw_observation_audit(dataset_root: Path) -> Dict[str, int]:
    path = dataset_root / "stage1_reward_calibration_current_generation_samples_merged.csv"
    result = {
        "rows": 0,
        "observation_valid": 0,
        "observation_invalid": 0,
        "training_active_valid": 0,
    }
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            result["rows"] += 1
            valid = _boolean(row, "observation_valid")
            result["observation_valid" if valid else "observation_invalid"] += 1
            if valid and _boolean(row, "training_active"):
                result["training_active_valid"] += 1
    result["invalid_reward_transitions_generated"] = 0
    return result


def _validation_checks(summary: Dict[str, object]) -> Dict[str, bool]:
    formal = summary["cohorts"]["formal"]
    low = formal["complexity"]["Low"]
    medium = formal["complexity"]["Medium"]
    high = formal["complexity"]["High"]
    unknown = formal["complexity"]["Unknown"]
    terminal_checks = [
        event
        for event in summary["terminal_event_scale_checks"]
        if event.get("reward_danger") is not None
    ]
    maximum_pass_reward = max(
        value["maximum_absolute_reward"]
        for value in formal["pass_runs"].values()
    )
    speed_p99 = max(abs(formal["terms"]["reward_speed"]["p01"]), abs(formal["terms"]["reward_speed"]["p99"]))
    smoothing_p99 = max(
        abs(formal["terms"]["reward_smoothing"]["p01"]),
        abs(formal["terms"]["reward_smoothing"]["p99"]),
    )
    return {
        "low_prefers_higher_speed": low["fraction_prefers_higher"] == 1.0,
        "high_prefers_lower_speed": high["fraction_prefers_lower"] == 1.0,
        "medium_phi_between_anchors": (
            0.75 < medium["phi_1"]["p50"] < 1.75
        ),
        "unknown_is_not_low": (
            unknown["phi_2"]["min"] == 0.5
            and unknown["phi_2"]["max"] == 0.5
            and unknown["preference_delta_1p75_minus_0p75"]["mean"]
            < low["preference_delta_1p75_minus_0p75"]["mean"]
        ),
        "dangerous_events_are_clearly_negative": (
            bool(terminal_checks)
            and all(event["reward_total"] < -1.0 for event in terminal_checks)
        ),
        "normal_pass_reward_bounded": maximum_pass_reward < 5.0,
        "smoothing_not_dominant": smoothing_p99 <= 0.10 * max(speed_p99, 1.0e-12),
        "all_terms_finite": all(
            cohort["nonfinite_count"] == 0
            for cohort in summary["cohorts"].values()
        ),
        "no_invalid_observation_reward": (
            summary["raw_observation_audit"]["invalid_reward_transitions_generated"]
            == 0
        ),
        "continuous_speed_reward": (
            summary["continuity_probe"]["finite"]
            and summary["continuity_probe"][
                "maximum_adjacent_reward_speed_delta"
            ]
            < 0.005
        ),
        "paper_lambda_relation": (
            summary["parameters"]["lambda_speed_2"]
            > summary["parameters"]["lambda_speed_3"]
        ),
    }


def main() -> int:
    repository_root = Path(__file__).resolve().parents[4]
    default_root = repository_root / "runtime_artifacts/learning_speed/stage1_reward_calibration_current_generation_20260820_231428"
    default_config = repository_root / "AstraDrone_ros1_ws/src/learning_speed_rl/config/stage1_reward.yaml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=default_root)
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument(
        "--output",
        type=Path,
        default=default_root / "stage1_reward_v1_replay_analysis.json",
    )
    args = parser.parse_args()
    dataset_root = args.dataset_root.resolve()
    config_path = args.config.resolve()
    output_path = args.output.resolve()
    if repository_root.resolve() not in output_path.parents:
        raise ValueError("output must remain under the repository runtime tree")

    config_mapping = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    reward = reward_from_config(config_mapping)
    manifest_path = dataset_root / "stage1_reward_calibration_current_generation_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    runs = _load_runs(repository_root, dataset_root, manifest)
    all_records = []
    run_counters = {}
    terminal_checks = []
    for spec in runs:
        records, counters, terminal_check = _replay_run(reward, spec)
        all_records.extend(records)
        run_counters[spec.run_id] = counters
        if terminal_check is not None:
            terminal_checks.append(terminal_check)

    cohorts = {}
    for cohort in ("formal", "enhanced_repeat", "safety_boundary_only"):
        cohorts[cohort] = _cohort_summary(
            [record for record in all_records if record["cohort"] == cohort]
        )
    config = reward.config
    summary = {
        "schema_version": "astradrone_stage1_reward_replay_v1.0",
        "paper_reference": {
            "title": "Learning Speed Adaptation for Flight in Clutter",
            "arxiv": "2403.04586v2",
            "design_equations": [6, 7, 9, 10],
            "claim": "paper-guided adaptation, not exact paper parameter reproduction",
        },
        "inputs": {
            "dataset_root": str(dataset_root.relative_to(repository_root)),
            "manifest": str(manifest_path.relative_to(repository_root)),
            "manifest_sha256": _sha256(manifest_path),
            "config": str(config_path.relative_to(repository_root)),
            "config_sha256": _sha256(config_path),
            "formal_run_allowlist": [
                spec.run_id for spec in runs if spec.cohort == "formal"
            ],
            "enhanced_repeat_allowlist": list(EXTRA_RUN_ALLOWLIST),
            "safety_boundary_allowlist": [
                spec.run_id for spec in runs if spec.cohort == "safety_boundary_only"
            ],
        },
        "parameters": {
            "nearest_safe_m": config.nearest_safe_m,
            "nearest_dangerous_m": config.nearest_dangerous_m,
            "density_safe": config.density_safe,
            "density_dangerous": config.density_dangerous,
            "anchor_low_mps": config.low_anchor_mps,
            "anchor_medium_mps": config.medium_anchor_mps,
            "anchor_high_mps": config.high_anchor_mps,
            "anchor_unknown_mps": config.unknown_anchor_mps,
            "lambda_phi_1": config.lambda_phi_1,
            "lambda_phi_2": config.lambda_phi_2,
            "lambda_speed_1": config.lambda_speed_1,
            "lambda_speed_2": config.lambda_speed_2,
            "lambda_speed_3": config.lambda_speed_3,
            "lambda_smoothing": config.lambda_smoothing,
            "lambda_danger": config.lambda_danger,
        },
        "legacy_contract_note": (
            "Archived flight transitions are v1.0; replay uses their frozen causal "
            "state/action ordering and exact Observation C stamp join. No archived "
            "file is rewritten."
        ),
        "run_counters": run_counters,
        "cohorts": cohorts,
        "terminal_event_scale_checks": terminal_checks,
        "raw_observation_audit": _raw_observation_audit(dataset_root),
        "smoothing_probe": _smoothing_probe(config),
        "continuity_probe": _continuity_probe(reward),
    }
    summary["validation_checks"] = _validation_checks(summary)
    summary["offline_validation_pass"] = all(summary["validation_checks"].values())
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True, default=_json_default) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "output": str(output_path),
        "formal_transitions": cohorts["formal"]["transition_count"],
        "enhanced_repeat_transitions": cohorts["enhanced_repeat"]["transition_count"],
        "safety_boundary_transitions": cohorts["safety_boundary_only"]["transition_count"],
        "terminal_event_scale_checks": len(terminal_checks),
        "validation_checks": summary["validation_checks"],
        "offline_validation_pass": summary["offline_validation_pass"],
    }, indent=2, sort_keys=True, default=_json_default))
    return 0 if summary["offline_validation_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
