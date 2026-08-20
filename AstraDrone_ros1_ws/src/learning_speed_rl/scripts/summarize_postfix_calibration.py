#!/usr/bin/env python3
"""Audit and merge only the isolated post-fix Stage 1 calibration matrix.

This utility never computes a reward.  It deliberately refuses pre-fix,
qualification, setup, and infrastructure attempts as formal input.
"""

import argparse
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path


SPEEDS = (0.30, 0.50, 0.75, 1.00, 1.25, 1.50, 1.75)
PROVENANCE = (
    "calibration_generation", "calibration_environment", "fixed_v_max_mps",
    "calibration_run_id", "source_csv", "source_row_index",
)
ACTION_FIELDS = (
    "latest_requested_v_max_mps", "latest_filtered_v_max_mps",
    "latest_applied_v_max_mps",
)
METRICS = (
    "actual_speed_mps", "tracking_error_norm_m", "nearest_obstacle_distance_m",
    "known_obstacle_bin_fraction", "known_obstacle_bin_count",
)
REGRESSION_TOKENS = (
    "kinematic_interpolation_gap_too_large", "trajectory timestamp mismatch",
    "fusion_invalid:trajectory", "causal", "future",
)
EXPECTED_INVALID_PREFIXES = (
    "trajectory_unavailable", "lidar_surrogate_invalid:insufficient_history:",
    "lidar_surrogate_invalid:timestamp_sync:cloud_newer_than_pose_history",
)


def number(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    at = (len(values) - 1) * fraction
    low, high = int(math.floor(at)), int(math.ceil(at))
    if low == high:
        return values[low]
    return values[low] + (values[high] - values[low]) * (at - low)


def stat(values):
    values = [value for value in values if value is not None]
    return {
        "mean": None if not values else sum(values) / len(values),
        "p95": percentile(values, 0.95),
        "max": max(values) if values else None,
        "min": min(values) if values else None,
        "count": len(values),
    }


def expected_ids():
    return [
        "postfix_{}_v{:03d}_r01".format(env, round(speed * 100))
        for env in ("A", "B") for speed in SPEEDS
    ]


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def invalid_unexplained(reasons):
    return {
        reason: count for reason, count in reasons.items()
        if not reason.startswith(EXPECTED_INVALID_PREFIXES)
    }


def load_run(root, logical_id):
    candidates = []
    for directory in sorted(root.glob(logical_id + "*")):
        manifest_path = directory / "experiment_manifest.json"
        if not manifest_path.is_file():
            continue
        manifest = read_json(manifest_path)
        if manifest.get("logical_run_id") != logical_id:
            continue
        candidates.append((int(manifest.get("attempt", 1)), directory, manifest))
    if not candidates:
        return {"logical_run_id": logical_id, "missing": True}
    _, directory, manifest = sorted(candidates)[-1]
    summary_path, samples_path = directory / "run_summary.json", directory / "calibration_samples.csv"
    result = {
        "logical_run_id": logical_id, "directory": str(directory), "manifest": manifest,
        "missing": False, "artifacts": {
            name: (directory / name).is_file() for name in (
                "calibration_samples.csv", "observation_diagnostics.jsonl",
                "planner_failure_episodes.csv", "transition_candidates.jsonl", "run_summary.json",
            )
        },
    }
    if not summary_path.is_file() or not samples_path.is_file():
        return result
    summary = read_json(summary_path)
    result["summary"] = summary
    result["rows"] = []
    with samples_path.open(newline="", encoding="utf-8") as stream:
        result["rows"] = list(csv.DictReader(stream))
    return result


def audit_run(run):
    if run.get("missing"):
        return {"pass": False, "errors": ["missing run directory"]}
    errors = []
    manifest = run["manifest"]
    if manifest.get("calibration_generation") != "post_fix":
        errors.append("manifest is not post_fix")
    if number(manifest.get("speed_ceiling_mps")) != 2.0:
        errors.append("ceiling is not the reviewed 2.00 m/s chain")
    if manifest.get("infrastructure_failure"):
        errors.append("selected attempt is infrastructure failure")
    for name, present in run["artifacts"].items():
        if not present:
            errors.append("missing " + name)
    summary = run.get("summary", {})
    if not summary:
        return {"pass": False, "errors": errors + ["missing run summary"]}
    if not (summary.get("mission", {}).get("done") or summary.get("safety", {}).get("dangerous_terminal")):
        errors.append("no formal terminal")
    if summary.get("reward_defined") is not False or summary.get("training_started") is not False:
        errors.append("reward/training boundary violation")
    if summary.get("transitions", {}).get("training_ready") not in (0, False):
        errors.append("training_ready boundary violation")
    fixed = number(manifest.get("fixed_v_max_mps"))
    actions = {field: [] for field in ACTION_FIELDS}
    invalid = Counter()
    regression = Counter()
    for row in run.get("rows", []):
        for field in ACTION_FIELDS:
            value = number(row.get(field))
            if value is not None:
                actions[field].append(value)
        reason = row.get("observation_invalid_reasons", "")
        if reason:
            invalid[reason] += 1
            lower = reason.lower()
            if any(token in lower for token in REGRESSION_TOKENS):
                regression[reason] += 1
    for field, values in actions.items():
        if not values or any(abs(value - fixed) > 0.005 for value in values):
            errors.append("invalid action chain " + field)
    if regression:
        errors.append("Observation C regression: " + json.dumps(dict(regression), sort_keys=True))
    unexplained = invalid_unexplained(summary.get("observation_c", {}).get("invalid_reasons", {}))
    if unexplained:
        errors.append("unexplained Observation C invalid: " + json.dumps(unexplained, sort_keys=True))
    return {
        "pass": not errors, "errors": errors, "invalid": dict(invalid),
        "regression": dict(regression),
        "training_active_valid_ratio": summary.get("observation_c", {}).get("training_active_valid_ratio"),
    }


def run_record(run):
    audit = audit_run(run)
    if run.get("missing"):
        return {"logical_run_id": run["logical_run_id"], "audit": audit}
    manifest, summary = run["manifest"], run.get("summary", {})
    rows = run.get("rows", [])
    valid_rows = [row for row in rows if row.get("observation_valid") == "1"]
    values = {
        metric: [number(row.get(metric)) for row in valid_rows] for metric in METRICS
    }
    return {
        "logical_run_id": run["logical_run_id"], "directory": run["directory"],
        "environment": manifest.get("environment"), "environment_name": manifest.get("environment_name"),
        "fixed_v_max_mps": number(manifest.get("fixed_v_max_mps")),
        "speed_ceiling_mps": number(manifest.get("speed_ceiling_mps")),
        "route_fingerprint": manifest.get("route_fingerprint"),
        "terminal": bool(summary.get("mission", {}).get("done") or summary.get("safety", {}).get("dangerous_terminal")),
        "mission_result": summary.get("mission", {}).get("result"),
        "terminated": summary.get("episode", {}).get("terminated"),
        "truncated": summary.get("episode", {}).get("truncated"),
        "planner_failure_episodes": summary.get("planner", {}).get("failure_episodes"),
        "collision_proxy": summary.get("safety", {}).get("collision_proxy_terminal"),
        "emergency": summary.get("safety", {}).get("emergency_terminal"),
        "tracking_safety_terminal": summary.get("safety", {}).get("tracking_safety_terminal"),
        "raw_observation_valid_ratio": summary.get("observation_c", {}).get("valid_ratio"),
        "training_active_valid_ratio": summary.get("observation_c", {}).get("training_active_valid_ratio"),
        "invalid_reasons": summary.get("observation_c", {}).get("invalid_reasons", {}),
        "unexplained_invalid": invalid_unexplained(summary.get("observation_c", {}).get("invalid_reasons", {})),
        "transition_candidates": summary.get("transitions", {}).get("candidates"),
        "samples": len(rows), "actual_speed": stat(values["actual_speed_mps"]),
        "tracking_error": stat(values["tracking_error_norm_m"]),
        "nearest_obstacle": stat(values["nearest_obstacle_distance_m"]),
        "density": stat(values["known_obstacle_bin_fraction"]),
        "clutter": stat(values["known_obstacle_bin_count"]), "audit": audit,
    }


def merge(records, runs, output):
    fields, source_rows = [], []
    for record, run in zip(records, runs):
        if run.get("missing"):
            continue
        source = Path(run["directory"]) / "calibration_samples.csv"
        for index, row in enumerate(run.get("rows", []), 1):
            source_rows.append((record, source, index, row))
            for field in row:
                if field not in fields:
                    fields.append(field)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(PROVENANCE) + fields)
        writer.writeheader()
        for record, source, index, row in source_rows:
            writer.writerow({
                "calibration_generation": "post_fix",
                "calibration_environment": record["environment"],
                "fixed_v_max_mps": record["fixed_v_max_mps"],
                "calibration_run_id": record["logical_run_id"],
                "source_csv": str(source), "source_row_index": index, **row,
            })
    return len(source_rows), hashlib.sha256(output.read_bytes()).hexdigest()


def write_report(path, records, result):
    lines = [
        "# Stage 1 Reward post-fix calibration", "",
        "This is a pre-reward/pre-training audit only: reward is null, reward_defined is false, and training_ready is false.", "",
        "The historical pre-fix calibration used a 1.50 m/s ceiling. This isolated post-fix matrix uses the existing reviewed 2.00 m/s high-speed chain solely to admit the new 1.75 m/s fixed-v_max condition. The two generations must not be pooled for reward-calibration statistics.", "",
        "| Run | terminal | mission | training-active valid | raw valid | tracking p95 | planner episodes | audit |",
        "|---|---:|---|---:|---:|---:|---:|---|",
    ]
    for record in records:
        lines.append("| {logical_run_id} | {terminal} | {mission_result} | {training_active_valid_ratio} | {raw_observation_valid_ratio} | {tracking} | {planner_failure_episodes} | {audit_result} |".format(
            tracking=record.get("tracking_error", {}).get("p95"),
            audit_result="PASS" if record.get("audit", {}).get("pass") else "FAIL",
            **record))
    lines += ["", "## Integrity", "", "```json", json.dumps(result["integrity"], indent=2, sort_keys=True), "```", "", "## Per-run analysis", "", "```json", json.dumps(records, indent=2, sort_keys=True), ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--postfix-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--audit-run", metavar="RUN_ID")
    args = parser.parse_args()
    runs = [load_run(args.postfix_root, logical_id) for logical_id in expected_ids()]
    if args.audit_run:
        available = [run for run in runs if run["logical_run_id"] == args.audit_run]
        if len(available) != 1 or available[0].get("missing"):
            raise SystemExit("--audit-run must name one completed formal run")
        audit = audit_run(available[0])
        (Path(available[0]["directory"]) / "postfix_run_audit.json").write_text(json.dumps(audit, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if not audit["pass"]:
            raise SystemExit(20)
        return
    records = [run_record(run) for run in runs]
    integrity = {
        "expected_run_ids": expected_ids(), "formal_run_count": len(records),
        "all_artifacts_present": all(not run.get("missing") and all(run["artifacts"].values()) for run in runs if not run.get("missing")) and not any(run.get("missing") for run in runs),
        "all_run_audits_pass": all(record.get("audit", {}).get("pass") for record in records),
        "route_fingerprints": {env: sorted({record.get("route_fingerprint") for record in records if record.get("environment") == env}) for env in ("A", "B")},
        "pre_fix_ceiling_mps": 1.50, "post_fix_ceiling_mps": 2.00,
        "pre_fix_data_isolated": True,
    }
    integrity["one_route_fingerprint_per_environment"] = all(len(value) == 1 for value in integrity["route_fingerprints"].values())
    args.output_root.mkdir(parents=True, exist_ok=True)
    merged = args.output_root / "stage1_reward_calibration_samples_postfix_merged.csv"
    if integrity["all_artifacts_present"]:
        rows, digest = merge(records, runs, merged)
        integrity.update({"source_rows": sum(record.get("samples", 0) for record in records), "merged_rows": rows, "merged_row_count_matches": rows == sum(record.get("samples", 0) for record in records), "merged_sha256": digest})
    else:
        integrity.update({"source_rows": None, "merged_rows": None, "merged_row_count_matches": False, "merged_sha256": None})
    result = {"schema_version": "stage1_reward_calibration_postfix_v1.0", "runs": records, "integrity": integrity}
    analysis = args.output_root / "stage1_reward_calibration_postfix_analysis.json"
    report = args.output_root / "stage1_reward_calibration_postfix_report.md"
    analysis.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_report(report, records, result)
    if not (integrity["all_artifacts_present"] and integrity["all_run_audits_pass"] and integrity["merged_row_count_matches"] and integrity["one_route_fingerprint_per_environment"]):
        raise SystemExit(20)


if __name__ == "__main__":
    main()
