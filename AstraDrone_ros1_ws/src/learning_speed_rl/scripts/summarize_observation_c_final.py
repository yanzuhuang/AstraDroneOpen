#!/usr/bin/env python3
"""Summarize one Observation C closure run without defining reward/training."""

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


EXPECTED_NO_TRAJECTORY_PHASES = {
    "", "WAIT_INPUTS", "SEGMENTED_CLIMB", "RETURN_HOME",
    "FAILURE_LANDING", "DONE", "ERROR",
}
TRAINING_ACTIVE_STATES = {
    "STAGING_POINT", "ENTRY_GATE_TRANSIT", "NAVIGATING", "RELOCATING",
    "RECOVERING", "LAYER_TRANSITION", "GO_TO_EXIT_GATE", "NORMAL_RETURN",
    "RETURN_EGRESS",
}


def finite(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def distribution(values, include_min=False):
    result = {
        "count": len(values),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "max": max(values) if values else None,
    }
    if include_min:
        result["min"] = min(values) if values else None
    return result


def base_reason(raw):
    value = (raw or "unspecified_invalid").strip()
    if value.startswith("lidar_surrogate_invalid"):
        return "lidar_surrogate_invalid"
    if value.startswith("fusion_invalid:trajectory timestamp mismatch"):
        return "fusion_invalid:trajectory timestamp mismatch"
    if value.startswith("contract_invalid"):
        return "contract_invalid"
    return value.split(";", 1)[0]


def expected_invalid(row, reason):
    phase = row.get("mission_state", "")
    if phase == "DONE":
        return True
    if reason == "trajectory_unavailable" and phase in EXPECTED_NO_TRAJECTORY_PHASES:
        return True
    if reason == "lidar_surrogate_invalid":
        detail = row.get("lidar_invalid_reason", "")
        return (
            detail.startswith("insufficient_history")
            and phase in {"", "WAIT_INPUTS", "SEGMENTED_CLIMB"}
        ) or detail.startswith("timestamp_sync:cloud_newer_than_pose_history")
    return False


def summarize(run_dir):
    sample_path = run_dir / "calibration_samples.csv"
    if not sample_path.is_file():
        raise RuntimeError("missing calibration_samples.csv: {}".format(sample_path))
    with sample_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    valid = sum(row.get("observation_valid") == "1" for row in rows)
    active_rows = [
        row for row in rows
        if (
            row.get("training_active") == "1"
            or (
                row.get("training_active", "") == ""
                and row.get("mission_state", "") in TRAINING_ACTIVE_STATES
            )
        )
    ]
    active_valid = sum(
        row.get("observation_valid") == "1" for row in active_rows
    )
    invalid_rows = [row for row in rows if row.get("observation_valid") != "1"]
    reason_counts = Counter()
    reason_phases = defaultdict(Counter)
    reason_stamps = defaultdict(list)
    expected_counts = Counter()
    unexpected_counts = Counter()
    lidar_details = Counter()
    for row in invalid_rows:
        reason = base_reason(row.get("observation_invalid_reasons", ""))
        phase = row.get("mission_state", "") or "<empty>"
        reason_counts[reason] += 1
        reason_phases[reason][phase] += 1
        stamp = finite(row.get("observation_stamp_sec"))
        if stamp is not None:
            reason_stamps[reason].append(stamp)
        if reason == "lidar_surrogate_invalid":
            lidar_details[row.get("lidar_invalid_reason", "") or "unspecified"] += 1
        target = expected_counts if expected_invalid(row, reason) else unexpected_counts
        target[reason] += 1

    reason_summary = {}
    for reason, count in reason_counts.most_common():
        stamps = reason_stamps[reason]
        reason_summary[reason] = {
            "count": count,
            "invalid_ratio": count / len(invalid_rows) if invalid_rows else None,
            "all_observation_ratio": count / len(rows) if rows else None,
            "mission_phases": dict(reason_phases[reason]),
            "first_stamp_sec": min(stamps) if stamps else None,
            "last_stamp_sec": max(stamps) if stamps else None,
        }

    gap_rows = [
        row for row in invalid_rows
        if base_reason(row.get("observation_invalid_reasons", ""))
        == "kinematic_interpolation_gap_too_large"
    ]
    recoverable_gap_rows = [
        row for row in gap_rows
        if row.get("trajectory_lookup_result") == "selected"
    ]
    recoverable_active_gap_rows = [
        row for row in recoverable_gap_rows
        if row.get("training_active") == "1"
    ]
    recoverable_contract_rows = [
        row for row in invalid_rows
        if (
            base_reason(row.get("observation_invalid_reasons", ""))
            == "contract_invalid"
            and row.get("kinematic_lookup_result") in {"exact", "interpolated"}
            and row.get("trajectory_lookup_result") == "selected"
        )
    ]
    recoverable_active_contract_rows = [
        row for row in recoverable_contract_rows
        if row.get("training_active") == "1"
    ]
    one_nanosecond_signature = [
        row for row in gap_rows
        if (
            finite(row.get("dt_after_sec")) is not None
            and 0.0 <= finite(row.get("dt_after_sec")) <= 1.1e-9
            and row.get("state_before_missing") == "0"
            and row.get("state_after_missing") == "0"
        )
    ]
    metric = lambda name, source=gap_rows: [
        value for value in (finite(row.get(name)) for row in source)
        if value is not None and value >= 0.0
    ]

    causal_old_selection = 0
    trajectory_causality_violations = 0
    for row in rows:
        source = finite(row.get("observation_source_stamp_sec"))
        selected_id = finite(row.get("selected_trajectory_id"))
        selected_start = finite(row.get("selected_trajectory_start_stamp_sec"))
        selected_end = finite(row.get("selected_trajectory_end_stamp_sec"))
        latest_id = finite(row.get("latest_trajectory_id_at_lookup"))
        latest_start = finite(row.get("latest_trajectory_start_stamp_sec"))
        if source is None or selected_id is None:
            continue
        if (
            selected_start is None or selected_end is None
            or source < selected_start - 1.0e-9
            or source > selected_end + 1.0e-9
        ):
            trajectory_causality_violations += 1
        if (
            latest_id is not None and selected_id != latest_id
            and latest_start is not None and latest_start > source + 1.0e-9
            and selected_start is not None and selected_end is not None
            and selected_start <= source <= selected_end
        ):
            causal_old_selection += 1

    all_metric = lambda name: [
        value for value in (finite(row.get(name)) for row in rows)
        if value is not None and value >= 0.0
    ]
    result = {
        "schema_version": "observation_c_final_analysis_v1.0",
        "run_directory": str(run_dir),
        "environment": next(
            (row.get("environment") for row in rows if row.get("environment")), ""
        ),
        "observation_c": {
            "total": len(rows),
            "valid": valid,
            "invalid": len(invalid_rows),
            "raw_valid_ratio": valid / len(rows) if rows else None,
            "training_active_total": len(active_rows),
            "training_active_valid": active_valid,
            "training_active_invalid": len(active_rows) - active_valid,
            "training_active_valid_ratio": (
                active_valid / len(active_rows) if active_rows else None
            ),
        },
        "invalid_reasons": reason_summary,
        "expected_invalid": {
            "count": sum(expected_counts.values()),
            "reasons": dict(expected_counts),
        },
        "unexpected_invalid": {
            "count": sum(unexpected_counts.values()),
            "reasons": dict(unexpected_counts),
        },
        "kinematic_gap_evidence": {
            "one_nanosecond_signature_count": len(one_nanosecond_signature),
            "all_gaps_have_one_nanosecond_signature": (
                bool(gap_rows) and len(one_nanosecond_signature) == len(gap_rows)
            ),
            "dt_before_sec": distribution(metric("dt_before_sec")),
            "dt_after_sec": distribution(metric("dt_after_sec")),
            "bracket_span_sec": distribution(metric("bracket_span_sec")),
            "source_to_receipt_latency_sec": distribution(
                metric("source_to_receipt_latency_sec")
            ),
            "state_buffer_coverage_sec": distribution(
                metric("state_buffer_coverage_sec"), include_min=True
            ),
        },
        "all_lookup_evidence": {
            "fast_lio_source_rate_hz": distribution(
                all_metric("fast_lio_source_rate_hz")
            ),
            "fast_lio_state_insert_rate_hz": distribution(
                all_metric("fast_lio_state_insert_rate_hz")
            ),
            "lidar_source_rate_hz": distribution(
                all_metric("lidar_source_rate_hz")
            ),
            "source_to_receipt_latency_sec": distribution(
                all_metric("source_to_receipt_latency_sec")
            ),
            "lidar_build_duration_ms": distribution(
                all_metric("lidar_build_duration_ms")
            ),
            "maximum_out_of_order_state_count": max(
                all_metric("out_of_order_state_count"), default=0
            ),
            "maximum_duplicate_state_stamp_count": max(
                all_metric("duplicate_state_stamp_count"), default=0
            ),
            "maximum_state_buffer_eviction_count": max(
                all_metric("state_buffer_eviction_count"), default=0
            ),
        },
        "trajectory_history": {
            "causal_old_trajectory_selections_after_replan": causal_old_selection,
            "causality_violations": trajectory_causality_violations,
        },
        "causal_replay_projection": {
            "scope": "same recorded B run; preserves the original v2 ROS sec/nsec source stamp and applies the causal producer-receipt lower bound for per-process /clock lag",
            "measured_post_fix_flight": False,
            "recoverable_kinematic_gap_rows": len(recoverable_gap_rows),
            "recoverable_training_active_gap_rows": len(
                recoverable_active_gap_rows
            ),
            "recoverable_contract_clock_lag_rows": len(
                recoverable_contract_rows
            ),
            "projected_raw_valid_ratio": (
                (
                    valid + len(recoverable_gap_rows)
                    + len(recoverable_contract_rows)
                ) / len(rows)
                if rows else None
            ),
            "projected_training_active_valid_ratio": (
                (
                    active_valid + len(recoverable_active_gap_rows)
                    + len(recoverable_active_contract_rows)
                )
                / len(active_rows) if active_rows else None
            ),
        },
        "lidar_invalid_details": dict(lidar_details),
        "reward_defined": False,
        "training_ready": False,
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = summarize(args.run_dir.resolve())
    output = args.output or args.run_dir / "observation_c_final_analysis.json"
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    observation = result["observation_c"]
    print(
        "Observation C: raw={:.6f} active={:.6f} unexpected_invalid={}".format(
            observation["raw_valid_ratio"] or 0.0,
            observation["training_active_valid_ratio"] or 0.0,
            result["unexpected_invalid"]["count"],
        )
    )


if __name__ == "__main__":
    main()
