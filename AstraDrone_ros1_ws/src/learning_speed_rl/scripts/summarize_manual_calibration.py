#!/usr/bin/env python3
"""Summarize fixed-speed manual calibration without defining a reward."""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import median


SPEEDS = (0.30, 0.50, 0.75, 1.00, 1.25, 1.50)
ENVIRONMENTS = ("A", "B")
INACTIVE_STATES = {"", "WAIT_INPUTS", "DONE", "ERROR"}
METRIC_FIELDS = (
    "actual_speed_mps",
    "tracking_error_norm_m",
    "nearest_obstacle_distance_m",
    "known_obstacle_bin_fraction",
    "known_obstacle_bin_count",
    "raw_filtered_nearest_point_distance_m",
    "inflated_occupied_center_distance_m",
    "latest_requested_v_max_mps",
    "latest_filtered_v_max_mps",
    "latest_applied_v_max_mps",
)
ACTION_FIELDS = (
    "latest_requested_v_max_mps",
    "latest_filtered_v_max_mps",
    "latest_applied_v_max_mps",
)
INDEPENDENT_DISTANCE_FIELDS = (
    "raw_filtered_nearest_point_distance_m",
    "inflated_occupied_center_distance_m",
)


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


def mean(values):
    return None if not values else sum(values) / len(values)


def rank(values):
    order = sorted(range(len(values)), key=lambda index: values[index])
    result = [0.0] * len(values)
    cursor = 0
    while cursor < len(order):
        end = cursor + 1
        while end < len(order) and values[order[end]] == values[order[cursor]]:
            end += 1
        average_rank = (cursor + end - 1) / 2.0
        for position in range(cursor, end):
            result[order[position]] = average_rank
        cursor = end
    return result


def correlation(left, right):
    if len(left) < 2 or len(left) != len(right):
        return None
    left_mean, right_mean = mean(left), mean(right)
    numerator = sum(
        (x - left_mean) * (y - right_mean) for x, y in zip(left, right)
    )
    denominator = math.sqrt(
        sum((x - left_mean) ** 2 for x in left)
        * sum((y - right_mean) ** 2 for y in right)
    )
    return None if denominator <= 0.0 else numerator / denominator


def spearman(left, right):
    return correlation(rank(left), rank(right))


def slope(left, right):
    if len(left) < 2 or len(left) != len(right):
        return None
    left_mean = mean(left)
    denominator = sum((value - left_mean) ** 2 for value in left)
    if denominator <= 0.0:
        return None
    right_mean = mean(right)
    return sum(
        (x - left_mean) * (y - right_mean) for x, y in zip(left, right)
    ) / denominator


def nested(record, path, default=None):
    current = record
    for key in path.split("."):
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def transition_audit(path):
    result = {
        "records": 0,
        "reward_defined": 0,
        "reward_undefined": 0,
        "contract_valid": True,
        "violations": [],
    }
    if not path.is_file():
        result["contract_valid"] = False
        result["violations"].append("transition_candidates.jsonl missing")
        return result
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            result["records"] += 1
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                result["violations"].append("invalid JSON line {}".format(line_number))
                continue
            defined = item.get("reward_defined") is True
            if defined:
                reward = item.get("reward")
                components = item.get("reward_components")
                reward_version = item.get("reward_version")
                required = [
                    "reward_total", "reward_speed", "reward_smoothing",
                    "reward_danger", "phi_1", "phi_2",
                ]
                if reward_version in (
                    "astradrone_paper_guided_reward_v2.0",
                    "astradrone_paper_guided_reward_v3.1",
                ):
                    required.append("reward_error")
                valid = (
                    isinstance(reward, (int, float))
                    and math.isfinite(reward)
                    and item.get("training_ready") is True
                    and reward_version in (
                        "astradrone_stage1_reward_v1.0",
                        "astradrone_paper_guided_reward_v2.0",
                        "astradrone_paper_guided_reward_v3.1",
                    )
                    and isinstance(components, dict)
                    and components.get("reward_valid") is True
                    and all(
                        isinstance(components.get(name), (int, float))
                        and math.isfinite(components[name])
                        for name in required
                    )
                    and abs(float(components["reward_total"]) - reward)
                    <= 1.0e-12
                )
                if valid:
                    result["reward_defined"] += 1
                else:
                    result["violations"].append(
                        "invalid defined Stage 1 reward line {}".format(line_number)
                    )
            elif (
                item.get("reward") is None
                and item.get("reward_defined") is False
                and item.get("training_ready") is False
                and item.get("reward_version") in (None, "")
                and item.get("reward_components") is None
            ):
                result["reward_undefined"] += 1
            else:
                result["violations"].append(
                    "reward/training boundary violation line {}".format(line_number)
                )
    result["contract_valid"] = not result["violations"]
    return result


def terminal_reason(summary, last_row):
    safety = summary.get("safety", {})
    reasons = []
    if safety.get("collision_proxy_terminal"):
        reasons.append("collision_proxy")
    if safety.get("emergency_terminal"):
        reasons.append("ego_emergency_stop")
    if safety.get("tracking_safety_terminal"):
        reasons.append("tracking_safety_gate")
    if not reasons:
        mission = summary.get("mission", {})
        if mission.get("success"):
            reasons.append("mission_success")
        elif mission.get("failure"):
            reasons.append("mission_failure")
        elif mission.get("done"):
            reasons.append("mission_done_without_result")
        else:
            reasons.append("infrastructure_truncated")
    planner_reason = str(last_row.get("planner_failure_reason", "")).strip()
    if (
        nested(summary, "planner.failure_episodes", 0)
        and planner_reason
        and planner_reason != "NONE"
        and planner_reason not in reasons
    ):
        reasons.append("planner:" + planner_reason)
    return ";".join(reasons)


def mission_terminal_detail(path):
    if not path.is_file():
        return ""
    detail = ""
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            candidate = str(row.get("target_switch_reason", "")).strip()
            if candidate:
                detail = candidate
    return detail


def load_attempt(directory):
    manifest_path = directory / "experiment_manifest.json"
    if not manifest_path.is_file():
        return None
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    summary_path = directory / "run_summary.json"
    summary = (
        json.loads(summary_path.read_text(encoding="utf-8"))
        if summary_path.is_file()
        else {}
    )
    values = {field: [] for field in METRIC_FIELDS}
    last_row = {}
    active_rows = 0
    active_valid_rows = 0
    samples_path = directory / "calibration_samples.csv"
    if samples_path.is_file():
        with samples_path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                last_row = row
                if row.get("mission_state", "") in INACTIVE_STATES:
                    continue
                active_rows += 1
                observation_valid = row.get("observation_valid", "") == "1"
                if observation_valid:
                    active_valid_rows += 1
                for field in METRIC_FIELDS:
                    # Observation-derived flight/clutter metrics are comparable
                    # only on valid Observation C rows.  The action chain and
                    # independently timestamped raw/inflated distances remain
                    # auditable on partial rows.
                    if (
                        not observation_valid
                        and field not in ACTION_FIELDS
                        and field not in INDEPENDENT_DISTANCE_FIELDS
                    ):
                        continue
                    value = finite(row.get(field))
                    if value is not None:
                        values[field].append(value)

    safety = summary.get("safety", {})
    mission = summary.get("mission", {})
    terminal = bool(mission.get("done") or safety.get("dangerous_terminal"))
    passed = bool(
        mission.get("done")
        and mission.get("success")
        and not safety.get("dangerous_terminal")
    )
    action_tolerance = 0.005
    fixed_speed = float(manifest["fixed_v_max_mps"])
    action_valid = all(
        values[field]
        and max(abs(value - fixed_speed) for value in values[field]) <= action_tolerance
        for field in ACTION_FIELDS
    )
    reward = transition_audit(directory / "transition_candidates.jsonl")
    mission_detail = mission_terminal_detail(directory / "mission.csv")
    reason = terminal_reason(summary, last_row)
    if mission.get("failure") and mission_detail:
        reason += ";" + mission_detail
    metrics_complete = all(
        values[field]
        for field in (
            "actual_speed_mps",
            "tracking_error_norm_m",
            "nearest_obstacle_distance_m",
            "known_obstacle_bin_fraction",
            "known_obstacle_bin_count",
        )
    )
    data_quality_note = ""
    if terminal and not metrics_complete:
        if nested(summary, "observation_c.valid", 0) and active_valid_rows == 0:
            data_quality_note = "valid Observation C counted but recorder has no valid metric rows"
        else:
            data_quality_note = "terminal run has incomplete valid Observation C metrics"
    return {
        "logical_run_id": manifest["logical_run_id"],
        "attempt": int(manifest.get("attempt", 1)),
        "attempt_run_id": directory.name,
        "directory": str(directory),
        "environment": manifest["environment"],
        "environment_name": manifest["environment_name"],
        "world": manifest["world"],
        "spawn_x_m": float(manifest["spawn_x_m"]),
        "spawn_y_m": float(manifest["spawn_y_m"]),
        "route_fingerprint": manifest["route_fingerprint"],
        "fixed_v_max_mps": fixed_speed,
        "terminal": terminal,
        "pass": passed,
        "infrastructure_failure": bool(manifest.get("infrastructure_failure", False)),
        "action_chain_valid": action_valid,
        "active_samples": active_rows,
        "active_valid_samples": active_valid_rows,
        "metrics_complete": metrics_complete,
        "data_quality_note": data_quality_note,
        "actual_speed_mean_mps": mean(values["actual_speed_mps"]),
        "actual_speed_p95_mps": percentile(values["actual_speed_mps"], 0.95),
        "tracking_error_mean_m": mean(values["tracking_error_norm_m"]),
        "tracking_error_p95_m": percentile(values["tracking_error_norm_m"], 0.95),
        "tracking_error_max_m": max(values["tracking_error_norm_m"], default=None),
        "nearest_obstacle_min_m": min(values["nearest_obstacle_distance_m"], default=None),
        "nearest_obstacle_mean_m": mean(values["nearest_obstacle_distance_m"]),
        "density_proxy_mean": mean(values["known_obstacle_bin_fraction"]),
        "density_proxy_p95": percentile(values["known_obstacle_bin_fraction"], 0.95),
        "clutter_proxy_mean_bins": mean(values["known_obstacle_bin_count"]),
        "clutter_proxy_p95_bins": percentile(values["known_obstacle_bin_count"], 0.95),
        "raw_filtered_nearest_min_m": min(
            values["raw_filtered_nearest_point_distance_m"], default=None
        ),
        "inflated_occupied_center_nearest_min_m": min(
            values["inflated_occupied_center_distance_m"], default=None
        ),
        "requested_v_max_mean_mps": mean(values["latest_requested_v_max_mps"]),
        "filtered_v_max_mean_mps": mean(values["latest_filtered_v_max_mps"]),
        "applied_v_max_mean_mps": mean(values["latest_applied_v_max_mps"]),
        "planner_failure": int(nested(summary, "planner.failure_episodes", 0) or 0) > 0,
        "planner_failure_episodes": int(
            nested(summary, "planner.failure_episodes", 0) or 0
        ),
        "terminal_reason": reason,
        "observation_c_valid_ratio": nested(summary, "observation_c.valid_ratio"),
        "reward_contract_valid": reward["contract_valid"],
        "transition_candidates": reward["records"],
        "reward_contract_violations": ";".join(reward["violations"]),
    }


def choose_attempts(root):
    attempts = defaultdict(list)
    for directory in sorted(path for path in root.iterdir() if path.is_dir()):
        attempt = load_attempt(directory)
        if attempt is not None:
            attempts[attempt["logical_run_id"]].append(attempt)
    selected = []
    for environment in ENVIRONMENTS:
        for speed in SPEEDS:
            logical_id = "{}_v{:03d}_r01".format(environment, round(speed * 100))
            candidates = sorted(attempts.get(logical_id, []), key=lambda item: item["attempt"])
            terminal = [item for item in candidates if item["terminal"]]
            if terminal:
                selected.append(terminal[-1])
            elif candidates:
                selected.append(candidates[-1])
            else:
                selected.append({
                    "logical_run_id": logical_id,
                    "environment": environment,
                    "environment_name": "missing",
                    "fixed_v_max_mps": speed,
                    "terminal": False,
                    "pass": False,
                    "terminal_reason": "missing_run",
                })
    return selected


def load_qualification_attempts(root):
    attempts = []
    for directory in sorted(path for path in root.iterdir() if path.is_dir()):
        manifest_path = directory / "experiment_manifest.json"
        if not manifest_path.is_file():
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if not manifest.get("qualification_only", False):
            continue
        summary_path = directory / "run_summary.json"
        summary = (
            json.loads(summary_path.read_text(encoding="utf-8"))
            if summary_path.is_file() else {}
        )
        attempts.append({
            "attempt_run_id": directory.name,
            "fixed_v_max_mps": manifest.get("fixed_v_max_mps"),
            "infrastructure_failure": bool(
                manifest.get("infrastructure_failure", False)
            ),
            "mission_result": nested(summary, "mission.result", "missing"),
            "episode_terminated": bool(nested(summary, "episode.terminated", False)),
            "episode_truncated": bool(nested(summary, "episode.truncated", False)),
            "qualification_reason": manifest.get("qualification_reason", ""),
            "directory": str(directory),
        })
    return attempts


def common_superiority(left, right):
    comparisons = [
        (1.0 if b > a else 0.5 if b == a else 0.0)
        for a in left for b in right
    ]
    return mean(comparisons)


def quality_analysis(runs):
    result = {
        "data_quality": {},
        "environment_discrimination": {},
        "tracking_by_environment": {},
    }
    valid = [run for run in runs if run.get("terminal")]
    observation_ratios = [
        run["observation_c_valid_ratio"] for run in valid
        if run.get("observation_c_valid_ratio") is not None
    ]
    route_fingerprints = {
        environment: sorted({
            run.get("route_fingerprint") for run in runs
            if run.get("environment") == environment
            and run.get("route_fingerprint")
        })
        for environment in ENVIRONMENTS
    }
    result["data_quality"] = {
        "formal_run_count": len(runs),
        "terminal_run_count": len(valid),
        "pass_count": sum(bool(run.get("pass")) for run in runs),
        "complete_metric_run_count": sum(
            bool(run.get("metrics_complete")) for run in runs
        ),
        "action_chain_valid_count": sum(
            bool(run.get("action_chain_valid")) for run in runs
        ),
        "reward_contract_valid_count": sum(
            bool(run.get("reward_contract_valid")) for run in runs
        ),
        "observation_c_valid_ratio_min": min(observation_ratios, default=None),
        "observation_c_valid_ratio_max": max(observation_ratios, default=None),
        "route_fingerprints": route_fingerprints,
        "one_route_fingerprint_per_environment": all(
            len(route_fingerprints[environment]) == 1
            for environment in ENVIRONMENTS
        ),
    }
    by_environment = {
        environment: [run for run in valid if run["environment"] == environment]
        for environment in ENVIRONMENTS
    }
    density = {
        environment: [
            run["density_proxy_mean"] for run in by_environment[environment]
            if run.get("density_proxy_mean") is not None
        ]
        for environment in ENVIRONMENTS
    }
    nearest = {
        environment: [
            run["nearest_obstacle_mean_m"] for run in by_environment[environment]
            if run.get("nearest_obstacle_mean_m") is not None
        ]
        for environment in ENVIRONMENTS
    }
    density_superiority = (
        common_superiority(density["A"], density["B"])
        if density["A"] and density["B"] else None
    )
    density_disjoint = bool(
        density["A"] and density["B"]
        and (max(density["A"]) < min(density["B"])
             or max(density["B"]) < min(density["A"]))
    )
    nearest_superiority = (
        common_superiority(nearest["B"], nearest["A"])
        if nearest["A"] and nearest["B"] else None
    )
    obvious = bool(
        density_superiority is not None
        and density_disjoint
        and (density_superiority >= 0.9 or density_superiority <= 0.1)
    )
    result["environment_discrimination"] = {
        "density_proxy_A_median": median(density["A"]) if density["A"] else None,
        "density_proxy_B_median": median(density["B"]) if density["B"] else None,
        "density_proxy_ranges_disjoint": density_disjoint,
        "probability_B_density_exceeds_A": density_superiority,
        "nearest_obstacle_A_median_m": median(nearest["A"]) if nearest["A"] else None,
        "nearest_obstacle_B_median_m": median(nearest["B"]) if nearest["B"] else None,
        "probability_A_nearest_distance_exceeds_B": nearest_superiority,
        "clearly_distinguishable": obvious,
        "conclusion": (
            "A/B obstacle density-clutter proxies are clearly separated"
            if obvious else
            "A/B obstacle proxies are not clearly separated by this first-pass dataset"
        ),
    }
    for environment in ENVIRONMENTS:
        ordered = sorted(
            [run for run in by_environment[environment]
             if run.get("tracking_error_mean_m") is not None
             and run.get("tracking_error_p95_m") is not None],
            key=lambda item: item["fixed_v_max_mps"],
        )
        speeds = [run["fixed_v_max_mps"] for run in ordered]
        tracking_mean = [run["tracking_error_mean_m"] for run in ordered]
        tracking_p95 = [run["tracking_error_p95_m"] for run in ordered]
        rho_mean = spearman(speeds, tracking_mean) if len(ordered) >= 3 else None
        rho_p95 = spearman(speeds, tracking_p95) if len(ordered) >= 3 else None
        low = mean(tracking_p95[:2]) if len(tracking_p95) >= 4 else None
        high = mean(tracking_p95[-2:]) if len(tracking_p95) >= 4 else None
        ratio = high / low if low and high is not None else None
        worsens = bool(
            rho_p95 is not None and rho_mean is not None
            and max(rho_p95, rho_mean) >= 0.7
            and ratio is not None and ratio >= 1.2
        )
        result["tracking_by_environment"][environment] = {
            "run_count": len(ordered),
            "spearman_speed_vs_tracking_mean": rho_mean,
            "spearman_speed_vs_tracking_p95": rho_p95,
            "tracking_mean_slope_m_per_mps": slope(speeds, tracking_mean),
            "tracking_p95_slope_m_per_mps": slope(speeds, tracking_p95),
            "high_two_to_low_two_tracking_p95_ratio": ratio,
            "tracking_clearly_worsens": worsens,
            "conclusion": (
                "tracking degradation with speed is clear"
                if worsens else
                "tracking degradation with speed is not clear from this first pass"
            ),
        }
    return result


def fmt(value, digits=3):
    return "" if value is None else ("{:.{}f}".format(value, digits))


def write_csv(path, runs):
    fields = (
        "logical_run_id", "attempt_run_id", "environment", "environment_name",
        "fixed_v_max_mps", "result", "actual_speed_mean_mps",
        "actual_speed_p95_mps", "tracking_error_mean_m", "tracking_error_p95_m",
        "tracking_error_max_m", "nearest_obstacle_min_m", "nearest_obstacle_mean_m",
        "density_proxy_mean", "density_proxy_p95", "clutter_proxy_mean_bins",
        "clutter_proxy_p95_bins", "raw_filtered_nearest_min_m",
        "inflated_occupied_center_nearest_min_m", "requested_v_max_mean_mps",
        "filtered_v_max_mean_mps", "applied_v_max_mean_mps", "action_chain_valid",
        "planner_failure", "planner_failure_episodes", "terminal_reason",
        "observation_c_valid_ratio", "metrics_complete", "data_quality_note",
        "reward_contract_valid", "transition_candidates",
        "route_fingerprint", "world", "spawn_x_m", "spawn_y_m", "directory",
    )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for run in runs:
            row = dict(run)
            row["result"] = "PASS" if run.get("pass") else "FAIL"
            writer.writerow(row)


def write_report(path, runs, analysis, qualification_attempts):
    lines = [
        "# UAV1 固定速度首轮人工标定汇总",
        "",
        "本报告只做数据质量与可辨识性分析；未定义 φ1/φ2、reward 或其权重，未训练或运行 SAC policy。",
        "",
        "| 环境 | fixed v_max | 结果 | actual speed mean/p95 (m/s) | tracking mean/p95/max (m) | nearest obstacle min (m) | density / clutter mean | planner failure | terminal reason | Obs C valid |",
        "|---|---:|---|---|---|---:|---|---|---|---:|",
    ]
    for run in runs:
        lines.append(
            "| {environment} | {speed} | {result} | {actual_mean}/{actual_p95} | "
            "{tracking_mean}/{tracking_p95}/{tracking_max} | {nearest} | "
            "{density}/{clutter} | {planner} | {reason} | {valid} |".format(
                environment=run["environment"],
                speed=fmt(run["fixed_v_max_mps"], 2),
                result="PASS" if run.get("pass") else "FAIL",
                actual_mean=fmt(run.get("actual_speed_mean_mps")),
                actual_p95=fmt(run.get("actual_speed_p95_mps")),
                tracking_mean=fmt(run.get("tracking_error_mean_m")),
                tracking_p95=fmt(run.get("tracking_error_p95_m")),
                tracking_max=fmt(run.get("tracking_error_max_m")),
                nearest=fmt(run.get("nearest_obstacle_min_m")),
                density=fmt(run.get("density_proxy_mean"), 5),
                clutter=fmt(run.get("clutter_proxy_mean_bins"), 1),
                planner=("yes ({})".format(run.get("planner_failure_episodes", 0))
                         if run.get("planner_failure") else "no"),
                reason=(run.get("terminal_reason", "")[:120]
                        + ("…" if len(run.get("terminal_reason", "")) > 120 else "")),
                valid=fmt(run.get("observation_c_valid_ratio"), 3),
            )
        )
    environment = analysis["environment_discrimination"]
    data_quality = analysis["data_quality"]
    incomplete = [
        run["logical_run_id"] for run in runs
        if run.get("terminal") and not run.get("metrics_complete", False)
    ]
    lines.extend([
        "",
        "## 数据质量与可辨识性",
        "",
        "- 有效指标完整性：{}。".format(
            "12 个终态 run 均有完整有效 Observation C 指标"
            if not incomplete else
            "以下终态 run 缺少完整有效指标，相关统计留空且不参与可辨识性计算：{}".format(
                ", ".join(incomplete)
            )
        ),
        "- 正式 run 审计：{}/{} terminal 且 PASS，{}/{} 指标完整，{}/{} action chain 有效，{}/{} reward/training 边界有效；Observation C valid ratio 范围 {}/{}；每个环境只有一个路线指纹={}。".format(
            data_quality["pass_count"], data_quality["formal_run_count"],
            data_quality["complete_metric_run_count"], data_quality["formal_run_count"],
            data_quality["action_chain_valid_count"], data_quality["formal_run_count"],
            data_quality["reward_contract_valid_count"], data_quality["formal_run_count"],
            fmt(data_quality["observation_c_valid_ratio_min"]),
            fmt(data_quality["observation_c_valid_ratio_max"]),
            data_quality["one_route_fingerprint_per_environment"],
        ),
        "- A/B 障碍特征：{}。A/B density 中位数为 {}/{}，B density 大于 A 的成对比例为 {}；A/B nearest-obstacle 均值的运行中位数为 {}/{} m。".format(
            environment["conclusion"],
            fmt(environment["density_proxy_A_median"], 5),
            fmt(environment["density_proxy_B_median"], 5),
            fmt(environment["probability_B_density_exceeds_A"], 3),
            fmt(environment["nearest_obstacle_A_median_m"]),
            fmt(environment["nearest_obstacle_B_median_m"]),
        ),
    ])
    for code in ENVIRONMENTS:
        item = analysis["tracking_by_environment"][code]
        lines.append(
            "- 环境 {} tracking：{}。speed 对 tracking mean/p95 的 Spearman ρ={}/{}，高两档相对低两档的 p95 比值={}。".format(
                code, item["conclusion"],
                fmt(item["spearman_speed_vs_tracking_mean"]),
                fmt(item["spearman_speed_vs_tracking_p95"]),
                fmt(item["high_two_to_low_two_tracking_p95_ratio"]),
            )
        )
    lines.extend([
        "- density/clutter 是 Observation C 已知障碍角度 bin 的代理，不是物体数、物理体积或 clearance。nearest obstacle 是 surrogate 最近已知障碍距离。raw-filtered 与 EGO inflated occupied-center 最近距只在 CSV 中按各自语义单独保留。",
        "- PASS 要求任务 success+done 且无 collision/emergency/tracking safety terminal；恢复过的 planner failure 会如实保留，但本身不覆盖任务结果。",
        "- transition reward 边界同时兼容冻结的 legacy reward-null 工件与在线 Stage 1 工件；后者必须是 finite、versioned、reward_defined=true 且 training_ready=true。",
    ])
    if qualification_attempts:
        lines.extend([
            "",
            "## 不计入 12 行的路线/启动资格验证",
            "",
            "以下工件均保留且 manifest 标记 `qualification_only=true`；它们没有覆盖正式 run，也不参与可辨识性计算：",
            "",
        ])
        for attempt in qualification_attempts:
            lines.append(
                "- `{}`：{}（mission_result={}，terminated={}，truncated={}）。".format(
                    attempt["attempt_run_id"], attempt["qualification_reason"],
                    attempt["mission_result"], attempt["episode_terminated"],
                    attempt["episode_truncated"],
                )
            )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    runs = choose_attempts(root)
    qualification_attempts = load_qualification_attempts(root)
    analysis = quality_analysis(runs)
    write_csv(root / "manual_calibration_runs.csv", runs)
    (root / "manual_calibration_analysis.json").write_text(
        json.dumps({
            "runs": runs,
            "qualification_attempts": qualification_attempts,
            "analysis": analysis,
        }, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_report(
        root / "manual_calibration_report.md", runs, analysis,
        qualification_attempts,
    )
    print(root / "manual_calibration_runs.csv")


if __name__ == "__main__":
    main()
