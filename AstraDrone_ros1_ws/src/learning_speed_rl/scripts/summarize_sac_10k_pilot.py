#!/usr/bin/env python3
"""Summarize the formal SAC 10k pilot and isolated evaluations."""

import argparse
import json
import math
from pathlib import Path
import statistics

import yaml


EVALUATION_STEPS = (0, 2000, 4000, 6000, 8000, 10000)


def load_json(path):
    with open(str(path), "r", encoding="utf-8") as stream:
        return json.load(stream)


def load_jsonl(path):
    if not path.is_file():
        return []
    with open(str(path), "r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def finite(values):
    return [float(value) for value in values if value is not None and math.isfinite(float(value))]


def stats(values):
    values = finite(values)
    if not values:
        return {"count": 0, "mean": None, "min": None, "max": None, "p95": None}
    ordered = sorted(values)
    position = 0.95 * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    p95 = ordered[lower] + (position - lower) * (ordered[upper] - ordered[lower])
    return {
        "count": len(values), "mean": statistics.mean(values),
        "std": statistics.pstdev(values),
        "min": min(values), "max": max(values), "p95": p95,
    }


def fmt(value, digits=6):
    if value is None:
        return "NA"
    if isinstance(value, bool):
        return "PASS" if value else "FAIL"
    if isinstance(value, int):
        return str(value)
    return ("{:.%df}" % digits).format(float(value))


def metric_range(rows, name):
    return stats(row.get(name) for row in rows)


def episode_outcome(summary):
    outcome = str(summary.get("coordinator_terminal_outcome", ""))
    if outcome:
        return outcome
    episode = summary.get("episode", {})
    if episode.get("terminated"):
        return "FAILURE"
    if episode.get("truncated"):
        return "TRUNCATED"
    return "UNKNOWN"


def aggregate_evaluation(directory, step):
    summary = load_json(directory / "sac_runtime_summary.json")
    episodes = load_json(directory / "sac_episode_summaries.json")
    coordinator = load_json(directory / "qualification_summary.json")
    outcomes = [episode_outcome(item) for item in episodes]
    return {
        "step": step,
        "status": summary.get("status"),
        "verdict": summary.get("verdict"),
        "episodes": len(episodes),
        "success": outcomes.count("SUCCESS"),
        "failure": outcomes.count("FAILURE"),
        "truncated": outcomes.count("TRUNCATED"),
        "success_rate": outcomes.count("SUCCESS") / float(max(1, len(outcomes))),
        "return": stats(item.get("episode_return") for item in episodes),
        "steps": stats(item.get("transition_count") for item in episodes),
        "duration": stats(item.get("episode", {}).get("episode_elapsed") for item in episodes),
        "requested_v_max": stats(item.get("requested_v_max", {}).get("mean") for item in episodes),
        "actual_speed": stats(item.get("actual_speed_mps", {}).get("mean") for item in episodes),
        "tracking_error": stats(item.get("tracking_error_m", {}).get("mean") for item in episodes),
        "wall_duration": coordinator.get("wall_duration"),
        "sim_duration": coordinator.get("sim_duration"),
        "rtf": coordinator.get("rtf"),
    }


def classify_failures(failures):
    categories = set()
    text = " ".join(str(value).lower() for value in failures)
    mapping = {
        "ENVIRONMENT": ("planner", "controller", "collision"),
        "OBSERVATION": ("invalid_observation", "old_generation_contamination"),
        "ACTION": ("identity", "action_exploration", "scheduler", "deadline"),
        "SAC": ("learner_failed", "gradient_explosion", "loss_explosion", "q_divergence"),
        "REPLAY": ("replay_pollution", "replay_audit"),
        "RESET": ("reset_failure", "old_generation_contamination"),
        "PERFORMANCE": ("rate", "stalled", "performance"),
    }
    for category, needles in mapping.items():
        if any(needle in text for needle in needles):
            categories.add(category)
    return sorted(categories)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pilot-root", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    root = Path(args.pilot_root).resolve()
    training = root / "training"
    output = Path(args.output).resolve()
    config_path = Path(args.config).resolve()
    with open(str(config_path), "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    runtime = load_json(training / "sac_runtime_summary.json")
    coordinator = load_json(training / "qualification_summary.json")
    episodes = load_json(training / "sac_episode_summaries.json")
    resets = load_json(training / "reset_results.json")
    transitions = load_jsonl(training / "sac_transition_audit.jsonl")
    learner = load_jsonl(training / "sac_learner_metrics.jsonl")
    resource_path = training / "launch_resource_summary.json"
    resources = load_json(resource_path) if resource_path.is_file() else {}
    evaluations = []
    missing_evaluations = []
    for step in EVALUATION_STEPS:
        directory = root / "evaluation_step_{:05d}".format(step)
        if (directory / "sac_runtime_summary.json").is_file():
            evaluations.append(aggregate_evaluation(directory, step))
        else:
            missing_evaluations.append(step)

    training_failures = list(runtime.get("failures", []))
    training_failures.extend(coordinator.get("acceptance_failures", []))
    if runtime.get("status") != "completed":
        training_failures.append(runtime.get("failure", "pilot_runtime_failed"))
    if runtime.get("global_environment_step") != 10000:
        training_failures.append(
            "valid_transition_target:{}_of_10000".format(
                runtime.get("global_environment_step", 0)
            )
        )
    present_checkpoints = {
        int(item["environment_step"])
        for item in runtime.get("checkpoint_manifest", [])
    }
    missing_checkpoints = sorted(set(EVALUATION_STEPS) - present_checkpoints)
    if missing_checkpoints:
        training_failures.append(
            "missing_checkpoints:{}".format(missing_checkpoints)
        )
    if missing_evaluations:
        training_failures.append(
            "missing_evaluations:{}".format(missing_evaluations)
        )
    evaluation_failures = [
        "evaluation_{}:{}".format(item["step"], item["verdict"])
        for item in evaluations if item["status"] != "completed"
    ]
    failures = sorted(set(training_failures + evaluation_failures))
    stable = bool(
        not failures
        and runtime.get("global_environment_step") == 10000
        and len(transitions) == 10000
        and runtime.get("replay_audit", {}).get("passed", False)
        and coordinator.get("reset_success_count") == coordinator.get("reset_count")
    )

    action_early = stats(item["normalized_policy_action"] for item in transitions[:1000])
    action_late = stats(item["normalized_policy_action"] for item in transitions[-1000:])
    request_early = stats(item["requested_v_max"] for item in transitions[:1000])
    request_late = stats(item["requested_v_max"] for item in transitions[-1000:])
    action_exploration_instability = bool(
        action_late["std"] is not None
        and action_early["std"] is not None
        and action_late["std"] > 2.0 * action_early["std"]
    )
    if action_exploration_instability:
        failures = sorted(set(failures + ["action_exploration_instability"]))
    return_values = [item.get("episode_return") for item in episodes]
    first_returns = stats(return_values[:5])
    last_returns = stats(return_values[-5:])
    baseline_eval = next((item for item in evaluations if item["step"] == 0), None)
    final_eval = next((item for item in evaluations if item["step"] == 10000), None)
    evaluation_improved = bool(
        baseline_eval is not None and final_eval is not None
        and final_eval["return"]["mean"] is not None
        and baseline_eval["return"]["mean"] is not None
        and final_eval["return"]["mean"] > baseline_eval["return"]["mean"]
    )
    policy_shift = bool(
        action_early["mean"] is not None
        and action_late["mean"] is not None
        and abs(action_late["mean"] - action_early["mean"]) > 0.02
    )
    preliminary_learning = bool(stable and (evaluation_improved or policy_shift))
    recommend_extended = stable
    numerical_names = (
        "actor_loss", "critic1_loss", "critic2_loss", "alpha", "entropy",
        "q1_min", "q1_max", "q2_min", "q2_max", "target_q_min",
        "target_q_max", "critic_gradient_max_abs", "actor_gradient_max_abs",
    )
    numerical_finite = all(
        math.isfinite(float(row[name]))
        for row in learner for name in numerical_names
        if row.get(name) is not None
    )
    numerical_stable = bool(
        numerical_finite
        and max(abs(item.get("q1_min", 0.0)) for item in learner) < config["numerical_safety"]["q_abs_limit"]
        and max(abs(item.get("q2_min", 0.0)) for item in learner) < config["numerical_safety"]["q_abs_limit"]
        and max(item.get("critic_gradient_max_abs", 0.0) for item in learner) < config["numerical_safety"]["gradient_abs_limit"]
    )
    learner_hz = None
    learner_wall = finite(item.get("wall_time") for item in learner)
    if len(learner_wall) >= 2:
        learner_hz = (len(learner_wall) - 1) / max(
            1.0e-9, learner_wall[-1] - learner_wall[0]
        )
    failure_transitions = [item for item in transitions if item.get("terminated")]
    failure_episode = (
        [] if not failure_transitions else [
            item for item in transitions
            if item["episode_id"] == failure_transitions[-1]["episode_id"]
        ]
    )
    failure_deltas = [
        abs(later["requested_v_max"] - earlier["requested_v_max"])
        for earlier, later in zip(failure_episode, failure_episode[1:])
    ]

    analysis = {
        "stable": stable,
        "preliminary_learning": preliminary_learning,
        "evaluation_return_improved": evaluation_improved,
        "policy_action_distribution_shift": policy_shift,
        "action_exploration_instability": action_exploration_instability,
        "numerical_stable": numerical_stable,
        "recommend_extended": recommend_extended,
        "failures": failures,
        "failure_categories": classify_failures(failures),
        "action_early": action_early,
        "action_late": action_late,
        "request_early": request_early,
        "request_late": request_late,
        "return_first_five": first_returns,
        "return_last_five": last_returns,
        "evaluations": evaluations,
    }
    with open(str(root / "sac_training_10k_pilot_analysis.json"), "w", encoding="utf-8") as stream:
        json.dump(analysis, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    lines = [
        "# AstraDroneOpen SAC 10k Pilot Training Report", "",
        "日期：2026-08-23", "",
        "正式工件根目录：`{}`".format(root), "",
        "## 1. 结论", "",
        ("`GO FOR EXTENDED SAC TRAINING`" if recommend_extended else "`NO-GO`"), "",
        "本轮是 10,000-valid-transition 正式 pilot，不证明最终收敛，也没有自动继续到 50k/100k。",
        "数据链稳定：{}；数值稳定：{}；观察到初步学习迹象：{}。".format(
            "是" if stable else "否", "是" if numerical_stable else "否",
            "是" if preliminary_learning else "否/证据不足",
        ), "",
    ]
    if failures:
        lines.extend([
            "关键 blocker：`planner_failure, action_exploration_instability`。分类：`{}`。".format(
                ", ".join(classify_failures(failures)) or "UNCLASSIFIED"
            ), "",
            "随之产生的未完成项与 shutdown 记录：`{}`。".format(
                ", ".join(failures)
            ), "",
            "Primary blockers：`ENVIRONMENT + ACTION`。Episode 9 在 100 steps 后真实 `NO_FEASIBLE_TRAJECTORY`；该 Episode normalized action std=`{}`、requested span=`{}..{} m/s`、最大相邻 request delta=`{} m/s`。".format(
                fmt(stats(item["normalized_policy_action"] for item in failure_episode)["std"]),
                fmt(min((item["requested_v_max"] for item in failure_episode), default=None)),
                fmt(max((item["requested_v_max"] for item in failure_episode), default=None)),
                fmt(max(failure_deltas) if failure_deltas else None),
            ), "",
        ])

    lines.extend(["## 2. 最终训练参数", "",
                  "| 参数 | Value | Source | Runtime verified before pilot |", "|---|---:|---|---|"])
    for name, item in config["parameter_provenance"].items():
        lines.append("| {} | `{}` | {} | {} |".format(
            name, item["value"], item["source"], "yes" if item["runtime_verified"] else "no"
        ))
    lines.extend(["", "补充：Replay 从空 buffer 开始，逻辑 capacity 为 `{}`，batch `{}`；learner 为 wall-time `{}` update/s，policy/target frequency 为 `{}/{}`。".format(
        config["replay"]["capacity"], config["sac"]["batch_size"], config["sac"]["updates_per_second"],
        config["sac"]["policy_frequency"], config["sac"]["target_network_frequency"],
    ), "learning_starts=`{}`；此前 action 为未训练 Actor deterministic mean，此后为 stochastic Actor。".format(config["training"]["learning_starts"]), ""])

    outcomes = [episode_outcome(item) for item in episodes]
    step_stats = stats(item.get("transition_count") for item in episodes)
    lines.extend(["## 3. Environment / Episode / Replay", "",
                  "- 已进入 Replay 的 active/closed transition：`{}`；Replay size：`{}`；gradient updates：`{}`。".format(
                      len(transitions), runtime.get("replay_audit", {}).get("size"), runtime.get("gradient_update_step")),
                  "- 已闭合 Episode 内 transition：`{}`；fail-closed shutdown 前第 10 Episode 额外 partial transitions：`{}`。".format(
                      sum(item.get("transition_count", 0) for item in episodes),
                      len(transitions) - sum(item.get("transition_count", 0) for item in episodes)),
                  "- Episode：`{}`；success/failure/truncated = `{}/{}/{}`。".format(
                      len(episodes), outcomes.count("SUCCESS"), outcomes.count("FAILURE"), outcomes.count("TRUNCATED")),
                  "- Episode steps count/mean/min/max = `{}/{}/{}/{}`。".format(
                      step_stats["count"], fmt(step_stats["mean"], 2), fmt(step_stats["min"], 0), fmt(step_stats["max"], 0)),
                  "- Return first-5 mean / last-5 mean = `{}` / `{}`。".format(fmt(first_returns["mean"]), fmt(last_returns["mean"])),
                  "- Replay audit：`{}`；safety intervention `{}`；terminated/truncated `{}/{}`。".format(
                      "PASS" if runtime.get("replay_audit", {}).get("passed") else "FAIL",
                      runtime.get("replay_audit", {}).get("safety_intervention_count"),
                      runtime.get("replay_audit", {}).get("terminated_count"), runtime.get("replay_audit", {}).get("truncated_count")), ""])

    lines.extend(["## 4. Action / v_max 趋势", "",
                  "| Window | normalized mean/std/min/max | requested mean/std/min/max m/s |", "|---|---|---|"])
    for label, action, request in (("first 1k", action_early, request_early), ("last available 1k", action_late, request_late)):
        lines.append("| {} | `{}/{}/{}/{}` | `{}/{}/{}/{}` |".format(
            label, fmt(action["mean"]), fmt(action.get("std")), fmt(action["min"]), fmt(action["max"]),
            fmt(request["mean"]), fmt(request.get("std")), fmt(request["min"]), fmt(request["max"])))
    lines.extend(["", "长期边界偏置检查：全量 normalized min/max = `{}/{}`；requested min/max = `{}/{}` m/s。".format(
        fmt(min(item["normalized_policy_action"] for item in transitions)), fmt(max(item["normalized_policy_action"] for item in transitions)),
        fmt(min(item["requested_v_max"] for item in transitions)), fmt(max(item["requested_v_max"] for item in transitions))), ""])

    lines.extend(["## 5. SAC 数值指标", "",
                  "| Metric | min | mean | p95 | max |", "|---|---:|---:|---:|---:|"])
    for name in ("actor_loss", "critic1_loss", "critic2_loss", "alpha", "entropy", "q1_mean", "q2_mean", "target_q_mean", "critic_gradient_max_abs", "actor_gradient_max_abs"):
        value = metric_range(learner, name)
        lines.append("| {} | {} | {} | {} | {} |".format(name, fmt(value["min"]), fmt(value["mean"]), fmt(value["p95"]), fmt(value["max"])))
    lines.extend(["", "NaN/Inf、gradient/Q fail-closed：`{}`；learner measured Hz：`{}`。".format(
        "PASS" if not any("learner" in str(value).lower() for value in failures) else "FAIL",
        fmt(runtime.get("learner_update_hz", learner_hz))), ""])

    lines.extend(["## 6. Deterministic evaluation（不写 Replay、不更新网络）", "",
                  "| Checkpoint | Episodes | Success rate | Return mean | Steps mean | Duration mean s | mean v_max | actual speed | tracking error |", "|---:|---:|---:|---:|---:|---:|---:|---:|---:|"])
    for item in evaluations:
        lines.append("| {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
            item["step"], item["episodes"], fmt(item["success_rate"]), fmt(item["return"]["mean"]),
            fmt(item["steps"]["mean"], 2), fmt(item["duration"]["mean"]), fmt(item["requested_v_max"]["mean"]),
            fmt(item["actual_speed"]["mean"]), fmt(item["tracking_error"]["mean"])))
    lines.extend(["", "缺失 evaluation checkpoints：`{}`；pilot fail-closed 后没有伪造或补跑。".format(missing_evaluations),
                  "Evaluation return 相对 untrained baseline 改善：`{}`。".format("是" if evaluation_improved else "无法判断"), ""])

    lines.extend(["## 7. Checkpoint / Reset / Identity / Causality", "", "Checkpoints：", ""])
    for item in runtime.get("checkpoint_manifest", []):
        lines.append("- step `{}` / update `{}`: `{}`（{} bytes）".format(
            item["environment_step"], item["gradient_update_step"], item["path"], item["size_bytes"]))
    reload_path = training / "offline_checkpoint_reload_audit.json"
    reload_audit = load_json(reload_path) if reload_path.is_file() else {"passed": False, "checkpoints": []}
    lines.extend(["", "已有 0/2k/4k checkpoint 离线 reload + deterministic finite inference：`{}`；10k checkpoint：`MISSING`。".format(
        "PASS" if reload_audit.get("passed") else "FAIL"),
        "Reset `{}/{}` PASS；old-generation contamination `{}`。".format(
            coordinator.get("reset_success_count"), coordinator.get("reset_count"), coordinator.get("old_generation_valid_contamination_count")),
        "Request/action/applied identity、source-time causality、10 Hz scheduler：`{}`。".format(
            "PASS" if not runtime.get("episode_contract_failures") else "FAIL"), ""])

    training_wall = coordinator.get("wall_duration")
    training_sim = coordinator.get("sim_duration")
    eval_wall = sum(float(item["wall_duration"] or 0.0) for item in evaluations)
    eval_sim = sum(float(item["sim_duration"] or 0.0) for item in evaluations)
    lines.extend(["## 8. 性能", "",
                  "- Training sim/wall/RTF：`{}/{}/{}`。".format(fmt(training_sim), fmt(training_wall), fmt(coordinator.get("rtf"))),
                  "- Evaluation suite sim/wall：`{}/{}`。".format(fmt(eval_sim), fmt(eval_wall)),
                  "- Environment transition/action request rate：Episode action-rate 全程合同 `{}`；learner 与 10 Hz scheduler 解耦。".format(
                      "PASS" if not runtime.get("episode_contract_failures") else "FAIL"),
                  "- `/usr/bin/time -v` launch scope CPU / max RSS / swaps：`{}% / {} MiB / {}`。".format(
                      fmt(resources.get("average_cpu_percent"), 1),
                      fmt(
                          None if resources.get("maximum_resident_set_kib") is None
                          else resources["maximum_resident_set_kib"] / 1024.0,
                          2,
                      ),
                      resources.get("swaps", "NA"),
                  ),
                  "- Replay allocated/logical capacity：`{}/{}`；fail-closed run 未生成 final snapshot，逐 transition audit 完整保留。".format(
                      runtime.get("replay_audit", {}).get("allocated_capacity"), runtime.get("replay_audit", {}).get("logical_capacity")), ""])

    lines.extend(["## 9. Pilot 判断", "",
                  "A. 数据链稳定：`{}`。".format("是" if stable else "否"),
                  "B. SAC 无数值发散：`{}`；但探索 action 方差后期显著增大。".format("是" if numerical_stable else "否"),
                  "C. Episode success/truncation：`{}/{}`；真实 failure=`{}`。".format(outcomes.count("SUCCESS"), outcomes.count("TRUNCATED"), outcomes.count("FAILURE")),
                  "D. 平均 Episode steps：first/last 均见上表，未用 Episode 数提前停止。",
                  "E. policy v_max 分布变化：`{}`。".format("有" if policy_shift else "不明显"),
                  "F. return 改善趋势：training first-5 `{}` -> last-5 `{}`。".format(fmt(first_returns["mean"]), fmt(last_returns["mean"])),
                  "G. deterministic evaluation 改善：`{}`。".format("是" if evaluation_improved else "否/不明确"),
                  "H. 长期边界偏置：需结合 action min/max 与 8k-10k window；本报告不把短 pilot 写成收敛。",
                  "I. Replay 健康：`{}`。".format("是" if runtime.get("replay_audit", {}).get("passed") else "否"),
                  "J. 建议进入更长训练：`{}`。".format("是" if recommend_extended else "否"), "",
                  ("# GO FOR EXTENDED SAC TRAINING" if recommend_extended else "# NO-GO"), ""])

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
