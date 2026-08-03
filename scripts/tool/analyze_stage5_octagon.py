#!/usr/bin/env python3
"""Analyze and plot comparable Stage-5 formal-orbit evidence.

The formal window for each aircraft starts at its first recorded orbit release and
ends before the first WAIT_EXIT_PERMISSION/GO_TO_EXIT_GATE sample.  The script is
deliberately evidence-only: it reads recorder CSV/log files and never touches ROS.
"""

import argparse
import csv
import json
import math
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


TOWER = (-10.0551, 19.7104)
COLORS = {1: "#1565c0", 2: "#e76f00", 3: "#2e8b57"}
WORLD_X_OFFSET = {1: 0.0, 2: 4.0, 3: 8.0}
EXIT_STATES = {"WAIT_EXIT_PERMISSION", "GO_TO_EXIT_GATE", "RETURN_HOME",
               "LANDING", "DONE"}


def finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def released_at(swarm_rows, uid):
    key = "u{}_orbit_released".format(uid)
    for row in swarm_rows:
        if row.get(key) == "1":
            return float(row["sim_time"])
    raise RuntimeError("UAV{} has no ORBIT_RELEASE sample".format(uid))


def formal_rows(rows, start):
    selected = []
    for row in rows:
        stamp = finite(row.get("sim_time"))
        if stamp is None or stamp < start:
            continue
        if row.get("state") in EXIT_STATES:
            break
        selected.append(row)
    if not selected:
        raise RuntimeError("empty formal orbit window")
    return selected


def smooth(values, width=7):
    values = np.asarray(values, dtype=float)
    half = width // 2
    result = np.empty_like(values)
    for index in range(len(values)):
        lo = max(0, index - half)
        hi = min(len(values), index + half + 1)
        result[index] = values[lo:hi].mean()
    return result


def world_xy(row, uid):
    return float(row["x"]) + WORLD_X_OFFSET[uid], float(row["y"])


def backtrack_events(rows, uid, threshold_deg=0.1):
    """Find excursions below the running peak in smoothed unwrapped CCW angle."""
    times = np.asarray([float(row["sim_time"]) for row in rows])
    coordinates = [world_xy(row, uid) for row in rows]
    raw_x = np.asarray([point[0] for point in coordinates])
    raw_y = np.asarray([point[1] for point in coordinates])
    x = smooth([point[0] for point in coordinates])
    y = smooth([point[1] for point in coordinates])
    angle = np.unwrap(np.arctan2(y - TOWER[1], x - TOWER[0]))
    angle = np.degrees(angle)
    events = []
    peak_index = 0
    peak_value = angle[0]
    active = None
    for index in range(1, len(angle)):
        if active is None:
            if angle[index] >= peak_value:
                peak_value = angle[index]
                peak_index = index
            elif peak_value - angle[index] >= threshold_deg:
                active = {"start": peak_index, "trough": index,
                          "peak": peak_value, "trough_value": angle[index]}
        else:
            if angle[index] < active["trough_value"]:
                active["trough"] = index
                active["trough_value"] = angle[index]
            if angle[index] >= active["peak"]:
                active["end"] = index
                events.append(active)
                peak_index = index
                peak_value = angle[index]
                active = None
    if active is not None:
        active["end"] = len(angle) - 1
        events.append(active)

    output = []
    for event in events:
        start, trough, end = event["start"], event["trough"], event["end"]
        # Report distance from the original 10 Hz trajectory while using the
        # smoothed angle only to classify reverse samples.
        reverse_distance = 0.0
        for index in range(start + 1, trough + 1):
            reverse_distance += math.hypot(
                raw_x[index] - raw_x[index - 1],
                raw_y[index] - raw_y[index - 1])
        output.append({
            "start": float(times[start]),
            "trough": float(times[trough]),
            "end": float(times[end]),
            "rollback_deg": float(event["peak"] - event["trough_value"]),
            "reverse_distance_m": float(reverse_distance),
        })
    return output


def intervals(rows, predicate):
    found = []
    active = None
    for row in rows:
        stamp = float(row["sim_time"])
        if predicate(row):
            if active is None:
                active = [stamp, stamp, row]
            else:
                active[1] = stamp
        elif active is not None:
            found.append(active)
            active = None
    if active is not None:
        found.append(active)
    return found


def waypoint_records(rows, uid):
    """Return the final commanded candidate for each of sectors 1..8."""
    selected = {}
    for row in rows:
        target_id = row.get("target_id", "")
        match = re.search(r"_s(\d+)_c(\d+)$", target_id)
        if not match:
            continue
        sector = int(match.group(1))
        if sector < 0 or sector > 7:
            continue
        x = finite(row.get("target_x"))
        y = finite(row.get("target_y"))
        if x is None or y is None or not target_id:
            continue
        x += WORLD_X_OFFSET[uid]
        angle = math.degrees(math.atan2(y - TOWER[1], x - TOWER[0])) % 360.0
        selected[sector] = {
            "waypoint": sector + 1,
            "sector": sector + 1,
            "sector_internal": sector,
            "candidate_id": target_id,
            "candidate_index": int(match.group(2)),
            "x": x,
            "y": y,
            "angle_deg": angle,
            "radius_m": math.hypot(x - TOWER[0], y - TOWER[1]),
        }
    return [selected[sector] for sector in range(8) if sector in selected]


def nearest_point(rows, stamp, uid):
    row = min(rows, key=lambda item: abs(float(item["sim_time"]) - stamp))
    return world_xy(row, uid)


def log_events(path, formal_by_uid):
    output = {uid: {"hold": [], "retry": [], "no_feasible": []}
              for uid in (1, 2, 3)}
    stamp_re = re.compile(r"\[\d+(?:\.\d+)?,\s*(\d+(?:\.\d+)?)\]")
    uav_re = re.compile(r"(?:/uav|uav(?:=|\s+))([123])", re.IGNORECASE)
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            stamp_match = stamp_re.search(line)
            uav_match = uav_re.search(line)
            if not stamp_match or not uav_match:
                continue
            stamp = float(stamp_match.group(1))
            uid = int(uav_match.group(1))
            rows = formal_by_uid[uid]
            if stamp < float(rows[0]["sim_time"]) or stamp > float(rows[-1]["sim_time"]):
                continue
            lowered = line.lower()
            kind = None
            if "no_feasible_trajectory" in lowered:
                kind = "no_feasible"
            elif "bounded retry" in lowered:
                kind = "retry"
            elif "hold requested" in lowered:
                kind = "hold"
            if kind:
                x, y = nearest_point(rows, stamp, uid)
                output[uid][kind].append({"time": stamp, "x": x, "y": y,
                                          "message": line.strip()[-280:]})
    return output


def summarize_run(directory):
    swarm = read_csv(directory / "swarm.csv")
    formal = {}
    summary = {"directory": str(directory), "uavs": {}}
    for uid in (1, 2, 3):
        rows = read_csv(directory / "uav{}.csv".format(uid))
        start = released_at(swarm, uid)
        selected = formal_rows(rows, start)
        formal[uid] = selected
        radii = np.asarray([
            math.hypot(world_xy(row, uid)[0] - TOWER[0],
                       world_xy(row, uid)[1] - TOWER[1])
            for row in selected
        ])
        holds = intervals(selected, lambda row: row.get("state") == "HOLDING")
        backtracks = backtrack_events(selected, uid)
        prior_wp1 = [row for row in rows
                     if float(row["sim_time"]) <= start and
                     row.get("mission_target_id") == "WP1" and
                     row.get("target_id", "").startswith("l0_s0_")]
        waypoint_source = ([prior_wp1[-1]] if prior_wp1 else []) + selected
        waypoints = waypoint_records(waypoint_source, uid)
        tier_counts = {"12.5": 0, "14.5": 0, "16.5": 0, "other": 0}
        for waypoint in waypoints:
            matched = False
            for radius in (12.5, 14.5, 16.5):
                if abs(waypoint["radius_m"] - radius) < 0.25:
                    tier_counts[str(radius)] += 1
                    matched = True
                    break
            if not matched:
                tier_counts["other"] += 1

        swarm_window = [row for row in swarm
                        if start <= float(row["sim_time"]) <= float(selected[-1]["sim_time"])]
        trajectory_ids = [int(float(row["u{}_trajectory_id".format(uid)]))
                          for row in swarm_window
                          if finite(row.get("u{}_trajectory_id".format(uid))) is not None]
        switches = sum(a != b for a, b in zip(trajectory_ids, trajectory_ids[1:]))
        scales = [float(row["u{}_orbit_speed_scale".format(uid)]) for row in swarm_window
                  if finite(row.get("u{}_orbit_speed_scale".format(uid))) is not None]
        summary["uavs"][str(uid)] = {
            "start": float(selected[0]["sim_time"]),
            "end": float(selected[-1]["sim_time"]),
            "sample_count": len(selected),
            "radius": {
                "min": float(radii.min()), "max": float(radii.max()),
                "mean": float(radii.mean()), "stddev": float(radii.std()),
                "within_12_5_pm_0_5_fraction": float(np.mean((radii >= 12.0) & (radii <= 13.0))),
            },
            "waypoints": waypoints,
            "tier_counts": tier_counts,
            "backtracks": backtracks,
            "backtrack_count": len(backtracks),
            "max_rollback_deg": max((item["rollback_deg"] for item in backtracks), default=0.0),
            "max_reverse_distance_m": max((item["reverse_distance_m"] for item in backtracks), default=0.0),
            "hold_intervals": [{"start": item[0], "end": item[1],
                                "duration": item[1] - item[0],
                                "reason": item[2].get("target_switch_reason", "")}
                               for item in holds],
            "trajectory_id_first": trajectory_ids[0] if trajectory_ids else None,
            "trajectory_id_last": trajectory_ids[-1] if trajectory_ids else None,
            "trajectory_id_switches": switches,
            "speed_scale": {"min": min(scales), "max": max(scales),
                            "mean": sum(scales) / len(scales)} if scales else None,
        }

    common_start = max(summary["uavs"][str(uid)]["start"] for uid in (1, 2, 3))
    common_end = min(summary["uavs"][str(uid)]["end"] for uid in (1, 2, 3))
    pair_minima = {}
    for pair in ("1-2", "1-3", "2-3"):
        key = pair + "_three_d"
        values = [finite(row.get(key)) for row in swarm
                  if common_start <= float(row["sim_time"]) <= common_end]
        values = [value for value in values if value is not None]
        pair_minima[pair] = min(values)
    summary["common_window"] = {"start": common_start, "end": common_end,
                                "pairwise_min_3d_m": pair_minima,
                                "minimum_3d_m": min(pair_minima.values())}
    summary["events"] = log_events(directory / "roslaunch.log", formal)
    return summary, formal


def axis_limit(formal_sets, summaries):
    extent = 0.0
    for formal in formal_sets:
        for uid, rows in formal.items():
            for row in rows:
                x, y = world_xy(row, uid)
                extent = max(extent, abs(x - TOWER[0]),
                             abs(y - TOWER[1]))
    for summary in summaries:
        for uav in summary["uavs"].values():
            for waypoint in uav["waypoints"]:
                extent = max(extent, abs(waypoint["x"] - TOWER[0]),
                             abs(waypoint["y"] - TOWER[1]))
    return math.ceil(extent + 1.0)


def setup_axis(axis, limit, title):
    axis.scatter(*TOWER, color="#222222", marker="+", s=100, linewidth=2,
                 label="tower centre")
    axis.set_xlim(TOWER[0] - limit, TOWER[0] + limit)
    axis.set_ylim(TOWER[1] - limit, TOWER[1] + limit)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.25)
    axis.set_xlabel("world X / East (m)")
    axis.set_ylabel("world Y / North (m)")
    axis.set_title(title)


def comparison_plot(before, after, before_formal, after_formal, output, limit):
    figure, axes = plt.subplots(1, 2, figsize=(14, 7), constrained_layout=True)
    for axis, title, formal in ((axes[0], "Before: first-point Tier fix baseline", before_formal),
                                (axes[1], "After: full-orbit hard Tier", after_formal)):
        setup_axis(axis, limit, title)
        for uid in (1, 2, 3):
            rows = formal[uid]
            coordinates = [world_xy(row, uid) for row in rows]
            axis.plot([point[0] for point in coordinates],
                      [point[1] for point in coordinates], color=COLORS[uid],
                      linewidth=1.15, label="UAV{}".format(uid))
        axis.legend(loc="upper right")
    figure.suptitle("Stage-5 formal orbit XY comparison (identical scale/window rules)")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(output), dpi=180)
    plt.close(figure)


def diagnostic_plots(summary, formal, output_dir, limit):
    figure, axes = plt.subplots(1, 3, figsize=(20, 7), constrained_layout=True)
    for uid, axis in zip((1, 2, 3), axes):
        rows = formal[uid]
        events = summary["events"][uid]
        coordinates = [world_xy(row, uid) for row in rows]
        setup_axis(axis, limit, "UAV{} formal orbit diagnostic".format(uid))
        axis.plot([point[0] for point in coordinates],
                  [point[1] for point in coordinates], color=COLORS[uid],
                  linewidth=1.25, label="actual XY")
        waypoints = summary["uavs"][str(uid)]["waypoints"]
        axis.plot([item["x"] for item in waypoints] + [waypoints[0]["x"]],
                  [item["y"] for item in waypoints] + [waypoints[0]["y"]],
                  color=COLORS[uid], linestyle="--", linewidth=0.9,
                  alpha=0.7, label="commanded octagon")
        for item in waypoints:
            axis.scatter(item["x"], item["y"], color=COLORS[uid], marker="D", s=35)
            axis.annotate("#{0} S{1}\n{2:.1f}° {3:.1f}m".format(
                item["waypoint"], item["sector"], item["angle_deg"], item["radius_m"]),
                (item["x"], item["y"]), xytext=(4, 4), textcoords="offset points",
                fontsize=7)
        hold_starts = [item["start"] for item in summary["uavs"][str(uid)]["hold_intervals"]]
        for stamp in hold_starts:
            x, y = nearest_point(rows, stamp, uid)
            axis.scatter(x, y, color="#b71c1c", marker="s", s=32,
                         label="HOLD start" if stamp == hold_starts[0] else None)
        for kind, marker, color, label in (("retry", "^", "#8e24aa", "bounded retry"),
                                            ("no_feasible", "x", "#d32f2f", "NO_FEASIBLE")):
            for index, event in enumerate(events[kind]):
                axis.scatter(event["x"], event["y"], color=color, marker=marker, s=35,
                             label=label if index == 0 else None)
        axis.legend(loc="upper right", fontsize=8)

        single = plt.figure(figsize=(8, 8), constrained_layout=True)
        single_axis = single.add_subplot(111)
        # Copying artists is unsafe; draw the same content explicitly.
        setup_axis(single_axis, limit, "UAV{} formal orbit diagnostic".format(uid))
        single_axis.plot([point[0] for point in coordinates],
                         [point[1] for point in coordinates], color=COLORS[uid],
                         linewidth=1.25, label="actual XY")
        single_axis.plot([item["x"] for item in waypoints] + [waypoints[0]["x"]],
                         [item["y"] for item in waypoints] + [waypoints[0]["y"]],
                         color=COLORS[uid], linestyle="--", linewidth=0.9,
                         alpha=0.7, label="commanded octagon")
        for item in waypoints:
            single_axis.scatter(item["x"], item["y"], color=COLORS[uid], marker="D", s=38)
            single_axis.annotate("#{0} S{1}  {2:.1f}°  {3:.1f}m\n{4}".format(
                item["waypoint"], item["sector"], item["angle_deg"],
                item["radius_m"], item["candidate_id"]),
                (item["x"], item["y"]), xytext=(5, 5), textcoords="offset points",
                fontsize=8)
        for index, stamp in enumerate(hold_starts):
            x, y = nearest_point(rows, stamp, uid)
            single_axis.scatter(x, y, color="#b71c1c", marker="s", s=35,
                                label="HOLD start" if index == 0 else None)
        for kind, marker, color, label in (("retry", "^", "#8e24aa", "bounded retry"),
                                            ("no_feasible", "x", "#d32f2f", "NO_FEASIBLE")):
            for index, event in enumerate(events[kind]):
                single_axis.scatter(event["x"], event["y"], color=color, marker=marker,
                                    s=38, label=label if index == 0 else None)
        single_axis.legend(loc="upper right", fontsize=8)
        single.savefig(str(output_dir / "uav{}_octagon_diagnostic.png".format(uid)), dpi=180)
        plt.close(single)
    figure.suptitle("Stage-5 commanded waypoints, HOLD and replanning events (identical scale)")
    figure.savefig(str(output_dir / "three_uav_octagon_diagnostics.png"), dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    before, before_formal = summarize_run(args.before)
    after, after_formal = summarize_run(args.after)
    limit = axis_limit((before_formal, after_formal), (before, after))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    comparison_plot(before, after, before_formal, after_formal,
                    args.output_dir / "before_after_xy_same_scale.png", limit)
    diagnostic_plots(after, after_formal, args.output_dir, limit)
    payload = {"coordinate_limit_from_tower_m": limit,
               "before": before, "after": after}
    with (args.output_dir / "octagon_analysis.json").open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
