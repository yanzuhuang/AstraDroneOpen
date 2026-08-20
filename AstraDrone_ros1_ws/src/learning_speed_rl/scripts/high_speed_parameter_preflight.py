#!/usr/bin/env python3
"""Offline preflight for the reviewed fixed-speed high-speed parameter chain."""

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


QUALIFICATION_SPEEDS = (1.75, 2.0, 2.5, 3.0, 3.5)
EXPECTED_CEILING = 4.0
EXPECTED_MAX_ACC = 3.0
EXPECTED_HORIZON = 7.5
EXPECTED_FEASIBILITY_TOLERANCE = 0.0
EPSILON = 1.0e-9


def _arg_default(path, name):
    root = ET.parse(str(path)).getroot()
    for element in root.findall("arg"):
        if element.get("name") == name:
            return element.get("default")
    raise ValueError("missing launch arg {} in {}".format(name, path))


def _check(results, name, condition, actual=None, expected=None):
    results.append({
        "name": name,
        "pass": bool(condition),
        "actual": actual,
        "expected": expected,
    })


def _load_px4_ulog(path):
    try:
        from pyulog import ULog
    except ImportError as error:
        raise RuntimeError("pyulog is required when --px4-ulog is used") from error
    parameters = ULog(str(path)).initial_parameters
    keys = (
        "MPC_XY_VEL_MAX", "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN",
        "MPC_ACC_HOR_MAX", "MPC_ACC_UP_MAX", "MPC_ACC_DOWN_MAX",
    )
    missing = [key for key in keys if key not in parameters]
    if missing:
        raise ValueError("PX4 ULog is missing parameters: {}".format(", ".join(missing)))
    return {key: float(parameters[key]) for key in keys}


def run_preflight(repo_root, requested_speeds, speed_ceiling, max_acc,
                  planning_horizon, bridge_config, px4_ulog=None,
                  feasibility_tolerance=EXPECTED_FEASIBILITY_TOLERANCE):
    repo_root = Path(repo_root).resolve()
    bridge_config = Path(bridge_config).resolve()
    requested_speeds = tuple(float(value) for value in requested_speeds)
    results = []
    warnings = []

    manual_launch = repo_root / (
        "AstraDrone_ros1_ws/src/learning_speed_rl/launch/"
        "manual_fixed_speed_calibration_uav1.launch")
    stack_launch = repo_root / (
        "AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/launch/"
        "uav_tower_stack.launch")
    bridge_launch = repo_root / (
        "AstraDrone_ros1_ws/src/MissionControl/ego_gazebo_bridge/launch/"
        "ego_gazebo_bridge.launch")
    advanced_launch = repo_root / (
        "AstraDrone_ros1_ws/src/Planner/ego-planner/planner/plan_manage/launch/"
        "advanced_param.xml")
    runner = repo_root / "scripts/run_sh/learning_speed_manual_run.sh"

    _check(results, "reviewed_static_ceiling", math.isclose(speed_ceiling, EXPECTED_CEILING),
           speed_ceiling, EXPECTED_CEILING)
    _check(results, "reviewed_max_acc", math.isclose(max_acc, EXPECTED_MAX_ACC),
           max_acc, EXPECTED_MAX_ACC)
    _check(results, "reviewed_planning_horizon",
           math.isclose(planning_horizon, EXPECTED_HORIZON),
           planning_horizon, EXPECTED_HORIZON)
    _check(results, "reviewed_feasibility_tolerance",
           math.isclose(feasibility_tolerance, EXPECTED_FEASIBILITY_TOLERANCE),
           feasibility_tolerance, EXPECTED_FEASIBILITY_TOLERANCE)
    _check(results, "requested_speed_set_nonempty", bool(requested_speeds),
           list(requested_speeds), list(QUALIFICATION_SPEEDS))
    for speed in requested_speeds:
        _check(results, "request_{:.2f}_within_ceiling".format(speed),
               speed > 0.0 and speed <= speed_ceiling + EPSILON,
               speed, "(0, {}]".format(speed_ceiling))

    launch_defaults = {
        "speed_ceiling": float(_arg_default(manual_launch, "speed_ceiling")),
        "max_acc": float(_arg_default(manual_launch, "max_acc")),
        "planning_horizon": float(_arg_default(manual_launch, "planning_horizon")),
        "feasibility_tolerance": float(
            _arg_default(manual_launch, "feasibility_tolerance")),
    }
    _check(results, "manual_launch_ceiling", math.isclose(
        launch_defaults["speed_ceiling"], speed_ceiling),
        launch_defaults["speed_ceiling"], speed_ceiling)
    _check(results, "manual_launch_max_acc", math.isclose(
        launch_defaults["max_acc"], max_acc), launch_defaults["max_acc"], max_acc)
    _check(results, "manual_launch_horizon", math.isclose(
        launch_defaults["planning_horizon"], planning_horizon),
        launch_defaults["planning_horizon"], planning_horizon)
    _check(results, "manual_launch_feasibility_tolerance", math.isclose(
        launch_defaults["feasibility_tolerance"], feasibility_tolerance),
        launch_defaults["feasibility_tolerance"], feasibility_tolerance)

    runner_text = runner.read_text(encoding="utf-8")
    for speed in QUALIFICATION_SPEEDS:
        token = "{:.2f}".format(speed)
        _check(results, "runner_allowlist_{}".format(token.replace(".", "p")),
               token in runner_text, token, "present")
    _check(results, "runner_high_speed_provenance",
           "high_speed_qualification" in runner_text,
           "high_speed_qualification" in runner_text, True)

    stack_text = stack_launch.read_text(encoding="utf-8")
    bridge_launch_text = bridge_launch.read_text(encoding="utf-8")
    advanced_text = advanced_launch.read_text(encoding="utf-8")
    _check(results, "filter_ceiling_propagates_from_max_vel",
           'name="safety/v_max_max" value="$(arg max_vel)"' in stack_text,
           None, "uav stack max_vel")
    _check(results, "dynamic_gate_enabled_with_learning_speed",
           'name="dynamic_speed_limit_enabled"' in stack_text and
           'value="$(arg learning_speed_enabled)"' in stack_text,
           None, "learning_speed_enabled")
    _check(results, "bridge_passes_static_max_vel_to_ego",
           'name="max_vel" value="$(arg max_vel)"' in bridge_launch_text,
           None, "max_vel passthrough")
    _check(results, "ego_manager_ceiling_from_max_vel",
           'name="manager/max_vel" value="$(arg max_vel)"' in advanced_text,
           None, "max_vel passthrough")
    _check(results, "ego_optimizer_ceiling_from_max_vel",
           'name="optimization/max_vel" value="$(arg max_vel)"' in advanced_text,
           None, "max_vel passthrough")
    _check(results, "ego_manager_acc_from_max_acc",
           'name="manager/max_acc" value="$(arg max_acc)"' in advanced_text,
           None, "max_acc passthrough")
    _check(results, "ego_optimizer_acc_from_max_acc",
           'name="optimization/max_acc" value="$(arg max_acc)"' in advanced_text,
           None, "max_acc passthrough")

    with bridge_config.open(encoding="utf-8") as stream:
        bridge = yaml.safe_load(stream)
    bridge_velocity = float(bridge["max_velocity"])
    bridge_acceleration = float(bridge["max_acceleration"])
    feasibility_epsilon = 1.0e-4
    required_bridge_velocity = math.sqrt(3.0) * (
        speed_ceiling * (1.0 + feasibility_tolerance) + feasibility_epsilon)
    required_bridge_acceleration = math.sqrt(3.0) * (
        max_acc * (1.0 + feasibility_tolerance) + feasibility_epsilon)
    _check(results, "bridge_velocity_norm_covers_ego_xyz_box",
           bridge_velocity + EPSILON >= required_bridge_velocity,
           bridge_velocity, required_bridge_velocity)
    _check(results, "bridge_acceleration_norm_covers_ego_xyz_box",
           bridge_acceleration + EPSILON >= required_bridge_acceleration,
           bridge_acceleration, required_bridge_acceleration)

    reaction_budget = (0.2, 0.3)
    stopping_at_ceiling = speed_ceiling * speed_ceiling / (2.0 * max_acc)
    required_horizon = tuple(
        stopping_at_ceiling + speed_ceiling * delay for delay in reaction_budget)
    _check(results, "horizon_covers_ceiling_stop_budget",
           planning_horizon > max(required_horizon), planning_horizon,
           "> {:.3f}".format(max(required_horizon)))

    px4 = None
    if px4_ulog is not None:
        px4 = _load_px4_ulog(Path(px4_ulog).resolve())
        required_xy_velocity = math.sqrt(2.0) * speed_ceiling
        required_xy_acceleration = math.sqrt(2.0) * max_acc
        _check(results, "px4_xy_velocity_covers_ego_xy_box",
               px4["MPC_XY_VEL_MAX"] + EPSILON >= required_xy_velocity,
               px4["MPC_XY_VEL_MAX"], required_xy_velocity)
        _check(results, "px4_xy_acceleration_covers_ego_xy_box",
               px4["MPC_ACC_HOR_MAX"] + EPSILON >= required_xy_acceleration,
               px4["MPC_ACC_HOR_MAX"], required_xy_acceleration)
        _check(results, "px4_up_acceleration_covers_ego_axis",
               px4["MPC_ACC_UP_MAX"] + EPSILON >= max_acc,
               px4["MPC_ACC_UP_MAX"], max_acc)
        _check(results, "px4_down_acceleration_covers_ego_axis",
               px4["MPC_ACC_DOWN_MAX"] + EPSILON >= max_acc,
               px4["MPC_ACC_DOWN_MAX"], max_acc)
        if (px4["MPC_Z_VEL_MAX_UP"] < speed_ceiling or
                px4["MPC_Z_VEL_MAX_DN"] < speed_ceiling):
            warnings.append(
                "PX4 vertical velocity limits do not cover the full EGO per-axis "
                "4.0 m/s box; qualification requests are horizontal ceilings, "
                "not a validated 4.0 m/s vertical-flight contract.")
    else:
        warnings.append(
            "PX4 limits were not re-read live; supply --px4-ulog for offline "
            "evidence and recheck live MPC parameters before flight.")

    return {
        "schema_version": "high_speed_parameter_chain_preflight_v1.0",
        "pass": all(item["pass"] for item in results),
        "requested_v_max_mps": list(requested_speeds),
        "speed_ceiling_mps": speed_ceiling,
        "max_acc_mps2": max_acc,
        "planning_horizon_m": planning_horizon,
        "feasibility_tolerance": feasibility_tolerance,
        "bridge_config": str(bridge_config),
        "bridge_velocity_norm_mps": bridge_velocity,
        "bridge_acceleration_norm_mps2": bridge_acceleration,
        "required_horizon_at_ceiling_m": {
            "reaction_0p2_s": required_horizon[0],
            "reaction_0p3_s": required_horizon[1],
        },
        "px4_parameters": px4,
        "checks": results,
        "warnings": warnings,
        "flight_or_simulation_started": False,
    }


def main():
    default_root = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", default=str(default_root))
    parser.add_argument("--requested-v-max", type=float, action="append")
    parser.add_argument("--speed-ceiling", type=float, default=EXPECTED_CEILING)
    parser.add_argument("--max-acc", type=float, default=EXPECTED_MAX_ACC)
    parser.add_argument("--planning-horizon", type=float, default=EXPECTED_HORIZON)
    parser.add_argument("--feasibility-tolerance", type=float,
                        default=EXPECTED_FEASIBILITY_TOLERANCE)
    parser.add_argument("--bridge-config")
    parser.add_argument("--px4-ulog")
    parser.add_argument("--output")
    args = parser.parse_args()
    repo_root = Path(args.repo_root).resolve()
    bridge_config = args.bridge_config or str(repo_root / (
        "AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/config/"
        "uav1_high_speed_bridge.yaml"))
    result = run_preflight(
        repo_root,
        args.requested_v_max or QUALIFICATION_SPEEDS,
        args.speed_ceiling,
        args.max_acc,
        args.planning_horizon,
        bridge_config,
        args.px4_ulog,
        args.feasibility_tolerance,
    )
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        if output.exists():
            raise SystemExit("refusing to overwrite existing preflight: {}".format(output))
        output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
