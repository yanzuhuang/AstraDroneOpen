#!/usr/bin/env python3
"""Plot two recorded tower-inspection trajectories in one world/ENU frame.

This is deliberately a thin dual-vehicle layer over plot_stage3_trajectory.py:
the proven single-vehicle CSV reader, phase classifier, and equal-metric 3-D
projection are reused instead of maintaining a second plotting implementation.
"""

import argparse
import csv
import hashlib
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from plot_stage3_trajectory import equal_metric_limits, read_csv


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SWARM_LAUNCH = (
    REPO_ROOT
    / "AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/launch"
    / "dual_tower_inspection.launch"
)
DEFAULT_BRIDGE_LAUNCH = (
    REPO_ROOT
    / "AstraDrone_ros1_ws/src/MissionControl/ego_gazebo_bridge/launch"
    / "ego_gazebo_bridge.launch"
)
DEFAULT_STACK_LAUNCH = (
    REPO_ROOT
    / "AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/launch"
    / "uav_tower_stack.launch"
)
COLORS = {"uav1": "#1565c0", "uav2": "#e76f00"}


class DataContractError(RuntimeError):
    """Raised when a truthful common-frame plot cannot be produced."""


@dataclass
class RigidTransform:
    parent: str
    child: str
    translation: np.ndarray
    quaternion_xyzw: np.ndarray
    source: Path

    def rotation_matrix(self):
        x, y, z, w = self.quaternion_xyzw
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not math.isfinite(norm) or norm < 1e-12:
            raise DataContractError(
                "Invalid quaternion in transform {} -> {}".format(
                    self.parent, self.child
                )
            )
        x, y, z, w = (value / norm for value in (x, y, z, w))
        return np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            dtype=np.float64,
        )

    def apply(self, points):
        return points @ self.rotation_matrix().T + self.translation

    def description(self):
        translation = ", ".join(
            "{:.6g}".format(value) for value in self.translation
        )
        quaternion = ", ".join(
            "{:.6g}".format(value) for value in self.quaternion_xyzw
        )
        return (
            "{} -> {}: t=({}), q_xyzw=({})".format(
                self.parent, self.child, translation, quaternion
            )
        )


@dataclass
class VehicleContract:
    name: str
    csv_path: Path
    odom_topic: str
    local_frame: str
    map_frame: str
    transform: RigidTransform
    local_tower_xy: np.ndarray


@dataclass
class VehicleData:
    contract: VehicleContract
    samples: list
    raw_rows: int
    finite_rows: int
    duplicate_rows: int
    local_xyz: np.ndarray
    world_xyz: np.ndarray
    times: np.ndarray
    continuity_runs: list


def _include_arguments(include):
    return {
        argument.get("name"): argument.get("value")
        for argument in include.findall("arg")
    }


def _resolve_launch_arg(value, launch_root):
    prefix = "$(arg "
    if not value.startswith(prefix) or not value.endswith(")"):
        return value
    name = value[len(prefix):-1]
    matches = [
        argument.get("default")
        for argument in launch_root.findall("arg")
        if argument.get("name") == name
    ]
    if len(matches) != 1 or matches[0] is None:
        raise DataContractError(
            "Could not resolve launch argument {!r}".format(name)
        )
    return matches[0]


def _parse_static_transform(node, source):
    tokens = (node.get("args") or "").split()
    if len(tokens) != 9:
        raise DataContractError(
            "Static TF node {!r} in {} does not contain 9 literal fields".format(
                node.get("name"), source
            )
        )
    try:
        numbers = np.array([float(value) for value in tokens[:7]])
    except ValueError as exc:
        raise DataContractError(
            "Static TF node {!r} in {} is not numeric".format(
                node.get("name"), source
            )
        ) from exc
    if not np.isfinite(numbers).all():
        raise DataContractError("Static TF contains non-finite values")
    return RigidTransform(
        parent=tokens[7],
        child=tokens[8],
        translation=numbers[:3],
        quaternion_xyzw=numbers[3:],
        source=source,
    )


def _validate_identity_map_to_planning(bridge_launch):
    root = ET.parse(str(bridge_launch)).getroot()
    candidates = [
        node
        for node in root.findall(".//node")
        if node.get("name") == "verified_map_to_planning_frame"
    ]
    if len(candidates) != 1:
        raise DataContractError(
            "Could not uniquely verify the map-to-planning-frame TF in {}".format(
                bridge_launch
            )
        )
    compact = " ".join((candidates[0].get("args") or "").split())
    if not compact.startswith("0 0 0 0 0 0 1 "):
        raise DataContractError(
            "map-to-planning-frame TF is not the audited identity transform"
        )
    if "$(arg mavros_frame)" not in compact:
        raise DataContractError("Identity TF parent is not mavros_frame")
    if "$(arg planning_frame)" not in compact:
        raise DataContractError("Identity TF child is not planning_frame")


def load_contracts(swarm_launch, bridge_launch, stack_launch):
    if not swarm_launch.is_file():
        raise DataContractError("Swarm launch file not found: {}".format(swarm_launch))
    if not bridge_launch.is_file():
        raise DataContractError(
            "Bridge launch file not found: {}".format(bridge_launch)
        )
    if not stack_launch.is_file():
        raise DataContractError("UAV stack launch not found: {}".format(stack_launch))

    _validate_identity_map_to_planning(bridge_launch)
    swarm_root = ET.parse(str(swarm_launch)).getroot()
    stack_root = ET.parse(str(stack_launch)).getroot()
    tower_y_args = [
        argument
        for argument in stack_root.findall("arg")
        if argument.get("name") == "tower_center_y"
    ]
    if len(tower_y_args) != 1:
        raise DataContractError("Could not verify the local tower Y coordinate")
    tower_y = float(tower_y_args[0].get("default"))

    transforms = {}
    for name in ("uav1", "uav2"):
        nodes = [
            node
            for node in swarm_root.findall(".//node")
            if node.get("name") == "world_to_{}_map".format(name)
        ]
        if len(nodes) != 1:
            raise DataContractError(
                "Could not uniquely resolve world_to_{}_map".format(name)
            )
        transform = _parse_static_transform(nodes[0], swarm_launch)
        if transform.parent != "world" or transform.child != "{}/map".format(name):
            raise DataContractError(
                "Unexpected frame chain for {}: {}".format(
                    name, transform.description()
                )
            )
        transforms[name] = transform

    includes = {}
    for include in swarm_root.findall(".//include"):
        if not (include.get("file") or "").endswith("uav_tower_stack.launch"):
            continue
        arguments = _include_arguments(include)
        name = arguments.get("uav_ns")
        if name in ("uav1", "uav2"):
            if name in includes:
                raise DataContractError(
                    "More than one mission stack is configured for {}".format(name)
                )
            includes[name] = arguments
    if set(includes) != {"uav1", "uav2"}:
        raise DataContractError(
            "The swarm launch does not define exactly one UAV1 and one UAV2 stack"
        )

    contracts = {}
    for name in ("uav1", "uav2"):
        arguments = includes[name]
        try:
            report_file = Path(
                _resolve_launch_arg(arguments["report_file"], swarm_root)
            ).resolve()
            local_tower_xy = np.array(
                [float(arguments["tower_center_x"]), tower_y], dtype=np.float64
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise DataContractError(
                "Incomplete report/tower contract for {}".format(name)
            ) from exc
        contracts[name] = VehicleContract(
            name=name,
            csv_path=report_file,
            odom_topic="/{}/Odometry".format(name),
            local_frame="{}/camera_init".format(name),
            map_frame="{}/map".format(name),
            transform=transforms[name],
            local_tower_xy=local_tower_xy,
        )

    if contracts["uav1"].odom_topic == contracts["uav2"].odom_topic:
        raise DataContractError("UAV1 and UAV2 resolve to the same odometry topic")
    if contracts["uav1"].csv_path == contracts["uav2"].csv_path:
        raise DataContractError("UAV1 and UAV2 resolve to the same report file")

    world_towers = {}
    for name, contract in contracts.items():
        local_tower = np.array(
            [[contract.local_tower_xy[0], contract.local_tower_xy[1], 0.0]]
        )
        world_towers[name] = contract.transform.apply(local_tower)[0, :2]
    if not np.allclose(
        world_towers["uav1"], world_towers["uav2"], atol=1e-6, rtol=0.0
    ):
        raise DataContractError(
            "The two local tower centres do not transform to one world point: "
            "UAV1={}, UAV2={}".format(
                world_towers["uav1"].tolist(), world_towers["uav2"].tolist()
            )
        )
    return contracts, world_towers["uav1"]


def raw_row_count(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return sum(1 for _ in csv.DictReader(stream))


def clean_samples(samples):
    finite_rows = len(samples)
    ordered = sorted(samples, key=lambda sample: sample["time"])
    by_time = {}
    for sample in ordered:
        # Keep the last complete record for an exact duplicate sim timestamp.
        by_time[sample["time"]] = sample
    cleaned = [by_time[stamp] for stamp in sorted(by_time)]
    if len(cleaned) < 2:
        raise DataContractError("Fewer than two unique finite samples remain")
    return cleaned, finite_rows, finite_rows - len(cleaned)


def continuity_runs(times, xyz):
    if len(times) < 2:
        return [(0, len(times))]
    delta_t = np.diff(times)
    positive = delta_t[delta_t > 0.0]
    if len(positive) == 0:
        raise DataContractError("No increasing timestamps remain after cleaning")
    typical_dt = float(np.median(positive))
    time_gap_limit = max(1.0, 5.0 * typical_dt)
    distance = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    speed = np.divide(
        distance,
        delta_t,
        out=np.full_like(distance, np.inf),
        where=delta_t > 0.0,
    )
    # Stage 3 is configured for 0.30 m/s. A >1 m jump implying >8 m/s is a
    # frame/reset discontinuity, not a line that should be drawn.
    split_after = np.flatnonzero(
        (delta_t > time_gap_limit) | ((distance > 1.0) & (speed > 8.0))
    )
    runs = []
    start = 0
    for index in split_after:
        end = int(index) + 1
        if end - start >= 1:
            runs.append((start, end))
        start = end
    if len(times) - start >= 1:
        runs.append((start, len(times)))
    return runs


def load_vehicle(contract, override_path=None):
    path = (override_path or contract.csv_path).resolve()
    if path != contract.csv_path:
        raise DataContractError(
            "{} input {} is not the report path bound to {} in {}".format(
                contract.name.upper(),
                path,
                contract.odom_topic,
                contract.transform.source,
            )
        )
    if not path.is_file():
        raise DataContractError("{} input file not found: {}".format(
            contract.name.upper(), path
        ))
    raw_rows = raw_row_count(path)
    parsed = read_csv(path)
    samples, finite_rows, duplicate_rows = clean_samples(parsed)
    local_xyz = np.array(
        [[sample["x"], sample["y"], sample["z"]] for sample in samples],
        dtype=np.float64,
    )
    world_xyz = contract.transform.apply(local_xyz)
    times = np.array([sample["time"] for sample in samples], dtype=np.float64)
    return VehicleData(
        contract=contract,
        samples=samples,
        raw_rows=raw_rows,
        finite_rows=finite_rows,
        duplicate_rows=duplicate_rows,
        local_xyz=local_xyz,
        world_xyz=world_xyz,
        times=times,
        continuity_runs=continuity_runs(times, world_xyz),
    )


def file_digest(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def reject_duplicate_inputs(uav1, uav2):
    path1 = uav1.contract.csv_path
    path2 = uav2.contract.csv_path
    if path1.samefile(path2):
        raise DataContractError("UAV1 and UAV2 are the same filesystem object")
    if file_digest(path1) == file_digest(path2):
        raise DataContractError("UAV1 and UAV2 CSV files are byte-for-byte identical")

    common_times = sorted(set(uav1.times).intersection(uav2.times))
    if len(common_times) < 2:
        return
    lookup1 = {
        sample["time"]: (sample["x"], sample["y"], sample["z"])
        for sample in uav1.samples
    }
    lookup2 = {
        sample["time"]: (sample["x"], sample["y"], sample["z"])
        for sample in uav2.samples
    }
    same = sum(
        np.allclose(lookup1[stamp], lookup2[stamp], atol=1e-9, rtol=0.0)
        for stamp in common_times
    )
    overlap_fraction = len(common_times) / min(len(uav1.times), len(uav2.times))
    if overlap_fraction > 0.95 and same == len(common_times):
        raise DataContractError(
            "UAV1 and UAV2 contain the same local coordinates at all common "
            "timestamps; the same odometry topic was probably recorded twice"
        )


def first_airborne_time(data):
    baseline = float(np.median(data.world_xyz[: min(10, len(data.world_xyz)), 2]))
    indices = np.flatnonzero(data.world_xyz[:, 2] > baseline + 0.5)
    return float(data.times[indices[0]]) if len(indices) else None


def state_counts(samples):
    counts = {}
    for sample in samples:
        state = sample["state"] or "<empty>"
        counts[state] = counts.get(state, 0) + 1
    return counts


def print_summary(data):
    name = data.contract.name.upper()
    invalid_rows = data.raw_rows - data.finite_rows
    print("=== {} data summary ===".format(name))
    print("input: {}".format(data.contract.csv_path))
    print("source topic (launch contract): {}".format(data.contract.odom_topic))
    print("time field: sim_time (shared Gazebo /clock)")
    print(
        "coordinate fields: x, y, z ({}, local ENU)".format(
            data.contract.local_frame
        )
    )
    print(
        "samples: raw={}, valid_unique={}, nonfinite_or_invalid_removed={}, "
        "duplicate_timestamps_removed={}".format(
            data.raw_rows,
            len(data.samples),
            invalid_rows,
            data.duplicate_rows,
        )
    )
    print(
        "time range: {:.6f} .. {:.6f} s".format(
            data.times[0], data.times[-1]
        )
    )
    print(
        "local start/end: {} -> {}".format(
            np.array2string(data.local_xyz[0], precision=4),
            np.array2string(data.local_xyz[-1], precision=4),
        )
    )
    print(
        "world start/end: {} -> {}".format(
            np.array2string(data.world_xyz[0], precision=4),
            np.array2string(data.world_xyz[-1], precision=4),
        )
    )
    print(
        "coordinate transform: YES; {} (source: {})".format(
            data.contract.transform.description(),
            data.contract.transform.source,
        )
    )
    print("continuous plot runs: {} (breaks={})".format(
        len(data.continuity_runs), max(0, len(data.continuity_runs) - 1)
    ))
    print("recorded state evidence: {}".format(state_counts(data.samples)))
    print("common world/ENU frame: YES")


def _plot_runs(axis, data, values, color, label, dimensions):
    first = True
    for start, end in data.continuity_runs:
        if end - start < 2:
            continue
        if dimensions == 3:
            axis.plot(
                values[start:end, 0],
                values[start:end, 1],
                values[start:end, 2],
                color=color,
                linewidth=1.7,
                alpha=0.95,
                label=label if first else None,
            )
        else:
            axis.plot(
                values[0][start:end],
                values[1][start:end],
                color=color,
                linewidth=1.35,
                alpha=0.95,
                label=label if first else None,
            )
        first = False


def plot(uav1, uav2, tower_xy, output, latest_output):
    shared_t0 = min(uav1.times[0], uav2.times[0])
    # One shared origin is intentional: never independently zero the vehicles.
    elapsed1 = uav1.times - shared_t0
    elapsed2 = uav2.times - shared_t0

    figure = plt.figure(figsize=(16, 8.5), constrained_layout=True)
    trajectory = figure.add_subplot(1, 2, 1, projection="3d")
    _plot_runs(
        trajectory,
        uav1,
        uav1.world_xyz,
        COLORS["uav1"],
        "UAV1 actual",
        3,
    )
    _plot_runs(
        trajectory,
        uav2,
        uav2.world_xyz,
        COLORS["uav2"],
        "UAV2 actual",
        3,
    )

    for data in (uav1, uav2):
        name = data.contract.name.upper()
        color = COLORS[data.contract.name]
        trajectory.scatter(
            *data.world_xyz[0],
            color=color,
            s=62,
            marker="o",
            edgecolor="white",
            linewidth=0.9,
            label="{} start".format(name),
            zorder=5,
        )
        trajectory.scatter(
            *data.world_xyz[-1],
            color=color,
            s=78,
            marker="X",
            edgecolor="white",
            linewidth=0.8,
            label="{} end".format(name),
            zorder=6,
        )

    maximum_z = max(
        36.0, float(uav1.world_xyz[:, 2].max()),
        float(uav2.world_xyz[:, 2].max())
    )
    trajectory.plot(
        [tower_xy[0], tower_xy[0]],
        [tower_xy[1], tower_xy[1]],
        [0.0, maximum_z],
        color="#555555",
        linestyle="--",
        linewidth=1.15,
        label="Tower centre axis",
    )
    all_xyz = np.vstack(
        (
            uav1.world_xyz,
            uav2.world_xyz,
            np.array(
                [
                    [tower_xy[0], tower_xy[1], 0.0],
                    [tower_xy[0], tower_xy[1], maximum_z],
                ]
            ),
        )
    )
    equal_metric_limits(
        trajectory, all_xyz[:, 0], all_xyz[:, 1], all_xyz[:, 2]
    )
    trajectory.view_init(elev=27.0, azim=-57.0)
    trajectory.set_xlabel("X / East (m)", labelpad=8)
    trajectory.set_ylabel("Y / North (m)", labelpad=8)
    trajectory.set_zlabel("Z / Up (m)", labelpad=7)
    trajectory.set_title(
        "Dual-UAV actual 3D trajectories\nworld / ENU"
    )
    trajectory.legend(loc="upper left", fontsize=8)

    altitude = figure.add_subplot(1, 2, 2)
    _plot_runs(
        altitude,
        uav1,
        (elapsed1, uav1.world_xyz[:, 2]),
        COLORS["uav1"],
        "UAV1 actual altitude",
        2,
    )
    _plot_runs(
        altitude,
        uav2,
        (elapsed2, uav2.world_xyz[:, 2]),
        COLORS["uav2"],
        "UAV2 actual altitude",
        2,
    )

    takeoff_times = {
        "uav1": first_airborne_time(uav1),
        "uav2": first_airborne_time(uav2),
    }
    for name, stamp in takeoff_times.items():
        if stamp is None:
            continue
        elapsed = stamp - shared_t0
        altitude.axvline(
            elapsed,
            color=COLORS[name],
            linestyle="--",
            linewidth=1.0,
            alpha=0.65,
            label="{} airborne +{:.1f}s".format(name.upper(), elapsed),
        )
    altitude_title = "Actual altitude on one shared time axis"
    if all(stamp is not None for stamp in takeoff_times.values()):
        delay = takeoff_times["uav2"] - takeoff_times["uav1"]
        altitude_title += (
            "\nObserved UAV2 airborne delay: {:+.1f} s".format(delay)
        )
    altitude.set_xlim(
        0.0, max(float(elapsed1[-1]), float(elapsed2[-1]))
    )
    altitude.set_xlabel(
        "Elapsed simulation time from shared t0={:.3f} s".format(shared_t0)
    )
    altitude.set_ylabel("Z / Up (m)")
    altitude.set_title(altitude_title)
    altitude.grid(alpha=0.25)
    altitude.legend(loc="best", fontsize=8)

    figure.suptitle(
        "Latest dual-UAV tower flight evidence — {} + {}".format(
            uav1.contract.csv_path.name, uav2.contract.csv_path.name
        ),
        fontsize=13,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)
    shutil.copy2(str(output), str(latest_output))


def default_output(uav1_path, uav2_path):
    newest_mtime = max(uav1_path.stat().st_mtime, uav2_path.stat().st_mtime)
    stamp = datetime.fromtimestamp(newest_mtime).strftime("%Y%m%d_%H%M%S")
    return REPO_ROOT / "trc_picture" / (
        "swarm_actual_trajectory_{}.png".format(stamp)
    )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Plot the two launch-bound swarm CSV reports after applying the "
            "audited TF chain into world/ENU."
        )
    )
    parser.add_argument("--uav1-csv", type=Path)
    parser.add_argument("--uav2-csv", type=Path)
    parser.add_argument(
        "--swarm-launch", type=Path, default=DEFAULT_SWARM_LAUNCH
    )
    parser.add_argument(
        "--bridge-launch", type=Path, default=DEFAULT_BRIDGE_LAUNCH
    )
    parser.add_argument(
        "--stack-launch", type=Path, default=DEFAULT_STACK_LAUNCH
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    try:
        contracts, tower_xy = load_contracts(
            args.swarm_launch.resolve(),
            args.bridge_launch.resolve(),
            args.stack_launch.resolve(),
        )
        if args.uav1_csv:
            contracts["uav1"].csv_path = args.uav1_csv.resolve()
        if args.uav2_csv:
            contracts["uav2"].csv_path = args.uav2_csv.resolve()
        if contracts["uav1"].csv_path == contracts["uav2"].csv_path:
            raise DataContractError("UAV1 and UAV2 resolve to the same report file")
        uav1 = load_vehicle(contracts["uav1"])
        uav2 = load_vehicle(contracts["uav2"])
        reject_duplicate_inputs(uav1, uav2)

        print_summary(uav1)
        print_summary(uav2)
        shared_t0 = min(uav1.times[0], uav2.times[0])
        print("=== Pair validation ===")
        print("topics distinct: YES")
        print("input files distinct: YES")
        print("numeric trajectories distinct: YES")
        print("shared time origin: {:.6f} s (not per-UAV zeroed)".format(shared_t0))
        print(
            "common tower centre: ({:.4f}, {:.4f}) m in world/ENU".format(
                tower_xy[0], tower_xy[1]
            )
        )
        print("successfully unified to one public coordinate frame: YES")

        output = (
            args.output.resolve()
            if args.output
            else default_output(
                uav1.contract.csv_path, uav2.contract.csv_path
            ).resolve()
        )
        latest = output.parent / "latest_swarm_trajectory.png"
        plot(uav1, uav2, tower_xy, output, latest)
        print("output: {}".format(output))
        print("latest: {}".format(latest))
        return 0
    except (
        DataContractError,
        ET.ParseError,
        FileNotFoundError,
        OSError,
        ValueError,
    ) as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
