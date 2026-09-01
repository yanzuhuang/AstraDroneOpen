#!/usr/bin/env python3
"""Extract the source-stamp-aligned inflated occupancy used by replan72 tests."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import rosbag
from sensor_msgs import point_cloud2


TOPIC = "/uav1/drone_0_ego_planner_node/grid_map/occupancy_inflate"
REPLAN_TIME = 207.031


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("csv_output", type=Path)
    parser.add_argument("metadata_output", type=Path)
    args = parser.parse_args()

    candidates = []
    with rosbag.Bag(str(args.bag)) as bag:
        for _, message, receipt_time in bag.read_messages(topics=[TOPIC]):
            source_time = message.header.stamp.to_sec()
            if receipt_time.to_sec() <= REPLAN_TIME:
                candidates.append((source_time, receipt_time.to_sec(), message))
    if not candidates:
        raise RuntimeError("no source-causal occupancy snapshot found")

    # Freeze only state that was observably published before the planner call.
    # The internal GridMap buffer is not recorded atomically in the bag, so a
    # source-causal message received after the call must not be treated as the
    # exact runtime snapshot.
    source_time, receipt_time, message = max(candidates, key=lambda row: row[1])
    points = sorted({
        (round(float(x), 6), round(float(y), 6), round(float(z), 6))
        for x, y, z in point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)
    })

    args.csv_output.parent.mkdir(parents=True, exist_ok=True)
    with args.csv_output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("x", "y", "z"))
        writer.writerows(points)

    csv_sha256 = hashlib.sha256(args.csv_output.read_bytes()).hexdigest()
    bag_sha256 = hashlib.sha256(args.bag.read_bytes()).hexdigest()
    metadata = {
        "schema": "ego_frozen_replan72_occupancy_v1",
        "source_run": "forest_seed6_raw36_shared_mapper_v075_20260825_r01",
        "bag": str(args.bag),
        "bag_sha256": bag_sha256,
        "topic": TOPIC,
        "replan_time": REPLAN_TIME,
        "snapshot_source_stamp": source_time,
        "snapshot_receipt_time": receipt_time,
        "selection": "latest published occupancy receipt <= replan time",
        "point_count": len(points),
        "csv_sha256": csv_sha256,
        "map_resolution": 0.1,
        "map_origin": [-60.0, -10.0, 0.0],
        "map_size": [120.0, 20.0, 4.0],
        "obstacles_inflation": 0.3,
        "astar_step": 0.1,
        "astar_start": [29.566398, -0.929099, 2.728702],
        "astar_goal": [29.898084, -0.803345, 2.793707],
        "old_astar_path": [
            [29.432241, -0.966222, 2.761205],
            [29.532241, -0.966222, 2.761205],
            [29.632241, -0.966222, 2.761205],
            [29.732241, -0.866222, 2.761205],
            [29.832241, -0.766222, 2.761205],
            [29.932241, -0.766222, 2.761205],
        ],
        "control_points": [
            [28.761292, -1.135844, 2.537161],
            [29.156949, -1.067315, 2.634937],
            [29.566398, -0.929099, 2.728702],
            [29.898084, -0.803345, 2.793707],
            [30.281439, -0.648640, 2.856534],
            [30.643644, -0.503026, 2.903575],
            [31.020887, -0.367523, 2.941399],
            [31.402068, -0.251819, 2.967528],
            [31.791228, -0.156051, 2.985375],
            [32.182006, -0.082228, 2.996577],
            [32.587019, -0.036448, 3.002199],
            [32.964875, -0.012396, 3.004366],
            [33.422710, -0.002459, 3.004892],
            [33.690041, -0.000162, 3.005059],
            [34.273577, -0.000104, 3.004930],
            [34.859909, -0.000056, 3.004797],
        ],
        "optimizer": {
            "ts": 0.8,
            "order": 3,
            "lambda_smooth": 1.0,
            "lambda_collision": 0.5,
            "lambda_feasibility": 0.1,
            "lambda_fitness": 1.0,
            "dist0": 0.5,
            "swarm_clearance": 1.5,
            "max_vel": 0.75,
            "max_acc": 0.8,
        },
    }
    args.metadata_output.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
