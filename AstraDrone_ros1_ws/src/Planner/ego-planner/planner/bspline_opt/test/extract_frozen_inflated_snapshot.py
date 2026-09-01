#!/usr/bin/env python3
"""Extract one source-causal inflated GridMap snapshot as a sorted CSV."""

import argparse
import csv
from pathlib import Path

import rosbag
from sensor_msgs import point_cloud2


TOPIC = "/uav1/drone_0_ego_planner_node/grid_map/occupancy_inflate"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("target_source_time", type=float)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    candidates = []
    with rosbag.Bag(str(args.bag)) as bag:
        for _, message, receipt_time in bag.read_messages(topics=[TOPIC]):
            source_time = message.header.stamp.to_sec()
            if receipt_time.to_sec() <= args.target_source_time:
                candidates.append((source_time, receipt_time.to_sec(), message))
    if not candidates:
        raise RuntimeError("no source-causal inflated snapshot")
    source_time, receipt_time, message = max(candidates, key=lambda row: row[1])
    points = sorted({
        (round(float(x), 6), round(float(y), 6), round(float(z), 6))
        for x, y, z in point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)
    })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("x", "y", "z"))
        writer.writerows(points)
    print(
        f"source_stamp={source_time:.3f} receipt_time={receipt_time:.3f} "
        f"points={len(points)} output={args.output}"
    )


if __name__ == "__main__":
    main()
