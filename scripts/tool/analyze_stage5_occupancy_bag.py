#!/usr/bin/env python3
"""Measure vehicle-to-inflated-map clearance over a Stage-5 bag interval."""

import argparse
import json
import math

import numpy as np
import rosbag
import rospy
from sensor_msgs import point_cloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--uav", type=int, required=True, choices=(1, 2, 3))
    parser.add_argument("--start", type=float, required=True,
                        help="absolute ROS/simulation time")
    parser.add_argument("--end", type=float, required=True,
                        help="absolute ROS/simulation time")
    parser.add_argument("--sample-period", type=float, default=0.1)
    args = parser.parse_args()

    prefix = "/uav{}".format(args.uav)
    odom_topic = prefix + "/Odometry"
    map_topic = prefix + "/stage3/occupancy_inflate"
    odom = None
    next_sample = args.start
    samples = []
    with rosbag.Bag(args.bag, "r") as bag:
        for topic, message, recorded_at in bag.read_messages(
                topics=(odom_topic, map_topic),
                start_time=rospy.Time.from_sec(args.start),
                end_time=rospy.Time.from_sec(args.end)):
            now = recorded_at.to_sec()
            if topic == odom_topic:
                odom = message
                continue
            if odom is None or now + 1.0e-9 < next_sample:
                continue
            next_sample = now + args.sample_period
            points = np.asarray(list(point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True)),
                dtype=np.float64)
            position = odom.pose.pose.position
            if points.size == 0:
                minimum = math.inf
                counts = {"0.25": 0, "0.5": 0, "1.0": 0}
                nearest = None
            else:
                distances = np.linalg.norm(
                    points - np.array((position.x, position.y, position.z)),
                    axis=1)
                nearest_index = int(np.argmin(distances))
                minimum = float(distances[nearest_index])
                nearest = points[nearest_index].tolist()
                counts = {
                    str(radius): int(np.count_nonzero(distances <= radius))
                    for radius in (0.25, 0.5, 1.0)
                }
            samples.append({
                "time": now,
                "odom_stamp": odom.header.stamp.to_sec(),
                "map_stamp": message.header.stamp.to_sec(),
                "position": [position.x, position.y, position.z],
                "point_count": int(points.shape[0]) if points.ndim == 2 else 0,
                "minimum_3d": None if math.isinf(minimum) else minimum,
                "nearest_point": nearest,
                "counts_within_3d_radius": counts,
            })

    finite = [sample for sample in samples
              if sample["minimum_3d"] is not None]
    closest = min(finite, key=lambda sample: sample["minimum_3d"],
                  default=None)
    first_inside = {}
    for radius in (0.25, 0.5, 1.0):
        key = str(radius)
        first_inside[key] = next(
            (sample for sample in samples
             if sample["counts_within_3d_radius"][key] > 0), None)
    result = {
        "bag": args.bag,
        "uav_id": args.uav,
        "topic": map_topic,
        "start": args.start,
        "end": args.end,
        "sample_period": args.sample_period,
        "sample_count": len(samples),
        "closest_sample": closest,
        "first_inside_radius": first_inside,
        "samples": samples,
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
