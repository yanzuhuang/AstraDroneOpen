#!/usr/bin/env python3
"""Measure live UAV position clearance in each Stage-5 point-cloud product."""

import argparse
import json
import math

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uav", type=int, required=True, choices=(1, 2, 3))
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("stage5_map_probe", anonymous=True, disable_signals=True)
    prefix = "/uav{}".format(args.uav)
    odom = rospy.wait_for_message(prefix + "/Odometry", Odometry,
                                  timeout=args.timeout)
    position = odom.pose.pose.position
    result = {
        "uav_id": args.uav,
        "odom_frame": odom.header.frame_id,
        "odom_stamp": odom.header.stamp.to_sec(),
        "position": [position.x, position.y, position.z],
        "clouds": {},
    }
    topics = (
        prefix + "/fast_lio/cloud_registered_raw",
        prefix + "/cloud_registered",
        prefix + "/cloud_registered_peer_filtered",
        prefix + "/stage3/cloud_registered_filtered",
        prefix + "/drone_{}_ego_planner_node/grid_map/occupancy".format(args.uav - 1),
        prefix + "/stage3/occupancy_inflate",
    )
    radii = (0.25, 0.5, 1.0, 1.5)
    for topic in topics:
        cloud = rospy.wait_for_message(topic, PointCloud2, timeout=args.timeout)
        minimum_3d = math.inf
        minimum_horizontal = math.inf
        counts = {str(radius): 0 for radius in radii}
        point_count = 0
        for x, y, z in point_cloud2.read_points(
                cloud, field_names=("x", "y", "z"), skip_nans=True):
            point_count += 1
            dx = x - position.x
            dy = y - position.y
            dz = z - position.z
            horizontal = math.hypot(dx, dy)
            distance = math.sqrt(horizontal * horizontal + dz * dz)
            minimum_3d = min(minimum_3d, distance)
            minimum_horizontal = min(minimum_horizontal, horizontal)
            for radius in radii:
                if distance <= radius:
                    counts[str(radius)] += 1
        result["clouds"][topic] = {
            "frame": cloud.header.frame_id,
            "stamp": cloud.header.stamp.to_sec(),
            "point_count": point_count,
            "minimum_3d": None if math.isinf(minimum_3d) else minimum_3d,
            "minimum_horizontal": (
                None if math.isinf(minimum_horizontal) else minimum_horizontal),
            "counts_within_3d_radius": counts,
        }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
