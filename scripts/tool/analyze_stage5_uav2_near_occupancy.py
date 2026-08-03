#!/usr/bin/env python3
"""Forensic, read-only analysis of one Stage-5 U2 evidence bag.

This intentionally reproduces the active GridMap cloud callback semantics
(0.25 m grid, 0.40 m XY inflation and one Z cell) without changing the
planner.  It writes compact JSON that identifies the raw final-cloud point
which can produce an inflated voxel inside the status adapter's 0.55 m search.
"""
import argparse
import hashlib
import json
import math
import os
from collections import Counter

import rosbag
import sensor_msgs.point_cloud2 as pc2

UAV = "/uav2"
TOPICS = {
    "odom": UAV + "/Odometry",
    "raw": UAV + "/cloud_registered",
    "peer": UAV + "/cloud_registered_peer_filtered",
    "self_before": UAV + "/stage5/cloud_before_self",
    "self_after": UAV + "/stage5/cloud_after_self",
    "final": UAV + "/stage3/cloud_registered_filtered",
    "occ_raw": UAV + "/grid_map/occupancy",
    "occ_inflated": UAV + "/stage3/occupancy_inflate",
    "status": UAV + "/planner/status",
    "tf": "/tf",
    "tf_static": "/tf_static",
    "model_states": "/gazebo/model_states",
    "link_states": "/gazebo/link_states",
}


def stamp(msg, fallback):
    value = getattr(getattr(msg, "header", None), "stamp", None)
    return value.to_sec() if value is not None and not value.is_zero() else fallback.to_sec()


def vec(obj):
    return [float(obj.x), float(obj.y), float(obj.z)]


def norm(a):
    return math.sqrt(sum(x * x for x in a))


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def q_rotate(q, p):
    # q = [x,y,z,w], q*p*q^-1
    x, y, z, w = q
    tx = 2.0 * (y * p[2] - z * p[1])
    ty = 2.0 * (z * p[0] - x * p[2])
    tz = 2.0 * (x * p[1] - y * p[0])
    return [p[0] + w * tx + (y * tz - z * ty),
            p[1] + w * ty + (z * tx - x * tz),
            p[2] + w * tz + (x * ty - y * tx)]


def q_inv_rotate(q, p):
    return q_rotate([-q[0], -q[1], -q[2], q[3]], p)


def cloud_points(msg):
    return [[float(x), float(y), float(z)] for x, y, z in
            pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]


def closest(points, origin):
    if not points:
        return None
    point = min(points, key=lambda p: norm(sub(p, origin)))
    return {"point": point, "distance_m": norm(sub(point, origin))}


def summary_cluster(points, seed, radius=0.35):
    members = [p for p in points if norm(sub(p, seed)) <= radius]
    if not members:
        return {"count": 0}
    centroid = [sum(p[i] for p in members) / len(members) for i in range(3)]
    return {"count": len(members), "centroid": centroid,
            "bbox_min": [min(p[i] for p in members) for i in range(3)],
            "bbox_max": [max(p[i] for p in members) for i in range(3)]}


def nearest(messages, at):
    return min(messages, key=lambda pair: abs(pair[0] - at)) if messages else None


def grid_center(p, resolution, origin):
    return [(math.floor((p[i] - origin[i]) / resolution) + 0.5) * resolution + origin[i]
            for i in range(3)]


def cloud_digest(msg):
    return hashlib.sha256(bytes(msg.data)).hexdigest()[:16]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output", required=True)
    parser.add_argument("--resolution", type=float, default=0.25)
    parser.add_argument("--inflation", type=float, default=0.40)
    parser.add_argument("--ground-height", type=float, default=-0.50)
    parser.add_argument("--map-size-x", type=float, default=90.0)
    parser.add_argument("--map-size-y", type=float, default=90.0)
    args = parser.parse_args()
    series = {name: [] for name in TOPICS}
    tf_edges, statuses = Counter(), []
    with rosbag.Bag(args.bag) as bag:
        for topic, msg, receipt in bag.read_messages(topics=list(TOPICS.values())):
            name = next(key for key, value in TOPICS.items() if value == topic)
            at = stamp(msg, receipt)
            series[name].append((at, msg))
            if name in ("tf", "tf_static"):
                for transform in msg.transforms:
                    tf_edges[(transform.header.frame_id, transform.child_frame_id)] += 1
            if name == "status" and getattr(msg, "current_position_in_collision", False):
                statuses.append(at)
    if not series["odom"]:
        raise RuntimeError("UAV2 adapted odometry is absent; cannot locate occupancy")
    selected_time = statuses[0] if statuses else series["odom"][-1][0]
    selected = {name: nearest(values, selected_time) for name, values in series.items()}
    odom_t, odom = selected["odom"]
    position = vec(odom.pose.pose.position)
    q = [float(odom.pose.pose.orientation.x), float(odom.pose.pose.orientation.y),
         float(odom.pose.pose.orientation.z), float(odom.pose.pose.orientation.w)]
    result = {
        "bag": os.path.abspath(args.bag), "selected_stamp": selected_time,
        "collision_status_frames": len(statuses),
        "odom_stamp": odom_t, "planning_frame": odom.header.frame_id,
        "uav2_position_planning": position,
        "uav2_orientation_xyzw": q,
        "topic_message_counts": {key: len(value) for key, value in series.items()},
        "tf_edges": [{"parent": parent, "child": child, "messages": count}
                     for (parent, child), count in sorted(tf_edges.items())],
        "frame": {},
    }
    decoded = {}
    for name in ("raw", "peer", "self_before", "self_after", "final", "occ_raw", "occ_inflated"):
        picked = selected[name]
        if not picked:
            result["frame"][name] = {"available": False}
            continue
        at, message = picked
        points = cloud_points(message)
        decoded[name] = points
        nearest_point = closest(points, position)
        info = {"available": True, "stamp": at, "stamp_delta_s": at - selected_time,
                "frame_id": message.header.frame_id, "point_count": len(points),
                "data_sha256_16": cloud_digest(message), "nearest_to_uav2": nearest_point}
        if nearest_point and name in ("raw", "peer", "self_before", "self_after", "final"):
            info["nearest_cluster_0p35m"] = summary_cluster(points, nearest_point["point"])
        result["frame"][name] = info
    before = result["frame"].get("self_before", {})
    after = result["frame"].get("self_after", {})
    result["self_filter_frame_identity"] = {
        "same_point_count": before.get("point_count") == after.get("point_count"),
        "same_serialized_cloud": before.get("data_sha256_16") == after.get("data_sha256_16"),
    }
    # Reconstruct GridMap's actual direct cloud callback at the chosen frame.
    origin = [-args.map_size_x / 2.0, -args.map_size_y / 2.0, args.ground_height]
    step_xy = int(math.ceil(args.inflation / args.resolution))
    near_voxels = []
    for point in decoded.get("final", []):
        center = grid_center(point, args.resolution, origin)
        for ix in range(-step_xy, step_xy + 1):
            for iy in range(-step_xy, step_xy + 1):
                for iz in (-1, 0, 1):
                    occupied = [center[0] + ix * args.resolution,
                                center[1] + iy * args.resolution,
                                center[2] + iz * args.resolution]
                    distance = norm(sub(occupied, position))
                    if distance < 0.55:
                        near_voxels.append({"input_point": point,
                                            "source_voxel_center": center,
                                            "inflated_voxel_center": occupied,
                                            "offset_cells": [ix, iy, iz],
                                            "distance_to_uav_m": distance})
    near_voxels.sort(key=lambda item: item["distance_to_uav_m"])
    result["gridmap_reconstruction"] = {
        "map_origin": origin, "resolution_m": args.resolution,
        "xy_inflation_cells": step_xy, "z_inflation_cells": 1,
        "predicted_inflated_voxels_inside_0p55m": len(near_voxels),
        "closest_contributor": near_voxels[0] if near_voxels else None,
    }
    if near_voxels:
        source = near_voxels[0]["input_point"]
        body_point = q_inv_rotate(q, sub(source, position))
        result["closest_contributor_coordinates"] = {
            "camera_init": source,
            "body_from_odom_inverse": body_point,
            # FAST-LIO config: p_imu = R_L_I * p_lidar + T_L_I.  Current
            # MID360 config uses identity R and [-.011,-.02329,.04412] m.
            "lidar_from_fastlio_extrinsic": [body_point[0] + 0.011,
                                             body_point[1] + 0.02329,
                                             body_point[2] - 0.04412],
        }
    # Link/model state evidence is retained verbatim by the bag; report the
    # closest state-origin only. Surface distance requires the audited SDF mesh
    # geometry and is intentionally not guessed from an origin point.
    for name in ("model_states", "link_states"):
        picked = selected[name]
        if not picked:
            continue
        _, message = picked
        candidates = []
        for label, pose in zip(message.name, message.pose):
            p = vec(pose.position)
            candidates.append((norm(sub(p, position)), label, p))
        candidates.sort()
        result[name + "_nearest_origins"] = [
            {"name": label, "distance_to_uav2_planning_origin_m": distance,
             "position": p} for distance, label, p in candidates[:12]]
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    if os.path.exists(args.output):
        raise RuntimeError("refusing to overwrite existing analysis: " + args.output)
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
