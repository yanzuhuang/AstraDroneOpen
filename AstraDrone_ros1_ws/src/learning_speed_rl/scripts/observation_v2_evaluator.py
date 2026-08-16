#!/usr/bin/env python3
"""Read-only directional evaluator for tower-inspection Observation v2."""

import csv
import json
import math
import os
import threading

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, UInt8MultiArray

from learning_speed_rl.observation.v2 import AngularPartition, AngularPartitionSpec, BinSemantic
from learning_speed_rl.observation.v2.frame_transform import quaternion_to_matrix


class ObservationV2Evaluator:
    def __init__(self):
        self._lock = threading.RLock()
        self._output_directory = os.path.abspath(rospy.get_param("~output_directory"))
        self._tower_center = np.asarray(rospy.get_param("~tower_center_xyz"), dtype=np.float64)
        self._neighborhood_bins = int(rospy.get_param("~angular_neighborhood_bins", 1))
        if self._tower_center.shape != (3,) or not np.all(np.isfinite(self._tower_center)):
            raise ValueError("tower_center_xyz must be finite xyz")
        if self._neighborhood_bins < 0:
            raise ValueError("angular_neighborhood_bins must be non-negative")
        self._partition = AngularPartition(
            AngularPartitionSpec(
                angular_resolution_deg=float(rospy.get_param("~angular_resolution_deg", 4.5)),
                expected_number_of_bins=int(rospy.get_param("~number_of_bins", 3200)),
            )
        )
        self._position = None
        self._rotation_world_body = None
        self._stamp = None
        self._surrogate = None
        self._rows = []
        rospy.Subscriber(rospy.get_param("~odom_topic"), Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(rospy.get_param("~surrogate_topic"), Float32MultiArray, self._surrogate_callback, queue_size=2)
        rospy.Subscriber(rospy.get_param("~semantic_topic"), UInt8MultiArray, self._semantic_callback, queue_size=2)
        rospy.on_shutdown(self._write_results)

    def _odom_callback(self, message):
        position = message.pose.pose.position
        quaternion = message.pose.pose.orientation
        with self._lock:
            self._position = np.asarray([position.x, position.y, position.z], dtype=np.float64)
            self._rotation_world_body = quaternion_to_matrix(
                [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            )
            self._stamp = message.header.stamp.to_sec()

    def _surrogate_callback(self, message):
        values = np.asarray(message.data, dtype=np.float32)
        if values.shape == (self._partition.spec.number_of_bins,):
            with self._lock:
                self._surrogate = values

    def _neighborhood(self, flat_index):
        azimuth_bins = self._partition.spec.azimuth_bins
        elevation_bins = self._partition.spec.elevation_bins
        elevation = flat_index // azimuth_bins
        azimuth = flat_index % azimuth_bins
        indices = []
        for de in range(-self._neighborhood_bins, self._neighborhood_bins + 1):
            candidate_elevation = elevation + de
            if candidate_elevation < 0 or candidate_elevation >= elevation_bins:
                continue
            for da in range(-self._neighborhood_bins, self._neighborhood_bins + 1):
                candidate_azimuth = (azimuth + da) % azimuth_bins
                indices.append(candidate_elevation * azimuth_bins + candidate_azimuth)
        return np.asarray(indices, dtype=np.int64)

    def _direction_result(self, direction_body, surrogate, semantic):
        index, _ = self._partition.indices(np.asarray(direction_body, dtype=np.float64).reshape(1, 3))
        if index[0] < 0:
            return {"semantic": "invalid", "value": math.nan, "obstacle_distance": math.nan}
        neighborhood = self._neighborhood(int(index[0]))
        neighborhood_semantic = semantic[neighborhood]
        obstacles = neighborhood[neighborhood_semantic == int(BinSemantic.KNOWN_OBSTACLE)]
        if obstacles.size:
            obstacle_distance = float(np.min(surrogate[obstacles]))
            result_semantic = "known_obstacle"
            result_value = obstacle_distance
        elif np.any(neighborhood_semantic == int(BinSemantic.OBSERVED_FREE)):
            obstacle_distance = math.nan
            result_semantic = "observed_free"
            free = neighborhood[neighborhood_semantic == int(BinSemantic.OBSERVED_FREE)]
            result_value = float(np.max(surrogate[free]))
        else:
            obstacle_distance = math.nan
            result_semantic = "unknown"
            result_value = float(np.max(surrogate[neighborhood]))
        return {
            "semantic": result_semantic,
            "value": result_value,
            "obstacle_distance": obstacle_distance,
        }

    def _semantic_callback(self, message):
        # rospy exposes uint8[] as ``bytes`` on Python 3, unlike numeric ROS
        # arrays such as float32[].  frombuffer handles that representation
        # without creating 3200 Python integers on every callback.
        if isinstance(message.data, (bytes, bytearray, memoryview)):
            semantic = np.frombuffer(message.data, dtype=np.uint8)
        else:
            semantic = np.asarray(message.data, dtype=np.uint8)
        if semantic.shape != (self._partition.spec.number_of_bins,):
            return
        with self._lock:
            if self._position is None or self._surrogate is None:
                return
            position = self._position.copy()
            rotation = self._rotation_world_body.copy()
            stamp = self._stamp
            surrogate = self._surrogate.copy()
        inward_world = self._tower_center - position
        horizontal_norm = float(np.linalg.norm(inward_world[:2]))
        if horizontal_norm < 1.0e-6:
            return
        inward_world[2] = 0.0
        inward_world /= np.linalg.norm(inward_world)
        ccw_world = np.asarray([-inward_world[1], inward_world[0], 0.0])
        cw_world = -ccw_world
        inward = self._direction_result(rotation.T @ inward_world, surrogate, semantic)
        ccw = self._direction_result(rotation.T @ ccw_world, surrogate, semantic)
        cw = self._direction_result(rotation.T @ cw_world, surrogate, semantic)
        row = {
            "stamp_sec": stamp,
            "uav_x": float(position[0]),
            "uav_y": float(position[1]),
            "uav_z": float(position[2]),
            "tower_horizontal_distance_m": horizontal_norm,
            "inward_semantic": inward["semantic"],
            "inward_value": inward["value"],
            "inward_obstacle_distance_m": inward["obstacle_distance"],
            "ccw_tangent_semantic": ccw["semantic"],
            "ccw_tangent_value": ccw["value"],
            "ccw_tangent_obstacle_distance_m": ccw["obstacle_distance"],
            "cw_tangent_semantic": cw["semantic"],
            "cw_tangent_value": cw["value"],
            "cw_tangent_obstacle_distance_m": cw["obstacle_distance"],
        }
        with self._lock:
            self._rows.append(row)

    def _write_results(self):
        with self._lock:
            rows = list(self._rows)
        if not rows:
            return
        os.makedirs(self._output_directory, exist_ok=True)
        csv_path = os.path.join(self._output_directory, "tower_direction_samples.csv")
        with open(csv_path, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

        def semantic_counts(prefix):
            result = {"known_obstacle": 0, "observed_free": 0, "unknown": 0, "invalid": 0}
            for row in rows:
                result[row[prefix + "_semantic"]] += 1
            return result

        finite_inward = [row["inward_obstacle_distance_m"] for row in rows if math.isfinite(row["inward_obstacle_distance_m"])]
        finite_ccw = [row["ccw_tangent_obstacle_distance_m"] for row in rows if math.isfinite(row["ccw_tangent_obstacle_distance_m"])]
        finite_cw = [row["cw_tangent_obstacle_distance_m"] for row in rows if math.isfinite(row["cw_tangent_obstacle_distance_m"])]
        summary = {
            "samples": len(rows),
            "tower_center_xyz": self._tower_center.tolist(),
            "angular_neighborhood_bins": self._neighborhood_bins,
            "inward_semantic_counts": semantic_counts("inward"),
            "ccw_tangent_semantic_counts": semantic_counts("ccw_tangent"),
            "cw_tangent_semantic_counts": semantic_counts("cw_tangent"),
            "inward_obstacle_distance_median": None if not finite_inward else float(np.median(finite_inward)),
            "ccw_tangent_obstacle_distance_median": None if not finite_ccw else float(np.median(finite_ccw)),
            "cw_tangent_obstacle_distance_median": None if not finite_cw else float(np.median(finite_cw)),
            "uav_translation_m": float(np.linalg.norm(
                np.asarray([rows[-1]["uav_x"], rows[-1]["uav_y"], rows[-1]["uav_z"]])
                - np.asarray([rows[0]["uav_x"], rows[0]["uav_y"], rows[0]["uav_z"]])
            )),
            "note": "Direction neighborhoods are environment semantics only; they do not control EGO.",
        }
        with open(os.path.join(self._output_directory, "tower_direction_summary.json"), "w") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
        rospy.loginfo("Observation v2 evaluator wrote %d samples to %s", len(rows), self._output_directory)


if __name__ == "__main__":
    rospy.init_node("observation_v2_evaluator")
    ObservationV2Evaluator()
    rospy.spin()
