#!/usr/bin/env python3
"""Hot-swap qualified Forest cylinders between closed training Episodes."""

import json
import math
import threading
import time

import rospy
from gazebo_msgs.srv import (
    DeleteModel,
    DeleteModelRequest,
    GetModelState,
    GetModelStateRequest,
    GetWorldProperties,
    SpawnModel,
    SpawnModelRequest,
)
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from hector_ego_training_backend.forest_map_contract import (
    FOREST_PREFIXES,
    ForestPoolContract,
)


class ForestMapManager:
    def __init__(self):
        config_path = rospy.get_param("~forest_config")
        world_dir = rospy.get_param("~world_dir")
        self._contract = ForestPoolContract(config_path, world_dir)
        self._timeout = float(rospy.get_param("~switch_timeout_wall", 30.0))
        self._pose_tolerance = float(rospy.get_param("~pose_tolerance_m", 1.0e-4))
        if self._timeout <= 0.0 or self._pose_tolerance <= 0.0:
            raise ValueError("Forest map-manager timeout/tolerance must be positive")
        self._lock = threading.Lock()
        self._last_state = {
            "version": "astradrone_forest_map_state_v1.0",
            "status": "waiting_for_request",
            "map_ready": False,
        }
        self._state_pub = rospy.Publisher(
            "/uav1/learning_speed/forest_map_state", String, queue_size=1, latch=True
        )
        self._spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self._delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self._world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self._model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        for service in (self._spawn, self._delete, self._world, self._model):
            service.wait_for_service(timeout=self._timeout)
        rospy.Subscriber(
            "/uav1/learning_speed/forest_map_request",
            String,
            self._request_callback,
            queue_size=1,
        )
        self._publish(self._last_state)

    def _publish(self, payload):
        self._last_state = dict(payload)
        self._state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _world_names(self):
        response = self._world()
        if not response.success:
            raise RuntimeError("Gazebo get_world_properties failed: " + response.status_message)
        return set(response.model_names)

    def _wait_absent(self, names):
        deadline = time.monotonic() + self._timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if not (set(names) & self._world_names()):
                return True
            time.sleep(0.05)
        return False

    def _delete_existing_forest(self):
        names = {
            name for name in self._world_names() if str(name).startswith(FOREST_PREFIXES)
        }
        for name in sorted(names):
            response = self._delete(DeleteModelRequest(model_name=name))
            if not response.success:
                raise RuntimeError("failed to delete {}: {}".format(name, response.status_message))
        if not self._wait_absent(names):
            raise RuntimeError("old Forest obstacles did not disappear")
        return sorted(names)

    @staticmethod
    def _pose(spec):
        pose = Pose()
        pose.position.x = spec.x
        pose.position.y = spec.y
        pose.position.z = spec.z
        cr, sr = math.cos(spec.roll / 2.0), math.sin(spec.roll / 2.0)
        cp, sp = math.cos(spec.pitch / 2.0), math.sin(spec.pitch / 2.0)
        cy, sy = math.cos(spec.yaw / 2.0), math.sin(spec.yaw / 2.0)
        pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.orientation.z = cr * cp * sy - sr * sp * cy
        return pose

    def _spawn_layout(self, logical_seed):
        expected = self._contract.layouts[int(logical_seed)]
        for spec in expected:
            request = SpawnModelRequest()
            request.model_name = spec.name
            request.model_xml = spec.model_sdf
            request.robot_namespace = "/forest"
            request.initial_pose = self._pose(spec)
            request.reference_frame = "world"
            response = self._spawn(request)
            if not response.success:
                raise RuntimeError("failed to spawn {}: {}".format(spec.name, response.status_message))
        return expected

    def _loaded_positions(self, expected):
        result = {}
        for spec in expected:
            response = self._model(GetModelStateRequest(model_name=spec.name, relative_entity_name="world"))
            if response.success:
                result[spec.name] = (
                    response.pose.position.x,
                    response.pose.position.y,
                    response.pose.position.z,
                )
        return result

    def _switch(self, request):
        switching = {
            "version": "astradrone_forest_map_state_v1.0",
            "status": "switching",
            "map_ready": False,
            **request,
        }
        self._publish(switching)
        deleted = self._delete_existing_forest()
        expected = self._spawn_layout(request["logical_seed"])
        audit = self._contract.verify_loaded_layout(
            expected,
            self._world_names(),
            self._loaded_positions(expected),
            tolerance=self._pose_tolerance,
        )
        if not audit["passed"]:
            raise RuntimeError("Forest loaded-layout audit failed: {}".format(audit["failures"]))
        ready = {
            "version": "astradrone_forest_map_state_v1.0",
            "status": "ready",
            "map_ready": True,
            **request,
            **audit,
            "deleted_previous_obstacles": deleted,
            "old_obstacles_absent_verified": True,
            "layout_sha256": self._contract.layout_hash(request["logical_seed"]),
            "ready_sim_time": rospy.Time.now().to_sec(),
            "ready_wall_time": time.time(),
        }
        self._publish(ready)
        rospy.logwarn(
            "[FOREST MAP] READY logical=%d raw=%d mode=%s block=%d round=%d obstacles=%d",
            request["logical_seed"], request["raw_seed"], request["mode"],
            request["map_block_id"], request["map_round_id"], audit["loaded_obstacle_count"],
        )

    def _request_callback(self, message):
        try:
            payload = self._contract.validate_request(json.loads(message.data))
        except Exception as error:
            self._publish(
                {
                    "version": "astradrone_forest_map_state_v1.0",
                    "status": "failed",
                    "map_ready": False,
                    "failure": "invalid_request:{}".format(error),
                }
            )
            return
        with self._lock:
            try:
                self._switch(payload)
            except Exception as error:
                failed = {
                    "version": "astradrone_forest_map_state_v1.0",
                    "status": "failed",
                    "map_ready": False,
                    **payload,
                    "failure": "{}: {}".format(type(error).__name__, error),
                }
                self._publish(failed)
                rospy.logerr("Forest map switch failed: %s", failed["failure"])


def main():
    rospy.init_node("forest_map_manager", anonymous=False)
    ForestMapManager()
    rospy.spin()


if __name__ == "__main__":
    main()
