#!/usr/bin/env python3
"""Verify the simulation-only PX4 contract before flight."""

import json
import math
from pathlib import Path

import rospy
from mavros_msgs.srv import ParamGet
from std_msgs.msg import Bool


class Px4ParamGuard:
    def __init__(self):
        self.namespaces = rospy.get_param(
            "~vehicle_namespaces", ["/uav1", "/uav2"])
        if (not self.namespaces
                or len(set(self.namespaces)) != len(self.namespaces)):
            raise rospy.ROSException(
                "~vehicle_namespaces must be non-empty and unique")
        self.expected = {
            "EKF2_HGT_REF": int(rospy.get_param("~ekf2_hgt_ref", 0)),
            # FAST-LIO contributes horizontal/vertical position and yaw. Its
            # current ROS odometry does not provide a validated 3-D velocity
            # estimate, so only the EV velocity bit is disabled.
            "EKF2_EV_CTRL": int(rospy.get_param("~ekf2_ev_ctrl", 11)),
        }
        self.check_high_speed_limits = bool(
            rospy.get_param("~check_high_speed_limits", False))
        self.high_speed_minimum = {
            "MPC_XY_VEL_MAX": float(rospy.get_param(
                "~minimum_mpc_xy_vel_max", math.sqrt(2.0) * 4.0)),
            "MPC_ACC_HOR_MAX": float(rospy.get_param(
                "~minimum_mpc_acc_hor_max", math.sqrt(2.0) * 3.0)),
            "MPC_ACC_UP_MAX": float(rospy.get_param(
                "~minimum_mpc_acc_up_max", 3.0)),
            "MPC_ACC_DOWN_MAX": float(rospy.get_param(
                "~minimum_mpc_acc_down_max", 3.0)),
        }
        self.audit_parameters = (
            "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN")
        self.output_file = str(rospy.get_param("~output_file", "")).strip()
        self.vehicle_results = {}
        self.ready_pub = rospy.Publisher(
            "/swarm/px4_params_ready", Bool, queue_size=1, latch=True)
        self.ready_pub.publish(Bool(data=False))
        self.completed = False
        rospy.Timer(rospy.Duration(1.0), self.timer_cb)

    def configure_vehicle(self, namespace):
        get_name = namespace.rstrip("/") + "/mavros/param/get"
        try:
            rospy.wait_for_service(get_name, timeout=0.2)
            getter = rospy.ServiceProxy(get_name, ParamGet)
            values = {}
            for param_id, expected in self.expected.items():
                current = getter(param_id)
                if (not current.success
                        or current.value.integer != expected):
                    return False
                values[param_id] = int(current.value.integer)
            if self.check_high_speed_limits:
                for param_id, minimum in self.high_speed_minimum.items():
                    current = getter(param_id)
                    value = float(current.value.real)
                    if (not current.success or not math.isfinite(value)
                            or value + 1.0e-6 < minimum):
                        return False
                    values[param_id] = value
                for param_id in self.audit_parameters:
                    current = getter(param_id)
                    value = float(current.value.real)
                    if not current.success or not math.isfinite(value):
                        return False
                    values[param_id] = value
            self.vehicle_results[namespace] = values
            return True
        except (rospy.ROSException, rospy.ServiceException):
            return False

    def timer_cb(self, _event):
        if self.completed:
            return
        ready = all(
            self.configure_vehicle(namespace)
            for namespace in self.namespaces)
        if ready:
            if self.output_file:
                output = Path(self.output_file)
                output.parent.mkdir(parents=True, exist_ok=True)
                if output.exists():
                    raise rospy.ROSException(
                        "refusing to overwrite PX4 preflight output: {}".format(
                            output))
                result = {
                    "schema_version": "high_speed_live_px4_preflight_v1.0",
                    "pass": True,
                    "check_high_speed_limits": self.check_high_speed_limits,
                    "minimums": self.high_speed_minimum,
                    "vehicles": self.vehicle_results,
                    "sim_time_sec": rospy.Time.now().to_sec(),
                }
                output.write_text(
                    json.dumps(result, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")
            self.ready_pub.publish(Bool(data=True))
            self.completed = True
            rospy.logwarn(
                "[SWARM_PX4_GUARD] verified %d PX4 instances: "
                "EKF2_HGT_REF=%d EKF2_EV_CTRL=%d",
                len(self.namespaces),
                self.expected["EKF2_HGT_REF"],
                self.expected["EKF2_EV_CTRL"])
        else:
            self.ready_pub.publish(Bool(data=False))


if __name__ == "__main__":
    rospy.init_node("px4_param_guard")
    Px4ParamGuard()
    rospy.spin()
