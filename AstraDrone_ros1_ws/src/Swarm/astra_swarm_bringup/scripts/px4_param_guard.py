#!/usr/bin/env python3
"""Verify the simulation-only multi-PX4 EKF contract before flight."""

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
            for param_id, expected in self.expected.items():
                current = getter(param_id)
                if (not current.success
                        or current.value.integer != expected):
                    return False
            return True
        except (rospy.ROSException, rospy.ServiceException):
            return False

    def timer_cb(self, _event):
        if self.completed:
            return
        ready = all(
            self.configure_vehicle(namespace)
            for namespace in self.namespaces)
        self.ready_pub.publish(Bool(data=ready))
        if ready:
            self.completed = True
            rospy.logwarn(
                "[SWARM_PX4_GUARD] verified %d PX4 instances: "
                "EKF2_HGT_REF=%d EKF2_EV_CTRL=%d",
                len(self.namespaces),
                self.expected["EKF2_HGT_REF"],
                self.expected["EKF2_EV_CTRL"])


if __name__ == "__main__":
    rospy.init_node("px4_param_guard")
    Px4ParamGuard()
    rospy.spin()
