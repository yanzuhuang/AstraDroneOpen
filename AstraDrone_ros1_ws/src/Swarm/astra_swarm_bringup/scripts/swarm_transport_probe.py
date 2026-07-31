#!/usr/bin/env python3
"""Verify per-UAV cancel isolation or stale swarm-trajectory rejection."""

import copy
import sys
import time

import rospy
from astra_custom_msgs.msg import PlannerStatus
from rosgraph_msgs.msg import Log
from std_msgs.msg import String
from std_srvs.srv import Trigger
from traj_utils.msg import Bspline


class TransportProbe:
    def __init__(self):
        self.mode = rospy.get_param("~mode")
        self.target_uav = int(rospy.get_param("~target_uav_id", 3))
        self.timeout = float(rospy.get_param("~timeout", 45.0))
        if self.mode not in {"cancel", "stale"}:
            raise rospy.ROSException("~mode must be cancel or stale")
        if self.target_uav not in (1, 2, 3):
            raise rospy.ROSException("~target_uav_id must be in 1..3")
        self.status = {}
        self.bridge = {}
        self.peer_violation = False
        self.cancel_seen = False
        self.hold_seen = False
        self.baseline_traj_id = None
        self.stale_message = None
        self.stale_rejections = set()
        self.injected = False
        self.event_pub = rospy.Publisher(
            "/swarm/transport_probe_event", String, queue_size=1, latch=True)
        self.broadcast_pub = rospy.Publisher(
            "/swarm/broadcast_bspline", Bspline, queue_size=1)
        for uid in (1, 2, 3):
            rospy.Subscriber(
                "/uav{}/planner/status".format(uid), PlannerStatus,
                lambda msg, u=uid: self.status_cb(u, msg), queue_size=20)
            rospy.Subscriber(
                "/uav{}/ego_mavros_bridge/state".format(uid), String,
                lambda msg, u=uid: self.bridge_cb(u, msg), queue_size=20)
        rospy.Subscriber(
            "/swarm/broadcast_bspline", Bspline,
            self.bspline_cb, queue_size=50)
        rospy.Subscriber("/rosout_agg", Log, self.rosout_cb, queue_size=100)

    def event(self, text):
        rospy.logwarn("[SWARM_TRANSPORT_PROBE] %s", text)
        self.event_pub.publish(String(data=text))

    def status_cb(self, uid, msg):
        self.status[uid] = msg
        if self.mode != "cancel" or self.baseline_traj_id is None:
            return
        if uid == self.target_uav and msg.failure_reason == "CANCELLED":
            self.cancel_seen = True
        elif uid != self.target_uav and msg.failure_reason == "CANCELLED":
            self.peer_violation = True

    def bridge_cb(self, uid, msg):
        self.bridge[uid] = msg.data
        if self.mode != "cancel" or self.baseline_traj_id is None:
            return
        if uid == self.target_uav and msg.data == "HOLD":
            self.hold_seen = True
        elif uid != self.target_uav and msg.data == "HOLD":
            self.peer_violation = True

    def bspline_cb(self, msg):
        if (self.mode == "stale"
                and msg.drone_id == self.target_uav - 1
                and self.stale_message is None):
            self.stale_message = copy.deepcopy(msg)

    def rosout_cb(self, msg):
        if self.mode != "stale" or not self.injected:
            return
        expected = "Time difference is too large! Local - Remote Agent {} =".format(
            self.target_uav - 1)
        if expected in msg.msg and msg.name.startswith("/uav"):
            self.stale_rejections.add(msg.name)

    def wait_ready(self, wall_deadline):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
            if (len(self.status) == 3 and len(self.bridge) == 3
                    and (self.mode != "stale"
                         or self.stale_message is not None)):
                return True
            rate.sleep()
        return False

    def run_cancel(self, wall_deadline):
        if self.bridge[self.target_uav] != "TRACK_EGO":
            self.event("FAILED cancel target is not TRACK_EGO")
            return False
        if any(self.bridge[uid] != "TRACK_EGO"
               for uid in (1, 2, 3) if uid != self.target_uav):
            self.event("FAILED cancel peers are not TRACK_EGO")
            return False
        self.baseline_traj_id = int(
            self.status[self.target_uav].trajectory_id)
        service_name = "/uav{}/ego_mavros_bridge/cancel_current_trajectory".format(
            self.target_uav)
        rospy.wait_for_service(service_name, timeout=2.0)
        response = rospy.ServiceProxy(service_name, Trigger)()
        if not response.success:
            self.event("FAILED cancel service rejected: {}".format(
                response.message))
            return False
        self.event(
            "CANCEL_INJECTED target_uav={} baseline_traj_id={}".format(
                self.target_uav, self.baseline_traj_id))
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
            target = self.status.get(self.target_uav)
            recovered = (
                target is not None
                and int(target.trajectory_id) > self.baseline_traj_id
                and self.bridge.get(self.target_uav) == "TRACK_EGO")
            if (self.cancel_seen and self.hold_seen and recovered
                    and not self.peer_violation):
                self.event(
                    "CANCEL_ISOLATION_VERIFIED target_uav={} "
                    "new_traj_id={} peers_unaffected=true".format(
                        self.target_uav, target.trajectory_id))
                return True
            rate.sleep()
        self.event(
            "FAILED cancel_seen={} hold_seen={} peer_violation={}".format(
                self.cancel_seen, self.hold_seen, self.peer_violation))
        return False

    def run_stale(self, wall_deadline):
        while (not rospy.is_shutdown()
               and time.monotonic() < wall_deadline
               and (rospy.Time.now() - self.stale_message.start_time).to_sec()
               <= 0.35):
            rospy.sleep(0.02)
        age = (rospy.Time.now() - self.stale_message.start_time).to_sec()
        if age <= 0.25:
            self.event("FAILED stale message did not age past 0.25 s")
            return False
        self.injected = True
        self.broadcast_pub.publish(self.stale_message)
        self.event(
            "STALE_INJECTED source_uav={} age={:.3f}".format(
                self.target_uav, age))
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
            expected_peers = {
                "/uav{}/drone_{}_ego_planner_node".format(uid, uid - 1)
                for uid in (1, 2, 3) if uid != self.target_uav}
            if expected_peers.issubset(self.stale_rejections):
                self.event(
                    "STALE_REJECTION_VERIFIED source_uav={} receivers={}".format(
                        self.target_uav,
                        ",".join(sorted(self.stale_rejections))))
                return True
            rate.sleep()
        self.event(
            "FAILED stale receivers={}".format(
                ",".join(sorted(self.stale_rejections))))
        return False

    def run(self):
        wall_deadline = time.monotonic() + self.timeout
        if not self.wait_ready(wall_deadline):
            self.event("FAILED timed out waiting for three live planners")
            return False
        if self.mode == "cancel":
            return self.run_cancel(wall_deadline)
        return self.run_stale(wall_deadline)


if __name__ == "__main__":
    rospy.init_node("swarm_transport_probe")
    try:
        success = TransportProbe().run()
    except (rospy.ROSException, rospy.ServiceException) as error:
        rospy.logfatal("%s", error)
        success = False
    sys.exit(0 if success else 1)
