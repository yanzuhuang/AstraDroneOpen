#!/usr/bin/env python3
"""Inject one controlled peer B-spline conflict and verify EGO replans."""

import copy
import sys
import time

import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import String
from traj_utils.msg import Bspline


class ConflictInjector:
    def __init__(self):
        self.target_uav = int(rospy.get_param("~target_uav_id"))
        self.peer_uav = int(rospy.get_param("~conflicting_peer_uav_id"))
        self.timeout = float(rospy.get_param("~timeout", 30.0))
        if (self.target_uav not in (1, 2, 3)
                or self.peer_uav not in (1, 2, 3)
                or self.target_uav == self.peer_uav):
            raise rospy.ROSException(
                "target/conflicting peer IDs must be distinct members of 1..3")
        self.target_drone = self.target_uav - 1
        self.peer_drone = self.peer_uav - 1
        self.injected = False
        self.core_detected = False
        self.verified = False
        self.source_traj_id = None
        self.latest_target_traj_id = None
        self.publisher = rospy.Publisher(
            "/swarm/broadcast_bspline", Bspline, queue_size=1)
        self.event_pub = rospy.Publisher(
            "/swarm/conflict_injection_event", String,
            queue_size=1, latch=True)
        self.subscriber = rospy.Subscriber(
            "/swarm/broadcast_bspline", Bspline,
            self.trajectory_cb, queue_size=20)
        self.rosout_subscriber = rospy.Subscriber(
            "/rosout_agg", Log, self.rosout_cb, queue_size=100)

    def publish_event(self, text):
        rospy.logwarn("[SWARM_CONFLICT_INJECTOR] %s", text)
        self.event_pub.publish(String(data=text))

    def trajectory_cb(self, msg):
        if msg.drone_id != self.target_drone:
            return
        self.latest_target_traj_id = int(msg.traj_id)
        if not self.injected:
            injected = copy.deepcopy(msg)
            injected.drone_id = self.peer_drone
            self.source_traj_id = int(msg.traj_id)
            self.injected = True
            self.publisher.publish(injected)
            self.publish_event(
                "INJECTED target_uav={} peer_uav={} source_traj_id={} "
                "start_time={:.6f}".format(
                    self.target_uav, self.peer_uav, self.source_traj_id,
                    msg.start_time.to_sec()))
            return
        self.maybe_verify()

    def rosout_cb(self, msg):
        expected = (
            "[EGO_SWARM_CONFLICT] self={} peer={} "
            "source=broadcast_bspline action=REPLAN_TRAJ"
            .format(self.target_drone, self.peer_drone))
        if self.injected and expected in msg.msg:
            self.core_detected = True
            self.publish_event(
                "CORE_CONFLICT_DETECTED target_uav={} peer_uav={}"
                .format(self.target_uav, self.peer_uav))
            self.maybe_verify()

    def maybe_verify(self):
        if (self.core_detected and self.latest_target_traj_id is not None
                and self.latest_target_traj_id > self.source_traj_id
                and not self.verified):
            self.verified = True
            self.publish_event(
                "REPLAN_VERIFIED target_uav={} peer_uav={} "
                "old_traj_id={} new_traj_id={}".format(
                    self.target_uav, self.peer_uav, self.source_traj_id,
                    self.latest_target_traj_id))

    def run(self):
        wall_deadline = time.monotonic() + self.timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
            if self.verified:
                return True
            rate.sleep()
        self.publish_event(
            "FAILED target_uav={} peer_uav={} injected={} core_detected={} "
            "source_traj_id={} latest_target_traj_id={}".format(
                self.target_uav, self.peer_uav, self.injected,
                self.core_detected, self.source_traj_id,
                self.latest_target_traj_id))
        return False


if __name__ == "__main__":
    rospy.init_node("swarm_conflict_injector")
    try:
        success = ConflictInjector().run()
    except rospy.ROSException as error:
        rospy.logfatal("%s", error)
        success = False
    sys.exit(0 if success else 1)
