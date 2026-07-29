#!/usr/bin/env python3
"""Fail-closed permission coordinator for the two-UAV tower mission."""

import math

import rospy
from astra_swarm_manager.policy import (
    landing_permissions,
    transition_permissions,
    uav2_takeoff_allowed,
)
from astra_swarm_msgs.msg import CoordinationStatus, LayerConfirmation, SwarmState
from std_msgs.msg import Bool


class SwarmManager:
    def __init__(self):
        self.delay = float(rospy.get_param("~takeoff_delay", 6.0))
        self.timeout = float(rospy.get_param("~heartbeat_timeout", 1.0))
        self.height_tolerance = float(rospy.get_param(
            "~height_tolerance", 0.4))
        self.takeoff_radius = float(rospy.get_param(
            "~takeoff_protection_radius", 3.0))
        self.takeoff_height = float(rospy.get_param(
            "~uav2_takeoff_height", 4.0))
        self.minimum_3d = float(rospy.get_param(
            "~minimum_3d_separation", 3.0))
        self.landing_radius = float(rospy.get_param(
            "~landing_protection_radius", 3.0))
        self.home1 = rospy.get_param("~uav1_home_position", [0.0, 0.0, 0.0])
        self.home2 = rospy.get_param("~uav2_home_position", [2.0, 0.0, 0.0])
        self.uav1_initial_height = float(rospy.get_param(
            "~uav1_initial_height", 34.0))
        self.uav1_final_height = float(rospy.get_param(
            "~uav1_final_height", 30.0))
        self.uav2_initial_height = float(rospy.get_param(
            "~uav2_initial_height", 28.0))
        self.uav2_final_height = float(rospy.get_param(
            "~uav2_final_height", 24.0))
        self.states = {}
        self.received = {}
        self.safety_clear = False
        self.px4_params_ready = False
        self.uav1_takeoff_started = None
        self.landing_owner = 0
        self.last_uav2_confirmed = False
        self.uav2_final_confirmed_ever = False

        for uav_id in (1, 2):
            rospy.Subscriber(
                "/uav{}/swarm/state".format(uav_id), SwarmState,
                lambda msg, uid=uav_id: self.state_cb(uid, msg), queue_size=5)
        rospy.Subscriber("/swarm/safety/clear", Bool,
                         self.safety_cb, queue_size=5)
        rospy.Subscriber("/swarm/px4_params_ready", Bool,
                         self.px4_params_cb, queue_size=1)
        self.takeoff_pubs = {
            uid: rospy.Publisher("/uav{}/swarm/takeoff_permission".format(uid),
                                 Bool, queue_size=1, latch=True)
            for uid in (1, 2)
        }
        self.transition_pubs = {
            uid: rospy.Publisher(
                "/uav{}/swarm/transition_permission".format(uid),
                Bool, queue_size=1, latch=True)
            for uid in (1, 2)
        }
        self.landing_pubs = {
            uid: rospy.Publisher("/uav{}/swarm/landing_permission".format(uid),
                                 Bool, queue_size=1, latch=True)
            for uid in (1, 2)
        }
        self.status_pub = rospy.Publisher(
            "/swarm/coordinator/status", CoordinationStatus,
            queue_size=1, latch=True)
        self.layer_pub = rospy.Publisher(
            "/swarm/layer_confirmation", LayerConfirmation,
            queue_size=2, latch=True)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def state_cb(self, uav_id, msg):
        self.states[uav_id] = msg
        self.received[uav_id] = rospy.Time.now()
        if uav_id == 1 and self.uav1_takeoff_started is None:
            if msg.flight_state in {
                    "TAKEOFF", "HOVER_READY", "TRACK_EGO", "HOLD",
                    "RETURN_HOME", "HOME_HOVER", "LANDING"}:
                self.uav1_takeoff_started = rospy.Time.now()

    def safety_cb(self, msg):
        self.safety_clear = msg.data

    def px4_params_cb(self, msg):
        self.px4_params_ready = msg.data

    def healthy(self, uav_id, now):
        if uav_id not in self.states or uav_id not in self.received:
            return False
        state = self.states[uav_id]
        return (
            (now - self.received[uav_id]).to_sec() <= self.timeout
            and state.heartbeat_ok
            and state.localization_valid
            and state.flight_state not in {"ERROR", "FAILSAFE", "HOLD_SAFE"}
            and state.mission_phase not in {
                "ERROR", "FAILSAFE", "HOLD_SAFE"}
        )

    @staticmethod
    def xyz(state):
        p = state.pose.position
        return (p.x, p.y, p.z)

    def timer_cb(self, _event):
        now = rospy.Time.now()
        healthy1 = self.healthy(1, now)
        healthy2 = self.healthy(2, now)
        takeoff1 = healthy1 and healthy2 and self.px4_params_ready
        elapsed = -1.0
        if self.uav1_takeoff_started is not None:
            elapsed = (now - self.uav1_takeoff_started).to_sec()
        takeoff2 = False
        if (healthy1 and healthy2 and self.px4_params_ready
                and elapsed >= 0.0):
            takeoff2 = uav2_takeoff_allowed(
                elapsed, self.delay, True, self.xyz(self.states[1]),
                self.home2, self.takeoff_radius, self.safety_clear,
                self.takeoff_height, self.minimum_3d)

        transition1 = transition2 = confirmed = False
        if healthy1 and healthy2:
            transition1, transition2, confirmed = transition_permissions(
                self.states[1].mission_phase,
                self.states[2].mission_phase,
                self.states[2].current_height,
                self.uav2_final_height,
                self.height_tolerance,
                self.safety_clear,
            )
        if confirmed and not self.last_uav2_confirmed:
            confirmation = LayerConfirmation()
            confirmation.header.stamp = now
            confirmation.header.frame_id = "world"
            confirmation.uav_id = 2
            confirmation.from_height = self.uav2_initial_height
            confirmation.to_height = self.uav2_final_height
            confirmation.stable = True
            confirmation.transition_anchor = "configured_transition_anchor"
            self.layer_pub.publish(confirmation)
        self.uav2_final_confirmed_ever |= confirmed
        self.last_uav2_confirmed = confirmed

        waiting1 = (healthy1 and
                    self.states[1].flight_state == "HOME_HOVER")
        waiting2 = (healthy2 and
                    self.states[2].flight_state == "HOME_HOVER")
        if not (healthy1 and healthy2):
            # A stale or invalid peer must inhibit every new landing grant.
            waiting1 = waiting2 = False
        zones_overlap = math.hypot(
            self.home1[0] - self.home2[0],
            self.home1[1] - self.home2[1]) < 2.0 * self.landing_radius
        if self.landing_owner == 1 and (
                not healthy1 or self.states[1].flight_state in {
                    "DONE", "ERROR", "FAILSAFE"}):
            self.landing_owner = 0
        if self.landing_owner == 2 and (
                not healthy2 or self.states[2].flight_state in {
                    "DONE", "ERROR", "FAILSAFE"}):
            self.landing_owner = 0
        land1, land2, self.landing_owner = landing_permissions(
            waiting1, waiting2, zones_overlap, self.landing_owner)

        permissions = {
            "takeoff": {1: takeoff1, 2: takeoff2},
            "transition": {1: transition1, 2: transition2},
            "landing": {1: land1, 2: land2},
        }
        for category, pubs in (
                ("takeoff", self.takeoff_pubs),
                ("transition", self.transition_pubs),
                ("landing", self.landing_pubs)):
            for uid, publisher in pubs.items():
                publisher.publish(Bool(data=permissions[category][uid]))

        status = CoordinationStatus()
        status.header.stamp = now
        status.header.frame_id = "world"
        status.uav1_takeoff_allowed = takeoff1
        status.uav2_takeoff_allowed = takeoff2
        status.uav1_transition_allowed = transition1
        status.uav2_transition_allowed = transition2
        status.uav1_landing_allowed = land1
        status.uav2_landing_allowed = land2
        phase1 = self.states[1].mission_phase if healthy1 else ""
        phase2 = self.states[2].mission_phase if healthy2 else ""
        flight1 = self.states[1].flight_state if healthy1 else ""
        flight2 = self.states[2].flight_state if healthy2 else ""
        returning = (
            phase1 in {"RETURN_HOME", "WAIT_LANDING_PERMISSION", "LAND", "DONE"}
            or phase2 in {
                "RETURN_HOME", "WAIT_LANDING_PERMISSION", "LAND", "DONE"}
            or flight1 in {"RETURN_HOME", "HOME_HOVER", "LANDING", "DONE"}
            or flight2 in {"RETURN_HOME", "HOME_HOVER", "LANDING", "DONE"})
        if not healthy1 or not healthy2 or not self.px4_params_ready:
            status.coordinator_state = "SYSTEM_READY"
            status.reason = (
                "waiting for healthy states and verified PX4 parameters")
        elif phase1 == "DONE" and phase2 == "DONE":
            status.coordinator_state = "MISSION_COMPLETE"
            status.reason = "both missions complete"
        elif returning:
            status.coordinator_state = "RETURN_COORDINATION"
            status.reason = "independent return and landing-zone arbitration"
        elif self.uav1_takeoff_started is None:
            status.coordinator_state = "SYSTEM_READY"
            status.reason = "UAV1 is ready for the first takeoff"
        elif not takeoff2:
            status.coordinator_state = "UAV1_TAKEOFF_STARTED"
            status.reason = "UAV2 delay/protection interlock active"
        elif flight2 in {
                "WAIT_FCU", "WAIT_INPUTS", "PRESTREAM", "ARM_OFFBOARD"}:
            status.coordinator_state = "UAV2_TAKEOFF_ALLOWED"
            status.reason = "delay, health, and takeoff corridor are clear"
        elif transition2:
            status.coordinator_state = "UAV2_TRANSITION_ALLOWED"
            status.reason = "UAV2 descends {:.1f} to {:.1f} before UAV1".format(
                self.uav2_initial_height, self.uav2_final_height)
        elif confirmed and transition1:
            status.coordinator_state = "UAV1_TRANSITION_ALLOWED"
            status.reason = "UAV2 stable at {:.1f} m".format(
                self.uav2_final_height)
        elif self.uav2_final_confirmed_ever:
            status.coordinator_state = "SECOND_LAYER_PARALLEL"
            status.reason = (
                "{:.1f} m and {:.1f} m asynchronous inspection".format(
                    self.uav1_final_height, self.uav2_final_height))
        else:
            status.coordinator_state = "FIRST_LAYER_PARALLEL"
            status.reason = (
                "{:.1f} m and {:.1f} m asynchronous inspection".format(
                    self.uav1_initial_height, self.uav2_initial_height))
        self.status_pub.publish(status)


if __name__ == "__main__":
    rospy.init_node("astra_swarm_manager")
    SwarmManager()
    rospy.spin()
