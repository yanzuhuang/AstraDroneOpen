#!/usr/bin/env python3
"""Fail-closed permission coordinator for the tower mission.

The original policy was written for two vehicles.  The same interlocks now
accept an explicit ``~uav_ids`` list so a third vehicle can join the common
ROS master without creating a second, conflicting coordinator.  The legacy
two-vehicle transition/landing fields remain unchanged for compatibility.
"""

import math

import rospy
from astra_swarm_manager.policy import (
    fixed_layers_clear,
    scheduled_takeoff_allowed,
    serialized_landing_permissions,
    transition_permissions,
)
from astra_swarm_msgs.msg import CoordinationStatus, LayerConfirmation, SwarmState
from std_msgs.msg import Bool


class SwarmManager:
    def __init__(self):
        configured_ids = rospy.get_param("~uav_ids", [1, 2])
        self.uav_ids = [int(uid) for uid in configured_ids]
        if self.uav_ids != sorted(set(self.uav_ids)) or not self.uav_ids:
            raise rospy.ROSException("~uav_ids must be a non-empty sorted list")
        self.takeoff_interval = float(rospy.get_param(
            "~takeoff_interval_sec", rospy.get_param("~takeoff_delay", 6.0)))
        if (not math.isfinite(self.takeoff_interval)
                or self.takeoff_interval < 0.0):
            raise rospy.ROSException(
                "~takeoff_interval_sec must be finite and non-negative")
        self.timeout = float(rospy.get_param("~heartbeat_timeout", 1.0))
        self.height_tolerance = float(rospy.get_param(
            "~height_tolerance", 0.4))
        self.minimum_vertical = float(rospy.get_param(
            "~minimum_vertical_separation", 4.0))
        self.optimizer_swarm_clearance = float(rospy.get_param(
            "~optimizer_swarm_clearance", 0.0))
        if (not math.isfinite(self.optimizer_swarm_clearance)
                or self.optimizer_swarm_clearance < 0.0):
            raise rospy.ROSException(
                "~optimizer_swarm_clearance must be finite and non-negative")
        # EGO-Swarm uses CLEARANCE=2*swarm_clearance and scales z by 1/2
        # in its ellipsoid, so same-XY vertical separation must be at least
        # 4*swarm_clearance.
        self.required_vertical = max(
            self.minimum_vertical, 4.0 * self.optimizer_swarm_clearance)
        self.mission_heights = [
            float(height) for height in rospy.get_param(
                "~mission_heights",
                [34.0, 28.0])]
        if len(self.mission_heights) != len(self.uav_ids):
            raise rospy.ROSException(
                "~mission_heights must contain one value per UAV")
        self.configuration_safe = fixed_layers_clear(
            self.mission_heights, self.required_vertical)
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
        self.schedule_started = None
        self.landing_owner = 0
        self.last_uav2_confirmed = False
        self.uav2_final_confirmed_ever = False

        for uav_id in self.uav_ids:
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
            for uid in self.uav_ids
        }
        self.transition_pubs = {
            uid: rospy.Publisher(
                "/uav{}/swarm/transition_permission".format(uid),
                Bool, queue_size=1, latch=True)
            for uid in self.uav_ids
        }
        self.landing_pubs = {
            uid: rospy.Publisher("/uav{}/swarm/landing_permission".format(uid),
                                 Bool, queue_size=1, latch=True)
            for uid in self.uav_ids
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

    def timer_cb(self, _event):
        now = rospy.Time.now()
        healthy1 = self.healthy(1, now)
        healthy2 = self.healthy(2, now) if 2 in self.uav_ids else False
        healthy_all = all(self.healthy(uid, now) for uid in self.uav_ids)
        if (self.schedule_started is None and healthy_all
                and self.px4_params_ready and self.configuration_safe):
            self.schedule_started = now
        elapsed = -1.0
        if self.schedule_started is not None:
            elapsed = (now - self.schedule_started).to_sec()
        takeoff_permissions = {}
        for index, uid in enumerate(self.uav_ids):
            takeoff_permissions[uid] = scheduled_takeoff_allowed(
                index, elapsed, self.takeoff_interval, healthy_all,
                self.px4_params_ready, self.safety_clear,
                self.configuration_safe)
        takeoff1 = takeoff_permissions.get(1, False)
        takeoff2 = takeoff_permissions.get(2, False)

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

        if self.landing_owner and (
                not self.healthy(self.landing_owner, now)
                or self.states[self.landing_owner].flight_state in {
                    "DONE", "ERROR", "FAILSAFE"}):
            self.landing_owner = 0
        waiting_ids = [
            uid for uid in self.uav_ids
            if self.healthy(uid, now)
            and self.states[uid].flight_state == "HOME_HOVER"]
        landing_grants, self.landing_owner = (
            serialized_landing_permissions(
                waiting_ids if healthy_all else [], self.landing_owner))
        landing_permissions_by_id = {
            uid: landing_grants.get(uid, False) for uid in self.uav_ids}
        land1 = landing_permissions_by_id.get(1, False)
        land2 = landing_permissions_by_id.get(2, False)

        permissions = {
            "takeoff": takeoff_permissions,
            "transition": {1: transition1, 2: transition2},
            "landing": landing_permissions_by_id,
        }
        for uid in self.uav_ids:
            if uid > 2:
                # No third-vehicle height exchange exists in the legacy
                # CoordinationStatus message.  Keep its grants fail-closed:
                # heartbeat, PX4 readiness and the global safety latch must
                # all be present.
                permissions["transition"][uid] = (
                    self.healthy(uid, now) and self.safety_clear)
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
        if not self.configuration_safe:
            status.coordinator_state = "SAFETY_INHIBIT"
            status.reason = (
                "fixed mission layers violate {:.2f} m required vertical "
                "separation".format(self.required_vertical))
        elif not healthy_all or not self.px4_params_ready:
            status.coordinator_state = "SYSTEM_READY"
            status.reason = (
                "waiting for healthy states and verified PX4 parameters")
        elif phase1 == "DONE" and phase2 == "DONE":
            status.coordinator_state = "MISSION_COMPLETE"
            status.reason = "both missions complete"
        elif returning:
            status.coordinator_state = "RETURN_COORDINATION"
            status.reason = "independent return and landing-zone arbitration"
        elif self.schedule_started is None:
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
