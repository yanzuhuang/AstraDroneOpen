#!/usr/bin/env python3
"""Fail-closed temporal/spatial coordinator for the tower swarm."""

import math

import rospy
from astra_swarm_manager.policy import (
    advance_entry_owner,
    corridor_clear,
    eligible_entry_ids,
    entry_owner_orbit_established,
    mission_geometry_clear,
    orbit_phase_hold_ids,
    orbit_release_allowed,
    rotate_xy_about_center,
    scheduled_takeoff_allowed,
    serialized_permissions,
    task_start_barrier_ready,
)
from astra_swarm_msgs.msg import (
    CoordinationStatus,
    PredictedTrajectory,
    SwarmState,
)
from std_msgs.msg import Bool


ACTIVE_ORBIT_PHASES = {
    "TARGET_LOCKED", "NAVIGATING", "EVALUATING", "RELOCATING", "RECOVERING",
    "HOLDING"
}
# Keep the exclusive ENTRY owner while it is hovering at the gate.  Releasing
# ownership in WAIT_ORBIT_PERMISSION creates a one-timer-cycle window where
# orbit_released is still empty, so the next vehicle can bypass projected
# phase admission and then wait at its gate for most of a lap.  Ownership is
# released only after the first fresh orbit goal has made the vehicle active.
ENTRY_ESTABLISHED_PHASES = ACTIVE_ORBIT_PHASES
RETURN_PHASES = {
    "WAIT_EXIT_PERMISSION", "GO_TO_EXIT_GATE", "NORMAL_RETURN",
    "RETURN_EGRESS", "RETURN_HOME", "FAILURE_LANDING", "DONE", "ERROR"
}


class SwarmManager:
    def __init__(self):
        self.uav_ids = [int(v) for v in rospy.get_param(
            "~uav_ids", [1, 2, 3])]
        if self.uav_ids != sorted(set(self.uav_ids)) or not self.uav_ids:
            raise rospy.ROSException("~uav_ids must be sorted and unique")
        self.interval = float(rospy.get_param("~takeoff_interval_sec", 3.0))
        self.timeout = float(rospy.get_param("~heartbeat_timeout", 1.0))
        self.trajectory_timeout = float(rospy.get_param(
            "~trajectory_timeout", self.timeout))
        self.minimum_3d = float(rospy.get_param(
            "~minimum_3d_separation", 3.0))
        self.clearance = float(rospy.get_param(
            "~optimizer_swarm_clearance", 1.5))
        self.heights = [float(v) for v in rospy.get_param(
            "~mission_heights", [2.0, 3.0, 4.0])]
        self.phases = [float(v) for v in rospy.get_param(
            "~phase_degrees", [270.0, 45.0, 135.0])]
        self.orbit_radius = float(rospy.get_param("~orbit_radius", 12.5))
        self.tower_center = [
            float(v) for v in rospy.get_param(
                "~tower_center", [-10.0551, 19.7104])]
        self.desired_phase = float(rospy.get_param(
            "~desired_phase_separation_degrees", 120.0))
        self.phase_tolerance = float(rospy.get_param(
            "~phase_tolerance_degrees", 15.0))
        self.entry_gate_radius = float(rospy.get_param(
            "~entry_gate_radius", 15.0))
        self.entry_nominal_speed = float(rospy.get_param(
            "~entry_nominal_speed", 0.20))
        self.orbit_nominal_speed = float(rospy.get_param(
            "~orbit_nominal_speed", 0.14))
        self.entry_settle_time = float(rospy.get_param(
            "~entry_settle_time", 0.0))
        self.homes = rospy.get_param(
            "~home_positions", [[0.0, 0.0, 0.0],
                                [4.0, 0.0, 0.0],
                                [8.0, 0.0, 0.0]])
        self.takeoff_heights = [float(v) for v in rospy.get_param(
            "~takeoff_heights", self.heights)]
        expected = len(self.uav_ids)
        if not all(len(values) == expected for values in (
                self.heights, self.phases, self.homes,
                self.takeoff_heights)):
            raise rospy.ROSException("per-UAV coordination arrays mismatch")
        if not all(math.isfinite(v) for v in (
                [self.interval, self.timeout, self.trajectory_timeout,
                 self.minimum_3d, self.clearance, self.orbit_radius,
                 self.desired_phase, self.phase_tolerance,
                 self.entry_gate_radius, self.entry_nominal_speed,
                 self.orbit_nominal_speed, self.entry_settle_time]
                + self.heights + self.phases + self.takeoff_heights)):
            raise rospy.ROSException("coordination parameters must be finite")
        if (len(self.tower_center) != 2
                or not all(math.isfinite(v) for v in self.tower_center)
                or self.desired_phase <= 0.0
                or self.desired_phase >= 180.0
                or self.phase_tolerance < 0.0
                or self.entry_gate_radius <= 0.0
                or self.entry_nominal_speed <= 0.0
                or self.orbit_nominal_speed <= 0.0
                or self.entry_settle_time < 0.0):
            raise rospy.ROSException("invalid orbit phase coordination")
        self.geometry_safe = mission_geometry_clear(
            self.heights, self.phases, self.orbit_radius,
            self.minimum_3d, self.clearance)
        self.states = {}
        self.received = {}
        self.predictions = {}
        self.prediction_received = {}
        self.safety_clear = False
        self.safety_received = None
        self.px4_ready = False
        self.schedule_started = None
        self.entry_owner = 0
        self.exit_owner = 0
        self.orbit_released = set()
        self.phase_held = set()
        self.task_started = False
        self.last_permissions = {}
        self.last_coordinator_state = ""
        self.last_coordinator_reason = ""
        self.condition_started = rospy.Time.now()

        for uid in self.uav_ids:
            rospy.Subscriber(
                "/uav{}/swarm/state".format(uid), SwarmState,
                lambda msg, u=uid: self.state_cb(u, msg), queue_size=5)
            rospy.Subscriber(
                "/uav{}/swarm/predicted_trajectory".format(uid),
                PredictedTrajectory,
                lambda msg, u=uid: self.prediction_cb(u, msg), queue_size=5)
        rospy.Subscriber("/swarm/safety/clear", Bool,
                         self.safety_cb, queue_size=5)
        rospy.Subscriber(
            "/swarm/px4_params_ready", Bool,
            lambda msg: setattr(self, "px4_ready", msg.data), queue_size=1)
        self.pubs = {}
        for category in (
                "takeoff", "task_start", "entry", "orbit", "exit",
                "transition", "landing"):
            self.pubs[category] = {
                uid: rospy.Publisher(
                    "/uav{}/swarm/{}_permission".format(uid, category),
                    Bool, queue_size=1, latch=True)
                for uid in self.uav_ids}
        self.status_pub = rospy.Publisher(
            "/swarm/coordinator/status", CoordinationStatus,
            queue_size=1, latch=True)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def state_cb(self, uid, msg):
        self.states[uid] = msg
        self.received[uid] = rospy.Time.now()

    def prediction_cb(self, uid, msg):
        self.predictions[uid] = msg
        self.prediction_received[uid] = rospy.Time.now()

    def safety_cb(self, msg):
        self.safety_clear = msg.data
        self.safety_received = rospy.Time.now()

    def healthy(self, uid, now):
        state = self.states.get(uid)
        received = self.received.get(uid)
        prediction = self.predictions.get(uid)
        prediction_received = self.prediction_received.get(uid)
        return bool(
            state is not None and received is not None
            and prediction is not None and prediction_received is not None
            and (now - received).to_sec() <= self.timeout
            and (now - prediction_received).to_sec()
            <= self.trajectory_timeout
            and prediction.uav_id == uid
            and prediction.header.frame_id == "world"
            and bool(prediction.points)
            and state.heartbeat_ok and state.localization_valid
            and state.flight_state not in {"ERROR", "FAILSAFE", "HOLD_SAFE"}
            and state.mission_phase not in {"ERROR", "FAILSAFE", "HOLD_SAFE"})

    def takeoff_corridor_clear(self, uid):
        index = self.uav_ids.index(uid)
        for peer in self.uav_ids:
            if peer == uid:
                continue
            points = [
                (p.x, p.y, p.z)
                for p in self.predictions[peer].points]
            if not corridor_clear(
                    points, self.homes[index], self.takeoff_heights[index],
                    self.minimum_3d, self.clearance):
                return False
        return True

    def projected_entry_phase_clear(self, uid, now):
        """Predict the measured phase when this UAV reaches its ENTRY gate."""
        prior_ids = [
            peer for peer in self.uav_ids
            if peer in self.orbit_released and peer != uid]
        if not prior_ids:
            return True
        if not all(
                self.healthy(peer, now)
                and self.states[peer].mission_phase in ACTIVE_ORBIT_PHASES
                for peer in prior_ids):
            return False
        index = self.uav_ids.index(uid)
        angle = math.radians(self.phases[index])
        gate = (
            self.tower_center[0] + self.entry_gate_radius * math.cos(angle),
            self.tower_center[1] + self.entry_gate_radius * math.sin(angle))
        current = self.states[uid].pose.position
        eta = (
            math.hypot(current.x - gate[0], current.y - gate[1])
            / self.entry_nominal_speed + self.entry_settle_time)
        angular_advance = (
            self.orbit_nominal_speed / self.orbit_radius * eta)
        projected = []
        for peer in prior_ids:
            position = self.states[peer].pose.position
            point = rotate_xy_about_center(
                (position.x, position.y), self.tower_center,
                angular_advance, self.orbit_radius)
            if point is None:
                return False
            projected.append(point)
        return orbit_release_allowed(
            gate, projected, self.tower_center,
            self.desired_phase, self.phase_tolerance)

    def timer_cb(self, _event):
        now = rospy.Time.now()
        healthy_all = all(self.healthy(uid, now) for uid in self.uav_ids)
        globally_clear = (
            healthy_all and self.px4_ready and self.safety_clear
            and self.safety_received is not None
            and (now - self.safety_received).to_sec() <= self.timeout
            and self.geometry_safe)
        if self.schedule_started is None and globally_clear:
            self.schedule_started = now
        elapsed = (
            (now - self.schedule_started).to_sec()
            if self.schedule_started is not None else -1.0)

        takeoff = {}
        for index, uid in enumerate(self.uav_ids):
            corridor_ok = (
                globally_clear and self.takeoff_corridor_clear(uid))
            takeoff[uid] = scheduled_takeoff_allowed(
                index, elapsed, self.interval, healthy_all,
                self.px4_ready, self.safety_clear,
                self.geometry_safe, corridor_ok)

        health = {uid: self.healthy(uid, now) for uid in self.uav_ids}
        barrier_ready = task_start_barrier_ready(
            self.uav_ids, self.states, health, globally_clear)
        if not self.task_started and barrier_ready:
            self.task_started = True
            rospy.logwarn(
                "[SWARM_COORD] all UAVs HOVER_READY; unified task-start "
                "barrier released uavs=%s takeoff_interval=%.3f",
                self.uav_ids, self.interval)

        # ENTRY is exclusive until the owner has established its configured
        # orbit phase.  Loss of owner health keeps ownership latched and stops
        # all new entrants.
        if self.entry_owner:
            owner_uid = self.entry_owner
            owner_healthy = self.healthy(owner_uid, now)
            owner_phase = (
                self.states[owner_uid].mission_phase
                if owner_healthy else "")
            # HOLDING is shared by pre-entry map/goal failures and by the
            # coordinator's dynamic orbit pause.  It establishes an orbit
            # only for a vehicle that was already released while genuinely
            # active; otherwise a failed ENTRY candidate could be mistaken
            # for a third orbit participant and release the corridor.
            owner_orbit_established = entry_owner_orbit_established(
                owner_phase, owner_uid in self.orbit_released,
                ENTRY_ESTABLISHED_PHASES)
            if owner_orbit_established:
                self.orbit_released.add(owner_uid)
            # A failed ingress may retrace the same corridor.  Transfer sticky
            # ownership to serialized return/landing instead of deadlocking
            # ENTRY or admitting a second vehicle into that corridor.
            self.entry_owner, self.exit_owner = advance_entry_owner(
                self.entry_owner, self.exit_owner, owner_healthy,
                owner_orbit_established,
                owner_phase in RETURN_PHASES)
        entry_waiting = eligible_entry_ids(
            self.uav_ids, self.states,
            {uid: self.healthy(uid, now) for uid in self.uav_ids})
        next_entry = min(entry_waiting) if entry_waiting else 0
        # ENTRY admission is a separate corridor lease from orbit release, but
        # it must also prove the future phase at the gate.  A slow ingress
        # vehicle can otherwise arrive after the leader has rotated onto the
        # same XY gate; vertical 2/3/4 m spacing is not a substitute for this
        # predicted geometric check.
        phase_ready = bool(
            next_entry and self.projected_entry_phase_clear(next_entry, now))
        if (globally_clear and not self.exit_owner
                and (self.entry_owner or phase_ready)):
            entry_grants, self.entry_owner = serialized_permissions(
                entry_waiting, self.entry_owner)
        else:
            entry_grants = {}

        # A vehicle hovers at its own ENTRY_GATE until its XY angle is about
        # 120 degrees from every already released vehicle.  This converts the
        # nominal gate phases into a measured, time-domain orbit phase and
        # prevents a late ingress vehicle from catching the preceding one.
        orbit_waiting = [
            uid for uid in self.uav_ids
            if self.healthy(uid, now)
            and self.states[uid].mission_phase == "WAIT_ORBIT_PERMISSION"]
        for uid in orbit_waiting:
            if uid in self.orbit_released:
                continue
            prior_ids = [
                peer for peer in self.uav_ids
                if peer in self.orbit_released and peer != uid]
            if (not all(
                    self.healthy(peer, now)
                    and self.states[peer].mission_phase in ACTIVE_ORBIT_PHASES
                    for peer in prior_ids)):
                continue
            waiting = self.states[uid].pose.position
            orbiting = [
                (self.states[peer].pose.position.x,
                 self.states[peer].pose.position.y)
                for peer in prior_ids]
            if orbit_release_allowed(
                    (waiting.x, waiting.y), orbiting, self.tower_center,
                    self.desired_phase, self.phase_tolerance):
                self.orbit_released.add(uid)

        # Entry release establishes the requested phase once.  Real EGO
        # detours can subsequently slow one vehicle, so continuously pause
        # only the CCW trailing vehicle when its forward phase gap reaches the
        # lower edge of the configured band.  The mission cancels that
        # vehicle's current B-spline and resumes with a fresh goal generation.
        active_orbit_positions = {
            uid: (
                self.states[uid].pose.position.x,
                self.states[uid].pose.position.y)
            for uid in self.uav_ids
            if uid in self.orbit_released
            and self.healthy(uid, now)
            and self.states[uid].mission_phase in ACTIVE_ORBIT_PHASES
        }
        self.phase_held = orbit_phase_hold_ids(
            active_orbit_positions, self.tower_center,
            self.desired_phase, self.phase_tolerance, self.phase_held)

        # EXIT owner remains sticky through return and landing.  A stale owner
        # is never replaced because its possible occupancy is unknown.
        if self.exit_owner and self.healthy(self.exit_owner, now):
            if self.states[self.exit_owner].mission_phase == "DONE":
                self.exit_owner = 0
        exit_waiting = [
            uid for uid in self.uav_ids
            if self.healthy(uid, now)
            and self.states[uid].mission_phase == "WAIT_EXIT_PERMISSION"]
        if globally_clear and not self.entry_owner:
            exit_grants, self.exit_owner = serialized_permissions(
                exit_waiting, self.exit_owner)
        else:
            exit_grants = {}

        permissions = {
            "takeoff": takeoff,
            "task_start": {
                uid: bool(globally_clear and self.task_started)
                for uid in self.uav_ids},
            "entry": {
                uid: bool(globally_clear and uid == self.entry_owner)
                for uid in self.uav_ids},
            "orbit": {
                uid: bool(
                    globally_clear and uid in self.orbit_released
                    and uid not in self.phase_held)
                for uid in self.uav_ids},
            "exit": {
                uid: bool(globally_clear and uid == self.exit_owner)
                for uid in self.uav_ids},
            "transition": {uid: globally_clear for uid in self.uav_ids},
            "landing": {
                uid: bool(
                    globally_clear and uid == self.exit_owner
                    and self.states[uid].flight_state == "HOME_HOVER")
                for uid in self.uav_ids},
        }
        for category, publishers in self.pubs.items():
            for uid, publisher in publishers.items():
                publisher.publish(Bool(data=permissions[category][uid]))
                key = (category, uid)
                previous = self.last_permissions.get(key)
                current = permissions[category][uid]
                if previous is not None and previous != current:
                    state = self.states.get(uid)
                    rospy.logwarn(
                        "[SWARM_COORD] uav=%d permission=%s value=%s "
                        "mission_state=%s flight_state=%s entry_owner=%d "
                        "exit_owner=%d phase_held=%s action=%s",
                        uid, category, current,
                        getattr(state, "mission_phase", "MISSING"),
                        getattr(state, "flight_state", "MISSING"),
                        self.entry_owner, self.exit_owner,
                        sorted(self.phase_held),
                        "RELEASE" if current else "FAIL_CLOSED_HOLD")
                self.last_permissions[key] = current

        status = CoordinationStatus()
        status.header.stamp = now
        status.header.frame_id = "world"
        status.uav1_takeoff_allowed = takeoff.get(1, False)
        status.uav2_takeoff_allowed = takeoff.get(2, False)
        status.uav1_transition_allowed = permissions["transition"].get(
            1, False)
        status.uav2_transition_allowed = permissions["transition"].get(
            2, False)
        status.uav1_landing_allowed = permissions["landing"].get(1, False)
        status.uav2_landing_allowed = permissions["landing"].get(2, False)
        if not self.geometry_safe:
            status.coordinator_state = "SAFETY_INHIBIT"
            status.reason = "configured phase geometry violates EGO ellipsoid"
        elif not globally_clear:
            status.coordinator_state = "SAFETY_INHIBIT"
            status.reason = (
                "heartbeat/localization/trajectory/PX4/safety gate not clear")
        elif self.exit_owner:
            status.coordinator_state = "RETURN_COORDINATION"
            status.reason = "exclusive EXIT/return/landing owner={}".format(
                self.exit_owner)
        elif self.entry_owner:
            status.coordinator_state = "ENTRY_COORDINATION"
            status.reason = "exclusive ENTRY owner={}".format(self.entry_owner)
        elif self.phase_held:
            status.coordinator_state = "PHASE_COORDINATION"
            status.reason = (
                "dynamic phase hold trailing UAV(s)={}; "
                "fresh-goal resume uses {:.1f}/{:.1f} deg hysteresis"
                .format(
                    sorted(self.phase_held),
                    self.desired_phase - self.phase_tolerance,
                    self.desired_phase - 0.5 * self.phase_tolerance))
        elif orbit_waiting or (entry_waiting and not phase_ready):
            status.coordinator_state = "PHASE_COORDINATION"
            status.reason = (
                "waiting for measured {:.1f}+/-{:.1f} deg orbit phase"
                .format(self.desired_phase, self.phase_tolerance))
        elif all(self.states[uid].mission_phase == "DONE"
                 for uid in self.uav_ids):
            status.coordinator_state = "MISSION_COMPLETE"
            status.reason = "all missions complete"
        else:
            status.coordinator_state = "ORBIT_COORDINATION"
            status.reason = "phase-separated mission geometry is clear"
        if (status.coordinator_state != self.last_coordinator_state or
                status.reason != self.last_coordinator_reason):
            duration = max(0.0, (now - self.condition_started).to_sec())
            rospy.logwarn(
                "[SWARM_COORD] state=%s previous=%s duration=%.3fs "
                "reason=%s entry_owner=%d exit_owner=%d phase_held=%s "
                "task_barrier=%s action=%s",
                status.coordinator_state,
                self.last_coordinator_state or "INIT", duration,
                status.reason, self.entry_owner, self.exit_owner,
                sorted(self.phase_held), self.task_started,
                "INHIBIT_NEW_PERMISSIONS"
                if status.coordinator_state == "SAFETY_INHIBIT"
                else "CONTINUE_COORDINATION")
            self.last_coordinator_state = status.coordinator_state
            self.last_coordinator_reason = status.reason
            self.condition_started = now
        self.status_pub.publish(status)


if __name__ == "__main__":
    rospy.init_node("astra_swarm_manager")
    SwarmManager()
    rospy.spin()
