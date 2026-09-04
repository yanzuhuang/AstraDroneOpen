#!/usr/bin/env python3
"""Fail-closed temporal/spatial coordinator for the tower swarm."""

import json
import math

import rospy
from astra_swarm_manager.policy import (
    completed_predecessor_handoff_allowed,
    contiguous_role_segments,
    corridor_clear,
    directed_phase_gap_degrees,
    entry_ready_barrier,
    formation_phase_decision,
    formation_speed_scale_targets,
    incremental_entry_corridor_selection,
    mission_geometry_clear,
    orbit_staging_ready_barrier,
    predicted_pair_clear,
    reconcile_entry_corridor_commitments,
    role_chain_hold_ids,
    scheduled_takeoff_allowed,
    sequential_orbit_release_allowed,
    serialized_permissions,
    serialized_transition_permissions,
    slew_speed_scale,
    task_start_barrier_ready,
)
from astra_swarm_msgs.msg import (
    CoordinationStatus,
    PredictedTrajectory,
    SwarmState,
)
from astra_custom_msgs.msg import InspectionCandidateArray
from std_msgs.msg import Bool, Float64, String


ACTIVE_ORBIT_PHASES = {
    "TARGET_LOCKED", "NAVIGATING", "EVALUATING", "RELOCATING", "RECOVERING",
    "HOLDING"
}
RETURN_PHASES = {
    "WAIT_EXIT_PERMISSION", "GO_TO_EXIT_GATE", "NORMAL_RETURN",
    "RETURN_EGRESS", "HOME_OVERHEAD_TRANSIT", "SEGMENTED_HOME_DESCENT",
    "RETURN_HOME", "FAILURE_LANDING", "DONE", "ERROR"
}
SAFE_COMPLETED_HANDOFF_PHASES = {
    "WAIT_EXIT_PERMISSION", "GO_TO_EXIT_GATE", "NORMAL_RETURN",
    "RETURN_EGRESS", "HOME_OVERHEAD_TRANSIT", "SEGMENTED_HOME_DESCENT",
    "RETURN_HOME", "DONE"
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
        # ENTRY candidates publish distance to EGO's already-inflated map.
        # This is the task-side additional margin, not the raw-cloud minimum.
        self.map_additional_clearance = float(rospy.get_param(
            "~map_additional_clearance", 0.5))
        self.clearance = float(rospy.get_param(
            "~optimizer_swarm_clearance", 1.5))
        self.heights = [float(v) for v in rospy.get_param(
            "~mission_heights", [3.0, 3.0, 3.0])]
        self.phases = [float(v) for v in rospy.get_param(
            "~phase_degrees", [292.5, 315.0, 337.5])]
        self.orbit_radius = float(rospy.get_param("~orbit_radius", 12.5))
        self.tower_center = [
            float(v) for v in rospy.get_param(
                "~tower_center", [-10.0551, 19.7104])]
        self.desired_phase = float(rospy.get_param(
            "~target_phase_deg", 67.5))
        self.normal_phase_min = float(rospy.get_param(
            "~normal_phase_min_deg", 57.5))
        self.normal_phase_max = float(rospy.get_param(
            "~normal_phase_max_deg", 77.5))
        self.warning_phase_min = float(rospy.get_param(
            "~warning_phase_min_deg", 45.0))
        self.emergency_phase = float(rospy.get_param(
            "~emergency_phase_deg", 45.0))
        self.leader_wait_phase = float(rospy.get_param(
            "~leader_wait_phase_deg", 95.0))
        self.direction = int(rospy.get_param("~orbit_direction", 1))
        default_role_order = list(reversed(self.uav_ids))
        self.role_order = [int(uid) for uid in rospy.get_param(
            "~formation_role_order", default_role_order)]
        self.multi_layer_enabled = bool(rospy.get_param(
            "~multi_layer_enabled", False))
        self.transition_order = [int(uid) for uid in rospy.get_param(
            "~transition_order", self.role_order)]
        transition_targets = [float(value) for value in rospy.get_param(
            "~transition_target_heights", self.heights)]
        self.transition_target_heights = dict(zip(
            self.uav_ids, transition_targets))
        self.transition_height_tolerance = float(rospy.get_param(
            "~transition_height_tolerance", 0.35))
        self.transition_maximum_speed = float(rospy.get_param(
            "~transition_maximum_speed", 0.20))
        self.leader_uav_id = self.role_order[0]
        self.middle_uav_id = (
            self.role_order[1] if len(self.role_order) > 2
            else self.role_order[0])
        self.trailing_uav_id = self.role_order[-1]
        self.entry_height_tolerance = float(rospy.get_param(
            "~entry_height_tolerance", 0.35))
        self.entry_maximum_speed = float(rospy.get_param(
            "~entry_maximum_speed", 0.20))
        self.entry_ready_timeout = float(rospy.get_param(
            "~entry_ready_timeout", 360.0))
        self.entry_gate_radius = float(rospy.get_param(
            "~entry_gate_radius", 15.0))
        self.entry_nominal_speed = float(rospy.get_param(
            "~entry_nominal_speed", 0.20))
        self.orbit_nominal_speed = float(rospy.get_param(
            "~orbit_nominal_speed", 0.14))
        self.layer_transition_nominal_speed = float(rospy.get_param(
            "~layer_transition_nominal_speed", 0.30))
        self.minimum_warning_speed_scale = float(rospy.get_param(
            "~minimum_warning_speed_scale", 0.35))
        self.speed_scale_rise_rate = float(rospy.get_param(
            "~speed_scale_rise_rate", 0.15))
        self.speed_scale_fall_rate = float(rospy.get_param(
            "~speed_scale_fall_rate", 1.5))
        self.entry_settle_time = float(rospy.get_param(
            "~entry_settle_time", 0.0))
        self.release_phase_min = float(rospy.get_param(
            "~release_phase_min_deg", 65.0))
        self.release_phase_max = float(rospy.get_param(
            "~release_phase_max_deg", 70.0))
        self.release_minimum_forward_speed = float(rospy.get_param(
            "~release_minimum_forward_speed", 0.03))
        self.homes = rospy.get_param(
            "~home_positions", [[0.0, 0.0, 0.0],
                                [4.0, 0.0, 0.0],
                                [8.0, 0.0, 0.0]])
        self.takeoff_heights = [float(v) for v in rospy.get_param(
            "~takeoff_heights", self.heights)]
        expected = len(self.uav_ids)
        if not all(len(values) == expected for values in (
                self.heights, self.phases, self.homes, transition_targets,
                self.takeoff_heights)):
            raise rospy.ROSException("per-UAV coordination arrays mismatch")
        if not all(math.isfinite(v) for v in (
                [self.interval, self.timeout, self.trajectory_timeout,
                 self.minimum_3d, self.clearance, self.orbit_radius,
                 self.desired_phase, self.normal_phase_min,
                 self.normal_phase_max, self.warning_phase_min,
                 self.emergency_phase, self.leader_wait_phase,
                 self.entry_height_tolerance, self.entry_maximum_speed,
                 self.entry_ready_timeout,
                 self.entry_gate_radius, self.entry_nominal_speed,
                 self.orbit_nominal_speed, self.layer_transition_nominal_speed,
                 self.entry_settle_time, self.transition_height_tolerance,
                 self.transition_maximum_speed,
                 self.release_phase_min, self.release_phase_max,
                 self.release_minimum_forward_speed,
                 self.minimum_warning_speed_scale,
                 self.speed_scale_rise_rate, self.speed_scale_fall_rate]
                + self.heights + self.phases + self.takeoff_heights)):
            raise rospy.ROSException("coordination parameters must be finite")
        if (len(self.tower_center) != 2
                or not all(math.isfinite(v) for v in self.tower_center)
                or set(self.role_order) != set(self.uav_ids)
                or self.transition_order != self.role_order
                or set(self.transition_target_heights) != set(self.uav_ids)
                or self.direction not in (-1, 1)
                or not (0.0 < self.emergency_phase
                        <= self.warning_phase_min
                        <= self.normal_phase_min < self.desired_phase
                        < self.normal_phase_max < self.leader_wait_phase
                        < 180.0)
                or self.entry_height_tolerance <= 0.0
                or self.entry_maximum_speed <= 0.0
                or self.entry_ready_timeout <= 0.0
                or self.entry_gate_radius <= 0.0
                or self.entry_nominal_speed <= 0.0
                or self.orbit_nominal_speed <= 0.0
                or self.layer_transition_nominal_speed <= 0.0
                or self.layer_transition_nominal_speed
                > self.orbit_nominal_speed
                or self.transition_height_tolerance <= 0.0
                or self.transition_maximum_speed < 0.0
                or not (0.0 < self.minimum_warning_speed_scale <= 1.0)
                or self.speed_scale_rise_rate <= 0.0
                or self.speed_scale_fall_rate <= 0.0
                or self.entry_settle_time < 0.0):
            raise rospy.ROSException("invalid orbit phase coordination")
        if (not (0.0 < self.release_phase_min < self.desired_phase
                 < self.release_phase_max < self.normal_phase_max)
                or self.release_minimum_forward_speed <= 0.0):
            raise rospy.ROSException("invalid sequential release gate")
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
        self.entry_owner = 0  # compatibility diagnostic; independent gates use 0
        self.exit_owner = 0   # compatibility diagnostic; independent exits use 0
        self.landing_owner = 0
        self.orbit_released = set()
        self.orbit_release_times = {}
        self.orbit_completed = set()
        self.orbit_staging_started = False
        self.orbit_staging_start_time = None
        self.orbit_staging_granted = set()
        self.orbit_staging_grant_times = {}
        self.orbit_staging_ready_reasons = {
            uid: "WAITING_FOR_FIRST_ORBIT_POINT" for uid in self.uav_ids}
        self.formation_orbit_active = False
        self.transition_owner = 0
        self.transition_started = set()
        self.transition_completed = set()
        self.transition_reasons = {
            uid: ("WAITING_FOR_LAYER" if self.multi_layer_enabled
                  else "MULTI_LAYER_DISABLED") for uid in self.uav_ids}
        self.release_diagnostics = {}
        self.phase_held = set()
        self.phase_gaps = {}
        self.phase_bands = {}
        self.speed_scales = {uid: 1.0 for uid in self.uav_ids}
        self.speed_scale_targets = {uid: 1.0 for uid in self.uav_ids}
        self.last_speed_scale_update = rospy.Time.now()
        self.task_started = False
        self.entry_corridor_messages = {}
        self.entry_corridor_received = {}
        self.entry_corridor_selection = {}
        self.entry_corridor_commit_state = {
            uid: {
                "selected_candidate_id": "",
                "candidate_generation": None,
                "committed": False,
                "entered": False,
                "released": False,
                "mission_lock_observed": False,
            }
            for uid in self.uav_ids}
        self.entry_corridor_generation_key = None
        self.entry_corridor_selection_reason = "WAITING_FOR_CANDIDATES"
        self.entry_corridor_diagnostics = {}
        self.entry_corridor_audit = {}
        # Compatibility summary: true only when all three individual releases
        # have occurred.  Per-UAV release times are authoritative.
        self.orbit_started = False
        self.orbit_start_time = None
        self.entry_ready_started = None
        self.entry_ready_reasons = {
            uid: "WAITING_FOR_GATE" for uid in self.uav_ids}
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
            rospy.Subscriber(
                "/uav{}/tower_mission/entry_corridor_candidates".format(uid),
                InspectionCandidateArray,
                lambda msg, u=uid: self.entry_corridor_cb(u, msg),
                queue_size=1)
        rospy.Subscriber("/swarm/safety/clear", Bool,
                         self.safety_cb, queue_size=5)
        rospy.Subscriber(
            "/swarm/px4_params_ready", Bool,
            lambda msg: setattr(self, "px4_ready", msg.data), queue_size=1)
        self.pubs = {}
        for category in (
                "takeoff", "task_start", "entry", "orbit_staging",
                "orbit", "exit",
                "transition", "landing"):
            self.pubs[category] = {
                uid: rospy.Publisher(
                    "/uav{}/swarm/{}_permission".format(uid, category),
                    Bool, queue_size=1, latch=True)
                for uid in self.uav_ids}
        self.status_pub = rospy.Publisher(
            "/swarm/coordinator/status", CoordinationStatus,
            queue_size=1, latch=True)
        self.formation_status_pub = rospy.Publisher(
            "/swarm/formation/status", String, queue_size=1, latch=True)
        self.speed_scale_pubs = {
            uid: rospy.Publisher(
                "/uav{}/swarm/orbit_speed_scale".format(uid), Float64,
                queue_size=1, latch=True)
            for uid in self.uav_ids}
        self.entry_corridor_selection_pubs = {
            uid: rospy.Publisher(
                "/uav{}/swarm/entry_corridor_selection".format(uid), String,
                queue_size=1, latch=True)
            for uid in self.uav_ids}
        self.entry_corridor_audit_pub = rospy.Publisher(
            "/swarm/entry_corridor_audit", String,
            queue_size=1, latch=True)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def state_cb(self, uid, msg):
        self.states[uid] = msg
        self.received[uid] = rospy.Time.now()

    def prediction_cb(self, uid, msg):
        self.predictions[uid] = msg
        self.prediction_received[uid] = rospy.Time.now()

    def entry_corridor_cb(self, uid, msg):
        self.entry_corridor_messages[uid] = msg
        self.entry_corridor_received[uid] = rospy.Time.now()

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

    def pair_prediction_clear(self, first_uid, second_uid):
        first_state = self.states.get(first_uid)
        second_state = self.states.get(second_uid)
        first_prediction = self.predictions.get(first_uid)
        second_prediction = self.predictions.get(second_uid)
        if (first_state is None or second_state is None
                or first_prediction is None or second_prediction is None):
            return False
        first_position = (
            first_state.pose.position.x, first_state.pose.position.y,
            first_state.pose.position.z)
        second_position = (
            second_state.pose.position.x, second_state.pose.position.y,
            second_state.pose.position.z)
        first_points = [(p.x, p.y, p.z) for p in first_prediction.points]
        second_points = [(p.x, p.y, p.z) for p in second_prediction.points]
        return predicted_pair_clear(
            first_position, second_position, first_points, second_points,
            self.minimum_3d, self.clearance)

    def evaluate_sequential_release(self, leader_uid, follower_uid, now,
                                    globally_clear):
        leader = self.states.get(leader_uid)
        follower = self.states.get(follower_uid)
        if leader is None or follower is None:
            return False, float("nan"), float("nan"), {
                "states_received": False}
        follower_ready = (
            self.healthy(follower_uid, now)
            and follower.mission_phase == "ORBIT_STAGING_READY"
            and self.orbit_staging_ready_reasons.get(follower_uid) == "READY"
            and follower.flight_state not in {
                "HOLD_SAFE", "ERROR", "FAILSAFE", "SAFETY_INHIBIT"})
        leader_prediction_fresh = self.healthy(leader_uid, now)
        leader_position = (
            leader.pose.position.x, leader.pose.position.y)
        follower_position = (
            follower.pose.position.x, follower.pose.position.y)
        leader_velocity = (leader.velocity.x, leader.velocity.y)
        return sequential_orbit_release_allowed(
            follower_position, leader_position, leader_velocity,
            self.tower_center, self.direction,
            self.release_phase_min, self.release_phase_max,
            self.release_minimum_forward_speed,
            leader_prediction_fresh, follower_ready,
            self.pair_prediction_clear(leader_uid, follower_uid),
            globally_clear)

    def evaluate_completed_predecessor_handoff(
            self, leader_uid, follower_uid, now, globally_clear):
        leader = self.states.get(leader_uid)
        follower = self.states.get(follower_uid)
        if leader is None or follower is None:
            return False, {"states_received": False}
        follower_ready = (
            self.healthy(follower_uid, now)
            and follower.mission_phase == "ORBIT_STAGING_READY"
            and self.orbit_staging_ready_reasons.get(follower_uid) == "READY"
            and follower.flight_state not in {
                "HOLD_SAFE", "ERROR", "FAILSAFE", "SAFETY_INHIBIT"})
        leader_position = (
            leader.pose.position.x, leader.pose.position.y,
            leader.pose.position.z)
        follower_position = (
            follower.pose.position.x, follower.pose.position.y,
            follower.pose.position.z)
        return completed_predecessor_handoff_allowed(
            follower_position, leader_position,
            leader_uid in self.orbit_completed,
            leader.mission_phase in SAFE_COMPLETED_HANDOFF_PHASES,
            self.healthy(leader_uid, now), follower_ready,
            self.pair_prediction_clear(leader_uid, follower_uid),
            globally_clear, self.minimum_3d, self.clearance)

    def release_orbit(self, uid, now, reason):
        if uid in self.orbit_released:
            return
        self.orbit_released.add(uid)
        self.orbit_release_times[uid] = now
        if uid in self.entry_corridor_commit_state:
            self.entry_corridor_commit_state[uid]["entered"] = True
            self.entry_corridor_commit_state[uid]["released"] = True
        rospy.logwarn(
            "[SWARM_COORD] ORBIT_RELEASE_UAV%d published time=%.3f "
            "roles=%s reason=%s",
            uid, now.to_sec(), self.role_order, reason)

    def entry_corridor_candidates(self, uid):
        message = self.entry_corridor_messages.get(uid)
        if message is None:
            return []
        home = self.homes[self.uav_ids.index(uid)]
        grouped = {}
        for item in message.candidates:
            if not item.accepted or "/" not in item.candidate_id:
                continue
            corridor_id, kind = item.candidate_id.rsplit("/", 1)
            if (kind not in {"PRE_ENTRY", "ENTRY_GATE", "ORBIT_STAGING"}
                    and not kind.startswith("PATH_")):
                continue
            point = (
                float(item.target.x) + float(home[0]),
                float(item.target.y) + float(home[1]),
                float(item.target.z) + float(home[2]))
            values = grouped.setdefault(corridor_id, {})
            if kind.startswith("PATH_"):
                try:
                    values.setdefault("PATH", {})[int(kind[5:])] = point
                except ValueError:
                    continue
            else:
                values[kind] = point
            if kind == "ORBIT_STAGING":
                values["endpoint_clearance"] = float(item.clearance)
                values["endpoint_valid"] = bool(item.accepted)
                ego_marker = "EGO_STATUS="
                if ego_marker in item.rejection_reason:
                    values["ego_status"] = item.rejection_reason.split(
                        ego_marker, 1)[1].split(";", 1)[0]
                else:
                    values["ego_status"] = "NOT_PRECHECKED"
            else:
                values["path_clearance"] = min(
                    float(item.clearance),
                    values.get("path_clearance", float("inf")))
                values["path_valid"] = (
                    values.get("path_valid", True) and bool(item.accepted))
        candidates = []
        for corridor_id, values in grouped.items():
            if not all(key in values for key in (
                    "PRE_ENTRY", "ENTRY_GATE", "ORBIT_STAGING")):
                continue
            staging = values["ORBIT_STAGING"]
            pre = values["PRE_ENTRY"]
            entry = values["ENTRY_GATE"]
            path = [point for _, point in sorted(
                values.get("PATH", {}).items())]
            if not path:
                path = [
                    (self.states[uid].pose.position.x,
                     self.states[uid].pose.position.y,
                     self.states[uid].pose.position.z),
                    pre, entry, staging]
            angle = math.degrees(math.atan2(
                staging[1] - self.tower_center[1],
                staging[0] - self.tower_center[0])) % 360.0
            staging_radius = math.hypot(
                staging[0] - self.tower_center[0],
                staging[1] - self.tower_center[1])
            radius_tier = int(round((staging_radius - self.orbit_radius) / 2.0))
            ingress_length = sum(
                math.sqrt(sum((path[index][axis] - path[index - 1][axis]) ** 2
                              for axis in range(3)))
                for index in range(1, len(path)))
            candidates.append({
                "id": corridor_id,
                "angle_deg": angle,
                "pre_radius": math.hypot(
                    pre[0] - self.tower_center[0],
                    pre[1] - self.tower_center[1]),
                "entry_radius": math.hypot(
                    entry[0] - self.tower_center[0],
                    entry[1] - self.tower_center[1]),
                "orbit_staging_radius": staging_radius,
                "radius_tier": radius_tier,
                "endpoint_clearance": values.get(
                    "endpoint_clearance", float("nan")),
                "endpoint_valid": values.get("endpoint_valid", False),
                "path_valid": values.get("path_valid", False),
                "ego_status": values.get("ego_status", "NOT_PRECHECKED"),
                "ego_candidate_valid": values.get("ego_status")
                != "PLANNER_UNREACHABLE",
                "clearance": values.get("path_clearance", -1.0),
                "pre": pre,
                "entry": entry,
                "staging": staging,
                "path": path,
                "ingress_length": ingress_length,
            })
        return candidates

    def safely_entered_entry_roles(self):
        safe_phases = {
            "ENTRY_READY", "ORBIT_STAGING_READY",
            "EVALUATING", "TARGET_LOCKED", "NAVIGATING", "RELOCATING",
            "RECOVERING", "WAIT_TRANSITION_PERMISSION",
            "LAYER_TRANSITION", "WAIT_EXIT_PERMISSION", "GO_TO_EXIT_GATE",
            "NORMAL_RETURN", "RETURN_EGRESS", "HOME_OVERHEAD_TRANSIT",
            "SEGMENTED_HOME_DESCENT", "RETURN_HOME", "DONE",
        }
        return {
            uid for uid, state in self.states.items()
            if state.mission_phase in safe_phases}

    def reconcile_entry_corridor_selection(self):
        current_generations = {
            uid: int(message.current_sector)
            for uid, message in self.entry_corridor_messages.items()}
        current_candidate_ids = {
            uid: {
                item.candidate_id.rsplit("/", 1)[0]
                for item in message.candidates
                if item.accepted and "/" in item.candidate_id}
            for uid, message in self.entry_corridor_messages.items()}
        current_locked_candidate_ids = {
            uid: str(message.locked_candidate_id)
            for uid, message in self.entry_corridor_messages.items()}
        (selection, commit_state, revoked,
         reason) = reconcile_entry_corridor_commitments(
            self.entry_corridor_selection,
            self.entry_corridor_commit_state,
            self.role_order,
            current_generations,
            current_candidate_ids,
            current_locked_candidate_ids,
            self.safely_entered_entry_roles(),
            self.orbit_released)
        self.entry_corridor_selection = selection
        self.entry_corridor_commit_state = commit_state
        if revoked:
            for uid in revoked:
                self.entry_corridor_selection_pubs[uid].publish(
                    String(data=""))
            self.entry_corridor_selection_reason = reason
            self.entry_corridor_diagnostics = {
                "reconciliation": reason,
                "revoked_roles": revoked,
                "preserved_roles": [
                    uid for uid in self.role_order if uid in selection],
            }
            rospy.logerr(
                "[SWARM_ENTRY_RECONCILE] reason=%s revoked=%s "
                "preserved=%s commit_state=%s action=REARBITRATE_FROM_UAV%d",
                reason, revoked,
                [uid for uid in self.role_order if uid in selection],
                self.entry_corridor_commit_state, revoked[0])
        self.entry_corridor_generation_key = tuple(
            (uid, self.entry_corridor_commit_state[uid][
                "candidate_generation"])
            for uid in self.role_order if uid in selection)
        return revoked, reason

    def update_entry_corridor_selection(self, now, globally_clear):
        revoked, reconciliation_reason = (
            self.reconcile_entry_corridor_selection())
        grants = {
            uid: bool(globally_clear and uid in self.entry_corridor_selection)
            for uid in self.uav_ids}
        if not (globally_clear and self.task_started):
            self.entry_corridor_selection_reason = "GLOBAL_SAFETY_INHIBIT"
            return {uid: False for uid in self.uav_ids}
        if len(self.entry_corridor_selection) == len(self.role_order):
            self.entry_corridor_selection_reason = "ALL_ROLES_COMMITTED"
            return grants

        next_uid = self.role_order[len(self.entry_corridor_selection)]
        state = self.states.get(next_uid)
        if (state is None or not self.healthy(next_uid, now)
                or state.mission_phase != "WAIT_ENTRY_PERMISSION"):
            self.entry_corridor_selection_reason = (
                "WAITING_FOR_UAV{}_ENTRY_READY".format(next_uid))
            self.entry_corridor_diagnostics = {
                "next_role": next_uid,
                "committed_role_order": [
                    uid for uid in self.role_order
                    if uid in self.entry_corridor_selection],
            }
            return grants
        fresh = bool(
            next_uid in self.entry_corridor_messages
            and next_uid in self.entry_corridor_received
            and (now - self.entry_corridor_received[next_uid]).to_sec()
            <= self.timeout
            and self.entry_corridor_messages[next_uid].candidates)
        if not fresh:
            self.entry_corridor_selection_reason = (
                "WAITING_FOR_UAV{}_FRESH_CANDIDATES".format(next_uid))
            self.entry_corridor_diagnostics = {
                "next_role": next_uid,
                "committed_role_order": [
                    uid for uid in self.role_order
                    if uid in self.entry_corridor_selection],
            }
            return grants

        candidates = {next_uid: self.entry_corridor_candidates(next_uid)}
        nominal_angles = dict(zip(self.uav_ids, self.phases))
        prefix = self.role_order[:len(self.entry_corridor_selection) + 1]
        starts = {
            uid: (self.states[uid].pose.position.x,
                  self.states[uid].pose.position.y,
                  self.states[uid].pose.position.z)
            for uid in prefix}
        committed_prediction_paths = {}
        for uid in self.entry_corridor_selection:
            current = starts[uid]
            committed_prediction_paths[uid] = [current] + [
                (point.x, point.y, point.z)
                for point in self.predictions[uid].points]
        previous_selection = dict(self.entry_corridor_selection)
        selected, grants, reason, diagnostics = (
            incremental_entry_corridor_selection(
            candidates, self.entry_corridor_selection, self.role_order,
            nominal_angles, self.tower_center,
            starts, self.direction, 20.0, 30.0,
            self.map_additional_clearance,
            self.minimum_3d, self.clearance,
            nominal_orbit_radius=self.orbit_radius,
            committed_prediction_paths=committed_prediction_paths,
            globally_clear=globally_clear))
        self.entry_corridor_selection_reason = reason
        candidate_outcomes = diagnostics.get("candidate_outcomes", {})
        self.entry_corridor_diagnostics = {
            key: value for key, value in diagnostics.items()
            if key != "candidate_outcomes"}
        self.entry_corridor_audit = {
            "stamp": now.to_sec(),
            "generations": self.entry_corridor_generation_key,
            "reason": reason,
            "next_role": next_uid,
            "committed_before": [
                uid for uid in self.role_order if uid in previous_selection],
            "selected_tier": diagnostics.get("selected_tier"),
            "tier_contract": diagnostics.get("tier_contract", {}),
            "tiers": diagnostics.get("tiers", {}),
            "selected_rank": diagnostics.get("selected_rank"),
            "candidate_outcomes": candidate_outcomes,
        }
        for tier, tier_result in sorted(
                diagnostics.get("tiers", {}).items(), key=lambda item: int(item[0])):
            rospy.logwarn(
                "[SWARM_ENTRY_TIER] tier=%s result=%s candidates=%s "
                "evaluated=%d valid=%d local=%d role=%d crossing=%d "
                "predicted=%d%s",
                tier, tier_result.get("result", "UNKNOWN"),
                tier_result.get("candidate_counts", {}),
                tier_result.get("evaluated", 0),
                tier_result.get("valid_combinations", 0),
                tier_result.get("local_hard_rejected", 0),
                tier_result.get("role_rejected", 0),
                tier_result.get("crossing_rejected", 0),
                tier_result.get("conflict_rejected", 0),
                "; entering next tier" if tier_result.get("result")
                != "SELECTED" else "; tier selected after full exhaustion")
        if len(selected) == len(previous_selection):
            self.entry_corridor_audit_pub.publish(String(
                data=json.dumps(self.entry_corridor_audit, sort_keys=True)))
            rospy.logerr_throttle(
                2.0,
                "[SWARM_ENTRY] no incremental safe corridor next=UAV%d "
                "committed=%s generations=%s "
                "candidate_counts=%s reason=%s diagnostics=%s",
                next_uid, sorted(previous_selection),
                self.entry_corridor_generation_key,
                {uid: len(values) for uid, values in candidates.items()},
                reason, self.entry_corridor_diagnostics)
            return grants
        self.entry_corridor_selection = selected
        added = [uid for uid in self.role_order
                 if uid in selected and uid not in previous_selection]
        for uid in added:
            generation = int(
                self.entry_corridor_messages[uid].current_sector)
            state = self.entry_corridor_commit_state[uid]
            state.update({
                "selected_candidate_id": selected[uid]["id"],
                "candidate_generation": generation,
                "committed": True,
                "entered": False,
                "released": False,
                "mission_lock_observed": False,
            })
            self.entry_corridor_selection_pubs[uid].publish(String(
                data=selected[uid]["id"]))
        self.entry_corridor_generation_key = tuple(
            (uid, self.entry_corridor_commit_state[uid][
                "candidate_generation"])
            for uid in self.role_order if uid in selected)
        if revoked:
            self.entry_corridor_audit["reconciliation"] = (
                reconciliation_reason)
            self.entry_corridor_audit["revoked_roles"] = revoked
        self.entry_corridor_audit["generations"] = (
            self.entry_corridor_generation_key)
        self.entry_corridor_audit_pub.publish(String(
            data=json.dumps(self.entry_corridor_audit, sort_keys=True)))
        rospy.logwarn(
            "[SWARM_ENTRY] incremental corridor latched added=%s "
            "committed_roles=%s generations=%s selection=%s diagnostics=%s",
            added, [uid for uid in self.role_order if uid in selected],
            self.entry_corridor_generation_key,
            {uid: {"id": value["id"],
                   "angle_deg": value["angle_deg"],
                   "pre_radius": value["pre_radius"],
                   "entry_radius": value["entry_radius"],
                   "orbit_staging_radius": value["orbit_staging_radius"],
                   "radius_tier": value["radius_tier"],
                   "clearance": value["clearance"],
                   "pre": value["pre"], "entry": value["entry"],
                   "staging": value["staging"]}
             for uid, value in selected.items()}, self.entry_corridor_diagnostics)
        return grants

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

        # Each mission first proves bounded live-map corridors. The manager
        # commits one fixed-role prefix at a time (UAV3 -> UAV2 -> UAV1), and
        # grants only the roles already proved safe. A committed-but-not-yet-
        # entered generation change rolls back that role and its downstream
        # suffix; safely entered predecessors remain sticky.
        entry_grants = self.update_entry_corridor_selection(
            now, globally_clear)

        _all_entry_ready, self.entry_ready_reasons = entry_ready_barrier(
            self.uav_ids, self.states, health, self.heights,
            self.entry_height_tolerance, self.entry_maximum_speed,
            globally_clear)
        any_entry_ready = any(
            reason == "READY"
            for reason in self.entry_ready_reasons.values())
        if any_entry_ready and self.entry_ready_started is None:
            self.entry_ready_started = now
        for uid in self.role_order:
            if (uid not in self.orbit_staging_granted
                    and self.entry_ready_reasons.get(uid) == "READY"):
                self.orbit_staging_granted.add(uid)
                self.orbit_staging_grant_times[uid] = now
                if not self.orbit_staging_started:
                    self.orbit_staging_started = True
                    self.orbit_staging_start_time = now
                rospy.logwarn(
                    "[SWARM_COORD] UAV%d ENTRY_READY at configured height "
                    "%.2fm; individual MOVE_TO_ORBIT_STAGING latched "
                    "time=%.3f roles=%s",
                    uid, self.heights[self.uav_ids.index(uid)], now.to_sec(),
                    self.role_order)
        entry_ready_timed_out = bool(
            set(self.orbit_staging_granted) != set(self.uav_ids)
            and self.entry_ready_started is not None
            and now - self.entry_ready_started >=
            rospy.Duration(self.entry_ready_timeout))

        orbit_staging_ready, self.orbit_staging_ready_reasons = (
            orbit_staging_ready_barrier(
                self.uav_ids, self.states, health, self.heights,
                self.entry_height_tolerance, self.entry_maximum_speed,
                globally_clear))
        leader_staging_ready = (
            self.orbit_staging_ready_reasons.get(self.leader_uav_id)
            == "READY")
        if (leader_staging_ready
                and self.leader_uav_id not in self.orbit_released):
            self.release_orbit(
                self.leader_uav_id, now,
                "leader reached its own ORBIT_STAGING_READY; released first")

        # WAIT_EXIT_PERMISSION is emitted only after the independent closed
        # lap predicate passes. Preserve that completion proof for the
        # separately gated fallback handoff after the moving phase window.
        for uid in self.role_order:
            state = self.states.get(uid)
            if (uid in self.orbit_released and state is not None
                    and state.mission_phase == "WAIT_EXIT_PERMISSION"):
                self.orbit_completed.add(uid)

        # UAV3 leads UAV2 to the 65-70 degree release window.  UAV2 then
        # leads UAV1 through the same independently evaluated gate.  A waiting
        # UAV remains latched at its first orbit point until its own release.
        release_pairs = list(zip(self.role_order[:-1], self.role_order[1:]))
        for leader_uid, follower_uid in release_pairs:
            if (leader_uid not in self.orbit_released
                    or follower_uid in self.orbit_released):
                continue
            allowed, phase, forward_speed, conditions = (
                self.evaluate_sequential_release(
                    leader_uid, follower_uid, now, globally_clear))
            release_mode = "PHASE_WINDOW"
            handoff_conditions = {}
            if not allowed:
                allowed, handoff_conditions = (
                    self.evaluate_completed_predecessor_handoff(
                        leader_uid, follower_uid, now, globally_clear))
                if allowed:
                    release_mode = "COMPLETED_PREDECESSOR_HANDOFF"
            key = "{}-{}".format(leader_uid, follower_uid)
            self.release_diagnostics[key] = {
                "phase_deg": phase,
                "leader_forward_speed": forward_speed,
                "conditions": conditions,
                "release_mode": release_mode if allowed else "INHIBITED",
                "completed_handoff_conditions": handoff_conditions,
            }
            if allowed:
                rospy.logwarn(
                    "[SWARM_COORD] UAV%d-UAV%d release allowed mode=%s "
                    "phase=%.3fdeg speed=%.3fm/s trajectory_fresh=%s "
                    "predicted_conflict=%s handoff=%s",
                    leader_uid, follower_uid, release_mode, phase,
                    forward_speed,
                    conditions.get("leader_trajectory_fresh", False),
                    not conditions.get("predicted_clear", False),
                    handoff_conditions)
                self.release_orbit(
                    follower_uid, now,
                    ("phase {:.3f}deg in [{:.1f},{:.1f}]".format(
                        phase, self.release_phase_min,
                        self.release_phase_max)
                     if release_mode == "PHASE_WINDOW"
                     else "completed predecessor handoff with fresh "
                          "trajectory/current separation/global safety"))
            else:
                rospy.logwarn_throttle(
                    1.0,
                    "[SWARM_COORD] ORBIT_RELEASE_UAV%d inhibited "
                    "leader=UAV%d phase=%.3fdeg speed=%.3fm/s "
                    "trajectory_fresh=%s predicted_conflict=%s "
                    "follower_ready=%s globally_clear=%s failed=%s",
                    follower_uid, leader_uid, phase, forward_speed,
                    conditions.get("leader_trajectory_fresh", False),
                    not conditions.get("predicted_clear", False),
                    conditions.get("follower_ready", False),
                    conditions.get("globally_clear", False),
                    sorted(name for name, value in conditions.items()
                           if not value))
            # UAV1 must never be considered until UAV2 has actually been
            # released, even if both phase windows happen to be true in one
            # timer cycle.
            if follower_uid not in self.orbit_released:
                break

        if (not self.orbit_started
                and set(self.orbit_released) == set(self.uav_ids)):
            self.orbit_started = True
            self.formation_orbit_active = True
            self.orbit_start_time = now
            rospy.logwarn(
                "[SWARM_COORD] FORMATION_ORBIT_ACTIVE established "
                "release_times=%s target=%.1fdeg-%.1fdeg",
                {uid: stamp.to_sec()
                 for uid, stamp in self.orbit_release_times.items()},
                self.desired_phase, self.desired_phase)

        (transition_grants, self.transition_owner,
         self.transition_started, self.transition_completed,
         self.transition_reasons) = serialized_transition_permissions(
            self.transition_order, self.states, health,
            self.transition_target_heights,
            self.transition_height_tolerance,
            self.transition_maximum_speed, globally_clear,
            self.transition_owner, self.transition_started,
            self.transition_completed, self.multi_layer_enabled)

        # Role-bound phase control never derives the leader from angular sort:
        # UAV3 remains leader across the 0/360 wrap, UAV2 follows UAV3 and UAV1
        # follows UAV2.  Safety HOLD propagation follows the same chain.
        active_orbit_positions = {
            uid: (
                self.states[uid].pose.position.x,
                self.states[uid].pose.position.y)
            for uid in self.uav_ids
            if uid in self.orbit_released
            and self.healthy(uid, now)
            and self.states[uid].mission_phase in ACTIVE_ORBIT_PHASES
        }
        active_role_order = [
            uid for uid in self.role_order if uid in active_orbit_positions]
        active_segments = contiguous_role_segments(
            self.role_order, active_role_order)
        next_phase_held = set()
        next_phase_gaps = {}
        next_phase_bands = {}
        phase_decision_valid = True
        for segment in active_segments:
            if len(segment) < 2:
                next_phase_bands["uav{}".format(segment[0])] = (
                    "INDEPENDENT_ORBIT")
                continue
            (segment_holds, segment_gaps, segment_bands,
             segment_reason) = formation_phase_decision(
                active_orbit_positions, self.tower_center, segment,
                self.direction, self.normal_phase_min,
                self.normal_phase_max, self.warning_phase_min,
                self.emergency_phase, self.leader_wait_phase,
                self.phase_held.intersection(segment))
            if segment_reason != "OK":
                phase_decision_valid = False
                break
            next_phase_held.update(segment_holds)
            next_phase_gaps.update(segment_gaps)
            next_phase_bands.update(segment_bands)
        if phase_decision_valid:
            self.phase_held = next_phase_held
            self.phase_gaps = next_phase_gaps
            self.phase_bands = next_phase_bands
        else:
            self.phase_held = set(self.uav_ids)
            self.phase_gaps = {}
            self.phase_bands = {"role_chain": "INVALID_FAIL_CLOSED"}
        formation_phase_held = set(self.phase_held)
        for index, uid in enumerate(self.role_order):
            state = self.states.get(uid)
            if (state is None or uid not in self.orbit_released
                    or state.mission_phase not in ACTIVE_ORBIT_PHASES):
                continue
            direct_hold = (
                state.flight_state in {"HOLD", "HOLD_SAFE"}
                or state.mission_phase in {"HOLDING", "ERROR",
                                           "SAFETY_INHIBIT"})
            if direct_hold and uid not in formation_phase_held:
                # The source keeps permission to execute its supervised resume;
                # only vehicles behind it are inhibited.  This avoids a
                # self-latching coordinator HOLD while preserving no-overtake.
                # A HOLD already selected by phase control is handled entirely
                # by formation_phase_decision: in particular, LEADER_WAIT must
                # not be reflected back onto the follower that closes the gap.
                affected = sorted(role_chain_hold_ids(
                    self.role_order, uid), key=self.role_order.index)
                self.phase_held.update(affected)
                self.phase_bands["role_hold_uav{}".format(uid)] = (
                    "PROPAGATE_TO_{}".format(affected))

        # Soft phase correction is forward-only.  A follower inside the
        # warning band receives a reduced EGO feed-forward scale; hard holds
        # still use the existing orbit permission interlock.  Recovery is
        # intentionally much slower than deceleration, so no vehicle suddenly
        # accelerates to catch up.
        scale_targets = {uid: 1.0 for uid in self.uav_ids}
        if self.phase_gaps:
            active_scales, scale_reason = formation_speed_scale_targets(
                active_role_order, self.phase_gaps, self.phase_held,
                self.emergency_phase, self.normal_phase_min,
                self.minimum_warning_speed_scale)
            if scale_reason != "OK":
                scale_targets = {uid: 0.0 for uid in self.uav_ids}
                self.phase_bands["speed_scale"] = scale_reason
            else:
                scale_targets.update(active_scales)
        for uid in self.phase_held:
            scale_targets[uid] = 0.0
        transition_speed_scale = min(
            1.0, self.layer_transition_nominal_speed
            / self.orbit_nominal_speed)
        if (self.transition_owner
                and self.states[self.transition_owner].mission_phase in {
                    "WAIT_TRANSITION_PERMISSION", "LAYER_TRANSITION",
                    "HOLDING"}):
            scale_targets[self.transition_owner] = min(
                scale_targets[self.transition_owner], transition_speed_scale)
        if not globally_clear:
            scale_targets = {uid: 0.0 for uid in self.uav_ids}
        dt = max(0.0, (now - self.last_speed_scale_update).to_sec())
        self.last_speed_scale_update = now
        self.speed_scale_targets = scale_targets
        for uid in self.uav_ids:
            self.speed_scales[uid] = slew_speed_scale(
                self.speed_scales[uid], scale_targets[uid], dt,
                self.speed_scale_rise_rate, self.speed_scale_fall_rate)
            self.speed_scale_pubs[uid].publish(
                Float64(data=self.speed_scales[uid]))
        if (self.transition_owner
                and transition_grants.get(self.transition_owner, False)
                and self.speed_scales[self.transition_owner]
                > transition_speed_scale + 1.0e-6):
            transition_grants[self.transition_owner] = False
            self.transition_reasons[self.transition_owner] = (
                "OWNER_SPEED_SCALE_SETTLING")

        # Each completed vehicle owns a distinct EXIT gate.  Current/future
        # trajectory conflict checks remain global, so all clear exits may be
        # released together.  Landing stays serialized around the nearby home
        # pads and is deliberately separate from EXIT ownership.
        exit_waiting = [
            uid for uid in self.uav_ids
            if self.healthy(uid, now)
            and self.states[uid].mission_phase == "WAIT_EXIT_PERMISSION"]
        exit_grants = {
            uid: bool(globally_clear and uid in exit_waiting)
            for uid in self.uav_ids}
        if self.landing_owner and self.healthy(self.landing_owner, now):
            if self.states[self.landing_owner].mission_phase == "DONE":
                self.landing_owner = 0
        landing_waiting = [
            uid for uid in self.uav_ids
            if self.healthy(uid, now)
            and self.states[uid].flight_state == "HOME_HOVER"]
        landing_grants, self.landing_owner = serialized_permissions(
            landing_waiting, self.landing_owner)

        permissions = {
            "takeoff": takeoff,
            "task_start": {
                uid: bool(globally_clear and self.task_started)
                for uid in self.uav_ids},
            "entry": {
                uid: bool(entry_grants.get(uid, False))
                for uid in self.uav_ids},
            "orbit_staging": {
                uid: bool(globally_clear
                          and uid in self.orbit_staging_granted)
                for uid in self.uav_ids},
            "orbit": {
                uid: bool(
                    globally_clear and uid in self.orbit_released
                    and uid not in self.phase_held)
                for uid in self.uav_ids},
            "exit": {
                uid: bool(exit_grants.get(uid, False))
                for uid in self.uav_ids},
            "transition": {
                uid: bool(transition_grants.get(uid, False))
                for uid in self.uav_ids},
            "landing": {
                uid: bool(
                    globally_clear and landing_grants.get(uid, False))
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
                        "role=%s mission_state=%s flight_state=%s "
                        "landing_owner=%d phase_held=%s gaps=%s bands=%s "
                        "action=%s",
                        uid, category, current,
                        ({self.leader_uav_id: "LEADER",
                          self.middle_uav_id: "MIDDLE",
                          self.trailing_uav_id: "TRAILING"}.get(uid, "UNKNOWN")),
                        getattr(state, "mission_phase", "MISSING"),
                        getattr(state, "flight_state", "MISSING"),
                        self.landing_owner, sorted(self.phase_held),
                        self.phase_gaps, self.phase_bands,
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
        elif (self.task_started
              and set(self.entry_corridor_selection) != set(self.uav_ids)):
            status.coordinator_state = "ENTRY_CORRIDOR_SELECTION"
            status.reason = "{} committed={} pending={} diagnostics={}".format(
                self.entry_corridor_selection_reason,
                [uid for uid in self.role_order
                 if uid in self.entry_corridor_selection],
                [uid for uid in self.role_order
                 if uid not in self.entry_corridor_selection],
                self.entry_corridor_diagnostics)
        elif entry_ready_timed_out:
            status.coordinator_state = "ENTRY_READY_TIMEOUT"
            status.reason = "entry barrier timed out: {}".format(
                self.entry_ready_reasons)
        elif self.transition_owner:
            status.coordinator_state = "LAYER_TRANSITION_COORDINATION"
            status.reason = (
                "sticky owner=UAV{} order={} completed={} reasons={}"
                .format(self.transition_owner, self.transition_order,
                        sorted(self.transition_completed),
                        self.transition_reasons))
        elif self.landing_owner:
            status.coordinator_state = "RETURN_COORDINATION"
            status.reason = "independent EXIT; landing owner={}".format(
                self.landing_owner)
        elif self.phase_held:
            status.coordinator_state = "PHASE_COORDINATION"
            status.reason = (
                "role-bound hold UAV(s)={} gaps={} bands={}"
                .format(sorted(self.phase_held), self.phase_gaps,
                        self.phase_bands))
        elif (set(self.orbit_staging_granted) != set(self.uav_ids)
              and any_entry_ready):
            status.coordinator_state = "ENTRY_READY"
            status.reason = (
                "individual staging grants={} pending={} reasons={}"
                .format(sorted(self.orbit_staging_granted),
                        [uid for uid in self.role_order
                         if uid not in self.orbit_staging_granted],
                        self.entry_ready_reasons))
        elif (self.orbit_staging_started
              and set(self.orbit_released) != set(self.uav_ids)):
            if (self.leader_uav_id not in self.orbit_released
                    and not leader_staging_ready):
                status.coordinator_state = "ORBIT_STAGING_READY"
                status.reason = (
                    "waiting for leader UAV{} first point; per-UAV states: {}"
                    .format(self.leader_uav_id,
                            self.orbit_staging_ready_reasons))
            else:
                status.coordinator_state = "SEQUENTIAL_ORBIT_RELEASE"
                status.reason = (
                    "released={} pending={} diagnostics={}".format(
                        sorted(self.orbit_released),
                        [uid for uid in self.role_order
                         if uid not in self.orbit_released],
                        self.release_diagnostics))
        elif all(self.states[uid].mission_phase == "DONE"
                 for uid in self.uav_ids):
            status.coordinator_state = "MISSION_COMPLETE"
            status.reason = "all missions complete"
        elif self.orbit_started:
            status.coordinator_state = "FORMATION_ORBIT_ACTIVE"
            status.reason = (
                "individual releases latched roles=UAV{}->UAV{}->UAV{} "
                "gaps={} "
                "bands={}".format(
                    self.leader_uav_id, self.middle_uav_id,
                    self.trailing_uav_id, self.phase_gaps, self.phase_bands))
        else:
            status.coordinator_state = "ORBIT_COORDINATION"
            status.reason = "phase-separated mission geometry is clear"
        if (status.coordinator_state != self.last_coordinator_state or
                status.reason != self.last_coordinator_reason):
            duration = max(0.0, (now - self.condition_started).to_sec())
            rospy.logwarn(
                "[SWARM_COORD] state=%s previous=%s duration=%.3fs "
                "reason=%s landing_owner=%d phase_held=%s "
                "task_barrier=%s action=%s",
                status.coordinator_state,
                self.last_coordinator_state or "INIT", duration,
                status.reason, self.landing_owner,
                sorted(self.phase_held), self.task_started,
                "INHIBIT_NEW_PERMISSIONS"
                if status.coordinator_state == "SAFETY_INHIBIT"
                else "CONTINUE_COORDINATION")
            self.last_coordinator_state = status.coordinator_state
            self.last_coordinator_reason = status.reason
            self.condition_started = now
        self.status_pub.publish(status)
        formation = {
            "roles": {"leader_uav_id": self.leader_uav_id,
                      "middle_uav_id": self.middle_uav_id,
                      "trailing_uav_id": self.trailing_uav_id},
            "entry_ready": self.entry_ready_reasons,
            "entry_corridor_nominal_angles_deg": dict(zip(
                self.uav_ids, self.phases)),
            "entry_corridor_selection": self.entry_corridor_selection,
            "entry_corridor_selection_reason": (
                self.entry_corridor_selection_reason),
            "entry_corridor_diagnostics": self.entry_corridor_diagnostics,
            "entry_corridor_commit_state": self.entry_corridor_commit_state,
            "move_to_orbit_staging": self.orbit_staging_started,
            "move_to_orbit_staging_time": (
                None if self.orbit_staging_start_time is None
                else self.orbit_staging_start_time.to_sec()),
            "orbit_staging_granted": {
                str(uid): uid in self.orbit_staging_granted
                for uid in self.uav_ids},
            "orbit_staging_grant_times": {
                str(uid): stamp.to_sec()
                for uid, stamp in self.orbit_staging_grant_times.items()},
            "orbit_staging_ready": self.orbit_staging_ready_reasons,
            "orbit_released": {
                str(uid): uid in self.orbit_released
                for uid in self.uav_ids},
            "orbit_release_times": {
                str(uid): stamp.to_sec()
                for uid, stamp in self.orbit_release_times.items()},
            "orbit_completed": {
                str(uid): uid in self.orbit_completed
                for uid in self.uav_ids},
            "formation_orbit_active": self.formation_orbit_active,
            "formation_orbit_active_time": (
                None if self.orbit_start_time is None
                else self.orbit_start_time.to_sec()),
            # Compatibility keys retained for existing evidence readers.
            "orbit_start": self.orbit_started,
            "orbit_start_time": (
                None if self.orbit_start_time is None
                else self.orbit_start_time.to_sec()),
            "release_diagnostics": self.release_diagnostics,
            "phase_gaps_deg": self.phase_gaps,
            "phase_bands": self.phase_bands,
            "phase_held_uavs": sorted(self.phase_held),
            "speed_scales": self.speed_scales,
            "speed_scale_targets": self.speed_scale_targets,
            "multi_layer_enabled": self.multi_layer_enabled,
            "transition_order": self.transition_order,
            "transition_owner": self.transition_owner,
            "transition_started": sorted(self.transition_started),
            "transition_completed": sorted(self.transition_completed),
            "transition_reasons": self.transition_reasons,
            "transition_target_heights": self.transition_target_heights,
            "layer_transition_speed_scale": transition_speed_scale,
            "landing_owner": self.landing_owner,
        }
        self.formation_status_pub.publish(String(
            data=json.dumps(formation, sort_keys=True)))


if __name__ == "__main__":
    rospy.init_node("astra_swarm_manager")
    SwarmManager()
    rospy.spin()
