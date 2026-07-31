#!/usr/bin/env python3
"""Record compact synchronized evidence for a three-UAV tower flight."""

import csv
import json
import math

import rospy
from astra_custom_msgs.msg import PlannerStatus
from astra_swarm_msgs.msg import SwarmState
from mavros_msgs.msg import State
from std_msgs.msg import Bool, Float64
from traj_utils.msg import MultiBsplines


class EvidenceRecorder:
    def __init__(self):
        self.ids = [int(v) for v in rospy.get_param("~uav_ids", [1, 2, 3])]
        self.csv_path = rospy.get_param(
            "~csv_file", "/tmp/astra_swarm_evidence.csv")
        self.summary_path = rospy.get_param(
            "~summary_file", "/tmp/astra_swarm_evidence.json")
        self.tower_center = [
            float(v) for v in rospy.get_param(
                "~tower_center", [-10.0551, 19.7104])]
        self.active_orbit_phases = {
            "TARGET_LOCKED", "NAVIGATING", "EVALUATING",
            "RELOCATING", "RECOVERING", "HOLDING"}
        self.states = {}
        self.armed = {uid: False for uid in self.ids}
        self.tracking = {uid: float("nan") for uid in self.ids}
        self.planner = {}
        self.last_phase = {}
        self.last_flight = {}
        self.last_traj_id = {}
        self.last_emergency = {uid: False for uid in self.ids}
        self.safety_clear = False
        self.orbit_permission = {uid: False for uid in self.ids}
        self.chain_ids = []
        self.chain_messages = 0
        self.full_chain_messages = 0
        self.arming_times = {}
        self.takeoff_times = {}
        self.landing_times = {}
        self.holds = {uid: 0 for uid in self.ids}
        self.phase_holds = {uid: 0 for uid in self.ids}
        self.holding = {uid: False for uid in self.ids}
        self.hold_started = {}
        self.hold_duration = {
            uid: {"total": 0.0, "maximum": 0.0}
            for uid in self.ids}
        self.emergencies = {uid: 0 for uid in self.ids}
        self.replans = {uid: 0 for uid in self.ids}
        self.max_tracking = {uid: 0.0 for uid in self.ids}
        self.orbit_height = {
            uid: {"min": float("inf"), "max": -float("inf"), "sum": 0.0,
                  "count": 0}
            for uid in self.ids}
        self.pair_min = {
            "{}-{}".format(a, b): {
                "horizontal": float("inf"), "vertical": float("inf"),
                "three_d": float("inf"), "ellipsoid": float("inf")}
            for index, a in enumerate(self.ids)
            for b in self.ids[index + 1:]}
        self.orbit_pair_min = {
            pair: {
                "horizontal": float("inf"), "vertical": float("inf"),
                "three_d": float("inf"), "ellipsoid": float("inf")}
            for pair in self.pair_min}
        self.phase_stats = {
            pair: {"min": float("inf"), "max": -float("inf"),
                   "sum": 0.0, "count": 0}
            for pair in self.pair_min}
        self.yaw_stats = {
            uid: {"maximum": 0.0, "sum": 0.0, "count": 0}
            for uid in self.ids}
        self.file = open(self.csv_path, "w", newline="")
        fields = ["sim_time", "safety_clear", "chain_ids"]
        for uid in self.ids:
            fields += [
                "u{}_x".format(uid), "u{}_y".format(uid),
                "u{}_z".format(uid), "u{}_phase".format(uid),
                "u{}_flight".format(uid), "u{}_armed".format(uid),
                "u{}_tracking_error".format(uid),
                "u{}_trajectory_id".format(uid)]
        for pair in self.pair_min:
            fields += [
                pair + "_horizontal", pair + "_vertical",
                pair + "_three_d", pair + "_ellipsoid",
                pair + "_phase_degrees"]
        self.writer = csv.DictWriter(self.file, fieldnames=fields)
        self.writer.writeheader()
        for uid in self.ids:
            rospy.Subscriber(
                "/uav{}/swarm/state".format(uid), SwarmState,
                lambda msg, u=uid: self.state_cb(u, msg), queue_size=10)
            rospy.Subscriber(
                "/uav{}/mavros/state".format(uid), State,
                lambda msg, u=uid: self.mavros_cb(u, msg), queue_size=10)
            rospy.Subscriber(
                "/uav{}/ego_mavros_bridge/tracking_error".format(uid),
                Float64, lambda msg, u=uid: self.tracking_cb(u, msg),
                queue_size=10)
            rospy.Subscriber(
                "/uav{}/planner/status".format(uid), PlannerStatus,
                lambda msg, u=uid: self.planner_cb(u, msg), queue_size=10)
            rospy.Subscriber(
                "/uav{}/swarm/orbit_permission".format(uid), Bool,
                lambda msg, u=uid: self.orbit_permission.__setitem__(
                    u, msg.data), queue_size=10)
        rospy.Subscriber(
            "/swarm/safety/clear", Bool,
            lambda msg: setattr(self, "safety_clear", msg.data), queue_size=5)
        rospy.Subscriber(
            "/swarm/trajectories", MultiBsplines,
            self.chain_cb, queue_size=20)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        rospy.on_shutdown(self.finish)

    def state_cb(self, uid, msg):
        previous_phase = self.last_phase.get(uid)
        holding_now = (
            msg.mission_phase == "HOLDING" or msg.flight_state == "HOLD")
        holding_before = self.holding[uid]
        now = rospy.Time.now().to_sec()
        if holding_now and not holding_before:
            self.holds[uid] += 1
            self.hold_started[uid] = now
            active_orbit_phases = {
                "TARGET_LOCKED", "NAVIGATING", "EVALUATING",
                "RELOCATING", "RECOVERING"}
            if (self.safety_clear and not self.orbit_permission[uid]
                    and previous_phase in active_orbit_phases):
                self.phase_holds[uid] += 1
        elif not holding_now and holding_before:
            started = self.hold_started.pop(uid, now)
            duration = max(0.0, now - started)
            stats = self.hold_duration[uid]
            stats["total"] += duration
            stats["maximum"] = max(stats["maximum"], duration)
        self.holding[uid] = holding_now
        if (msg.mission_phase in {"ERROR", "FAILURE_LANDING"}
                and previous_phase not in {"ERROR", "FAILURE_LANDING"}):
            self.emergencies[uid] += 1
        self.last_phase[uid] = msg.mission_phase
        self.last_flight[uid] = msg.flight_state
        self.states[uid] = msg
        if (self.armed[uid] and uid not in self.takeoff_times
                and msg.pose.position.z >= 0.30):
            self.takeoff_times[uid] = rospy.Time.now().to_sec()
        if msg.mission_phase in self.active_orbit_phases:
            stats = self.orbit_height[uid]
            height = msg.pose.position.z
            stats["min"] = min(stats["min"], height)
            stats["max"] = max(stats["max"], height)
            stats["sum"] += height
            stats["count"] += 1

    def mavros_cb(self, uid, msg):
        if msg.armed and not self.armed[uid] and uid not in self.arming_times:
            self.arming_times[uid] = rospy.Time.now().to_sec()
        if not msg.armed and self.armed[uid] and uid in self.takeoff_times:
            self.landing_times[uid] = rospy.Time.now().to_sec()
        self.armed[uid] = msg.armed

    def tracking_cb(self, uid, msg):
        self.tracking[uid] = msg.data
        if math.isfinite(msg.data):
            self.max_tracking[uid] = max(self.max_tracking[uid], msg.data)

    def planner_cb(self, uid, msg):
        previous = self.last_traj_id.get(uid)
        if previous is not None and msg.trajectory_id != previous:
            self.replans[uid] += 1
        self.last_traj_id[uid] = msg.trajectory_id
        self.planner[uid] = msg
        if msg.emergency_stop_active and not self.last_emergency[uid]:
            self.emergencies[uid] += 1
        self.last_emergency[uid] = msg.emergency_stop_active

    def chain_cb(self, msg):
        self.chain_ids = [int(traj.drone_id) for traj in msg.traj]
        self.chain_messages += 1
        if set(self.chain_ids) == {uid - 1 for uid in self.ids}:
            self.full_chain_messages += 1

    @staticmethod
    def pair_distances(first, second):
        dx = first.x - second.x
        dy = first.y - second.y
        dz = first.z - second.z
        horizontal = math.hypot(dx, dy)
        return (
            horizontal, abs(dz), math.sqrt(horizontal ** 2 + dz ** 2),
            math.sqrt(horizontal ** 2 + (dz / 2.0) ** 2))

    def phase_separation(self, first, second):
        first_angle = math.degrees(math.atan2(
            first.y - self.tower_center[1],
            first.x - self.tower_center[0]))
        second_angle = math.degrees(math.atan2(
            second.y - self.tower_center[1],
            second.x - self.tower_center[0]))
        return abs((first_angle - second_angle + 180.0) % 360.0 - 180.0)

    def timer_cb(self, _event):
        if any(uid not in self.states for uid in self.ids):
            return
        row = {
            "sim_time": "{:.3f}".format(rospy.Time.now().to_sec()),
            "safety_clear": int(self.safety_clear),
            "chain_ids": ";".join(str(v) for v in self.chain_ids),
        }
        for uid in self.ids:
            state = self.states[uid]
            row.update({
                "u{}_x".format(uid): state.pose.position.x,
                "u{}_y".format(uid): state.pose.position.y,
                "u{}_z".format(uid): state.pose.position.z,
                "u{}_phase".format(uid): state.mission_phase,
                "u{}_flight".format(uid): state.flight_state,
                "u{}_armed".format(uid): int(self.armed[uid]),
                "u{}_tracking_error".format(uid): self.tracking[uid],
                "u{}_trajectory_id".format(uid):
                    self.last_traj_id.get(uid, 0),
            })
            if state.mission_phase in self.active_orbit_phases:
                desired_yaw = math.atan2(
                    self.tower_center[1] - state.pose.position.y,
                    self.tower_center[0] - state.pose.position.x)
                yaw_error = abs(
                    (state.yaw - desired_yaw + math.pi)
                    % (2.0 * math.pi) - math.pi)
                stats = self.yaw_stats[uid]
                stats["maximum"] = max(stats["maximum"], yaw_error)
                stats["sum"] += yaw_error
                stats["count"] += 1
        for index, first_id in enumerate(self.ids):
            for second_id in self.ids[index + 1:]:
                pair = "{}-{}".format(first_id, second_id)
                values = self.pair_distances(
                    self.states[first_id].pose.position,
                    self.states[second_id].pose.position)
                for key, value in zip(
                        ("horizontal", "vertical", "three_d", "ellipsoid"),
                        values):
                    row[pair + "_" + key] = value
                    self.pair_min[pair][key] = min(
                        self.pair_min[pair][key], value)
                    if (self.states[first_id].mission_phase
                            in self.active_orbit_phases
                            and self.states[second_id].mission_phase
                            in self.active_orbit_phases):
                        self.orbit_pair_min[pair][key] = min(
                            self.orbit_pair_min[pair][key], value)
                phase = self.phase_separation(
                    self.states[first_id].pose.position,
                    self.states[second_id].pose.position)
                row[pair + "_phase_degrees"] = phase
                if (self.states[first_id].mission_phase
                        in self.active_orbit_phases
                        and self.states[second_id].mission_phase
                        in self.active_orbit_phases):
                    stats = self.phase_stats[pair]
                    stats["min"] = min(stats["min"], phase)
                    stats["max"] = max(stats["max"], phase)
                    stats["sum"] += phase
                    stats["count"] += 1
        self.writer.writerow(row)
        self.file.flush()

    def finish(self):
        now = rospy.Time.now().to_sec()
        for uid, started in list(self.hold_started.items()):
            duration = max(0.0, now - started)
            stats = self.hold_duration[uid]
            stats["total"] += duration
            stats["maximum"] = max(stats["maximum"], duration)
            del self.hold_started[uid]
        if not self.file.closed:
            self.file.flush()
            self.file.close()
        orbit = {}
        for uid, stats in self.orbit_height.items():
            orbit[str(uid)] = {
                "samples": stats["count"],
                "min": None if not stats["count"] else stats["min"],
                "mean": None if not stats["count"]
                else stats["sum"] / stats["count"],
                "max": None if not stats["count"] else stats["max"],
            }
        pair_min = {}
        for pair, values in self.pair_min.items():
            pair_min[pair] = {
                key: None if not math.isfinite(value) else value
                for key, value in values.items()
            }
        orbit_pair_min = {}
        for pair, values in self.orbit_pair_min.items():
            orbit_pair_min[pair] = {
                key: None if not math.isfinite(value) else value
                for key, value in values.items()
            }
        summary = {
            "arming_times": self.arming_times,
            "takeoff_times": self.takeoff_times,
            "landing_times": self.landing_times,
            "orbit_height": orbit,
            "pairwise_minimum": pair_min,
            "concurrent_orbit_pairwise_minimum": orbit_pair_min,
            "orbit_phase_separation_degrees": {
                pair: {
                    "samples": stats["count"],
                    "min": None if not stats["count"] else stats["min"],
                    "mean": None if not stats["count"]
                    else stats["sum"] / stats["count"],
                    "max": None if not stats["count"] else stats["max"],
                }
                for pair, stats in self.phase_stats.items()
            },
            "ellipsoid_required": 3.0,
            "trajectory_chain_messages": self.chain_messages,
            "full_three_trajectory_messages": self.full_chain_messages,
            "planner_trajectory_changes": self.replans,
            "maximum_tracking_error": self.max_tracking,
            "tower_facing_yaw_error_radians": {
                str(uid): {
                    "samples": stats["count"],
                    "mean": None if not stats["count"]
                    else stats["sum"] / stats["count"],
                    "maximum": None if not stats["count"]
                    else stats["maximum"],
                }
                for uid, stats in self.yaw_stats.items()
            },
            "hold_count": self.holds,
            "dynamic_phase_hold_count": self.phase_holds,
            "hold_duration_seconds": self.hold_duration,
            "emergency_count": self.emergencies,
            "final_mission_phase": self.last_phase,
            "final_flight_state": self.last_flight,
            "final_armed": self.armed,
        }
        with open(self.summary_path, "w") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)


if __name__ == "__main__":
    rospy.init_node("swarm_evidence_recorder")
    EvidenceRecorder()
    rospy.spin()
