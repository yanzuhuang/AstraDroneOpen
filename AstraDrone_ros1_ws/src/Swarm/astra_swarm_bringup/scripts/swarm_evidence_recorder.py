#!/usr/bin/env python3
"""Record compact synchronized evidence for a three-UAV tower flight."""

import csv
import json
import math
import threading
import time
from pathlib import Path

import rospy
from astra_custom_msgs.msg import InspectionCandidateArray, PlannerStatus
from astra_swarm_msgs.msg import SwarmState
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String
from traj_utils.msg import MultiBsplines


REPO_ROOT = Path(__file__).resolve().parents[5]
RUNTIME_ARTIFACTS_ROOT = REPO_ROOT / "runtime_artifacts"


def runtime_output_param(name, default):
    path = Path(rospy.get_param(name, str(default))).resolve()
    try:
        path.relative_to(RUNTIME_ARTIFACTS_ROOT)
    except ValueError:
        raise rospy.ROSInitException(
            "{} must be under {}: {}".format(
                name, RUNTIME_ARTIFACTS_ROOT, path
            )
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    return str(path)


class EvidenceRecorder:
    def __init__(self):
        self.ids = [int(v) for v in rospy.get_param("~uav_ids", [1, 2, 3])]
        direct_artifact_dir = RUNTIME_ARTIFACTS_ROOT / "direct_triple_tower"
        self.csv_path = runtime_output_param(
            "~csv_file", direct_artifact_dir / "swarm.csv")
        self.summary_path = runtime_output_param(
            "~summary_file", direct_artifact_dir / "summary.json")
        self.candidate_path = runtime_output_param(
            "~candidate_file", direct_artifact_dir / "candidates.jsonl")
        self.candidate_record_mode = str(rospy.get_param(
            "~candidate_record_mode", "full")).strip().lower()
        if self.candidate_record_mode not in ("none", "light", "full"):
            raise rospy.ROSInitException(
                "~candidate_record_mode must be one of none, light, full: {}"
                .format(self.candidate_record_mode))
        self.candidate_file_lock = threading.Lock()
        self.last_candidate_signatures = {}
        self.tower_center = [
            float(v) for v in rospy.get_param(
                "~tower_center", [-10.0551, 19.7104])]
        self.role_order = [int(v) for v in rospy.get_param(
            "~formation_role_order", [3, 2, 1])]
        self.target_phase = float(rospy.get_param("~target_phase_deg", 67.5))
        self.active_orbit_phases = {
            "TARGET_LOCKED", "NAVIGATING", "EVALUATING",
            "RELOCATING", "RECOVERING", "HOLDING"}
        self.states = {}
        self.bridge_states = {uid: "" for uid in self.ids}
        self.bridge_health = {uid: "" for uid in self.ids}
        self.extended_landed = {uid: -1 for uid in self.ids}
        self.stream_health = {uid: {} for uid in self.ids}
        self.armed = {uid: False for uid in self.ids}
        self.tracking = {uid: float("nan") for uid in self.ids}
        self.planner = {}
        self.last_phase = {}
        self.last_flight = {}
        self.last_traj_id = {}
        self.last_emergency = {uid: False for uid in self.ids}
        self.safety_clear = False
        self.orbit_permission = {uid: False for uid in self.ids}
        self.orbit_speed_scale = {uid: float("nan") for uid in self.ids}
        self.chain_ids = []
        self.chain_messages = 0
        self.full_chain_messages = 0
        self.arming_times = {}
        self.takeoff_times = {}
        self.landing_times = {}
        self.entry_ready_times = {}
        self.orbit_staging_ready_times = {}
        self.orbit_release_times = {}
        self.move_to_orbit_staging_time = None
        self.orbit_start_time = None
        self.entry_corridor_nominal_angles = {}
        self.entry_corridor_selection = {}
        self.entry_corridor_diagnostics = {}
        self.entry_corridor_audit = {}
        self.phase_first_times = {uid: {} for uid in self.ids}
        self.candidate_rejection_observations = {
            uid: {} for uid in self.ids}
        self.orbit_angle = {uid: None for uid in self.ids}
        self.orbit_accumulated = {uid: 0.0 for uid in self.ids}
        self.orbit_reverse = {uid: 0.0 for uid in self.ids}
        self.visited_sector_mask = {uid: 0 for uid in self.ids}
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
        self.formation_phase_stats = {
            "{}-{}".format(leader, follower): {
                "min": float("inf"), "max": -float("inf"),
                "sum": 0.0, "error_sum": 0.0, "maximum_error": 0.0,
                "count": 0}
            for leader, follower in zip(
                self.role_order[:-1], self.role_order[1:])}
        self.file = open(self.csv_path, "w", newline="")
        self.candidate_file = open(self.candidate_path, "w")
        fields = ["sim_time", "safety_clear", "chain_ids"]
        for uid in self.ids:
            fields += [
                "u{}_x".format(uid), "u{}_y".format(uid),
                "u{}_z".format(uid), "u{}_phase".format(uid),
                "u{}_flight".format(uid), "u{}_armed".format(uid),
                "u{}_tracking_error".format(uid),
                "u{}_trajectory_id".format(uid),
                "u{}_orbit_released".format(uid),
                "u{}_orbit_speed_scale".format(uid),
                "u{}_heartbeat_ok".format(uid),
                "u{}_localization_valid".format(uid),
                "u{}_bridge_state".format(uid),
                "u{}_extended_landed_state".format(uid),
                "u{}_bridge_input_health".format(uid)]
            for stream in (
                    "mavros_pose_raw", "mavros_pose_framed",
                    "fast_lio_odom_raw", "planner_odom"):
                fields += [
                    "u{}_{}_stamp".format(uid, stream),
                    "u{}_{}_stamp_age".format(uid, stream),
                    "u{}_{}_receive_age".format(uid, stream),
                    "u{}_{}_wall_receive_age".format(uid, stream),
                    "u{}_{}_count".format(uid, stream)]
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
                "/uav{}/mavros/local_position/pose".format(uid),
                PoseStamped,
                lambda msg, u=uid: self.header_stream_cb(
                    u, "mavros_pose_raw", msg.header.stamp), queue_size=20)
            rospy.Subscriber(
                "/uav{}/mavros/local_position/pose_framed".format(uid),
                PoseStamped,
                lambda msg, u=uid: self.header_stream_cb(
                    u, "mavros_pose_framed", msg.header.stamp), queue_size=20)
            rospy.Subscriber(
                "/uav{}/fast_lio/Odometry_raw".format(uid), Odometry,
                lambda msg, u=uid: self.header_stream_cb(
                    u, "fast_lio_odom_raw", msg.header.stamp), queue_size=20)
            rospy.Subscriber(
                "/uav{}/Odometry".format(uid), Odometry,
                lambda msg, u=uid: self.header_stream_cb(
                    u, "planner_odom", msg.header.stamp), queue_size=20)
            rospy.Subscriber(
                "/uav{}/mavros/extended_state".format(uid), ExtendedState,
                lambda msg, u=uid: self.extended_cb(u, msg), queue_size=10)
            rospy.Subscriber(
                "/uav{}/ego_mavros_bridge/state".format(uid), String,
                lambda msg, u=uid: self.bridge_states.__setitem__(
                    u, msg.data), queue_size=10)
            rospy.Subscriber(
                "/uav{}/ego_mavros_bridge/input_health".format(uid), String,
                lambda msg, u=uid: self.bridge_health.__setitem__(
                    u, msg.data), queue_size=20)
            rospy.Subscriber(
                "/uav{}/tower_mission/candidate_targets".format(uid),
                InspectionCandidateArray,
                lambda msg, u=uid: self.candidate_cb(
                    u, msg, "candidate_targets"), queue_size=20)
            rospy.Subscriber(
                "/uav{}/tower_mission/entry_corridor_candidates".format(uid),
                InspectionCandidateArray,
                lambda msg, u=uid: self.candidate_cb(
                    u, msg, "entry_corridor_candidates"), queue_size=20)
            rospy.Subscriber(
                "/uav{}/swarm/orbit_permission".format(uid), Bool,
                lambda msg, u=uid: self.orbit_permission.__setitem__(
                    u, msg.data), queue_size=10)
            rospy.Subscriber(
                "/uav{}/swarm/orbit_speed_scale".format(uid), Float64,
                lambda msg, u=uid: self.orbit_speed_scale.__setitem__(
                    u, msg.data), queue_size=10)
        rospy.Subscriber(
            "/swarm/safety/clear", Bool,
            lambda msg: setattr(self, "safety_clear", msg.data), queue_size=5)
        rospy.Subscriber(
            "/swarm/trajectories", MultiBsplines,
            self.chain_cb, queue_size=20)
        rospy.Subscriber(
            "/swarm/formation/status", String,
            self.formation_cb, queue_size=10)
        rospy.Subscriber(
            "/swarm/entry_corridor_audit", String,
            self.entry_corridor_audit_cb, queue_size=5)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        rospy.on_shutdown(self.finish)

    def header_stream_cb(self, uid, stream, stamp):
        previous = self.stream_health[uid].get(stream, {})
        self.stream_health[uid][stream] = {
            "stamp": stamp.to_sec(),
            "ros_received": rospy.Time.now().to_sec(),
            "wall_received": time.monotonic(),
            "count": previous.get("count", 0) + 1,
        }

    def extended_cb(self, uid, msg):
        self.extended_landed[uid] = int(msg.landed_state)

    def write_candidate_record(self, record):
        with self.candidate_file_lock:
            if self.candidate_file.closed:
                return False
            self.candidate_file.write(
                json.dumps(record, sort_keys=True) + "\n")
            self.candidate_file.flush()
        return True

    @staticmethod
    def candidate_change_signature(record):
        """Return the meaningful candidate state, excluding noisy metrics."""
        return (
            record["record_type"],
            record["frame_id"],
            record["uav_id"],
            record["current_sector"],
            record["locked_candidate_id"],
            tuple((
                item["candidate_id"],
                item["sector_id"],
                item["layer_id"],
                tuple(round(value, 6) for value in item["target"]),
                item["accepted"],
                item["rejection_reason"],
            ) for item in record["candidates"]),
        )

    def should_write_candidate_record(self, record):
        if self.candidate_record_mode == "none":
            return False
        if self.candidate_record_mode == "full":
            return True
        key = (record["uav_id"], record["record_type"])
        signature = self.candidate_change_signature(record)
        if self.last_candidate_signatures.get(key) == signature:
            return False
        self.last_candidate_signatures[key] = signature
        return True

    def candidate_cb(self, uid, msg, source="candidate_targets"):
        record = {
            "record_type": source,
            "receive_sim_time": rospy.Time.now().to_sec(),
            "header_stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "uav_id": uid,
            "current_sector": int(msg.current_sector),
            "locked_candidate_id": msg.locked_candidate_id,
            "candidates": [{
                "candidate_id": item.candidate_id,
                "sector_id": int(item.sector_id),
                "layer_id": int(item.layer_id),
                "target": [item.target.x, item.target.y, item.target.z],
                "yaw": item.yaw,
                "accepted": bool(item.accepted),
                "rejection_reason": item.rejection_reason,
                "clearance": item.clearance,
                "score": item.score,
                "unknown_ratio": item.unknown_ratio,
            } for item in msg.candidates],
        }
        # rospy may dispatch a final queued candidate callback concurrently
        # with the shutdown hook.  Serialize the write and close so a normal
        # SIGINT cannot turn complete evidence into a spurious traceback.
        if (self.should_write_candidate_record(record)
                and not self.write_candidate_record(record)):
            return
        counts = self.candidate_rejection_observations[uid]
        for item in msg.candidates:
            if item.accepted:
                continue
            reason = item.rejection_reason or "UNSPECIFIED"
            counts[reason] = counts.get(reason, 0) + 1

    def entry_corridor_audit_cb(self, msg):
        try:
            audit = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self.entry_corridor_audit = audit
        if self.candidate_record_mode != "none":
            self.write_candidate_record({
                "record_type": "joint_entry_corridor_audit",
                "receive_sim_time": rospy.Time.now().to_sec(),
                "audit": audit,
            })

    def state_cb(self, uid, msg):
        previous_phase = self.last_phase.get(uid)
        holding_now = (
            msg.mission_phase == "HOLDING" or msg.flight_state == "HOLD")
        holding_before = self.holding[uid]
        now = rospy.Time.now().to_sec()
        if msg.mission_phase not in self.phase_first_times[uid]:
            self.phase_first_times[uid][msg.mission_phase] = now
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
        if msg.mission_phase == "ENTRY_READY" and uid not in self.entry_ready_times:
            self.entry_ready_times[uid] = now
        if (msg.mission_phase == "ORBIT_STAGING_READY"
                and uid not in self.orbit_staging_ready_times):
            self.orbit_staging_ready_times[uid] = now
        if (self.armed[uid] and uid not in self.takeoff_times
                and msg.pose.position.z >= 0.30):
            self.takeoff_times[uid] = rospy.Time.now().to_sec()
        if (uid in self.orbit_release_times
                and msg.mission_phase in self.active_orbit_phases):
            stats = self.orbit_height[uid]
            height = msg.pose.position.z
            stats["min"] = min(stats["min"], height)
            stats["max"] = max(stats["max"], height)
            stats["sum"] += height
            stats["count"] += 1
            angle = math.atan2(
                msg.pose.position.y - self.tower_center[1],
                msg.pose.position.x - self.tower_center[0])
            previous = self.orbit_angle[uid]
            if previous is not None:
                delta = (angle - previous + math.pi) % (2.0 * math.pi) - math.pi
                if delta > 0.0:
                    self.orbit_accumulated[uid] += delta
                elif delta < 0.0:
                    self.orbit_reverse[uid] += abs(delta)
            self.orbit_angle[uid] = angle
            sector = int(math.floor(
                ((math.degrees(angle) % 360.0) + 22.5) / 45.0)) % 8
            self.visited_sector_mask[uid] |= 1 << sector

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

    def formation_cb(self, msg):
        try:
            formation = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        start = formation.get("orbit_start_time")
        if start is not None and self.orbit_start_time is None:
            self.orbit_start_time = float(start)
        staging_start = formation.get("move_to_orbit_staging_time")
        if (staging_start is not None
                and self.move_to_orbit_staging_time is None):
            self.move_to_orbit_staging_time = float(staging_start)
        if formation.get("entry_corridor_nominal_angles_deg"):
            self.entry_corridor_nominal_angles = formation[
                "entry_corridor_nominal_angles_deg"]
        if formation.get("entry_corridor_selection"):
            self.entry_corridor_selection = formation[
                "entry_corridor_selection"]
        if formation.get("entry_corridor_diagnostics"):
            self.entry_corridor_diagnostics = formation[
                "entry_corridor_diagnostics"]
        for uid, stamp in formation.get("orbit_release_times", {}).items():
            try:
                numeric_uid = int(uid)
                if numeric_uid not in self.orbit_release_times:
                    self.orbit_release_times[numeric_uid] = float(stamp)
                    self.orbit_angle[numeric_uid] = None
            except (TypeError, ValueError):
                continue

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

    def directed_phase(self, follower, leader):
        follower_angle = math.degrees(math.atan2(
            follower.y - self.tower_center[1],
            follower.x - self.tower_center[0]))
        leader_angle = math.degrees(math.atan2(
            leader.y - self.tower_center[1],
            leader.x - self.tower_center[0]))
        return (leader_angle - follower_angle) % 360.0

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
                "u{}_orbit_released".format(uid):
                    int(uid in self.orbit_release_times),
                "u{}_orbit_speed_scale".format(uid):
                    self.orbit_speed_scale[uid],
                "u{}_heartbeat_ok".format(uid): int(state.heartbeat_ok),
                "u{}_localization_valid".format(uid):
                    int(state.localization_valid),
                "u{}_bridge_state".format(uid): self.bridge_states[uid],
                "u{}_extended_landed_state".format(uid):
                    self.extended_landed[uid],
                "u{}_bridge_input_health".format(uid):
                    self.bridge_health[uid],
            })
            now_ros = rospy.Time.now().to_sec()
            now_wall = time.monotonic()
            for stream in (
                    "mavros_pose_raw", "mavros_pose_framed",
                    "fast_lio_odom_raw", "planner_odom"):
                health = self.stream_health[uid].get(stream)
                if health is None:
                    continue
                row.update({
                    "u{}_{}_stamp".format(uid, stream): health["stamp"],
                    "u{}_{}_stamp_age".format(uid, stream):
                        now_ros - health["stamp"],
                    "u{}_{}_receive_age".format(uid, stream):
                        now_ros - health["ros_received"],
                    "u{}_{}_wall_receive_age".format(uid, stream):
                        now_wall - health["wall_received"],
                    "u{}_{}_count".format(uid, stream): health["count"],
                })
            if (uid in self.orbit_release_times
                    and state.mission_phase in self.active_orbit_phases):
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
                    if (self.orbit_start_time is not None
                            and self.states[first_id].mission_phase
                            in self.active_orbit_phases
                            and self.states[second_id].mission_phase
                            in self.active_orbit_phases):
                        self.orbit_pair_min[pair][key] = min(
                            self.orbit_pair_min[pair][key], value)
                phase = self.phase_separation(
                    self.states[first_id].pose.position,
                    self.states[second_id].pose.position)
                row[pair + "_phase_degrees"] = phase
                if (self.orbit_start_time is not None
                        and self.states[first_id].mission_phase
                        in self.active_orbit_phases
                        and self.states[second_id].mission_phase
                        in self.active_orbit_phases):
                    stats = self.phase_stats[pair]
                    stats["min"] = min(stats["min"], phase)
                    stats["max"] = max(stats["max"], phase)
                    stats["sum"] += phase
                    stats["count"] += 1
        if (self.orbit_start_time is not None
                and all(self.states[uid].mission_phase
                        in self.active_orbit_phases
                        for uid in self.role_order)):
            for leader, follower in zip(
                    self.role_order[:-1], self.role_order[1:]):
                key = "{}-{}".format(leader, follower)
                gap = self.directed_phase(
                    self.states[follower].pose.position,
                    self.states[leader].pose.position)
                stats = self.formation_phase_stats[key]
                error = abs(gap - self.target_phase)
                stats["min"] = min(stats["min"], gap)
                stats["max"] = max(stats["max"], gap)
                stats["sum"] += gap
                stats["error_sum"] += error
                stats["maximum_error"] = max(stats["maximum_error"], error)
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
        with self.candidate_file_lock:
            if not self.candidate_file.closed:
                self.candidate_file.flush()
                self.candidate_file.close()
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
            "entry_ready_times": self.entry_ready_times,
            "entry_corridor_nominal_angles_deg": (
                self.entry_corridor_nominal_angles),
            "entry_corridor_selection": self.entry_corridor_selection,
            "entry_corridor_diagnostics": self.entry_corridor_diagnostics,
            "entry_corridor_audit": self.entry_corridor_audit,
            "mission_phase_first_times": self.phase_first_times,
            "candidate_rejection_observations": (
                self.candidate_rejection_observations),
            "move_to_orbit_staging_time": self.move_to_orbit_staging_time,
            "orbit_staging_ready_times": self.orbit_staging_ready_times,
            "orbit_release_times": self.orbit_release_times,
            "orbit_start_time": self.orbit_start_time,
            "formation_role_order": self.role_order,
            "independent_orbit_accumulated_degrees": {
                str(uid): math.degrees(value)
                for uid, value in self.orbit_accumulated.items()},
            "reverse_orbit_motion_degrees": {
                str(uid): math.degrees(value)
                for uid, value in self.orbit_reverse.items()},
            "visited_sector_mask": {
                str(uid): "0x{:02X}".format(value)
                for uid, value in self.visited_sector_mask.items()},
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
            "role_bound_phase_degrees": {
                pair: {
                    "samples": stats["count"],
                    "min": None if not stats["count"] else stats["min"],
                    "mean": None if not stats["count"]
                    else stats["sum"] / stats["count"],
                    "max": None if not stats["count"] else stats["max"],
                    "mean_absolute_error": None if not stats["count"]
                    else stats["error_sum"] / stats["count"],
                    "maximum_absolute_error": (
                        None if not stats["count"]
                        else stats["maximum_error"]),
                }
                for pair, stats in self.formation_phase_stats.items()
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
            "final_bridge_state": self.bridge_states,
            "final_extended_landed_state": self.extended_landed,
            "stream_message_counts": {
                str(uid): {
                    name: values.get("count", 0)
                    for name, values in streams.items()
                }
                for uid, streams in self.stream_health.items()
            },
        }
        with open(self.summary_path, "w") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)


if __name__ == "__main__":
    rospy.init_node("swarm_evidence_recorder")
    EvidenceRecorder()
    rospy.spin()
