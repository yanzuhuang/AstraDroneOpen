#!/usr/bin/env python3
"""Three-UAV actual paths, main-waypoint markers, and topic diagnostics."""

import copy
import json
import math

import rospy
from astra_custom_msgs.msg import PlannerStatus
from astra_swarm_msgs.msg import CoordinationStatus, SafetyEvent, SwarmState
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, UInt32
from visualization_msgs.msg import Marker, MarkerArray


COLORS = {
    1: (1.0, 0.55, 0.0),
    2: (0.0, 0.75, 1.0),
    3: (0.35, 1.0, 0.2),
}


class SwarmRvizDiagnostics:
    def __init__(self):
        self.uav_ids = [int(value) for value in rospy.get_param(
            "~uav_ids", [1, 2, 3])]
        self.tower = [float(value) for value in rospy.get_param(
            "~tower_center", [-10.0551, 19.7104])]
        self.radius = float(rospy.get_param("~orbit_radius", 12.5))
        self.entry_radius = float(rospy.get_param("~entry_gate_radius", 15.0))
        self.heights = [float(value) for value in rospy.get_param(
            "~mission_heights", [3.0, 3.0, 3.0])]
        self.entry_angles = [float(value) for value in rospy.get_param(
            "~entry_angles_deg", [292.5, 315.0, 337.5])]
        self.pre_entry_radius = float(rospy.get_param(
            "~pre_entry_radius", 18.0))
        self.homes = rospy.get_param(
            "~home_positions", [[0.0, 0.0], [4.0, 0.0], [8.0, 0.0]])
        self.role_order = [int(value) for value in rospy.get_param(
            "~formation_role_order", [3, 2, 1])]
        self.maximum_path_points = int(rospy.get_param(
            "~maximum_path_points", 12000))
        if not (len(self.uav_ids) == len(self.heights) ==
                len(self.entry_angles) == len(self.homes)):
            raise rospy.ROSException("RViz per-UAV arrays mismatch")
        if set(self.role_order) != set(self.uav_ids):
            raise rospy.ROSException("RViz formation roles mismatch")

        self.paths = {uid: Path() for uid in self.uav_ids}
        self.states = {}
        self.mission_states = {uid: "WAIT_INPUTS" for uid in self.uav_ids}
        self.bridge_states = {uid: "WAIT_FCU" for uid in self.uav_ids}
        self.sectors = {uid: 0 for uid in self.uav_ids}
        self.planners = {}
        self.last_safety = "CLEAR_NOT_CONFIRMED"
        self.coordinator = "INIT"
        self.coordinator_reason = "waiting for coordinator"
        self.formation = {}
        self.orbit_angles = {uid: None for uid in self.uav_ids}
        self.accumulated = {uid: 0.0 for uid in self.uav_ids}
        self.sector_masks = {uid: 0 for uid in self.uav_ids}
        self.orbit_released = {uid: False for uid in self.uav_ids}
        self.path_pubs = {}

        for uid in self.uav_ids:
            prefix = "/uav{}".format(uid)
            self.path_pubs[uid] = rospy.Publisher(
                prefix + "/swarm/actual_path", Path, queue_size=1,
                latch=True)
            rospy.Subscriber(
                prefix + "/Odometry", Odometry,
                lambda msg, u=uid: self.odom_cb(u, msg), queue_size=20)
            rospy.Subscriber(
                prefix + "/swarm/state", SwarmState,
                lambda msg, u=uid: self.state_cb(u, msg), queue_size=5)
            rospy.Subscriber(
                prefix + "/tower_mission/state", String,
                lambda msg, u=uid: self.mission_cb(u, msg), queue_size=5)
            rospy.Subscriber(
                prefix + "/ego_mavros_bridge/state", String,
                lambda msg, u=uid: self.bridge_cb(u, msg), queue_size=5)
            rospy.Subscriber(
                prefix + "/tower_mission/current_sector", UInt32,
                lambda msg, u=uid: self.sector_cb(u, msg), queue_size=5)
            rospy.Subscriber(
                prefix + "/planner/status", PlannerStatus,
                lambda msg, u=uid: self.planner_cb(u, msg), queue_size=5)

        rospy.Subscriber(
            "/swarm/safety/event", SafetyEvent, self.safety_cb, queue_size=10)
        rospy.Subscriber(
            "/swarm/coordinator/status", CoordinationStatus,
            self.coordinator_cb, queue_size=5)
        rospy.Subscriber(
            "/swarm/formation/status", String,
            self.formation_cb, queue_size=5)
        self.static_pub = rospy.Publisher(
            "/swarm/rviz/mission_markers", MarkerArray,
            queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            "/swarm/rviz/status_markers", MarkerArray, queue_size=1)
        self.topic_diag_pub = rospy.Publisher(
            "/swarm/rviz/topic_diagnostics", String, queue_size=1, latch=True)
        rospy.Timer(rospy.Duration(0.5), self.timer_cb)
        rospy.Timer(rospy.Duration(3.0), self.topic_diagnostics_cb)
        self.publish_static_markers()

    def odom_cb(self, uid, msg):
        path = self.paths[uid]
        if not path.poses:
            path.header.frame_id = msg.header.frame_id
        if msg.header.frame_id != path.header.frame_id:
            rospy.logerr_throttle(
                1.0, "[SWARM_RVIZ] uav=%d odom frame changed %s -> %s",
                uid, path.header.frame_id, msg.header.frame_id)
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = copy.deepcopy(msg.pose.pose)
        if path.poses:
            previous = path.poses[-1].pose.position
            current = pose.pose.position
            if math.sqrt((current.x - previous.x) ** 2 +
                         (current.y - previous.y) ** 2 +
                         (current.z - previous.z) ** 2) < 0.03:
                return
        path.header.stamp = msg.header.stamp
        path.poses.append(pose)
        if len(path.poses) > self.maximum_path_points:
            path.poses = path.poses[-self.maximum_path_points:]
        self.path_pubs[uid].publish(path)

    def state_cb(self, uid, msg):
        self.states[uid] = msg
        if not self.orbit_released[uid]:
            return
        if msg.mission_phase not in {
                "TARGET_LOCKED", "NAVIGATING", "EVALUATING", "RELOCATING",
                "RECOVERING", "HOLDING"}:
            return
        angle = math.atan2(
            msg.pose.position.y - self.tower[1],
            msg.pose.position.x - self.tower[0])
        previous = self.orbit_angles[uid]
        if previous is not None:
            delta = (angle - previous + math.pi) % (2.0 * math.pi) - math.pi
            if delta > 0.0:
                self.accumulated[uid] += delta
        self.orbit_angles[uid] = angle
        degrees = math.degrees(angle) % 360.0
        sector = int(math.floor((degrees + 22.5) / 45.0)) % 8
        self.sector_masks[uid] |= 1 << sector

    def mission_cb(self, uid, msg):
        self.mission_states[uid] = msg.data

    def bridge_cb(self, uid, msg):
        self.bridge_states[uid] = msg.data

    def sector_cb(self, uid, msg):
        self.sectors[uid] = int(msg.data) + 1

    def planner_cb(self, uid, msg):
        self.planners[uid] = msg

    def safety_cb(self, msg):
        self.last_safety = "{} UAV{}-{}: {}".format(
            msg.code, msg.uav_id, msg.peer_id, msg.detail)

    def coordinator_cb(self, msg):
        self.coordinator = msg.coordinator_state
        self.coordinator_reason = msg.reason

    def formation_cb(self, msg):
        try:
            self.formation = json.loads(msg.data)
            released = self.formation.get("orbit_released", {})
            for uid in self.uav_ids:
                self.orbit_released[uid] = bool(
                    released.get(str(uid), released.get(uid, False)))
        except (TypeError, ValueError):
            self.formation = {"decode_error": msg.data}

    @staticmethod
    def color(marker, uid, alpha=1.0):
        marker.color.r, marker.color.g, marker.color.b = COLORS[uid]
        marker.color.a = alpha

    def publish_static_markers(self):
        now = rospy.Time.now()
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "world"
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for waypoint in range(8):
            angle = waypoint * math.pi / 4.0
            point = Point()
            point.x = self.tower[0] + self.radius * math.cos(angle)
            point.y = self.tower[1] + self.radius * math.sin(angle)
            point.z = max(self.heights) + 0.35
            sphere = Marker()
            sphere.header.frame_id = "world"
            sphere.header.stamp = now
            sphere.ns = "main_waypoints"
            sphere.id = waypoint
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = point
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.32
            sphere.color.r = sphere.color.g = sphere.color.b = 1.0
            sphere.color.a = 0.9
            markers.markers.append(sphere)
            label = Marker()
            label.header = sphere.header
            label.ns = "main_waypoint_labels"
            label.id = waypoint
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = copy.deepcopy(point)
            label.pose.position.z += 0.50
            label.pose.orientation.w = 1.0
            label.scale.z = 0.42
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = "WP{}".format(waypoint + 1)
            markers.markers.append(label)
        self.static_pub.publish(markers)

    def timer_cb(self, _event):
        markers = MarkerArray()
        now = rospy.Time.now()
        for uid in self.uav_ids:
            state = self.states.get(uid)
            if state is None:
                continue
            planner = self.planners.get(uid)
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = now
            marker.ns = "swarm_status"
            marker.id = uid
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position = copy.deepcopy(state.pose.position)
            marker.pose.position.z += 1.2
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.42
            self.color(marker, uid)
            planner_text = "NO_STATUS"
            if planner is not None:
                planner_text = "{} replans={} reason={}".format(
                    planner.planner_state,
                    planner.consecutive_plan_failures,
                    planner.failure_reason or "NONE")
            role = ("LEADER" if uid == self.role_order[0] else
                    "TRAILING" if uid == self.role_order[-1] else "MIDDLE")
            marker.text = (
                "UAV{} {} task={} sector={} mask=0x{:02X}\n"
                "released={} orbit={:.1f}deg speed_scale={} "
                "bridge={} safety={} planner={}").format(
                    uid, role, self.mission_states[uid], self.sectors[uid],
                    self.sector_masks[uid],
                    self.orbit_released[uid],
                    math.degrees(self.accumulated[uid]),
                    self.formation.get("speed_scales", {}).get(str(uid),
                                                               "pending"),
                    self.bridge_states[uid], self.last_safety, planner_text)
            markers.markers.append(marker)
        coordinator = Marker()
        coordinator.header.frame_id = "world"
        coordinator.header.stamp = now
        coordinator.ns = "coordinator_status"
        coordinator.id = 0
        coordinator.type = Marker.TEXT_VIEW_FACING
        coordinator.action = Marker.ADD
        coordinator.pose.position.x = self.tower[0]
        coordinator.pose.position.y = self.tower[1]
        coordinator.pose.position.z = max(self.heights) + 3.0
        coordinator.pose.orientation.w = 1.0
        coordinator.scale.z = 0.55
        coordinator.color.r = coordinator.color.g = coordinator.color.b = 1.0
        coordinator.color.a = 1.0
        coordinator.text = "UAV3 -> UAV2 -> UAV1\n{}\n{}\n{}".format(
            self.coordinator, self.coordinator_reason,
            json.dumps(self.formation, sort_keys=True))
        markers.markers.append(coordinator)
        self.status_pub.publish(markers)

    def topic_diagnostics_cb(self, _event):
        published = {name: topic_type
                     for name, topic_type in rospy.get_published_topics()}
        expected = []
        for uid in self.uav_ids:
            planner_id = uid - 1
            expected.extend([
                "/uav{}/cloud_registered_peer_filtered".format(uid),
                "/uav{}/drone_{}_ego_planner_node/optimal_list".format(
                    uid, planner_id),
                "/uav{}/tower_mission/current_target".format(uid),
                "/uav{}/Odometry".format(uid),
            ])
        missing = [topic for topic in expected if topic not in published]
        duplicate_names = len(expected) != len(set(expected))
        message = String()
        message.data = (
            "FAIL missing={} duplicate_expected_names={}".format(
                missing, duplicate_names)
            if missing or duplicate_names else
            "PASS independent cloud/EGO/target/odom topics for UAV1/UAV2/UAV3")
        self.topic_diag_pub.publish(message)
        if missing:
            rospy.logwarn_throttle(
                3.0, "[SWARM_RVIZ] topic independence pending: %s", message.data)


if __name__ == "__main__":
    rospy.init_node("swarm_rviz_diagnostics")
    SwarmRvizDiagnostics()
    rospy.spin()
