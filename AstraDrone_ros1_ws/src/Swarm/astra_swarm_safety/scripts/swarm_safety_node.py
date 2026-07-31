#!/usr/bin/env python3
"""Fail-closed heartbeat and predicted-trajectory safety monitor."""

import math

import rospy
from astra_swarm_msgs.msg import PredictedTrajectory, SafetyEvent, SwarmState
from astra_swarm_safety.prediction import pairwise_ids, separation_clear
from std_msgs.msg import Bool


ACTIVE = {
    "TAKEOFF", "HOVER_READY", "TRACK_EGO", "HOLD", "RETURN_HOME",
    "HOME_HOVER", "LANDING",
}


class SafetyMonitor:
    def __init__(self):
        self.minimum_3d = float(rospy.get_param(
            "~minimum_3d_separation", 3.0))
        self.optimizer_clearance = float(rospy.get_param(
            "~optimizer_swarm_clearance", 0.0))
        if (not math.isfinite(self.optimizer_clearance)
                or self.optimizer_clearance <= 0.0):
            raise rospy.ROSException(
                "~optimizer_swarm_clearance must be finite and positive")
        self.timeout = float(rospy.get_param("~heartbeat_timeout", 1.0))
        self.trajectory_timeout = float(rospy.get_param(
            "~trajectory_timeout", self.timeout))
        self.uav_ids = [
            int(uid) for uid in rospy.get_param("~uav_ids", [1, 2])]
        try:
            self.vehicle_pairs = pairwise_ids(self.uav_ids)
        except ValueError as exc:
            raise rospy.ROSException(str(exc))
        self.states = {}
        self.trajectories = {}
        self.trajectory_received = {}
        self.received = {}
        for uid in self.uav_ids:
            rospy.Subscriber(
                "/uav{}/swarm/state".format(uid), SwarmState,
                lambda msg, u=uid: self.state_cb(u, msg), queue_size=5)
            rospy.Subscriber(
                "/uav{}/swarm/predicted_trajectory".format(uid),
                PredictedTrajectory,
                lambda msg, u=uid: self.trajectory_cb(u, msg), queue_size=5)
        self.clear_pub = rospy.Publisher(
            "/swarm/safety/clear", Bool, queue_size=1, latch=True)
        self.event_pub = rospy.Publisher(
            "/swarm/safety/event", SafetyEvent, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def state_cb(self, uid, msg):
        self.states[uid] = msg
        self.received[uid] = rospy.Time.now()

    def trajectory_cb(self, uid, msg):
        self.trajectories[uid] = msg
        self.trajectory_received[uid] = rospy.Time.now()

    @staticmethod
    def tuple_point(point):
        return (point.x, point.y, point.z)

    def publish_event(self, uav_id, peer_id, code, detail, severity,
                      current=0.0, predicted=0.0):
        event = SafetyEvent()
        event.header.stamp = rospy.Time.now()
        event.header.frame_id = "world"
        event.severity = severity
        event.uav_id = uav_id
        event.peer_id = peer_id
        event.code = code
        event.detail = detail
        event.current_distance = current
        event.predicted_minimum_distance = predicted
        self.event_pub.publish(event)
        rospy.logwarn_throttle(
            1.0,
            "[SWARM_SAFETY] code=%s uav=%d peer=%d current=%.3f "
            "predicted=%.3f detail=%s",
            code, uav_id, peer_id, current, predicted, detail)

    def timer_cb(self, _event):
        now = rospy.Time.now()
        for uid in self.uav_ids:
            if (uid not in self.states or uid not in self.received or
                    (now - self.received[uid]).to_sec() > self.timeout or
                    not self.states[uid].heartbeat_ok or
                    not self.states[uid].localization_valid):
                self.clear_pub.publish(Bool(data=False))
                self.publish_event(
                    uid, 0,
                    "HEARTBEAT_OR_LOCALIZATION_TIMEOUT",
                    "new permissions are inhibited", SafetyEvent.STOP)
                return
        for uid in self.uav_ids:
            trajectory = self.trajectories.get(uid)
            received = self.trajectory_received.get(uid)
            valid = (
                trajectory is not None and received is not None
                and trajectory.uav_id == uid
                and trajectory.header.frame_id == "world"
                and not trajectory.header.stamp.is_zero()
                and (now - received).to_sec() <= self.trajectory_timeout
                and (now - trajectory.header.stamp).to_sec()
                <= self.trajectory_timeout
                and trajectory.horizon.to_sec() > 0.0
                and trajectory.sample_period.to_sec() > 0.0
                and bool(trajectory.points)
                and all(math.isfinite(value)
                        for point in trajectory.points
                        for value in (point.x, point.y, point.z)))
            if not valid:
                self.clear_pub.publish(Bool(data=False))
                self.publish_event(
                    uid, 0, "TRAJECTORY_TIMEOUT_OR_INVALID",
                    "fresh world-frame future trajectory is required",
                    SafetyEvent.STOP)
                return
        for first_id, second_id in self.vehicle_pairs:
            first = self.states[first_id]
            second = self.states[second_id]
            position1 = self.tuple_point(first.pose.position)
            position2 = self.tuple_point(second.pose.position)
            trajectory1 = [
                self.tuple_point(p)
                for p in self.trajectories[first_id].points]
            trajectory2 = [
                self.tuple_point(p)
                for p in self.trajectories[second_id].points]
            clear, current, predicted = separation_clear(
                position1, position2, trajectory1, trajectory2,
                self.minimum_3d, self.optimizer_clearance)
            if not clear:
                self.clear_pub.publish(Bool(data=False))
                self.publish_event(
                    first_id, second_id,
                    "PREDICTED_SEPARATION_CONFLICT",
                    "hold current safe layer; inhibit takeoff/transition",
                    SafetyEvent.STOP, current, predicted)
                return
        self.clear_pub.publish(Bool(data=True))


if __name__ == "__main__":
    rospy.init_node("astra_swarm_safety")
    SafetyMonitor()
    rospy.spin()
