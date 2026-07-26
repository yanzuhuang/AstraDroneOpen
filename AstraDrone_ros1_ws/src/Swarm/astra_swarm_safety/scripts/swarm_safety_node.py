#!/usr/bin/env python3
"""Fail-closed heartbeat and predicted-trajectory safety monitor."""

import rospy
from astra_swarm_msgs.msg import PredictedTrajectory, SafetyEvent, SwarmState
from astra_swarm_safety.prediction import separation_clear
from std_msgs.msg import Bool


ACTIVE = {
    "TAKEOFF", "HOVER_READY", "TRACK_EGO", "HOLD", "RETURN_HOME",
    "HOME_HOVER", "LANDING",
}


class SafetyMonitor:
    def __init__(self):
        self.minimum_3d = float(rospy.get_param(
            "~minimum_3d_separation", 3.0))
        self.minimum_vertical = float(rospy.get_param(
            "~minimum_vertical_separation", 4.0))
        self.timeout = float(rospy.get_param("~heartbeat_timeout", 1.0))
        self.states = {}
        self.trajectories = {}
        self.received = {}
        for uid in (1, 2):
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

    @staticmethod
    def tuple_point(point):
        return (point.x, point.y, point.z)

    def publish_event(self, code, detail, severity, current=0.0,
                      predicted=0.0):
        event = SafetyEvent()
        event.header.stamp = rospy.Time.now()
        event.header.frame_id = "world"
        event.severity = severity
        event.uav_id = 1
        event.peer_id = 2
        event.code = code
        event.detail = detail
        event.current_distance = current
        event.predicted_minimum_distance = predicted
        self.event_pub.publish(event)

    def timer_cb(self, _event):
        now = rospy.Time.now()
        for uid in (1, 2):
            if (uid not in self.states or uid not in self.received or
                    (now - self.received[uid]).to_sec() > self.timeout or
                    not self.states[uid].heartbeat_ok or
                    not self.states[uid].localization_valid):
                self.clear_pub.publish(Bool(data=False))
                self.publish_event(
                    "HEARTBEAT_OR_LOCALIZATION_TIMEOUT",
                    "new permissions are inhibited", SafetyEvent.STOP)
                return
        if 1 not in self.trajectories or 2 not in self.trajectories:
            self.clear_pub.publish(Bool(data=False))
            return
        state1, state2 = self.states[1], self.states[2]
        position1 = self.tuple_point(state1.pose.position)
        position2 = self.tuple_point(state2.pose.position)
        trajectory1 = [
            self.tuple_point(p) for p in self.trajectories[1].points]
        trajectory2 = [
            self.tuple_point(p) for p in self.trajectories[2].points]
        enforce_vertical = (
            state1.flight_state in ACTIVE and state2.flight_state in ACTIVE)
        clear, current, predicted = separation_clear(
            position1, position2, trajectory1, trajectory2,
            self.minimum_3d, self.minimum_vertical, enforce_vertical)
        self.clear_pub.publish(Bool(data=clear))
        if not clear:
            self.publish_event(
                "PREDICTED_SEPARATION_CONFLICT",
                "hold current safe layer; inhibit takeoff/transition",
                SafetyEvent.STOP, current, predicted)


if __name__ == "__main__":
    rospy.init_node("astra_swarm_safety")
    SafetyMonitor()
    rospy.spin()
