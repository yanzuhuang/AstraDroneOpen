#!/usr/bin/env python3
"""Publish one vehicle's normalized world-frame swarm state and prediction."""

import copy
import math

import rospy
from astra_swarm_manager.kinematics import (
    KinematicEstimator,
    bounded_prediction,
)
from astra_swarm_msgs.msg import PredictedTrajectory, SwarmState
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class StatePublisher:
    def __init__(self):
        self.uav_id = int(rospy.get_param("~uav_id"))
        self.vehicle_ns = rospy.get_param("~vehicle_namespace")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.home = rospy.get_param("~home_position", [0.0, 0.0, 0.0])
        self.horizon = float(rospy.get_param(
            "~trajectory_prediction_horizon", 4.0))
        self.sample_period = float(rospy.get_param(
            "~trajectory_prediction_period", 0.25))
        self.timeout = float(rospy.get_param("~heartbeat_timeout", 1.0))
        self.velocity_alpha = float(rospy.get_param(
            "~velocity_filter_alpha", 0.25))
        self.acceleration_alpha = float(rospy.get_param(
            "~acceleration_filter_alpha", 0.15))
        self.maximum_prediction_speed = float(rospy.get_param(
            "~maximum_prediction_speed", 3.0))
        self.maximum_prediction_acceleration = float(rospy.get_param(
            "~maximum_prediction_acceleration", 2.0))
        self.odom = None
        self.target = Point()
        self.mission_phase = "INIT"
        self.flight_state = "WAIT_FCU"
        self.last_odom_wall = rospy.Time(0)
        self.kinematics = KinematicEstimator(
            self.velocity_alpha, self.acceleration_alpha,
            self.maximum_prediction_speed,
            self.maximum_prediction_acceleration)

        rospy.Subscriber(rospy.get_param("~odom_topic"), Odometry,
                         self.odom_cb, queue_size=5)
        rospy.Subscriber(rospy.get_param("~target_topic"), PoseStamped,
                         self.target_cb, queue_size=5)
        rospy.Subscriber(rospy.get_param("~mission_state_topic"), String,
                         self.mission_cb, queue_size=5)
        rospy.Subscriber(rospy.get_param("~bridge_state_topic"), String,
                         self.bridge_cb, queue_size=5)
        # MAVROS state is subscribed explicitly so a disconnected FCU cannot
        # be represented as healthy solely because FAST-LIO remains alive.
        self.fcu_connected = False
        rospy.Subscriber(rospy.get_param("~mavros_state_topic"), State,
                         self.fcu_cb, queue_size=5)
        self.state_pub = rospy.Publisher("state", SwarmState, queue_size=5)
        self.trajectory_pub = rospy.Publisher(
            "predicted_trajectory", PredictedTrajectory, queue_size=5)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def odom_cb(self, msg):
        position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        # FAST-LIO's ROS Odometry currently leaves twist at zero. Estimate
        # motion from successive world-aligned poses so prediction is useful.
        self.kinematics.update(position, msg.header.stamp.to_sec())
        self.odom = msg
        self.last_odom_wall = rospy.Time.now()

    def target_cb(self, msg):
        self.target.x = msg.pose.position.x + self.home[0]
        self.target.y = msg.pose.position.y + self.home[1]
        self.target.z = msg.pose.position.z + self.home[2]

    def mission_cb(self, msg):
        self.mission_phase = msg.data

    def bridge_cb(self, msg):
        self.flight_state = msg.data

    def fcu_cb(self, msg):
        self.fcu_connected = msg.connected

    def timer_cb(self, _event):
        if self.odom is None:
            return
        now = rospy.Time.now()
        fresh = (now - self.last_odom_wall).to_sec() <= self.timeout
        state = SwarmState()
        state.header.stamp = now
        state.header.frame_id = self.world_frame
        state.uav_id = self.uav_id
        state.vehicle_namespace = self.vehicle_ns
        state.pose = copy.deepcopy(self.odom.pose.pose)
        state.pose.position.x += self.home[0]
        state.pose.position.y += self.home[1]
        state.pose.position.z += self.home[2]
        state.velocity.x, state.velocity.y, state.velocity.z = (
            self.kinematics.velocity)
        state.acceleration.x, state.acceleration.y, state.acceleration.z = (
            self.kinematics.acceleration)
        q = state.pose.orientation
        state.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        state.flight_state = self.flight_state
        state.current_height = state.pose.position.z
        state.current_target = self.target
        state.mission_phase = self.mission_phase
        state.heartbeat_ok = fresh and self.fcu_connected
        state.localization_valid = fresh
        state.covariance = list(self.odom.pose.covariance)
        self.state_pub.publish(state)

        prediction = PredictedTrajectory()
        prediction.header = state.header
        prediction.uav_id = self.uav_id
        prediction.horizon = rospy.Duration(self.horizon)
        prediction.sample_period = rospy.Duration(self.sample_period)
        predicted_points = bounded_prediction(
            (state.pose.position.x, state.pose.position.y,
             state.pose.position.z),
            (state.velocity.x, state.velocity.y, state.velocity.z),
            (state.acceleration.x, state.acceleration.y,
             state.acceleration.z),
            self.horizon, self.sample_period,
            self.maximum_prediction_speed,
            self.maximum_prediction_acceleration)
        for values in predicted_points:
            point = Point()
            point.x, point.y, point.z = values
            prediction.points.append(point)
        self.trajectory_pub.publish(prediction)


if __name__ == "__main__":
    rospy.init_node("swarm_state_publisher")
    StatePublisher()
    rospy.spin()
