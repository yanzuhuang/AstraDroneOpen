#!/usr/bin/env python3

import math
import threading
import unittest

import rospy
import rostest
import sensor_msgs.point_cloud2 as point_cloud2
from astra_custom_msgs.msg import PlannerStatus
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


class Stage3NoControlIntegration(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.state = ""
        self.goal_count = 0
        self.trajectory_id = 0
        self.failure_injected = False
        self.recovery_started = False
        self.bridge_hold_injected = False
        self.bridge_hold_active = False
        self.approach_goal_count = 0
        self.obstacle_enabled = False
        self.return_calls = 0
        self.cancel_calls = 0
        self.resume_calls = 0
        self.tracking_disable_calls = 0
        self.recovery_goal_heights = []
        self.face_tower_modes = []
        self.mavros_setpoints = 0
        self.position = [12.0, 0.0, 4.0]
        self.yaw = 0.0

        self.odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher(
            "/stage3/cloud_registered_filtered", PointCloud2, queue_size=2)
        self.occupancy_pub = rospy.Publisher(
            "/grid_map/occupancy_inflate", PointCloud2, queue_size=2)
        self.bridge_state_pub = rospy.Publisher(
            "/ego_mavros_bridge/state", String, queue_size=2, latch=True)
        self.command_pub = rospy.Publisher(
            "/planning/pos_cmd", PositionCommand, queue_size=10)
        self.status_pub = rospy.Publisher(
            "/planner/status", PlannerStatus, queue_size=10, latch=True)

        rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                         self.goal_callback, queue_size=10)
        rospy.Subscriber("/tower_mission/state", String,
                         self.state_callback, queue_size=10)
        rospy.Subscriber("/tower_mission/face_tower", Bool,
                         self.face_tower_callback, queue_size=10)
        rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget,
                         self.mavros_callback, queue_size=1)

        self.services = [
            rospy.Service("/ego_mavros_bridge/enable_tracking", SetBool,
                          self.tracking_service),
            rospy.Service("/ego_mavros_bridge/cancel_current_trajectory", Trigger,
                          self.cancel_service),
            rospy.Service("/ego_mavros_bridge/resume_ego", Trigger,
                          self.resume_service),
            rospy.Service("/ego_mavros_bridge/return_home", Trigger,
                          self.return_service),
            rospy.Service("/ego_mavros_bridge/land", Trigger,
                          lambda _request: TriggerResponse(True, "mock land")),
        ]
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_inputs)

    def state_callback(self, message):
        with self.lock:
            self.state = message.data
            if message.data == "EVALUATING" and not self.recovery_started:
                self.obstacle_enabled = True
            if message.data == "RECOVERING":
                self.recovery_started = True
                self.failure_injected = False

    def face_tower_callback(self, message):
        with self.lock:
            self.face_tower_modes.append(message.data)

    def mavros_callback(self, _message):
        with self.lock:
            self.mavros_setpoints += 1

    def goal_callback(self, message):
        with self.lock:
            self.goal_count += 1
            self.trajectory_id += 1
            state = self.state
            if state == "APPROACH":
                self.approach_goal_count += 1
                if not self.bridge_hold_injected:
                    # Reproduce the 2026-07-23 13:15 bag chain: bridge enters
                    # a local HOLD immediately after the first ENTRY_GATE
                    # command. The mission must cancel/supervise it, relocate
                    # the gate, and continue without any MAVROS publication.
                    self.bridge_hold_injected = True
                    self.bridge_hold_active = True
                    return
            if state == "RECOVERING":
                self.recovery_goal_heights.append(message.pose.position.z)
            # After alternate ENTRY_GATE succeeds, hold the first sector
            # target away from odom so a real PlannerStatus failure event
            # drives the separate HOLD/R1/R2 chain.
            if state != "APPROACH" and not self.recovery_started:
                self.failure_injected = True
                return
            self.position = [message.pose.position.x,
                             message.pose.position.y,
                             message.pose.position.z]
            q = message.pose.orientation
            self.yaw = math.atan2(2.0 * q.w * q.z,
                                  1.0 - 2.0 * q.z * q.z)

    def tracking_service(self, request):
        with self.lock:
            if not request.data:
                self.tracking_disable_calls += 1
        return SetBoolResponse(True, "mock tracking")

    def cancel_service(self, _request):
        with self.lock:
            self.cancel_calls += 1
            self.bridge_hold_active = False
        return TriggerResponse(True, "mock cancel")

    def resume_service(self, _request):
        with self.lock:
            self.resume_calls += 1
        return TriggerResponse(True, "mock resume")

    def return_service(self, _request):
        with self.lock:
            self.return_calls += 1
        return TriggerResponse(True, "mock return")

    def publish_inputs(self, _event):
        now = rospy.Time.now()
        with self.lock:
            position = list(self.position)
            yaw = self.yaw
            trajectory_id = self.trajectory_id
            failures = 3 if self.failure_injected else 0
            obstacle_enabled = self.obstacle_enabled
            bridge_state = ("HOLD" if self.bridge_hold_active
                            else "HOVER_READY")

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "camera_init"
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]
        odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.odom_pub.publish(odom)

        cloud = point_cloud2.create_cloud_xyz32(
            odom.header, [(100.0, 100.0, 100.0)])
        self.cloud_pub.publish(cloud)
        occupancy_points = [(8.0, 0.0, 5.0)] if obstacle_enabled else []
        occupancy = point_cloud2.create_cloud_xyz32(odom.header,
                                                     occupancy_points)
        self.occupancy_pub.publish(occupancy)
        self.bridge_state_pub.publish(String(data=bridge_state))

        command = PositionCommand()
        command.header = odom.header
        command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        command.trajectory_id = trajectory_id
        command.position.x = position[0]
        command.position.y = position[1]
        command.position.z = position[2]
        command.yaw = yaw
        self.command_pub.publish(command)

        status = PlannerStatus()
        status.header = odom.header
        status.planner_state = "REPLAN_TRAJ" if failures else "EXEC_TRAJ"
        status.target_id = "integration_target"
        status.trajectory_id = trajectory_id
        status.last_plan_success = failures == 0
        status.consecutive_plan_failures = failures
        status.failure_reason = (PlannerStatus.REPLAN_FAILED if failures
                                 else PlannerStatus.NONE)
        status.status_timestamp = now
        self.status_pub.publish(status)

    def test_minimum_stage3_safety_chain(self):
        deadline = rospy.Time.now() + rospy.Duration(25.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if self.return_calls > 0 and self.state == "RETURN_HOME":
                    break
            rate.sleep()

        with self.lock:
            self.assertEqual(self.state, "RETURN_HOME")
            self.assertTrue(self.bridge_hold_injected)
            self.assertGreaterEqual(self.approach_goal_count, 2)
            self.assertGreaterEqual(self.cancel_calls, 2)
            self.assertGreaterEqual(self.tracking_disable_calls, 2)
            self.assertGreaterEqual(self.resume_calls, 1)
            self.assertEqual(self.return_calls, 1)
            self.assertGreaterEqual(len(self.recovery_goal_heights), 3)
            self.assertAlmostEqual(self.recovery_goal_heights[0], 7.0, places=3)
            self.assertAlmostEqual(self.recovery_goal_heights[1], 7.0, places=3)
            self.assertAlmostEqual(self.recovery_goal_heights[2], 5.0, places=3)
            self.assertIn(True, self.face_tower_modes)
            self.assertEqual(self.mavros_setpoints, 0)


if __name__ == "__main__":
    rospy.init_node("stage3_no_control_integration_test")
    rostest.rosrun("astra_tower_mission",
                   "stage3_no_control_integration",
                   Stage3NoControlIntegration)
