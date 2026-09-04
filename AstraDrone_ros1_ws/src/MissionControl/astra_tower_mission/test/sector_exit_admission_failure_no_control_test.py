#!/usr/bin/env python3

import math
import threading
import unittest

import rospy
import rostest
import sensor_msgs.point_cloud2 as point_cloud2
from astra_custom_msgs.msg import PlannerStatus
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


class SectorExitAdmissionFailureNoControl(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.state = ""
        self.states = []
        self.position = [12.0, 0.0, 4.0]
        self.yaw = 0.0
        self.trajectory_id = 0
        self.exit_gate = None
        self.wait_started = None
        self.return_egress_started = None
        self.return_egress_goal_seen = False
        self.resume_calls = 0
        self.exit_resume_calls = 0
        self.land_calls = 0

        self.odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher(
            "/stage3/cloud_registered_filtered", PointCloud2, queue_size=2)
        self.occupancy_pub = rospy.Publisher(
            "/grid_map/occupancy_inflate", PointCloud2, queue_size=2)
        self.bridge_state_pub = rospy.Publisher(
            "/ego_mavros_bridge/state", String, queue_size=2, latch=True)
        self.mavros_state_pub = rospy.Publisher(
            "/mavros/state", State, queue_size=2, latch=True)
        self.command_pub = rospy.Publisher(
            "/planning/pos_cmd", PositionCommand, queue_size=10)
        self.status_pub = rospy.Publisher(
            "/planner/status", PlannerStatus, queue_size=10, latch=True)
        self.exit_permission_pub = rospy.Publisher(
            "/tower_mission/exit_permission", Bool, queue_size=2, latch=True)

        rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                         self.goal_callback, queue_size=10)
        rospy.Subscriber("/tower_mission/state", String,
                         self.state_callback, queue_size=10)

        self.services = [
            rospy.Service("/ego_mavros_bridge/enable_tracking", SetBool,
                          lambda _request: SetBoolResponse(True, "mock")),
            rospy.Service("/ego_mavros_bridge/cancel_current_trajectory", Trigger,
                          lambda _request: TriggerResponse(True, "mock")),
            rospy.Service("/ego_mavros_bridge/resume_ego", Trigger,
                          self.resume_service),
            rospy.Service("/ego_mavros_bridge/return_home", Trigger,
                          lambda _request: TriggerResponse(True, "mock")),
            rospy.Service("/ego_mavros_bridge/land", Trigger,
                          self.land_service),
        ]
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_inputs)

    def state_callback(self, message):
        with self.lock:
            self.state = message.data
            self.states.append(message.data)
            if message.data == "WAIT_EXIT_PERMISSION" and self.wait_started is None:
                self.wait_started = rospy.Time.now()
            if message.data == "RETURN_EGRESS" and self.return_egress_started is None:
                self.return_egress_started = rospy.Time.now()

    def goal_callback(self, message):
        with self.lock:
            self.trajectory_id += 1
            target = [message.pose.position.x,
                      message.pose.position.y,
                      message.pose.position.z]
            if self.state == "ENTRY_GATE_TRANSIT" and (
                    abs(math.hypot(target[0], target[1]) - 15.0) < 0.1):
                self.exit_gate = list(target)
            if self.state == "RETURN_EGRESS":
                self.return_egress_goal_seen = True
                return
            self.position = target
            q = message.pose.orientation
            self.yaw = math.atan2(2.0 * q.w * q.z,
                                  1.0 - 2.0 * q.z * q.z)

    def resume_service(self, _request):
        with self.lock:
            self.resume_calls += 1
            if self.state == "WAIT_EXIT_PERMISSION":
                self.exit_resume_calls += 1
        return TriggerResponse(True, "mock resume")

    def land_service(self, _request):
        with self.lock:
            self.land_calls += 1
        return TriggerResponse(True, "mock land")

    def publish_inputs(self, _event):
        now = rospy.Time.now()
        with self.lock:
            position = list(self.position)
            yaw = self.yaw
            trajectory_id = self.trajectory_id
            state = self.state
            exit_gate = None if self.exit_gate is None else list(self.exit_gate)
            return_egress_goal_seen = self.return_egress_goal_seen

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "camera_init"
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]
        odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.odom_pub.publish(odom)

        rays = []
        for azimuth_deg in range(-180, 180, 8):
            azimuth = math.radians(azimuth_deg)
            for elevation_deg in range(-6, 53, 8):
                elevation = math.radians(elevation_deg)
                horizontal = 40.0 * math.cos(elevation)
                rays.append((
                    position[0] + horizontal * math.cos(azimuth),
                    position[1] + horizontal * math.sin(azimuth),
                    position[2] + 40.0 * math.sin(elevation),
                ))
        cloud = point_cloud2.create_cloud_xyz32(odom.header, rays)
        self.cloud_pub.publish(cloud)

        occupancy_points = []
        if state == "WAIT_EXIT_PERMISSION" and exit_gate is not None:
            occupancy_points.append(tuple(exit_gate))
        occupancy = point_cloud2.create_cloud_xyz32(odom.header,
                                                     occupancy_points)
        self.occupancy_pub.publish(occupancy)
        self.exit_permission_pub.publish(
            Bool(data=state == "WAIT_EXIT_PERMISSION"))
        self.bridge_state_pub.publish(String(data="HOVER_READY"))

        mavros_state = State()
        mavros_state.header.stamp = now
        mavros_state.connected = True
        mavros_state.armed = True
        mavros_state.mode = "OFFBOARD"
        self.mavros_state_pub.publish(mavros_state)

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
        status.planner_state = (
            "REPLAN_TRAJ" if return_egress_goal_seen else "EXEC_TRAJ")
        status.target_id = "integration_target_{}".format(trajectory_id)
        status.trajectory_id = trajectory_id
        status.last_plan_success = not return_egress_goal_seen
        status.consecutive_plan_failures = 3 if return_egress_goal_seen else 0
        status.failure_reason = (
            PlannerStatus.REPLAN_FAILED
            if return_egress_goal_seen else PlannerStatus.NONE)
        status.status_timestamp = now
        self.status_pub.publish(status)

    def test_exit_admission_failure_is_bounded_and_fail_closed(self):
        deadline = rospy.Time.now() + rospy.Duration(18.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if self.state in ("FAILURE_LANDING", "ERROR"):
                    break
            rate.sleep()

        with self.lock:
            self.assertIsNotNone(self.exit_gate)
            self.assertIn("WAIT_EXIT_PERMISSION", self.states)
            self.assertIn("RETURN_EGRESS", self.states)
            self.assertNotIn("GO_TO_EXIT_GATE", self.states)
            self.assertTrue(self.return_egress_goal_seen)
            self.assertIn(self.state, ("FAILURE_LANDING", "ERROR"))
            self.assertEqual(self.exit_resume_calls, 1)
            self.assertGreaterEqual(self.land_calls, 1)
            self.assertIsNotNone(self.wait_started)
            self.assertIsNotNone(self.return_egress_started)
            self.assertLess((self.return_egress_started -
                             self.wait_started).to_sec(), 1.0)


if __name__ == "__main__":
    rospy.init_node("sector_exit_admission_failure_no_control_test")
    rostest.rosrun("astra_tower_mission",
                   "sector_exit_admission_failure_no_control",
                   SectorExitAdmissionFailureNoControl)
