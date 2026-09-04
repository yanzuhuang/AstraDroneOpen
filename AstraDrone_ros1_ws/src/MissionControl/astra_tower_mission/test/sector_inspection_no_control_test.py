#!/usr/bin/env python3

import math
import threading
import unittest

import rospy
import rostest
import sensor_msgs.point_cloud2 as point_cloud2
from astra_custom_msgs.msg import PlannerStatus
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


class SectorInspectionNoControl(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.state = ""
        self.states = []
        self.goal_count = 0
        self.goal_positions = []
        self.trajectory_id = 0
        self.failure_injected = False
        self.sector_failure_goal_count = 0
        self.recovery_started = False
        self.bridge_hold_injected = False
        self.bridge_hold_active = False
        self.approach_goal_count = 0
        self.entry_gate_positions = []
        self.inspection_positions = []
        self.obstacle_enabled = False
        self.return_calls = 0
        self.cancel_calls = 0
        self.resume_calls = 0
        self.tracking_disable_calls = 0
        self.recovery_goal_heights = []
        self.ascent_positions = []
        self.layer_transition_positions = []
        self.normal_return_positions = []
        self.exit_gate_positions = []
        self.home_overhead_positions = []
        self.segmented_descent_positions = []
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
        self.mavros_state_pub = rospy.Publisher(
            "/mavros/state", State, queue_size=2, latch=True)
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
            self.states.append(message.data)
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
            self.goal_positions.append((
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ))
            state = self.state
            if state == "ENTRY_GATE_TRANSIT":
                self.approach_goal_count += 1
                self.entry_gate_positions.append((
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ))
                if not self.bridge_hold_injected:
                    # Inject one bridge-local HOLD during the newly explicit
                    # safe-altitude ENTRY_GATE transit. The mission must
                    # supervise it and retry the same locked goal without
                    # switching ascent channels or publishing MAVROS output.
                    self.bridge_hold_injected = True
                    self.bridge_hold_active = True
                    return
            if state == "RECOVERING":
                self.recovery_goal_heights.append(message.pose.position.z)
            if state == "SEGMENTED_CLIMB":
                self.ascent_positions.append((
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ))
            if state == "LAYER_TRANSITION":
                self.layer_transition_positions.append((
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ))
            if state == "NORMAL_RETURN":
                self.normal_return_positions.append((
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ))
            if state == "HOME_OVERHEAD_TRANSIT":
                self.home_overhead_positions.append((
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ))
            if state == "SEGMENTED_HOME_DESCENT":
                self.segmented_descent_positions.append((
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ))
            if (state == "GO_TO_EXIT_GATE" or
                    (abs(message.pose.position.z - 22.0) < 0.05 and
                     abs(math.hypot(message.pose.position.x,
                                    message.pose.position.y) - 15.0) < 0.1)):
                self.exit_gate_positions.append((
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ))
            if state in ("TARGET_LOCKED", "NAVIGATING"):
                self.inspection_positions.append((
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ))
            # After ENTRY_GATE succeeds, hold the first sector
            # target away from odom so a real PlannerStatus failure event
            # drives the separate HOLD/R1/R2 chain.
            if state in ("TARGET_LOCKED", "NAVIGATING") and not self.recovery_started:
                if self.sector_failure_goal_count < 2:
                    self.sector_failure_goal_count += 1
                    self.failure_injected = True
                    return
                self.failure_injected = False
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

        rays = []
        for azimuth_deg in range(-180, 180, 4):
            azimuth = math.radians(azimuth_deg)
            for elevation_deg in range(-6, 53, 4):
                elevation = math.radians(elevation_deg)
                horizontal = 40.0 * math.cos(elevation)
                rays.append((
                    position[0] + horizontal * math.cos(azimuth),
                    position[1] + horizontal * math.sin(azimuth),
                    position[2] + 40.0 * math.sin(elevation),
                ))
        cloud = point_cloud2.create_cloud_xyz32(odom.header, rays)
        self.cloud_pub.publish(cloud)
        occupancy_points = [(8.0, 0.0, 5.0)] if obstacle_enabled else []
        occupancy = point_cloud2.create_cloud_xyz32(odom.header,
                                                     occupancy_points)
        self.occupancy_pub.publish(occupancy)
        self.bridge_state_pub.publish(String(data=bridge_state))
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
        status.planner_state = "REPLAN_TRAJ" if failures else "EXEC_TRAJ"
        status.target_id = "integration_target_{}".format(trajectory_id)
        status.trajectory_id = trajectory_id
        status.last_plan_success = failures == 0
        status.consecutive_plan_failures = failures
        status.failure_reason = (PlannerStatus.REPLAN_FAILED if failures
                                 else PlannerStatus.NONE)
        status.status_timestamp = now
        self.status_pub.publish(status)

    def test_minimum_sector_inspection_safety_chain(self):
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
            self.assertGreaterEqual(len(self.entry_gate_positions), 1)
            selected_entry = self.entry_gate_positions[-1]
            selected_angle = (
                math.degrees(math.atan2(selected_entry[1],
                                        selected_entry[0])) + 360.0
            ) % 360.0
            self.assertGreaterEqual(selected_angle, 247.5)
            self.assertLessEqual(selected_angle, 292.5)
            self.assertAlmostEqual(
                math.hypot(selected_entry[0], selected_entry[1]),
                15.0, places=2)
            self.assertGreaterEqual(self.cancel_calls, 2)
            self.assertGreaterEqual(self.tracking_disable_calls, 2)
            self.assertGreaterEqual(self.resume_calls, 1)
            self.assertEqual(self.return_calls, 1)
            self.assertNotIn("NORMAL_RETURN", self.states)
            self.assertIn("GO_TO_EXIT_GATE", self.states)
            self.assertIn("HOME_OVERHEAD_TRANSIT", self.states)
            self.assertIn("SEGMENTED_HOME_DESCENT", self.states)
            self.assertNotIn("RETURN_EGRESS", self.states)
            self.assertGreaterEqual(len(self.exit_gate_positions), 1)
            self.assertAlmostEqual(self.exit_gate_positions[-1][0],
                                   selected_entry[0], places=2)
            self.assertAlmostEqual(self.exit_gate_positions[-1][1],
                                   selected_entry[1], places=2)
            self.assertAlmostEqual(self.exit_gate_positions[-1][2],
                                   22.0, places=2)
            self.assertIn("LAYER_TRANSITION", self.states)
            self.assertEqual(self.states.count("STAGING_POINT"), 1)
            self.assertEqual(self.sector_failure_goal_count, 2)
            self.assertEqual(len(self.recovery_goal_heights), 0)
            self.assertEqual(
                [round(item[2], 2) for item in self.ascent_positions],
                [10.0, 18.0, 26.0],
            )
            self.assertGreaterEqual(
                self.states.count("LAYER_TRANSITION"), 3)
            self.assertEqual(len(self.normal_return_positions), 0)
            self.assertEqual(len(self.home_overhead_positions), 1)
            self.assertAlmostEqual(
                self.home_overhead_positions[0][0], 12.0, places=2)
            self.assertAlmostEqual(
                self.home_overhead_positions[0][1], 0.0, places=2)
            self.assertAlmostEqual(
                self.home_overhead_positions[0][2], 22.0, places=2)
            self.assertEqual(
                [round(item[2], 2)
                 for item in self.segmented_descent_positions],
                [18.0, 10.0],
            )
            for item in self.segmented_descent_positions:
                self.assertAlmostEqual(item[0], 12.0, places=2)
                self.assertAlmostEqual(item[1], 0.0, places=2)
            final_layer_gate_positions = [
                item for item in self.goal_positions
                if abs(item[2] - 22.0) < 0.05 and
                abs(math.hypot(item[0], item[1]) - 15.0) < 0.1
            ]
            self.assertGreaterEqual(len(final_layer_gate_positions), 1)
            self.assertLessEqual(
                max(math.hypot(item[0], item[1])
                    for item in final_layer_gate_positions),
                15.1,
            )
            self.assertIn(True, self.face_tower_modes)
            self.assertEqual(self.mavros_setpoints, 0)
            direction = rospy.get_param(
                "/tower_mission/mission/direction")
            expected_fallback_angle = (
                315.0 if direction == "counter_clockwise" else 225.0)
            standard_route_positions = [
                item for item in self.goal_positions
                if abs(math.hypot(item[0], item[1]) - 8.0) < 0.1
            ]
            for layer_height, expected_angle in (
                    (26.0, 270.0), (22.0, expected_fallback_angle)):
                layer_positions = [
                    item for item in standard_route_positions
                    if abs(item[2] - layer_height) < 0.05
                ]
                self.assertGreaterEqual(len(layer_positions), 1)
                first_angle = (
                    math.degrees(math.atan2(layer_positions[0][1],
                                            layer_positions[0][0])) + 360.0
                ) % 360.0
                self.assertAlmostEqual(first_angle, expected_angle, places=1)
            upper_unique_angles = []
            for item in standard_route_positions:
                if abs(item[2] - 26.0) >= 0.05:
                    continue
                angle = (
                    math.degrees(math.atan2(item[1], item[0])) + 360.0
                ) % 360.0
                if (not upper_unique_angles or
                        abs(angle - upper_unique_angles[-1]) > 0.1):
                    upper_unique_angles.append(angle)
            self.assertGreaterEqual(len(upper_unique_angles), 2)
            self.assertAlmostEqual(
                upper_unique_angles[1], expected_fallback_angle, places=1)


if __name__ == "__main__":
    rospy.init_node("sector_inspection_no_control_test")
    rostest.rosrun("astra_tower_mission",
                   "sector_inspection_no_control",
                   SectorInspectionNoControl)
