#ifndef EGO_GAZEBO_BRIDGE_EGO_MAVROS_BRIDGE_H_
#define EGO_GAZEBO_BRIDGE_EGO_MAVROS_BRIDGE_H_

#include "ego_gazebo_bridge/command_utils.h"
#include "ego_gazebo_bridge/control_authority.h"
#include "ego_gazebo_bridge/recovery_control.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <vector>

namespace ego_gazebo_bridge {

enum class BridgeState {
  kDryRun,
  kWaitFcu,
  kWaitInputs,
  kPrestream,
  kArmOffboard,
  kTakeoff,
  kHoverReady,
  kTrackEgo,
  kHold,
  kHomeHover,
  kLanding,
  kDone,
  kError,
};

const char* bridgeStateName(BridgeState state);

struct BridgeConfig {
  bool enable_control{false};
  bool require_sim_time{true};
  bool auto_track_on_command{false};
  bool require_start_permission{false};
  bool tower_yaw_override_enabled{false};
  double publish_rate{50.0};
  double prestream_duration{2.0};
  double request_interval{2.0};
  double takeoff_height{1.0};
  double return_height{1.0};
  double takeoff_tolerance{0.15};
  double hover_duration{1.0};
  double command_timeout{0.2};
  double timestamp_future_tolerance{0.02};
  double goal_timeout{2.0};
  double fcu_state_timeout{1.0};
  double extended_state_timeout{1.0};
  double mavros_pose_timeout{0.2};
  double planner_odom_timeout{0.2};
  double cloud_timeout{0.5};
  double land_after_loss{3.0};
  double wait_fcu_timeout{60.0};
  double wait_inputs_timeout{120.0};
  double arm_offboard_timeout{30.0};
  double takeoff_timeout{60.0};
  double landing_timeout{90.0};
  double max_position_rate{0.5};
  double max_yaw_rate{0.75};
  double max_velocity{0.5};
  double max_acceleration{1.0};
  double gravity_alignment_tolerance{0.017453292519943295};
  double tracking_error_limit{1.0};
  double tracking_error_duration{1.0};
  double alignment_position_tolerance{0.25};
  double alignment_yaw_tolerance{0.2617993878};
  double alignment_yaw_error_duration{1.0};
  double return_tolerance{0.25};
  double return_hold_duration{1.0};
  double tower_camera_yaw_offset{0.0};
  double forward_yaw_min_speed{0.05};
  CommandBounds bounds;

  std::string planning_frame{"camera_init"};
  std::string mavros_frame{"map"};
  std::string command_topic{"/planning/pos_cmd"};
  std::string planner_odom_topic{"/Odometry"};
  std::string cloud_topic{"/cloud_registered"};
  std::string mavros_state_topic{"/mavros/state"};
  std::string mavros_extended_state_topic{"/mavros/extended_state"};
  std::string mavros_pose_topic{"/mavros/local_position/pose"};
  std::string setpoint_topic{"/mavros/setpoint_raw/local"};
  std::string arming_service{"/mavros/cmd/arming"};
  std::string set_mode_service{"/mavros/set_mode"};
  std::string input_goal_topic{"/move_base_simple/goal"};
  std::string planner_goal_topic{"/planning/goal"};
  std::string planning_cancel_topic{"/planning/cancel"};
  std::string tower_center_topic{"/tower_mission/selected_tower_center"};
  std::string tower_yaw_mode_topic{"/tower_mission/face_tower"};
  std::string start_permission_topic{"/swarm/takeoff_permission"};
  std::vector<std::string> position_control_topics{
      "/mavros/setpoint_position/local",
      "/mavros/setpoint_position/global",
      "/mavros/setpoint_position/global_to_local"};
  std::vector<std::string> raw_local_control_topics{
      "/mavros/setpoint_raw/local"};
  std::vector<std::string> velocity_control_topics{
      "/mavros/setpoint_velocity/cmd_vel",
      "/mavros/setpoint_velocity/cmd_vel_unstamped"};
  std::vector<std::string> attitude_control_topics{
      "/mavros/setpoint_attitude/attitude",
      "/mavros/setpoint_attitude/cmd_vel",
      "/mavros/setpoint_raw/attitude"};
  std::vector<std::string> thrust_control_topics{
      "/mavros/setpoint_attitude/thrust"};
};

class EgoMavrosBridge {
 public:
  EgoMavrosBridge(ros::NodeHandle node_handle,
                  ros::NodeHandle private_node_handle);

 private:
  void loadConfig();
  bool validateConfig() const;
  void setupRosInterfaces();

  void fcuStateCallback(const mavros_msgs::State::ConstPtr& message);
  void extendedStateCallback(
      const mavros_msgs::ExtendedState::ConstPtr& message);
  void mavrosPoseCallback(
      const geometry_msgs::PoseStamped::ConstPtr& message);
  void plannerOdomCallback(const nav_msgs::Odometry::ConstPtr& message);
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message);
  void commandCallback(
      const quadrotor_msgs::PositionCommand::ConstPtr& message);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& message);
  void towerCenterCallback(
      const geometry_msgs::PointStamped::ConstPtr& message);
  void towerYawModeCallback(const std_msgs::Bool::ConstPtr& message);
  void startPermissionCallback(const std_msgs::Bool::ConstPtr& message);

  bool trackingService(std_srvs::SetBool::Request& request,
                       std_srvs::SetBool::Response& response);
  bool landService(std_srvs::Trigger::Request& request,
                   std_srvs::Trigger::Response& response);
  bool returnHomeService(std_srvs::Trigger::Request& request,
                         std_srvs::Trigger::Response& response);
  bool cancelCurrentTrajectoryService(std_srvs::Trigger::Request& request,
                                      std_srvs::Trigger::Response& response);
  bool resumeEgoService(std_srvs::Trigger::Request& request,
                        std_srvs::Trigger::Response& response);

  void controlTimerCallback(const ros::TimerEvent& event);
  void transitionTo(BridgeState next_state, const std::string& reason);
  void publishState();
  void publishSetpoint(const geometry_msgs::PoseStamped& desired,
                       const ros::Time& now);
  void publishTrajectorySetpoint(const ros::Time& now);
  void publishHold(const ros::Time& now);
  void latchHoldAtCurrentPose();

  bool baseInputsFresh(const ros::Time& now, std::string* reason) const;
  bool commandFresh(const ros::Time& now) const;
  bool plannerAlignmentErrors(double* position_error, double* yaw_error,
                              std::string* reason) const;
  bool plannerAlignmentValid(std::string* reason) const;
  bool flightPreflightValid(const ros::Time& now, std::string* reason);
  bool fullPreflightValid(const ros::Time& now, std::string* reason);
  bool captureHomeIfSafe(std::string* reason);
  bool transformToMavros(const geometry_msgs::PoseStamped& input,
                         geometry_msgs::PoseStamped* output,
                         std::string* reason) const;
  bool transformToPlanning(const geometry_msgs::PoseStamped& input,
                           geometry_msgs::PoseStamped* output,
                           std::string* reason) const;
  bool planningToMavrosTransform(
      const ros::Time& stamp, geometry_msgs::TransformStamped* transform,
      std::string* reason) const;
  bool hasControlConflict(std::string* detail) const;
  bool controlAuthorityValid(const ros::Time& now, std::string* detail);
  std::vector<MonitoredControlTopic> monitoredControlTopics() const;
  bool requestMode(const std::string& mode, const ros::Time& now);
  bool requestArm(const ros::Time& now);
  void startHold(const std::string& reason);
  bool publishReturnGoal(std::string* reason);
  void updateReturnProgress(const ros::Time& now);

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  BridgeConfig config_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber fcu_state_subscriber_;
  ros::Subscriber extended_state_subscriber_;
  ros::Subscriber mavros_pose_subscriber_;
  ros::Subscriber planner_odom_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber goal_subscriber_;
  ros::Subscriber tower_center_subscriber_;
  ros::Subscriber tower_yaw_mode_subscriber_;
  ros::Subscriber start_permission_subscriber_;
  ros::Publisher setpoint_publisher_;
  ros::Publisher debug_setpoint_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher tracking_error_publisher_;
  ros::Publisher goal_publisher_;
  ros::Publisher planning_cancel_publisher_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceServer tracking_service_;
  ros::ServiceServer land_service_;
  ros::ServiceServer return_home_service_;
  ros::ServiceServer cancel_current_trajectory_service_;
  ros::ServiceServer resume_ego_service_;
  ros::Timer control_timer_;

  BridgeState state_{BridgeState::kWaitFcu};
  mavros_msgs::State fcu_state_;
  mavros_msgs::ExtendedState extended_state_;
  geometry_msgs::PoseStamped mavros_pose_;
  nav_msgs::Odometry planner_odom_;
  geometry_msgs::PoseStamped home_pose_;
  geometry_msgs::PoseStamped hold_pose_;
  geometry_msgs::PoseStamped output_setpoint_;
  geometry_msgs::PoseStamped planner_target_mavros_;
  geometry_msgs::PoseStamped validated_goal_mavros_;
  geometry_msgs::PointStamped tower_center_;
  mavros_msgs::PositionTarget planner_raw_target_;

  bool have_fcu_state_{false};
  bool have_extended_state_{false};
  bool have_mavros_pose_{false};
  bool have_planner_odom_{false};
  bool have_cloud_{false};
  bool have_home_{false};
  bool have_output_setpoint_{false};
  bool have_planner_target_{false};
  bool have_valid_goal_{false};
  bool tracking_requested_{false};
  bool land_requested_{false};
  bool return_in_progress_{false};
  bool return_inside_tolerance_{false};
  bool takeoff_inside_tolerance_{false};
  bool tracking_error_active_{false};
  bool alignment_yaw_error_active_{false};
  bool have_tower_center_{false};
  bool tower_yaw_mode_{false};
  bool have_effective_yaw_{false};
  bool supervised_hold_{false};
  bool start_permission_{false};
  TrajectoryGate trajectory_gate_;

  ros::Time last_fcu_state_time_;
  ros::Time last_extended_state_time_;
  ros::Time last_mavros_pose_time_;
  ros::Time last_mavros_pose_stamp_;
  ros::Time last_planner_odom_time_;
  ros::Time last_planner_odom_stamp_;
  ros::Time last_cloud_time_;
  ros::Time last_cloud_stamp_;
  ros::Time last_command_time_;
  ros::Time last_command_stamp_;
  ros::Time state_entered_time_;
  ros::Time last_mode_request_time_;
  ros::Time last_arm_request_time_;
  ros::Time last_authority_check_time_;
  ros::Time return_inside_since_;
  ros::Time takeoff_inside_since_;
  ros::Time tracking_error_since_;
  ros::Time alignment_yaw_error_since_;
  ros::Time last_effective_yaw_time_;
  double maximum_tracking_error_{0.0};
  double effective_yaw_{0.0};
  bool cached_control_conflict_{false};
  std::string cached_control_conflict_detail_;
  std::string hold_reason_;
};

}  // namespace ego_gazebo_bridge

#endif  // EGO_GAZEBO_BRIDGE_EGO_MAVROS_BRIDGE_H_
