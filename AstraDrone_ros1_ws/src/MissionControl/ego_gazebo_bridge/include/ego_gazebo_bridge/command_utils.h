#ifndef EGO_GAZEBO_BRIDGE_COMMAND_UTILS_H_
#define EGO_GAZEBO_BRIDGE_COMMAND_UTILS_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/time.h>

#include <string>

namespace ego_gazebo_bridge {

struct CommandBounds {
  double min_relative_height{0.3};
  double max_relative_height{2.0};
  double max_horizontal_radius{10.0};
};

struct RawCommandLimits {
  double max_velocity{0.5};
  double max_acceleration{1.0};
  double max_yaw_rate{0.75};
  double gravity_alignment_tolerance{0.017453292519943295};
};

bool isFinitePositionCommand(const quadrotor_msgs::PositionCommand& command);
bool isFinitePose(const geometry_msgs::PoseStamped& pose);
bool isTimestampUsable(const ros::Time& now, const ros::Time& stamp,
                       double timeout, double future_tolerance);

geometry_msgs::Quaternion quaternionFromYaw(double yaw);
double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
double angularDistance(double from, double to);
double positionDistance(const geometry_msgs::PoseStamped& lhs,
                        const geometry_msgs::PoseStamped& rhs);
double horizontalDistance(const geometry_msgs::PoseStamped& lhs,
                          const geometry_msgs::PoseStamped& rhs);

geometry_msgs::PoseStamped commandToPose(
    const quadrotor_msgs::PositionCommand& command,
    const std::string& fallback_frame);

geometry_msgs::PoseStamped transformPose(
    const geometry_msgs::PoseStamped& input,
    const geometry_msgs::TransformStamped& transform);

bool commandToRawTarget(
    const quadrotor_msgs::PositionCommand& command,
    const geometry_msgs::TransformStamped& planning_to_mavros,
    const RawCommandLimits& limits, const ros::Time& stamp,
    mavros_msgs::PositionTarget* target, std::string* reason);

mavros_msgs::PositionTarget poseToRawTarget(
    const geometry_msgs::PoseStamped& pose, const ros::Time& stamp);

bool isWithinBounds(const geometry_msgs::PoseStamped& target,
                    const geometry_msgs::PoseStamped& home,
                    const CommandBounds& bounds,
                    std::string* reason);

geometry_msgs::PoseStamped stepToward(
    const geometry_msgs::PoseStamped& current,
    const geometry_msgs::PoseStamped& target,
    double max_position_step,
    double max_yaw_step);

}  // namespace ego_gazebo_bridge

#endif  // EGO_GAZEBO_BRIDGE_COMMAND_UTILS_H_
