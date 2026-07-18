#ifndef EGO_GAZEBO_BRIDGE_COMMAND_UTILS_H_
#define EGO_GAZEBO_BRIDGE_COMMAND_UTILS_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <string>

namespace ego_gazebo_bridge {

struct CommandBounds {
  double min_relative_height{0.3};
  double max_relative_height{2.0};
  double max_horizontal_radius{10.0};
};

bool isFinitePositionCommand(const quadrotor_msgs::PositionCommand& command);
bool isFinitePose(const geometry_msgs::PoseStamped& pose);

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
