#include "ego_gazebo_bridge/command_utils.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_gazebo_bridge {
namespace {

constexpr double kQuaternionNormEpsilon = 1e-9;

bool finite(double value) {
  return std::isfinite(value);
}

}  // namespace

bool isFinitePositionCommand(
    const quadrotor_msgs::PositionCommand& command) {
  return finite(command.position.x) && finite(command.position.y) &&
         finite(command.position.z) && finite(command.velocity.x) &&
         finite(command.velocity.y) && finite(command.velocity.z) &&
         finite(command.acceleration.x) && finite(command.acceleration.y) &&
         finite(command.acceleration.z) && finite(command.yaw) &&
         finite(command.yaw_dot);
}

bool isFinitePose(const geometry_msgs::PoseStamped& pose) {
  const auto& position = pose.pose.position;
  const auto& orientation = pose.pose.orientation;
  if (!finite(position.x) || !finite(position.y) || !finite(position.z) ||
      !finite(orientation.x) || !finite(orientation.y) ||
      !finite(orientation.z) || !finite(orientation.w)) {
    return false;
  }

  const double norm_squared = orientation.x * orientation.x +
                              orientation.y * orientation.y +
                              orientation.z * orientation.z +
                              orientation.w * orientation.w;
  return norm_squared > kQuaternionNormEpsilon;
}

geometry_msgs::Quaternion quaternionFromYaw(double yaw) {
  geometry_msgs::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
  return std::atan2(
      2.0 * (quaternion.w * quaternion.z +
             quaternion.x * quaternion.y),
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z));
}

double angularDistance(double from, double to) {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

double positionDistance(const geometry_msgs::PoseStamped& lhs,
                        const geometry_msgs::PoseStamped& rhs) {
  const double dx = rhs.pose.position.x - lhs.pose.position.x;
  const double dy = rhs.pose.position.y - lhs.pose.position.y;
  const double dz = rhs.pose.position.z - lhs.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double horizontalDistance(const geometry_msgs::PoseStamped& lhs,
                          const geometry_msgs::PoseStamped& rhs) {
  const double dx = rhs.pose.position.x - lhs.pose.position.x;
  const double dy = rhs.pose.position.y - lhs.pose.position.y;
  return std::hypot(dx, dy);
}

geometry_msgs::PoseStamped commandToPose(
    const quadrotor_msgs::PositionCommand& command,
    const std::string& fallback_frame) {
  geometry_msgs::PoseStamped pose;
  pose.header = command.header;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = fallback_frame;
  }
  pose.pose.position = command.position;
  pose.pose.orientation = quaternionFromYaw(command.yaw);
  return pose;
}

geometry_msgs::PoseStamped transformPose(
    const geometry_msgs::PoseStamped& input,
    const geometry_msgs::TransformStamped& transform) {
  geometry_msgs::PoseStamped output;
  tf2::doTransform(input, output, transform);
  return output;
}

bool isWithinBounds(const geometry_msgs::PoseStamped& target,
                    const geometry_msgs::PoseStamped& home,
                    const CommandBounds& bounds,
                    std::string* reason) {
  if (!isFinitePose(target) || !isFinitePose(home)) {
    if (reason != nullptr) {
      *reason = "target or home pose contains NaN/Inf or an invalid quaternion";
    }
    return false;
  }

  const double relative_height =
      target.pose.position.z - home.pose.position.z;
  if (relative_height < bounds.min_relative_height ||
      relative_height > bounds.max_relative_height) {
    if (reason != nullptr) {
      *reason = "relative height is outside the configured flight envelope";
    }
    return false;
  }

  if (horizontalDistance(target, home) > bounds.max_horizontal_radius) {
    if (reason != nullptr) {
      *reason = "horizontal distance from home exceeds the configured radius";
    }
    return false;
  }

  return true;
}

geometry_msgs::PoseStamped stepToward(
    const geometry_msgs::PoseStamped& current,
    const geometry_msgs::PoseStamped& target,
    double max_position_step,
    double max_yaw_step) {
  geometry_msgs::PoseStamped result = current;
  result.header = target.header;

  const double distance = positionDistance(current, target);
  const double scale = distance > max_position_step && distance > 0.0
                           ? max_position_step / distance
                           : 1.0;
  result.pose.position.x +=
      (target.pose.position.x - current.pose.position.x) * scale;
  result.pose.position.y +=
      (target.pose.position.y - current.pose.position.y) * scale;
  result.pose.position.z +=
      (target.pose.position.z - current.pose.position.z) * scale;

  const double current_yaw = yawFromQuaternion(current.pose.orientation);
  const double target_yaw = yawFromQuaternion(target.pose.orientation);
  const double yaw_error = angularDistance(current_yaw, target_yaw);
  const double yaw_step =
      std::max(-max_yaw_step, std::min(max_yaw_step, yaw_error));
  result.pose.orientation = quaternionFromYaw(current_yaw + yaw_step);
  return result;
}

}  // namespace ego_gazebo_bridge
