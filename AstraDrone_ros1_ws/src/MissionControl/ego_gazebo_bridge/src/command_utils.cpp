#include "ego_gazebo_bridge/command_utils.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_gazebo_bridge {
namespace {

constexpr double kQuaternionNormEpsilon = 1e-9;

bool finite(double value) {
  return std::isfinite(value);
}

geometry_msgs::Vector3 rotateVector(
    const geometry_msgs::Vector3& input,
    const geometry_msgs::Quaternion& rotation) {
  tf2::Quaternion quaternion;
  tf2::fromMsg(rotation, quaternion);
  const tf2::Vector3 rotated =
      tf2::quatRotate(quaternion,
                      tf2::Vector3(input.x, input.y, input.z));
  geometry_msgs::Vector3 output;
  output.x = rotated.x();
  output.y = rotated.y();
  output.z = rotated.z();
  return output;
}

void limitVector(double maximum_norm, geometry_msgs::Vector3* vector) {
  const double norm =
      std::sqrt(vector->x * vector->x + vector->y * vector->y +
                vector->z * vector->z);
  if (norm > maximum_norm && norm > 0.0) {
    const double scale = maximum_norm / norm;
    vector->x *= scale;
    vector->y *= scale;
    vector->z *= scale;
  }
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

bool isTimestampUsable(const ros::Time& now, const ros::Time& stamp,
                       double timeout, double future_tolerance) {
  if (now.isZero() || stamp.isZero() || !finite(timeout) || timeout <= 0.0 ||
      !finite(future_tolerance) || future_tolerance < 0.0) {
    return false;
  }
  const double age = (now - stamp).toSec();
  return age >= -future_tolerance && age <= timeout;
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

bool applyPointFacingYaw(const geometry_msgs::Point& target,
                         double camera_yaw_offset,
                         quadrotor_msgs::PositionCommand* command,
                         std::string* reason) {
  if (command == nullptr || reason == nullptr ||
      !finite(target.x) || !finite(target.y) ||
      !finite(camera_yaw_offset) || !isFinitePositionCommand(*command)) {
    if (reason != nullptr) {
      *reason = "invalid tower-facing yaw input";
    }
    return false;
  }

  const double dx = target.x - command->position.x;
  const double dy = target.y - command->position.y;
  const double radius_squared = dx * dx + dy * dy;
  constexpr double kMinimumRadiusSquared = 1e-6;
  if (!finite(radius_squared) || radius_squared < kMinimumRadiusSquared) {
    *reason = "tower-facing yaw is undefined at the tower center";
    return false;
  }

  const double bearing = std::atan2(dy, dx);
  command->yaw =
      std::atan2(std::sin(bearing - camera_yaw_offset),
                 std::cos(bearing - camera_yaw_offset));
  command->yaw_dot =
      (dy * command->velocity.x - dx * command->velocity.y) /
      radius_squared;
  reason->clear();
  return true;
}

bool applyVelocityFacingYaw(double minimum_horizontal_speed,
                            quadrotor_msgs::PositionCommand* command,
                            std::string* reason) {
  if (command == nullptr || reason == nullptr ||
      !finite(minimum_horizontal_speed) || minimum_horizontal_speed <= 0.0 ||
      !isFinitePositionCommand(*command)) {
    if (reason != nullptr) {
      *reason = "invalid velocity-facing yaw input";
    }
    return false;
  }

  const double velocity_x = command->velocity.x;
  const double velocity_y = command->velocity.y;
  const double speed_squared =
      velocity_x * velocity_x + velocity_y * velocity_y;
  const double minimum_speed_squared =
      minimum_horizontal_speed * minimum_horizontal_speed;
  if (speed_squared < minimum_speed_squared) {
    // Horizontal direction is undefined while nearly stationary. Preserve the
    // last trajectory yaw instead of amplifying velocity/acceleration noise.
    command->yaw_dot = 0.0;
    reason->clear();
    return true;
  }

  command->yaw = std::atan2(velocity_y, velocity_x);
  command->yaw_dot =
      (velocity_x * command->acceleration.y -
       velocity_y * command->acceleration.x) /
      speed_squared;
  reason->clear();
  return true;
}

bool limitYawCommand(double current_yaw, double desired_yaw,
                     double maximum_yaw_rate, double dt,
                     double* limited_yaw, double* limited_yaw_rate) {
  if (limited_yaw == nullptr || limited_yaw_rate == nullptr ||
      !finite(current_yaw) || !finite(desired_yaw) ||
      !finite(maximum_yaw_rate) || maximum_yaw_rate <= 0.0 ||
      !finite(dt) || dt <= 0.0) {
    return false;
  }
  const double desired_change = angularDistance(current_yaw, desired_yaw);
  const double maximum_change = maximum_yaw_rate * dt;
  const double limited_change =
      std::max(-maximum_change, std::min(maximum_change, desired_change));
  *limited_yaw =
      std::atan2(std::sin(current_yaw + limited_change),
                 std::cos(current_yaw + limited_change));
  *limited_yaw_rate = limited_change / dt;
  return true;
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

bool commandToRawTarget(
    const quadrotor_msgs::PositionCommand& command,
    const geometry_msgs::TransformStamped& planning_to_mavros,
    const RawCommandLimits& limits, const ros::Time& stamp,
    mavros_msgs::PositionTarget* target, std::string* reason) {
  if (target == nullptr || reason == nullptr) {
    return false;
  }
  if (!isFinitePositionCommand(command) || stamp.isZero() ||
      !finite(limits.max_velocity) || limits.max_velocity <= 0.0 ||
      !finite(limits.max_acceleration) || limits.max_acceleration <= 0.0 ||
      !finite(limits.max_yaw_rate) || limits.max_yaw_rate <= 0.0 ||
      !finite(limits.gravity_alignment_tolerance) ||
      limits.gravity_alignment_tolerance < 0.0) {
    *reason = "invalid command, timestamp or raw-command limit";
    return false;
  }

  tf2::Quaternion frame_rotation;
  tf2::fromMsg(planning_to_mavros.transform.rotation, frame_rotation);
  if (frame_rotation.length2() <= kQuaternionNormEpsilon) {
    *reason = "planning-to-MAVROS rotation is invalid";
    return false;
  }
  frame_rotation.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double frame_yaw = 0.0;
  tf2::Matrix3x3(frame_rotation).getRPY(roll, pitch, frame_yaw);
  if (std::abs(roll) > limits.gravity_alignment_tolerance ||
      std::abs(pitch) > limits.gravity_alignment_tolerance) {
    *reason =
        "planning and MAVROS world frames are not gravity-aligned";
    return false;
  }

  const geometry_msgs::PoseStamped planning_pose =
      commandToPose(command, planning_to_mavros.child_frame_id);
  const geometry_msgs::PoseStamped mavros_pose =
      transformPose(planning_pose, planning_to_mavros);
  if (!isFinitePose(mavros_pose)) {
    *reason = "transformed command pose is invalid";
    return false;
  }

  mavros_msgs::PositionTarget output;
  output.header.stamp = stamp;
  output.header.frame_id = planning_to_mavros.header.frame_id;
  output.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  // ROS fields are ENU. MAVROS converts them to MAVLink/PX4 NED. All
  // position, velocity, acceleration, yaw and yaw-rate fields are active.
  output.type_mask = 0;
  output.position = mavros_pose.pose.position;
  output.velocity = rotateVector(command.velocity,
                                 planning_to_mavros.transform.rotation);
  output.acceleration_or_force =
      rotateVector(command.acceleration,
                   planning_to_mavros.transform.rotation);
  limitVector(limits.max_velocity, &output.velocity);
  limitVector(limits.max_acceleration, &output.acceleration_or_force);
  output.yaw = static_cast<float>(
      std::atan2(std::sin(command.yaw + frame_yaw),
                 std::cos(command.yaw + frame_yaw)));
  output.yaw_rate = static_cast<float>(
      std::max(-limits.max_yaw_rate,
               std::min(limits.max_yaw_rate, command.yaw_dot)));

  *target = output;
  reason->clear();
  return true;
}

mavros_msgs::PositionTarget poseToRawTarget(
    const geometry_msgs::PoseStamped& pose, const ros::Time& stamp) {
  mavros_msgs::PositionTarget target;
  target.header.stamp = stamp;
  target.header.frame_id = pose.header.frame_id;
  target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  target.type_mask = 0;
  target.position = pose.pose.position;
  target.yaw = static_cast<float>(yawFromQuaternion(pose.pose.orientation));
  target.yaw_rate = 0.0F;
  return target;
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
