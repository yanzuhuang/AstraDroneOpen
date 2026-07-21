#include "astra_tower_mission/motion_limiter.h"

#include <algorithm>
#include <cmath>

#include "astra_tower_mission/tower_route.h"

namespace astra_tower_mission {

namespace {

double clampMagnitude(double value, double maximum_magnitude) {
  return std::max(-maximum_magnitude,
                  std::min(maximum_magnitude, value));
}

}  // namespace

MotionReference stepMotionReference(const MotionReference& current,
                                    const MotionTarget& target,
                                    double dt,
                                    double maximum_speed,
                                    double maximum_acceleration,
                                    double maximum_yaw_rate) {
  if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(maximum_speed) ||
      maximum_speed <= 0.0 || !std::isfinite(maximum_acceleration) ||
      maximum_acceleration <= 0.0 || !std::isfinite(maximum_yaw_rate) ||
      maximum_yaw_rate <= 0.0) {
    return current;
  }

  MotionReference result = current;
  const double dx = target.x - current.x;
  const double dy = target.y - current.y;
  const double dz = target.z - current.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  double desired_vx = 0.0;
  double desired_vy = 0.0;
  double desired_vz = 0.0;
  if (distance > 1e-9) {
    const double braking_speed = std::sqrt(2.0 * maximum_acceleration * distance);
    const double desired_speed = std::min(maximum_speed, braking_speed);
    desired_vx = desired_speed * dx / distance;
    desired_vy = desired_speed * dy / distance;
    desired_vz = desired_speed * dz / distance;
  }

  double delta_vx = desired_vx - current.vx;
  double delta_vy = desired_vy - current.vy;
  double delta_vz = desired_vz - current.vz;
  const double delta_speed =
      std::sqrt(delta_vx * delta_vx + delta_vy * delta_vy + delta_vz * delta_vz);
  const double maximum_delta_speed = maximum_acceleration * dt;
  if (delta_speed > maximum_delta_speed && delta_speed > 1e-12) {
    const double scale = maximum_delta_speed / delta_speed;
    delta_vx *= scale;
    delta_vy *= scale;
    delta_vz *= scale;
  }
  result.vx += delta_vx;
  result.vy += delta_vy;
  result.vz += delta_vz;

  const double step_x = result.vx * dt;
  const double step_y = result.vy * dt;
  const double step_z = result.vz * dt;
  const double step_distance =
      std::sqrt(step_x * step_x + step_y * step_y + step_z * step_z);
  if (step_distance >= distance && distance > 0.0) {
    result.x = target.x;
    result.y = target.y;
    result.z = target.z;
    result.vx = 0.0;
    result.vy = 0.0;
    result.vz = 0.0;
  } else {
    result.x += step_x;
    result.y += step_y;
    result.z += step_z;
  }

  const double yaw_error = normalizeAngle(target.yaw - current.yaw);
  const double yaw_step =
      clampMagnitude(yaw_error, maximum_yaw_rate * dt);
  result.yaw = normalizeAngle(current.yaw + yaw_step);
  return result;
}

CircularMotionReference initializeCircularMotionReference(
    const RouteConfig& config) {
  CircularMotionReference reference;
  const TowerWaypoint point = towerWaypointAtProgress(config, 0.0);
  reference.motion.x = point.x;
  reference.motion.y = point.y;
  reference.motion.z = point.z;
  reference.motion.yaw = point.yaw;
  return reference;
}

CircularMotionReference stepCircularMotionReference(
    const CircularMotionReference& current,
    const RouteConfig& config,
    double dt,
    double maximum_speed,
    double maximum_acceleration) {
  if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(maximum_speed) ||
      maximum_speed <= 0.0 || !std::isfinite(maximum_acceleration) ||
      maximum_acceleration <= 0.0 || !std::isfinite(config.radius) ||
      config.radius <= 0.0 || current.complete) {
    return current;
  }

  CircularMotionReference result = current;
  result.speed = std::min(maximum_speed,
                          current.speed + maximum_acceleration * dt);
  const double remaining_angle =
      std::max(0.0, 2.0 * kPi - current.angular_progress);
  const double angular_step =
      std::min(remaining_angle, result.speed * dt / config.radius);
  result.angular_progress = current.angular_progress + angular_step;
  result.complete = result.angular_progress >= 2.0 * kPi - 1e-12;
  if (result.complete) {
    result.angular_progress = 2.0 * kPi;
  }

  const TowerWaypoint point =
      towerWaypointAtProgress(config, result.angular_progress);
  const double direction_sign =
      config.direction == OrbitDirection::kCounterClockwise ? 1.0 : -1.0;
  result.motion.x = point.x;
  result.motion.y = point.y;
  result.motion.z = point.z;
  result.motion.vx = -direction_sign * result.speed * std::sin(point.theta);
  result.motion.vy = direction_sign * result.speed * std::cos(point.theta);
  result.motion.vz = 0.0;
  // The orbit pose is sampled from the circle every cycle. Its desired yaw is
  // therefore recomputed from that same sample instead of interpolated between
  // checkpoint headings.
  result.motion.yaw = point.yaw;
  return result;
}

}  // namespace astra_tower_mission
