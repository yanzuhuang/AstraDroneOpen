#include "astra_tower_mission/tower_route.h"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace astra_tower_mission {

double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool parseDirection(const std::string& text, OrbitDirection* direction) {
  std::string normalized = text;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  if (normalized == "counter_clockwise" || normalized == "ccw") {
    if (direction != nullptr) {
      *direction = OrbitDirection::kCounterClockwise;
    }
    return true;
  }
  if (normalized == "clockwise" || normalized == "cw") {
    if (direction != nullptr) {
      *direction = OrbitDirection::kClockwise;
    }
    return true;
  }
  return false;
}

const char* directionName(OrbitDirection direction) {
  return direction == OrbitDirection::kCounterClockwise
             ? "counter_clockwise"
             : "clockwise";
}

bool validateRouteConfig(const RouteConfig& config, std::string* reason) {
  const auto fail = [reason](const std::string& message) {
    if (reason != nullptr) {
      *reason = message;
    }
    return false;
  };
  const double values[] = {
      config.center_x, config.center_y, config.radius,
      config.height, config.start_angle_rad, config.camera_yaw_offset_rad,
      config.tower_collision_radius, config.minimum_safety_distance,
      config.minimum_height, config.maximum_height};
  for (double value : values) {
    if (!std::isfinite(value)) {
      return fail("all route values must be finite");
    }
  }
  if (config.tower_name.empty()) {
    return fail("tower name must not be empty");
  }
  if (config.frame_id.empty()) {
    return fail("tower frame_id must not be empty");
  }
  if (config.waypoint_count < 1 || config.waypoint_count > 360) {
    return fail("waypoint_count must be in [1, 360]");
  }
  if (config.radius <= 0.0 || config.tower_collision_radius < 0.0 ||
      config.minimum_safety_distance < 0.0) {
    return fail("radius must be positive and safety radii non-negative");
  }
  if (config.minimum_height >= config.maximum_height) {
    return fail("minimum_height must be lower than maximum_height");
  }
  if (config.height < config.minimum_height ||
      config.height > config.maximum_height) {
    return fail("mission height is outside configured height bounds");
  }
  const double radial_clearance =
      config.radius - config.tower_collision_radius;
  if (radial_clearance < config.minimum_safety_distance) {
    return fail("radius does not preserve the minimum tower clearance");
  }
  if (reason != nullptr) {
    reason->clear();
  }
  return true;
}

std::vector<TowerWaypoint> generateTowerWaypoints(const RouteConfig& config) {
  std::string reason;
  if (!validateRouteConfig(config, &reason)) {
    return {};
  }

  std::vector<TowerWaypoint> waypoints;
  waypoints.reserve(static_cast<std::size_t>(config.waypoint_count));
  const double angular_step = 2.0 * kPi / config.waypoint_count;
  for (int index = 0; index < config.waypoint_count; ++index) {
    waypoints.push_back(towerWaypointAtProgress(config, index * angular_step));
  }
  return waypoints;
}

TowerWaypoint towerWaypointAtProgress(const RouteConfig& config,
                                      double angular_progress) {
  const double direction_sign =
      config.direction == OrbitDirection::kCounterClockwise ? 1.0 : -1.0;
  TowerWaypoint waypoint;
  waypoint.theta = normalizeAngle(config.start_angle_rad +
                                  direction_sign * angular_progress);
  waypoint.x = config.center_x + config.radius * std::cos(waypoint.theta);
  waypoint.y = config.center_y + config.radius * std::sin(waypoint.theta);
  waypoint.z = config.height;
  const double camera_bearing =
      std::atan2(config.center_y - waypoint.y,
                 config.center_x - waypoint.x);
  waypoint.yaw = normalizeAngle(camera_bearing -
                                config.camera_yaw_offset_rad);
  return waypoint;
}

}  // namespace astra_tower_mission
