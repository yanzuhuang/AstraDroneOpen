#ifndef ASTRA_TOWER_MISSION_TOWER_ROUTE_H_
#define ASTRA_TOWER_MISSION_TOWER_ROUTE_H_

#include <string>
#include <vector>

namespace astra_tower_mission {

constexpr double kPi = 3.14159265358979323846;

enum class OrbitDirection {
  kCounterClockwise,
  kClockwise,
};

struct RouteConfig {
  std::string tower_name;
  std::string frame_id;
  double center_x{0.0};
  double center_y{0.0};
  double radius{0.0};
  double height{0.0};
  int waypoint_count{0};
  double start_angle_rad{0.0};
  OrbitDirection direction{OrbitDirection::kCounterClockwise};
  double camera_yaw_offset_rad{0.0};
  double tower_collision_radius{0.0};
  double minimum_safety_distance{0.0};
  double minimum_height{0.0};
  double maximum_height{0.0};
};

struct TowerWaypoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  double theta{0.0};
};

double normalizeAngle(double angle);
bool parseDirection(const std::string& text, OrbitDirection* direction);
const char* directionName(OrbitDirection direction);
bool validateRouteConfig(const RouteConfig& config, std::string* reason);
TowerWaypoint towerWaypointAtProgress(const RouteConfig& config,
                                      double angular_progress);
std::vector<TowerWaypoint> generateTowerWaypoints(const RouteConfig& config);

}  // namespace astra_tower_mission

#endif  // ASTRA_TOWER_MISSION_TOWER_ROUTE_H_
