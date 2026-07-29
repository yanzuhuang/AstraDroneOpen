#ifndef ASTRA_TOWER_MISSION_LOW_ALTITUDE_PATH_PLANNER_H_
#define ASTRA_TOWER_MISSION_LOW_ALTITUDE_PATH_PLANNER_H_

#include <geometry_msgs/Point.h>

#include <cstddef>
#include <string>
#include <vector>

namespace astra_tower_mission {

// Cost terms are evaluated only after the hard occupied/tower keep-out mask
// has rejected unsafe cells. They rank safe alternatives; no weight can buy
// its way through a blocked cell.
struct LevelPathCostWeights {
  double path_length{1.0};
  double goal_deviation{0.03};
  double tower_distance{0.08};
  double turn{0.35};
  double obstacle_clearance{0.05};
};

// A live-map, fixed-height path used to feed bounded local goals to EGO.
// occupied_points are expected to come from EGO's already-inflated occupancy
// topic. additional_clearance is therefore an operational margin only and
// must not include the vehicle radius a second time.
struct LevelPathConfig {
  double altitude{3.0};
  double vertical_half_extent{0.2};
  double additional_clearance{0.6};
  double resolution{0.4};
  double boundary_margin{6.0};
  double maximum_segment_length{5.0};
  std::size_t maximum_cell_count{250000U};

  bool use_tower_constraint{false};
  double tower_x{0.0};
  double tower_y{0.0};
  double tower_keep_out_radius{0.0};
  double preferred_tower_radius{0.0};
  LevelPathCostWeights cost;
};

struct LevelPathResult {
  bool reachable{false};
  std::string reason;
  std::vector<geometry_msgs::Point> points;
  std::size_t occupied_cell_count{0U};
  double path_length{0.0};
  double minimum_clearance{0.0};
  double score{0.0};
};

LevelPathResult planLevelPath(
    const geometry_msgs::Point& start,
    const geometry_msgs::Point& goal,
    const std::vector<geometry_msgs::Point>& occupied_points,
    const LevelPathConfig& config);

}  // namespace astra_tower_mission

#endif  // ASTRA_TOWER_MISSION_LOW_ALTITUDE_PATH_PLANNER_H_
