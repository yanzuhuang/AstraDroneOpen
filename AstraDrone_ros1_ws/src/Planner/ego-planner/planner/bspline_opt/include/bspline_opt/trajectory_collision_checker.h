#ifndef BSPLINE_OPT_TRAJECTORY_COLLISION_CHECKER_H
#define BSPLINE_OPT_TRAJECTORY_COLLISION_CHECKER_H

#include <algorithm>
#include <cmath>

#include "bspline_opt/uniform_bspline.h"
#include "plan_env/grid_map.h"

namespace ego_planner
{

struct TrajectoryCollisionResult
{
  bool collision{false};
  bool outside_map{false};
  double first_collision_parameter{0.0};
  std::size_t swept_voxels{0};
  GridMap::BlockReason reason{GridMap::BlockReason::NONE};
};

inline bool isUniformBsplineSweptCollisionFree(
    UniformBspline& trajectory, GridMap& map,
    double start_parameter, double end_parameter,
    TrajectoryCollisionResult* result = nullptr,
    double maximum_parameter_step = 0.01)
{
  TrajectoryCollisionResult local;
  TrajectoryCollisionResult& out = result ? *result : local;
  out = TrajectoryCollisionResult();
  if (!std::isfinite(start_parameter) || !std::isfinite(end_parameter) ||
      !std::isfinite(maximum_parameter_step) || maximum_parameter_step <= 0.0 ||
      end_parameter < start_parameter) {
    out.collision = true;
    out.outside_map = true;
    out.reason = GridMap::BlockReason::OUT_OF_MAP;
    out.first_collision_parameter = start_parameter;
    return false;
  }

  Eigen::Vector3d previous = trajectory.evaluateDeBoorT(start_parameter);
  const bool first_blocked = map.getPlanningOccupancy(previous);
  if (first_blocked) {
    out.collision = true;
    out.outside_map = !map.isInMap(previous);
    out.first_collision_parameter = start_parameter;
    out.reason = !map.isInMap(previous)
                     ? GridMap::BlockReason::OUT_OF_MAP
                     : GridMap::BlockReason::OCCUPIED;
    return false;
  }
  if (end_parameter == start_parameter)
    return true;

  double parameter = start_parameter;
  while (parameter < end_parameter) {
    const double next_parameter =
        std::min(end_parameter, parameter + maximum_parameter_step);
    const Eigen::Vector3d current =
        trajectory.evaluateDeBoorT(next_parameter);
    GridMap::SweptCollisionResult swept;
    if (!map.isSweptSegmentFree(previous, current, &swept)) {
      out.collision = true;
      out.outside_map = swept.outside_map;
      out.first_collision_parameter = next_parameter;
      out.swept_voxels += swept.visited_voxels;
      out.reason = swept.reason;
      return false;
    }
    out.swept_voxels += swept.visited_voxels;
    previous = current;
    parameter = next_parameter;
  }
  return true;
}

}  // namespace ego_planner

#endif
