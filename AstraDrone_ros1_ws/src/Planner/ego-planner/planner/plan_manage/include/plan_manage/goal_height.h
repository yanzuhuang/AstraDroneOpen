#ifndef EGO_PLANNER_GOAL_HEIGHT_H_
#define EGO_PLANNER_GOAL_HEIGHT_H_

#include <cmath>

namespace ego_planner
{

inline bool resolveManualGoalHeight(double requested_height,
                                    double fallback_height,
                                    bool use_goal_height,
                                    double *resolved_height)
{
  if (resolved_height == nullptr)
    return false;

  const double selected_height =
      use_goal_height ? requested_height : fallback_height;
  if (!std::isfinite(selected_height) || selected_height <= 0.0)
    return false;

  *resolved_height = selected_height;
  return true;
}

}  // namespace ego_planner

#endif  // EGO_PLANNER_GOAL_HEIGHT_H_
