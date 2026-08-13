#ifndef EGO_PLANNER_DYNAMIC_SPEED_LIMIT_H_
#define EGO_PLANNER_DYNAMIC_SPEED_LIMIT_H_

#include <cmath>

namespace ego_planner
{

class DynamicSpeedLimitGate
{
public:
  DynamicSpeedLimitGate() = default;

  DynamicSpeedLimitGate(double minimum, double maximum,
                        double replan_delta)
      : minimum_(minimum), maximum_(maximum), replan_delta_(replan_delta)
  {
  }

  bool validConfiguration() const
  {
    return std::isfinite(minimum_) && std::isfinite(maximum_) &&
           std::isfinite(replan_delta_) && minimum_ > 0.0 &&
           maximum_ >= minimum_ && replan_delta_ > 0.0;
  }

  bool accepts(double requested) const
  {
    return validConfiguration() && std::isfinite(requested) &&
           requested >= minimum_ && requested <= maximum_;
  }

  bool changed(double current, double requested,
               double epsilon = 1.0e-6) const
  {
    return accepts(requested) && std::isfinite(current) &&
           std::abs(requested - current) > epsilon;
  }

  bool requiresReplan(double limit_used_for_last_replan,
                      double requested) const
  {
    return accepts(requested) && std::isfinite(limit_used_for_last_replan) &&
           std::abs(requested - limit_used_for_last_replan) >= replan_delta_;
  }

  double minimum() const { return minimum_; }
  double maximum() const { return maximum_; }

private:
  double minimum_{0.1};
  double maximum_{1.0};
  double replan_delta_{0.1};
};

}  // namespace ego_planner

#endif  // EGO_PLANNER_DYNAMIC_SPEED_LIMIT_H_
