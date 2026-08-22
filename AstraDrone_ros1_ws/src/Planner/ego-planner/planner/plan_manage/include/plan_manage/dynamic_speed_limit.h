#ifndef EGO_PLANNER_DYNAMIC_SPEED_LIMIT_H_
#define EGO_PLANNER_DYNAMIC_SPEED_LIMIT_H_

#include <cmath>

namespace ego_planner
{

class DynamicSpeedLimitGate
{
public:
  DynamicSpeedLimitGate() = default;

  DynamicSpeedLimitGate(double minimum, double maximum)
      : minimum_(minimum), maximum_(maximum)
  {
  }

  bool validConfiguration() const
  {
    return std::isfinite(minimum_) && std::isfinite(maximum_) &&
           minimum_ > 0.0 && maximum_ >= minimum_;
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

  bool requiresForceReplan(double previous, double requested) const
  {
    if (!accepts(previous) || !accepts(requested))
      return false;
    return requested < previous - kNoForceReplanDecreaseMagnitudeMps ||
           requested > previous + kNoForceReplanIncreaseMps;
  }

  double minimum() const { return minimum_; }
  double maximum() const { return maximum_; }

private:
  static constexpr double kNoForceReplanDecreaseMagnitudeMps = 0.3;
  static constexpr double kNoForceReplanIncreaseMps = 0.5;
  double minimum_{0.1};
  double maximum_{1.0};
};

}  // namespace ego_planner

#endif  // EGO_PLANNER_DYNAMIC_SPEED_LIMIT_H_
