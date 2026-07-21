#ifndef ASTRA_TOWER_MISSION_MOTION_LIMITER_H_
#define ASTRA_TOWER_MISSION_MOTION_LIMITER_H_

#include "astra_tower_mission/tower_route.h"

namespace astra_tower_mission {

struct MotionReference {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double yaw{0.0};
};

struct MotionTarget {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct CircularMotionReference {
  MotionReference motion;
  double angular_progress{0.0};
  double speed{0.0};
  bool complete{false};
};

MotionReference stepMotionReference(const MotionReference& current,
                                    const MotionTarget& target,
                                    double dt,
                                    double maximum_speed,
                                    double maximum_acceleration,
                                    double maximum_yaw_rate);

CircularMotionReference initializeCircularMotionReference(
    const RouteConfig& config);

CircularMotionReference stepCircularMotionReference(
    const CircularMotionReference& current,
    const RouteConfig& config,
    double dt,
    double maximum_speed,
    double maximum_acceleration);

}  // namespace astra_tower_mission

#endif  // ASTRA_TOWER_MISSION_MOTION_LIMITER_H_
