#ifndef ASTRA_TOWER_MISSION_MOTION_LIMITER_H_
#define ASTRA_TOWER_MISSION_MOTION_LIMITER_H_

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

MotionReference stepMotionReference(const MotionReference& current,
                                    const MotionTarget& target,
                                    double dt,
                                    double maximum_speed,
                                    double maximum_acceleration,
                                    double maximum_yaw_rate);

}  // namespace astra_tower_mission

#endif  // ASTRA_TOWER_MISSION_MOTION_LIMITER_H_
