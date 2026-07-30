#ifndef EGO_PLANNER_SWARM_FRAME_TRANSFORM_H
#define EGO_PLANNER_SWARM_FRAME_TRANSFORM_H

#include <geometry_msgs/Point.h>

#include <cmath>

namespace ego_planner
{

class SwarmFrameTransform
{
public:
  SwarmFrameTransform() = default;

  SwarmFrameTransform(double origin_x, double origin_y, double origin_z,
                      double origin_yaw)
      : origin_x_(origin_x),
        origin_y_(origin_y),
        origin_z_(origin_z),
        cosine_(std::cos(origin_yaw)),
        sine_(std::sin(origin_yaw))
  {
  }

  bool isFinite() const
  {
    return std::isfinite(origin_x_) && std::isfinite(origin_y_) &&
           std::isfinite(origin_z_) && std::isfinite(cosine_) &&
           std::isfinite(sine_);
  }

  geometry_msgs::Point localToCommon(const geometry_msgs::Point &local) const
  {
    geometry_msgs::Point common;
    common.x = cosine_ * local.x - sine_ * local.y + origin_x_;
    common.y = sine_ * local.x + cosine_ * local.y + origin_y_;
    common.z = local.z + origin_z_;
    return common;
  }

  geometry_msgs::Point commonToLocal(const geometry_msgs::Point &common) const
  {
    const double x = common.x - origin_x_;
    const double y = common.y - origin_y_;
    geometry_msgs::Point local;
    local.x = cosine_ * x + sine_ * y;
    local.y = -sine_ * x + cosine_ * y;
    local.z = common.z - origin_z_;
    return local;
  }

private:
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_z_{0.0};
  double cosine_{1.0};
  double sine_{0.0};
};

}  // namespace ego_planner

#endif
