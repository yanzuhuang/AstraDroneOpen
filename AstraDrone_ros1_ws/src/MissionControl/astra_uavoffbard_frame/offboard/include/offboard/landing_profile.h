#ifndef OFFBOARD_LANDING_PROFILE_H_
#define OFFBOARD_LANDING_PROFILE_H_

#include <algorithm>

namespace offboard {

// 根据相对地面高度生成连续的下降速度（返回值为正数，表示向下速度大小）。
// 中间段使用 smoothstep，避免在减速高度处产生速度斜率突变。
inline double landingDescentSpeed(double height_above_ground,
                                  double cruise_speed,
                                  double touchdown_speed,
                                  double slowdown_height,
                                  double flare_height) {
    if (height_above_ground >= slowdown_height) {
        return cruise_speed;
    }
    if (height_above_ground <= flare_height) {
        return touchdown_speed;
    }

    const double normalized = std::max(
        0.0, std::min(1.0,
                      (height_above_ground - flare_height) /
                          (slowdown_height - flare_height)));
    const double smooth = normalized * normalized * (3.0 - 2.0 * normalized);
    return touchdown_speed + (cruise_speed - touchdown_speed) * smooth;
}

}  // namespace offboard

#endif  // OFFBOARD_LANDING_PROFILE_H_
