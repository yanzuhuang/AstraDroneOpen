#ifndef EGO_GAZEBO_BRIDGE_RECOVERY_CONTROL_H_
#define EGO_GAZEBO_BRIDGE_RECOVERY_CONTROL_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/time.h>

#include <cstdint>
#include <cmath>

namespace ego_gazebo_bridge {

// Generation gate at the EGO/bridge boundary. Cancellation closes the gate;
// only a command for a later goal and a different trajectory generation can
// reopen it. This remains independent of command publication and is unit-testable.
class TrajectoryGate {
 public:
  void cancel() {
    active_ = true;
    have_cancelled_id_ = have_last_id_;
    cancelled_id_ = last_id_;
    goal_time_ = ros::Time(0);
  }

  void noteGoal(const ros::Time& goal_time) { goal_time_ = goal_time; }

  bool allows(std::uint32_t trajectory_id, const ros::Time& command_stamp,
              bool have_valid_goal) const {
    if (!active_) return true;
    return have_valid_goal && !goal_time_.isZero() &&
           command_stamp >= goal_time_ &&
           (!have_cancelled_id_ || trajectory_id != cancelled_id_);
  }

  void accept(std::uint32_t trajectory_id) {
    last_id_ = trajectory_id;
    have_last_id_ = true;
    active_ = false;
  }

  bool active() const { return active_; }
  std::uint32_t cancelledId() const { return cancelled_id_; }

 private:
  bool active_{false};
  bool have_last_id_{false};
  bool have_cancelled_id_{false};
  std::uint32_t last_id_{0};
  std::uint32_t cancelled_id_{0};
  ros::Time goal_time_;
};

inline bool shouldAutoLandFromHold(bool mission_supervised,
                                   double hold_duration,
                                   double timeout) {
  return !mission_supervised && hold_duration >= timeout;
}

inline bool homeHoverAllowsAutoLand(bool in_home_hover_state,
                                    bool inputs_fresh,
                                    bool armed,
                                    bool offboard,
                                    double home_position_error,
                                    double home_tolerance) {
  return in_home_hover_state && inputs_fresh && armed && offboard &&
         std::isfinite(home_position_error) &&
         std::isfinite(home_tolerance) && home_tolerance > 0.0 &&
         home_position_error <= home_tolerance;
}

inline bool supervisedHoldAllowsEmergencyAutoLand(bool in_hold_state,
                                                  bool mission_supervised,
                                                  bool inputs_fresh,
                                                  bool armed,
                                                  bool offboard,
                                                  bool pose_fresh) {
  return in_hold_state && mission_supervised && inputs_fresh && armed &&
         offboard && pose_fresh;
}

// Raw EGO commands bypass the position-only output_setpoint_ slew state. A
// HOLD transition must therefore seed both HOLD and output state from the
// same measured pose; otherwise publishSetpoint() can restart from a stale
// takeoff/home setpoint and command a large discontinuity.
struct HoldSetpointLatch {
  geometry_msgs::PoseStamped hold_pose;
  geometry_msgs::PoseStamped output_setpoint;
  bool have_output_setpoint{false};
};

inline HoldSetpointLatch makeHoldSetpointLatch(
    const geometry_msgs::PoseStamped& measured_or_last_output) {
  HoldSetpointLatch latch;
  latch.hold_pose = measured_or_last_output;
  latch.output_setpoint = measured_or_last_output;
  latch.have_output_setpoint = true;
  return latch;
}

}  // namespace ego_gazebo_bridge

#endif  // EGO_GAZEBO_BRIDGE_RECOVERY_CONTROL_H_
