#ifndef EGO_PLANNER_TRAJECTORY_CANCELLATION_GATE_H_
#define EGO_PLANNER_TRAJECTORY_CANCELLATION_GATE_H_

#include <cstdint>

#include <ros/time.h>

namespace ego_planner
{

class TrajectoryCancellationGate
{
public:
  void noteCancel(const ros::Time &cancel_time, bool have_active_trajectory,
                  std::uint32_t active_trajectory_id)
  {
    have_cancel_ = true;
    cancel_time_ = cancel_time;
    have_cancelled_trajectory_id_ = have_active_trajectory;
    cancelled_trajectory_id_ = active_trajectory_id;
    post_cancel_goal_time_ = ros::Time(0);
  }

  bool noteGoal(const ros::Time &goal_time)
  {
    if (!have_cancel_)
      return !goal_time.isZero();

    if (goal_time.isZero() || goal_time <= cancel_time_)
      return false;

    if (!post_cancel_goal_time_.isZero() &&
        goal_time <= post_cancel_goal_time_)
      return false;

    post_cancel_goal_time_ = goal_time;
    return true;
  }

  bool acceptsTrajectory(std::uint32_t trajectory_id,
                         const ros::Time &start_time) const
  {
    if (!have_cancel_)
      return true;

    if (have_cancelled_trajectory_id_ &&
        trajectory_id == cancelled_trajectory_id_)
      return false;

    if (post_cancel_goal_time_.isZero() || start_time.isZero())
      return false;

    return start_time >= post_cancel_goal_time_;
  }

  bool hasCancellation() const { return have_cancel_; }
  const ros::Time &cancelTime() const { return cancel_time_; }
  const ros::Time &postCancelGoalTime() const
  {
    return post_cancel_goal_time_;
  }

private:
  bool have_cancel_{false};
  bool have_cancelled_trajectory_id_{false};
  std::uint32_t cancelled_trajectory_id_{0};
  ros::Time cancel_time_;
  ros::Time post_cancel_goal_time_;
};

}  // namespace ego_planner

#endif  // EGO_PLANNER_TRAJECTORY_CANCELLATION_GATE_H_
