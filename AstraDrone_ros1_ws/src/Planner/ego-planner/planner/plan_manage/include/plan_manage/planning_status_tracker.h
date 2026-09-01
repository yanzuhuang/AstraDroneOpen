#ifndef EGO_PLANNER_PLANNING_STATUS_TRACKER_H_
#define EGO_PLANNER_PLANNING_STATUS_TRACKER_H_

#include <cstdint>
#include <string>

namespace ego_planner {

inline std::string planningFailureReason() {
  return "NO_FEASIBLE_TRAJECTORY";
}

// Updated only at actual planner call sites. Periodic PlannerStatus
// publication is read-only and therefore cannot inflate failure counts.
class PlanningStatusTracker {
 public:
  void reset(const std::string& neutral_reason) {
    last_success_ = false;
    consecutive_failures_ = 0;
    failure_reason_ = neutral_reason;
  }

  void recordAttempt(bool success, const std::string& failure_reason,
                     std::uint32_t trajectory_id) {
    last_success_ = success;
    if (success) {
      consecutive_failures_ = 0;
      failure_reason_.clear();
      trajectory_id_ = trajectory_id;
    } else {
      ++consecutive_failures_;
      failure_reason_ = failure_reason;
    }
  }

  void setEventReason(const std::string& reason) { failure_reason_ = reason; }
  bool lastSuccess() const { return last_success_; }
  std::uint32_t consecutiveFailures() const { return consecutive_failures_; }
  std::uint32_t trajectoryId() const { return trajectory_id_; }
  const std::string& failureReason() const { return failure_reason_; }

 private:
  bool last_success_{false};
  std::uint32_t consecutive_failures_{0};
  std::uint32_t trajectory_id_{0};
  std::string failure_reason_;
};

}  // namespace ego_planner

#endif  // EGO_PLANNER_PLANNING_STATUS_TRACKER_H_
