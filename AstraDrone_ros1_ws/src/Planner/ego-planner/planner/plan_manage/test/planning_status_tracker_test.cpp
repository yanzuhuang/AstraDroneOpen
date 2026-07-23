#include <plan_manage/planning_status_tracker.h>

#include <gtest/gtest.h>

namespace ego_planner {
namespace {

TEST(PlanningStatusTracker, CountsActualAttemptsAndResetsOnSuccess) {
  PlanningStatusTracker tracker;
  tracker.reset("NONE");
  tracker.recordAttempt(false, "NO_FEASIBLE_TRAJECTORY", 0U);
  tracker.recordAttempt(false, "REPLAN_FAILED", 0U);
  EXPECT_EQ(tracker.consecutiveFailures(), 2U);
  EXPECT_FALSE(tracker.lastSuccess());
  EXPECT_EQ(tracker.failureReason(), "REPLAN_FAILED");
  tracker.recordAttempt(true, "ignored", 17U);
  EXPECT_EQ(tracker.consecutiveFailures(), 0U);
  EXPECT_TRUE(tracker.lastSuccess());
  EXPECT_EQ(tracker.trajectoryId(), 17U);
}

TEST(PlanningStatusTracker, StatusReadsAndEventsDoNotCreateAttempts) {
  PlanningStatusTracker tracker;
  tracker.reset("NONE");
  tracker.recordAttempt(false, "REPLAN_FAILED", 0U);
  for (int publish_cycle = 0; publish_cycle < 100; ++publish_cycle) {
    EXPECT_EQ(tracker.consecutiveFailures(), 1U);
    (void)tracker.lastSuccess();
    (void)tracker.failureReason();
  }
  tracker.setEventReason("EMERGENCY_STOP_TIMEOUT");
  EXPECT_EQ(tracker.consecutiveFailures(), 1U);
}

}  // namespace
}  // namespace ego_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
