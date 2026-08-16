#include "astra_tower_mission/mission_completion.h"

#include <gtest/gtest.h>

namespace astra_tower_mission {
namespace {

TEST(OrbitBookkeepingRelease, PermissionOffStartsAtFirstFormalWaypoint) {
  EXPECT_TRUE(shouldReleaseOrbitBookkeeping(false, false, true, false));
  EXPECT_FALSE(shouldReleaseOrbitBookkeeping(false, false, false, false));
  EXPECT_FALSE(shouldReleaseOrbitBookkeeping(false, false, true, true));
}

TEST(OrbitBookkeepingRelease, PermissionOnStillRequiresExplicitRelease) {
  EXPECT_FALSE(shouldReleaseOrbitBookkeeping(true, false, true, false));
  EXPECT_TRUE(shouldReleaseOrbitBookkeeping(true, true, false, false));
  EXPECT_FALSE(shouldReleaseOrbitBookkeeping(true, true, false, true));
}

TEST(MissionCompletion, SuccessfulTerminalRequiresCompletedOrbit) {
  const auto active = missionCompletionSignals(false, false, false, true);
  EXPECT_FALSE(active.mission_done);
  EXPECT_FALSE(active.mission_success);
  EXPECT_FALSE(active.mission_failure);
  EXPECT_TRUE(active.orbit_complete);

  const auto success = missionCompletionSignals(true, false, false, true);
  EXPECT_TRUE(success.mission_done);
  EXPECT_TRUE(success.mission_success);
  EXPECT_FALSE(success.mission_failure);
  EXPECT_TRUE(success.orbit_complete);
}

TEST(MissionCompletion, FailedOrIncompleteTerminalCannotReportSuccess) {
  const auto returning_after_failure =
      missionCompletionSignals(false, false, true, false);
  EXPECT_FALSE(returning_after_failure.mission_done);
  EXPECT_TRUE(returning_after_failure.mission_failure);

  const auto failed = missionCompletionSignals(false, true, true, false);
  EXPECT_TRUE(failed.mission_done);
  EXPECT_FALSE(failed.mission_success);
  EXPECT_TRUE(failed.mission_failure);

  const auto incomplete = missionCompletionSignals(true, false, false, false);
  EXPECT_TRUE(incomplete.mission_done);
  EXPECT_FALSE(incomplete.mission_success);
  EXPECT_TRUE(incomplete.mission_failure);
}

}  // namespace
}  // namespace astra_tower_mission

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
