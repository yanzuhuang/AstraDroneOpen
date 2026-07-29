#include <bspline_opt/occupied_segment_guard.h>

#include <gtest/gtest.h>

namespace ego_planner
{
namespace
{

TEST(OccupiedSegmentGuard, NormalEntryAndExit)
{
  EXPECT_TRUE(isValidOccupiedSegment(3, 7));
}

TEST(OccupiedSegmentGuard, EntryWithoutExit)
{
  EXPECT_FALSE(isValidOccupiedSegment(3, -1));
}

TEST(OccupiedSegmentGuard, OccupiedAtCheckedStart)
{
  EXPECT_TRUE(isValidOccupiedSegment(2, 4));
}

TEST(OccupiedSegmentGuard, OccupiedThroughCheckedEnd)
{
  EXPECT_FALSE(isValidOccupiedSegment(4, -1));
}

TEST(OccupiedSegmentGuard, VeryShortOccupiedInterval)
{
  EXPECT_TRUE(isValidOccupiedSegment(4, 5));
}

}  // namespace
}  // namespace ego_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
