#include <gtest/gtest.h>

#include "bspline_opt/short_segment_guidance.h"

namespace {

using ego_planner::ShortSegmentGuidance;
using ego_planner::ShortSegmentGuidanceSource;
using ego_planner::computeShortSegmentGuidance;

void expectDirection(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const std::vector<Eigen::Vector3d>& path,
    const Eigen::Vector3d& expected_axis,
    ShortSegmentGuidanceSource expected_source) {
  ShortSegmentGuidance guidance;
  ASSERT_TRUE(computeShortSegmentGuidance(start, end, path, guidance));
  EXPECT_NEAR(guidance.direction.norm(), 1.0, 1.0e-12);
  EXPECT_GT(guidance.direction.dot(expected_axis.normalized()), 0.99);
  EXPECT_EQ(guidance.source, expected_source);
}

TEST(ShortSegmentGuidance, HorizontalIntersectionChoosesPositiveY) {
  expectDirection({-0.2, 0.0, 0.0}, {0.2, 0.0, 0.0},
                  {{-1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}},
                  Eigen::Vector3d::UnitY(),
                  ShortSegmentGuidanceSource::MIDPOINT_INTERSECTION);
}

TEST(ShortSegmentGuidance, HorizontalSupportsNegativeY) {
  expectDirection({-0.2, 0.0, 0.0}, {0.2, 0.0, 0.0},
                  {{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}},
                  -Eigen::Vector3d::UnitY(),
                  ShortSegmentGuidanceSource::MIDPOINT_INTERSECTION);
}

TEST(ShortSegmentGuidance, HorizontalSupportsPositiveAndNegativeZ) {
  expectDirection({-0.2, 0.0, 0.0}, {0.2, 0.0, 0.0},
                  {{-1.0, 0.0, 1.0}, {1.0, 0.0, 1.0}},
                  Eigen::Vector3d::UnitZ(),
                  ShortSegmentGuidanceSource::MIDPOINT_INTERSECTION);
  expectDirection({-0.2, 0.0, 0.0}, {0.2, 0.0, 0.0},
                  {{-1.0, 0.0, -1.0}, {1.0, 0.0, -1.0}},
                  -Eigen::Vector3d::UnitZ(),
                  ShortSegmentGuidanceSource::MIDPOINT_INTERSECTION);
}

TEST(ShortSegmentGuidance, VerticalAndDiagonalSegmentsRemainGeometric) {
  expectDirection({0.0, 0.0, -0.2}, {0.0, 0.0, 0.2},
                  {{0.0, -1.0, -1.0}, {0.0, -1.0, 1.0}},
                  -Eigen::Vector3d::UnitY(),
                  ShortSegmentGuidanceSource::MIDPOINT_INTERSECTION);
  expectDirection({-0.2, -0.2, 0.0}, {0.2, 0.2, 0.0},
                  {{-1.0, 0.0, 0.0}, {1.0, 2.0, 0.0}},
                  Eigen::Vector3d(-1.0, 1.0, 0.0),
                  ShortSegmentGuidanceSource::MIDPOINT_INTERSECTION);
}

TEST(ShortSegmentGuidance, ProjectionFallbackHandlesNoPlaneCrossing) {
  expectDirection({-0.2, 0.0, 0.0}, {0.2, 0.0, 0.0},
                  {{0.4, 1.0, -1.0}, {0.4, 1.0, 1.0}},
                  Eigen::Vector3d(0.4, 1.0, 0.0),
                  ShortSegmentGuidanceSource::NEAREST_PROJECTED_PATH);
}

TEST(ShortSegmentGuidance, DegenerateOrCollinearGeometryFailsClosed) {
  ShortSegmentGuidance guidance;
  EXPECT_FALSE(computeShortSegmentGuidance(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      {{0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}}, guidance));
  EXPECT_FALSE(computeShortSegmentGuidance(
      {-0.2, 0.0, 0.0}, {0.2, 0.0, 0.0},
      {{0.4, 0.0, 0.0}, {1.0, 0.0, 0.0}}, guidance));
  EXPECT_FALSE(computeShortSegmentGuidance(
      {-0.2, 0.0, 0.0}, {0.2, 0.0, 0.0},
      {{0.0, 0.0, 0.0}}, guidance));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
