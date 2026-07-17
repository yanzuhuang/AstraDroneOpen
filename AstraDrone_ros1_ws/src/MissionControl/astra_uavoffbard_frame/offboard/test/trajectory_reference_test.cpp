#include <gtest/gtest.h>

#include <cmath>

#include "offboard/trajectory_reference.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(TrajectoryReference, CircleHasExpectedLengthAndStart) {
    offboard::TrajectoryReference path(
        "circle", 1.0, -2.0, 2.0, 3.0, 2.0, 1.0, false, 2000);
    EXPECT_NEAR(path.length(), 4.0 * kPi, 1e-4);
    const offboard::TrajectoryPoint start = path.sample(0.0);
    EXPECT_NEAR(start.x, 3.0, 1e-9);
    EXPECT_NEAR(start.y, -2.0, 1e-9);
    EXPECT_NEAR(start.tangent_x, 0.0, 0.01);
    EXPECT_NEAR(start.tangent_y, 1.0, 0.01);
}

TEST(TrajectoryReference, ArcLengthSamplingProducesEqualFigure8Steps) {
    offboard::TrajectoryReference path(
        "figure8", 0.0, 0.0, 2.0, 3.0, 2.0, 1.0, false, 4000);
    const double ds = path.length() / 100.0;
    double minimum = 1e9;
    double maximum = 0.0;
    offboard::TrajectoryPoint previous = path.sample(0.0);
    for (int i = 1; i < 100; ++i) {
        const offboard::TrajectoryPoint current = path.sample(i * ds);
        const double dx = current.x - previous.x;
        const double dy = current.y - previous.y;
        const double chord = std::sqrt(dx * dx + dy * dy);
        minimum = std::min(minimum, chord);
        maximum = std::max(maximum, chord);
        previous = current;
    }
    EXPECT_LT(maximum / minimum, 1.08);
}

TEST(TrajectoryReference, SquareSlowsAtCorners) {
    offboard::TrajectoryReference path(
        "square", 0.0, 0.0, 2.0, 4.0, 2.0, 1.0, false, 2000);
    EXPECT_NEAR(path.length(), 16.0, 1e-9);
    EXPECT_NEAR(path.speedScale(0.0, 0.5, 0.3), 0.3, 1e-9);
    EXPECT_NEAR(path.speedScale(2.0, 0.5, 0.3), 1.0, 1e-9);
}

TEST(TrajectoryReference, UnwrapAvoidsTwoPiJump) {
    const double previous = 179.0 * kPi / 180.0;
    const double raw = -179.0 * kPi / 180.0;
    EXPECT_NEAR(offboard::unwrapAngle(raw, previous) - previous,
                2.0 * kPi / 180.0, 1e-9);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
