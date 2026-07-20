#include "ego_gazebo_bridge/control_authority.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace ego_gazebo_bridge {
namespace {

const std::vector<MonitoredControlTopic> kMonitoredTopics{
    {"position", "/mavros/setpoint_position/local"},
    {"raw local", "/mavros/setpoint_raw/local"},
    {"velocity", "/mavros/setpoint_velocity/cmd_vel"},
    {"attitude", "/mavros/setpoint_raw/attitude"},
    {"thrust", "/mavros/setpoint_attitude/thrust"},
};

TEST(ControlAuthority, AcceptsEmptyPublisherState) {
  ControlConflict conflict;
  EXPECT_FALSE(findControlConflict(kMonitoredTopics, {}, "/bridge",
                                   &conflict));
  EXPECT_TRUE(conflict.topic.empty());
}

TEST(ControlAuthority, IgnoresOwnAndUnmonitoredPublishers) {
  const std::vector<TopicPublishers> publishers{
      {"/mavros/setpoint_position/local", {"/bridge"}},
      {"/planning/pos_cmd", {"/traj_server"}},
  };
  EXPECT_FALSE(findControlConflict(kMonitoredTopics, publishers, "/bridge",
                                   nullptr));
}

TEST(ControlAuthority, ReportsEachControlCategory) {
  for (const auto& monitored : kMonitoredTopics) {
    const std::vector<TopicPublishers> publishers{
        {monitored.topic, {"/other_controller"}},
    };
    ControlConflict conflict;
    ASSERT_TRUE(findControlConflict(kMonitoredTopics, publishers, "/bridge",
                                    &conflict));
    EXPECT_EQ(monitored.category, conflict.category);
    EXPECT_EQ(monitored.topic, conflict.topic);
    EXPECT_EQ("/other_controller", conflict.node);
  }
}

}  // namespace
}  // namespace ego_gazebo_bridge

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
