#include "ego_gazebo_bridge/control_authority.h"

namespace ego_gazebo_bridge {

bool findControlConflict(
    const std::vector<MonitoredControlTopic>& monitored_topics,
    const std::vector<TopicPublishers>& publisher_state,
    const std::string& own_node, ControlConflict* conflict) {
  for (const auto& monitored : monitored_topics) {
    for (const auto& publishers : publisher_state) {
      if (publishers.topic != monitored.topic) {
        continue;
      }
      for (const auto& node : publishers.nodes) {
        if (node == own_node) {
          continue;
        }
        if (conflict != nullptr) {
          conflict->category = monitored.category;
          conflict->topic = monitored.topic;
          conflict->node = node;
        }
        return true;
      }
    }
  }
  if (conflict != nullptr) {
    *conflict = ControlConflict{};
  }
  return false;
}

}  // namespace ego_gazebo_bridge
