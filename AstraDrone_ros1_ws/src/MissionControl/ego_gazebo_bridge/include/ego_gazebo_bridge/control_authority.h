#ifndef EGO_GAZEBO_BRIDGE_CONTROL_AUTHORITY_H_
#define EGO_GAZEBO_BRIDGE_CONTROL_AUTHORITY_H_

#include <string>
#include <vector>

namespace ego_gazebo_bridge {

struct MonitoredControlTopic {
  std::string category;
  std::string topic;
};

struct TopicPublishers {
  std::string topic;
  std::vector<std::string> nodes;
};

struct ControlConflict {
  std::string category;
  std::string topic;
  std::string node;
};

bool findControlConflict(
    const std::vector<MonitoredControlTopic>& monitored_topics,
    const std::vector<TopicPublishers>& publisher_state,
    const std::string& own_node, ControlConflict* conflict);

}  // namespace ego_gazebo_bridge

#endif  // EGO_GAZEBO_BRIDGE_CONTROL_AUTHORITY_H_
