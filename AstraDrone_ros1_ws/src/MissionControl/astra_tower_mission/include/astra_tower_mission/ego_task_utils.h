#ifndef ASTRA_TOWER_MISSION_EGO_TASK_UTILS_H_
#define ASTRA_TOWER_MISSION_EGO_TASK_UTILS_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/time.h>

#include <cerrno>
#include <cctype>
#include <cstdint>
#include <ctime>
#include <string>

#include <sys/stat.h>

#include <vector>

namespace astra_tower_mission {

struct TowerCandidate {
  std::string name;
  std::string frame_id;
  double center_x{0.0};
  double center_y{0.0};
  double collision_radius{0.0};
};

double posePositionDistance(const geometry_msgs::PoseStamped& first,
                            const geometry_msgs::PoseStamped& second);

bool validateFixedHeightGoals(
    const std::vector<geometry_msgs::PoseStamped>& goals,
    double expected_height, double tolerance, std::string* reason);

bool validateGoalHeightBounds(
    const std::vector<geometry_msgs::PoseStamped>& goals,
    double minimum_height, double maximum_height, std::string* reason);

std::vector<geometry_msgs::PoseStamped> appendClosureGoal(
    const std::vector<geometry_msgs::PoseStamped>& unique_goals);

int nearestTowerIndex(const std::vector<TowerCandidate>& candidates,
                      double vehicle_x, double vehicle_y);

bool isNewTrajectory(std::uint32_t baseline_id, std::uint32_t command_id,
                     const ros::Time& goal_stamp,
                     const ros::Time& command_stamp);

double selectArrivalTolerance(bool tower_scenario, std::size_t goal_index,
                              std::size_t descent_goal_index,
                              double nominal_tolerance,
                              double transit_transition_tolerance);

std::size_t nextSafeReturnGoalIndex(
    const std::vector<bool>& endpoint_safety, std::size_t failed_index);

inline bool runtimeArtifactDirectoryHasTimestamp(const std::string& directory) {
  const std::size_t first_underscore = directory.rfind('_');
  if (first_underscore == std::string::npos || first_underscore < 9 ||
      directory.size() - first_underscore != 7) {
    return false;
  }
  const std::size_t date_underscore = first_underscore - 9;
  if (directory[date_underscore] != '_') {
    return false;
  }
  for (std::size_t index = date_underscore + 1; index < directory.size(); ++index) {
    if (index == first_underscore) {
      continue;
    }
    if (!std::isdigit(static_cast<unsigned char>(directory[index]))) {
      return false;
    }
  }
  return true;
}

inline std::string currentArtifactTimestamp() {
  const std::time_t now = std::time(nullptr);
  std::tm local_time;
  localtime_r(&now, &local_time);
  char buffer[16];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local_time);
  return buffer;
}

inline std::string timestampedRuntimeArtifactFile(
    const std::string& configured_path, const std::string& task_name) {
  const std::string marker = "runtime_artifacts/";
  const std::size_t marker_position = configured_path.find(marker);
  if (marker_position == std::string::npos) {
    return configured_path;
  }

  const std::size_t directory_begin = marker_position + marker.size();
  const std::size_t directory_end = configured_path.find('/', directory_begin);
  if (directory_end == std::string::npos) {
    return configured_path;
  }
  const std::string directory = configured_path.substr(
      directory_begin, directory_end - directory_begin);
  if (runtimeArtifactDirectoryHasTimestamp(directory)) {
    return configured_path;
  }

  const std::size_t file_separator = configured_path.rfind('/');
  const std::string file_name = configured_path.substr(file_separator + 1);
  return configured_path.substr(0, directory_begin) + task_name + "_" +
         currentArtifactTimestamp() + "/" + file_name;
}

inline bool ensureArtifactParentDirectory(const std::string& file_path) {
  const std::size_t parent_end = file_path.rfind('/');
  if (parent_end == std::string::npos) {
    return true;
  }
  const std::string parent = file_path.substr(0, parent_end);
  for (std::size_t index = 1; index <= parent.size(); ++index) {
    if (index != parent.size() && parent[index] != '/') {
      continue;
    }
    const std::string directory = parent.substr(0, index);
    if (directory.empty() || directory == ".") {
      continue;
    }
    if (mkdir(directory.c_str(), 0755) != 0 && errno != EEXIST) {
      return false;
    }
  }
  return true;
}

}  // namespace astra_tower_mission

#endif  // ASTRA_TOWER_MISSION_EGO_TASK_UTILS_H_
