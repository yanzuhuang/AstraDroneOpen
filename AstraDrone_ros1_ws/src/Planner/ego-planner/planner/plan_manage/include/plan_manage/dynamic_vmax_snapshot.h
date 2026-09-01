#ifndef EGO_PLANNER_DYNAMIC_VMAX_SNAPSHOT_H_
#define EGO_PLANNER_DYNAMIC_VMAX_SNAPSHOT_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <learning_speed_rl/SpeedActionStamped.h>
#include <learning_speed_rl/SpeedAppliedStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Float64.h>

#include <plan_manage/dynamic_speed_limit.h>

namespace ego_planner
{

struct DynamicVmaxSnapshot
{
  std::uint64_t version{0};
  double v_max{0.0};
  std::string episode_id;
  std::uint64_t step_index{0};
  std::uint64_t request_id{0};
  ros::Time apply_ros_stamp;
  std::uint64_t force_replan_generation{0};
  bool force_replan_intent{false};
  std_msgs::Header source_header;
};

enum class DynamicVmaxCommitStatus
{
  ACCEPTED,
  INVALID_ACTION,
  STALE_IDENTITY,
  OUT_OF_RANGE,
  SHUTTING_DOWN
};

struct DynamicVmaxCommitResult
{
  DynamicVmaxCommitStatus status{DynamicVmaxCommitStatus::INVALID_ACTION};
  std::shared_ptr<const DynamicVmaxSnapshot> snapshot;
  double previous_v_max{0.0};
};

// The dedicated speed callback thread is the only intended writer.  The
// mutex makes that ownership explicit and also keeps tests/teardown safe.  It
// never spans ROS publication or planning; the atomic shared_ptr store below
// is the authoritative snapshot linearization point.
class DynamicVmaxSnapshotStore
{
public:
  DynamicVmaxSnapshotStore(const DynamicSpeedLimitGate &gate,
                           double initial_v_max);

  DynamicVmaxCommitResult commit(
      const learning_speed_rl::SpeedActionStamped &action,
      const ros::Time &apply_ros_time);

  std::shared_ptr<const DynamicVmaxSnapshot> latest() const;
  void shutdown();

private:
  DynamicSpeedLimitGate gate_;
  mutable std::mutex writer_mutex_;
  std::shared_ptr<const DynamicVmaxSnapshot> latest_snapshot_;
  std::uint64_t next_version_{1};
  std::uint64_t force_replan_generation_{0};
  std::string active_episode_id_;
  std::unordered_set<std::string> closed_episode_ids_;
  std::unordered_map<std::string, std::uint64_t>
      last_request_id_by_episode_;
  std::unordered_map<std::string, std::uint64_t> last_step_by_episode_;
  std::atomic<bool> shutting_down_{false};
};

// Owns exactly one custom callback queue and one AsyncSpinner worker.  No
// planner/FSM callback is placed on this queue.
class DedicatedSpeedLimitChannel
{
public:
  DedicatedSpeedLimitChannel() = default;
  ~DedicatedSpeedLimitChannel();

  DedicatedSpeedLimitChannel(const DedicatedSpeedLimitChannel &) = delete;
  DedicatedSpeedLimitChannel &operator=(
      const DedicatedSpeedLimitChannel &) = delete;

  bool start(ros::NodeHandle &node,
             const std::string &action_topic,
             const std::string &applied_topic,
             const std::string &applied_stamped_topic,
             const DynamicSpeedLimitGate &gate,
             double initial_v_max,
             std::uint32_t queue_size = 100);
  void stop();

  std::shared_ptr<const DynamicVmaxSnapshot> latestSnapshot() const;
  bool running() const { return running_.load(std::memory_order_acquire); }

private:
  void speedLimitCallback(
      const learning_speed_rl::SpeedActionStampedConstPtr &action);
  void publishApplied(
      const std::shared_ptr<const DynamicVmaxSnapshot> &snapshot);

  ros::CallbackQueue callback_queue_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  ros::Subscriber action_subscriber_;
  ros::Publisher applied_publisher_;
  ros::Publisher applied_stamped_publisher_;
  std::unique_ptr<DynamicVmaxSnapshotStore> store_;
  std::atomic<bool> running_{false};
};

}  // namespace ego_planner

#endif  // EGO_PLANNER_DYNAMIC_VMAX_SNAPSHOT_H_
