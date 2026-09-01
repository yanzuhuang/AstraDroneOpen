#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <set>
#include <thread>
#include <vector>
#include <iostream>

#include <learning_speed_rl/SpeedAppliedStamped.h>
#include <plan_manage/dynamic_vmax_snapshot.h>
#include <ros/ros.h>

namespace ego_planner
{
namespace
{

class DedicatedQueueStressHarness
{
public:
  void appliedCallback(
      const learning_speed_rl::SpeedAppliedStampedConstPtr &message)
  {
    if (!message)
      return;
    std::lock_guard<std::mutex> guard(ack_mutex);
    acknowledgements.push_back(*message);
    ack_condition.notify_all();
  }

  void blockedPlannerCallback(const ros::WallTimerEvent &)
  {
    planner_binding = channel.latestSnapshot();
    planner_busy_start = ros::WallTime::now();
    planner_busy.store(true, std::memory_order_release);
    ros::WallDuration(4.5).sleep();
    planner_binding_consistent.store(
        planner_binding && planner_binding->version == 0 &&
            planner_binding->v_max == 1.0,
        std::memory_order_release);
    const std::shared_ptr<const DynamicVmaxSnapshot> latest =
        channel.latestSnapshot();
    if (latest)
    {
      consumed_force_generation.store(
          latest->force_replan_generation, std::memory_order_release);
      consumed_snapshot_version.store(
          latest->version, std::memory_order_release);
    }
    planner_busy.store(false, std::memory_order_release);
    planner_busy_end = ros::WallTime::now();
    planner_finished.store(true, std::memory_order_release);
  }

  bool waitForAckCount(std::size_t count, double timeout_seconds)
  {
    std::unique_lock<std::mutex> lock(ack_mutex);
    return ack_condition.wait_for(
        lock, std::chrono::duration<double>(timeout_seconds),
        [this, count]() { return acknowledgements.size() >= count; });
  }

  DedicatedSpeedLimitChannel channel;
  std::mutex ack_mutex;
  std::condition_variable ack_condition;
  std::vector<learning_speed_rl::SpeedAppliedStamped> acknowledgements;
  std::shared_ptr<const DynamicVmaxSnapshot> planner_binding;
  std::atomic<bool> planner_busy{false};
  std::atomic<bool> planner_finished{false};
  std::atomic<bool> planner_binding_consistent{false};
  std::atomic<std::uint64_t> consumed_force_generation{0};
  std::atomic<std::uint64_t> consumed_snapshot_version{0};
  ros::WallTime planner_busy_start;
  ros::WallTime planner_busy_end;
};

TEST(DedicatedSpeedQueueRosIntegration,
     ReturnsOneHundredExactAcksWhileMainPlannerQueueIsBusy)
{
  DedicatedQueueStressHarness harness;
  ros::NodeHandle channel_node;
  ASSERT_TRUE(harness.channel.start(
      channel_node, "/ego_speed_async_stress/action",
      "/ego_speed_async_stress/applied_scalar",
      "/ego_speed_async_stress/applied_stamped",
      DynamicSpeedLimitGate(0.3, 1.75), 1.0, 100));

  ros::CallbackQueue ack_queue;
  ros::NodeHandle ack_node;
  ack_node.setCallbackQueue(&ack_queue);
  ros::Subscriber ack_subscriber = ack_node.subscribe(
      "/ego_speed_async_stress/applied_stamped", 200,
      &DedicatedQueueStressHarness::appliedCallback, &harness);
  ros::AsyncSpinner ack_spinner(1, &ack_queue);
  ack_spinner.start();

  ros::Publisher action_publisher =
      channel_node.advertise<learning_speed_rl::SpeedActionStamped>(
          "/ego_speed_async_stress/action", 200, false);

  ros::CallbackQueue main_queue;
  ros::NodeHandle main_node;
  main_node.setCallbackQueue(&main_queue);
  ros::WallTimer blocked_planner = main_node.createWallTimer(
      ros::WallDuration(0.01),
      &DedicatedQueueStressHarness::blockedPlannerCallback, &harness,
      true, true);
  ros::AsyncSpinner main_spinner(1, &main_queue);
  main_spinner.start();

  const ros::WallTime connection_deadline =
      ros::WallTime::now() + ros::WallDuration(3.0);
  while (ros::ok() &&
         (action_publisher.getNumSubscribers() != 1 ||
          ack_subscriber.getNumPublishers() != 1) &&
         ros::WallTime::now() < connection_deadline)
    ros::WallDuration(0.01).sleep();
  ASSERT_EQ(1u, action_publisher.getNumSubscribers());
  ASSERT_EQ(1u, ack_subscriber.getNumPublishers());

  const ros::WallTime busy_deadline =
      ros::WallTime::now() + ros::WallDuration(2.0);
  while (!harness.planner_busy.load(std::memory_order_acquire) &&
         ros::WallTime::now() < busy_deadline)
    ros::WallDuration(0.005).sleep();
  ASSERT_TRUE(harness.planner_busy.load(std::memory_order_acquire));

  const double values[] = {0.60, 0.65, 1.20, 1.10, 0.75};
  std::vector<ros::Time> action_stamps(101);
  std::uint64_t request_count = 0;
  std::uint64_t action_count = 0;
  DynamicSpeedLimitGate gate(0.3, 1.75);
  double previous_value = 1.0;
  std::uint64_t expected_force_generation = 0;
  for (std::uint64_t request = 1; request <= 100; ++request)
  {
    const double value = values[(request - 1) % 5];
    ++request_count;
    learning_speed_rl::SpeedActionStamped action;
    action.header.stamp = ros::Time::now();
    action.version = "learning_speed_action_v1.1";
    action.source_mode = "ros_integration_stress";
    action.episode_id = "stress_episode";
    action.step_index = request - 1;
    action.request_id = request;
    action.requested_v_max = value;
    action.filtered_v_max = value;
    action_stamps[request] = action.header.stamp;
    if (gate.requiresForceReplan(previous_value, value))
      ++expected_force_generation;
    previous_value = value;
    action_publisher.publish(action);
    ++action_count;
    ros::WallDuration(0.005).sleep();
  }

  ASSERT_TRUE(harness.waitForAckCount(100, 2.0));
  EXPECT_TRUE(harness.planner_busy.load(std::memory_order_acquire));
  EXPECT_EQ(100u, request_count);
  EXPECT_EQ(100u, action_count);

  std::vector<learning_speed_rl::SpeedAppliedStamped> acknowledgements;
  {
    std::lock_guard<std::mutex> guard(harness.ack_mutex);
    acknowledgements = harness.acknowledgements;
  }
  ASSERT_EQ(100u, acknowledgements.size());
  std::set<std::uint64_t> unique_requests;
  std::uint64_t last_request = 0;
  double maximum_ack_latency_ms = 0.0;
  for (const learning_speed_rl::SpeedAppliedStamped &ack : acknowledgements)
  {
    EXPECT_EQ("learning_speed_applied_v1.0", ack.version);
    EXPECT_EQ("stress_episode", ack.episode_id);
    EXPECT_EQ(ack.step_index + 1, ack.request_id);
    EXPECT_GT(ack.request_id, last_request);
    EXPECT_GE(ack.header.stamp, action_stamps.at(ack.request_id));
    EXPECT_DOUBLE_EQ(values[(ack.request_id - 1) % 5], ack.applied_v_max);
    maximum_ack_latency_ms = std::max(
        maximum_ack_latency_ms,
        (ack.header.stamp - action_stamps.at(ack.request_id)).toSec() *
            1000.0);
    unique_requests.insert(ack.request_id);
    last_request = ack.request_id;
  }
  EXPECT_EQ(100u, unique_requests.size());

  const std::shared_ptr<const DynamicVmaxSnapshot> latest =
      harness.channel.latestSnapshot();
  ASSERT_TRUE(latest);
  EXPECT_EQ(100u, latest->version);
  EXPECT_EQ(100u, latest->request_id);
  EXPECT_EQ(expected_force_generation, latest->force_replan_generation);
  ASSERT_TRUE(harness.planner_binding);
  EXPECT_EQ(0u, harness.planner_binding->version);
  EXPECT_DOUBLE_EQ(1.0, harness.planner_binding->v_max);

  const ros::WallTime finish_deadline =
      ros::WallTime::now() + ros::WallDuration(6.0);
  while (!harness.planner_finished.load(std::memory_order_acquire) &&
         ros::WallTime::now() < finish_deadline)
    ros::WallDuration(0.01).sleep();
  ASSERT_TRUE(harness.planner_finished.load(std::memory_order_acquire));
  EXPECT_TRUE(harness.planner_binding_consistent.load(
      std::memory_order_acquire));
  EXPECT_EQ(expected_force_generation,
            harness.consumed_force_generation.load(
                std::memory_order_acquire));
  EXPECT_EQ(100u, harness.consumed_snapshot_version.load(
                      std::memory_order_acquire));
  const double planner_busy_seconds =
      (harness.planner_busy_end - harness.planner_busy_start).toSec();
  EXPECT_GT(planner_busy_seconds, 4.0);
  std::cout << "EGO_SPEED_ASYNC_STRESS request=100 action=100 applied=100"
            << " duplicate=0 out_of_order=0 identity_mismatch=0"
            << " force_generation=" << expected_force_generation
            << " planner_busy_seconds=" << planner_busy_seconds
            << " max_ack_latency_ms=" << maximum_ack_latency_ms
            << " planner_binding_version="
            << harness.planner_binding->version
            << " latest_version=" << latest->version << std::endl;

  blocked_planner.stop();
  main_spinner.stop();
  ack_spinner.stop();
  harness.channel.stop();
}

}  // namespace
}  // namespace ego_planner

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_vmax_snapshot_ros_stress_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
