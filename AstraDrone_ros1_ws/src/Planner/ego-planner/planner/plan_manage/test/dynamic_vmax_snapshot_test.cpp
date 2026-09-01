#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include <plan_manage/dynamic_vmax_snapshot.h>

namespace ego_planner
{
namespace
{

learning_speed_rl::SpeedActionStamped makeAction(
    const std::string &episode, std::uint64_t step, std::uint64_t request,
    double value)
{
  learning_speed_rl::SpeedActionStamped action;
  action.header.stamp = ros::Time(10, static_cast<std::uint32_t>(request));
  action.version = "learning_speed_action_v1.1";
  action.source_mode = "unit_stress";
  action.episode_id = episode;
  action.step_index = step;
  action.request_id = request;
  action.requested_v_max = value;
  action.filtered_v_max = value;
  return action;
}

TEST(DynamicVmaxSnapshotStore, CommitPrecedesExactImmutableAckState)
{
  DynamicVmaxSnapshotStore store(DynamicSpeedLimitGate(0.3, 1.75), 1.0);
  const learning_speed_rl::SpeedActionStamped action =
      makeAction("episode_1", 0, 1, 0.6);
  const DynamicVmaxCommitResult result =
      store.commit(action, ros::Time(20, 0));

  ASSERT_EQ(DynamicVmaxCommitStatus::ACCEPTED, result.status);
  ASSERT_TRUE(result.snapshot);
  EXPECT_EQ(1u, result.snapshot->version);
  EXPECT_DOUBLE_EQ(0.6, result.snapshot->v_max);
  EXPECT_EQ(action.episode_id, result.snapshot->episode_id);
  EXPECT_EQ(action.step_index, result.snapshot->step_index);
  EXPECT_EQ(action.request_id, result.snapshot->request_id);
  EXPECT_EQ(ros::Time(20, 0), result.snapshot->apply_ros_stamp);
  EXPECT_TRUE(result.snapshot->force_replan_intent);
  EXPECT_EQ(1u, result.snapshot->force_replan_generation);

  const std::shared_ptr<const DynamicVmaxSnapshot> latest = store.latest();
  ASSERT_TRUE(latest);
  EXPECT_EQ(result.snapshot.get(), latest.get());
}

TEST(DynamicVmaxSnapshotStore, RejectsDuplicateOutOfOrderAndClosedEpisode)
{
  DynamicVmaxSnapshotStore store(DynamicSpeedLimitGate(0.3, 1.75), 1.0);
  EXPECT_EQ(DynamicVmaxCommitStatus::ACCEPTED,
            store.commit(makeAction("episode_1", 0, 1, 1.0),
                         ros::Time(20, 0)).status);
  EXPECT_EQ(DynamicVmaxCommitStatus::STALE_IDENTITY,
            store.commit(makeAction("episode_1", 0, 1, 1.0),
                         ros::Time(21, 0)).status);
  EXPECT_EQ(DynamicVmaxCommitStatus::STALE_IDENTITY,
            store.commit(makeAction("episode_1", 0, 2, 1.0),
                         ros::Time(22, 0)).status);
  EXPECT_EQ(DynamicVmaxCommitStatus::ACCEPTED,
            store.commit(makeAction("episode_2", 0, 1, 1.0),
                         ros::Time(23, 0)).status);
  EXPECT_EQ(DynamicVmaxCommitStatus::STALE_IDENTITY,
            store.commit(makeAction("episode_1", 1, 2, 1.0),
                         ros::Time(24, 0)).status);
  ASSERT_TRUE(store.latest());
  EXPECT_EQ("episode_2", store.latest()->episode_id);
  EXPECT_EQ(2u, store.latest()->version);
}

TEST(DynamicVmaxSnapshotStore, UnchangedValueStillCommitsExactIdentity)
{
  DynamicVmaxSnapshotStore store(DynamicSpeedLimitGate(0.3, 1.75), 1.0);
  const DynamicVmaxCommitResult first =
      store.commit(makeAction("episode_1", 0, 1, 1.0), ros::Time(20, 0));
  const DynamicVmaxCommitResult second =
      store.commit(makeAction("episode_1", 1, 2, 1.0), ros::Time(21, 0));
  ASSERT_EQ(DynamicVmaxCommitStatus::ACCEPTED, first.status);
  ASSERT_EQ(DynamicVmaxCommitStatus::ACCEPTED, second.status);
  EXPECT_EQ(1u, first.snapshot->version);
  EXPECT_EQ(2u, second.snapshot->version);
  EXPECT_FALSE(first.snapshot->force_replan_intent);
  EXPECT_FALSE(second.snapshot->force_replan_intent);
  EXPECT_EQ(0u, second.snapshot->force_replan_generation);
  EXPECT_EQ(2u, second.snapshot->request_id);
}

TEST(DynamicVmaxSnapshotStore, AtomicReadersObserveWholeMonotonicSnapshots)
{
  DynamicVmaxSnapshotStore store(DynamicSpeedLimitGate(0.3, 1.75), 1.0);
  std::atomic<bool> writer_done(false);
  std::atomic<std::uint64_t> violations(0);
  std::vector<std::thread> readers;
  for (int reader = 0; reader < 4; ++reader)
  {
    readers.emplace_back([&store, &writer_done, &violations]() {
      std::uint64_t last_version = 0;
      while (!writer_done.load(std::memory_order_acquire))
      {
        const std::shared_ptr<const DynamicVmaxSnapshot> snapshot =
            store.latest();
        if (!snapshot || snapshot->version < last_version ||
            (snapshot->version > 0 &&
             (snapshot->request_id != snapshot->version ||
              snapshot->step_index + 1 != snapshot->request_id ||
              snapshot->episode_id != "stress")))
          violations.fetch_add(1, std::memory_order_relaxed);
        if (snapshot)
          last_version = snapshot->version;
      }
    });
  }

  for (std::uint64_t request = 1; request <= 1000; ++request)
  {
    const double value = 0.5 + 0.05 * static_cast<double>(request % 10);
    const DynamicVmaxCommitResult result = store.commit(
        makeAction("stress", request - 1, request, value),
        ros::Time(30, static_cast<std::uint32_t>(request)));
    ASSERT_EQ(DynamicVmaxCommitStatus::ACCEPTED, result.status);
  }
  writer_done.store(true, std::memory_order_release);
  for (std::thread &reader : readers)
    reader.join();

  EXPECT_EQ(0u, violations.load());
  ASSERT_TRUE(store.latest());
  EXPECT_EQ(1000u, store.latest()->version);
  EXPECT_EQ(1000u, store.latest()->request_id);
}

TEST(DynamicVmaxSnapshotStore, ShutdownRejectsFurtherCommit)
{
  DynamicVmaxSnapshotStore store(DynamicSpeedLimitGate(0.3, 1.75), 1.0);
  store.shutdown();
  EXPECT_EQ(DynamicVmaxCommitStatus::SHUTTING_DOWN,
            store.commit(makeAction("episode_1", 0, 1, 1.0),
                         ros::Time(20, 0)).status);
  ASSERT_TRUE(store.latest());
  EXPECT_EQ(0u, store.latest()->version);
}

}  // namespace
}  // namespace ego_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
