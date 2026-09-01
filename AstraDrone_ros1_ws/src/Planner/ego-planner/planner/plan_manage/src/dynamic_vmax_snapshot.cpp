#include <plan_manage/dynamic_vmax_snapshot.h>

#include <boost/bind/bind.hpp>
#include <limits>

namespace ego_planner
{

DynamicVmaxSnapshotStore::DynamicVmaxSnapshotStore(
    const DynamicSpeedLimitGate &gate, double initial_v_max)
    : gate_(gate)
{
  std::shared_ptr<DynamicVmaxSnapshot> initial(new DynamicVmaxSnapshot);
  initial->v_max = initial_v_max;
  std::atomic_store_explicit(
      &latest_snapshot_,
      std::shared_ptr<const DynamicVmaxSnapshot>(initial),
      std::memory_order_release);
}

DynamicVmaxCommitResult DynamicVmaxSnapshotStore::commit(
    const learning_speed_rl::SpeedActionStamped &action,
    const ros::Time &apply_ros_time)
{
  DynamicVmaxCommitResult result;
  if (shutting_down_.load(std::memory_order_acquire))
  {
    result.status = DynamicVmaxCommitStatus::SHUTTING_DOWN;
    return result;
  }

  if (action.version != "learning_speed_action_v1.1" ||
      action.episode_id.empty() || action.request_id == 0 ||
      action.header.stamp.isZero() ||
      !std::isfinite(action.requested_v_max) ||
      apply_ros_time.isZero())
  {
    result.status = DynamicVmaxCommitStatus::INVALID_ACTION;
    return result;
  }
  if (!gate_.accepts(action.filtered_v_max))
  {
    result.status = DynamicVmaxCommitStatus::OUT_OF_RANGE;
    return result;
  }

  std::lock_guard<std::mutex> guard(writer_mutex_);
  if (shutting_down_.load(std::memory_order_relaxed))
  {
    result.status = DynamicVmaxCommitStatus::SHUTTING_DOWN;
    return result;
  }

  if ((!active_episode_id_.empty() &&
       action.episode_id != active_episode_id_ &&
       closed_episode_ids_.count(action.episode_id) != 0) ||
      (last_request_id_by_episode_.count(action.episode_id) != 0 &&
       action.request_id <=
           last_request_id_by_episode_.at(action.episode_id)) ||
      (last_step_by_episode_.count(action.episode_id) != 0 &&
       action.step_index <= last_step_by_episode_.at(action.episode_id)))
  {
    result.status = DynamicVmaxCommitStatus::STALE_IDENTITY;
    return result;
  }
  if (next_version_ == std::numeric_limits<std::uint64_t>::max())
  {
    result.status = DynamicVmaxCommitStatus::INVALID_ACTION;
    return result;
  }

  const std::shared_ptr<const DynamicVmaxSnapshot> previous =
      std::atomic_load_explicit(&latest_snapshot_, std::memory_order_acquire);
  if (!previous || !gate_.accepts(previous->v_max))
  {
    result.status = DynamicVmaxCommitStatus::INVALID_ACTION;
    return result;
  }

  const bool force_replan = gate_.requiresForceReplan(
      previous->v_max, action.filtered_v_max);
  if (force_replan &&
      force_replan_generation_ == std::numeric_limits<std::uint64_t>::max())
  {
    result.status = DynamicVmaxCommitStatus::INVALID_ACTION;
    return result;
  }

  if (active_episode_id_.empty())
    active_episode_id_ = action.episode_id;
  else if (action.episode_id != active_episode_id_)
  {
    closed_episode_ids_.insert(active_episode_id_);
    active_episode_id_ = action.episode_id;
  }

  if (force_replan)
    ++force_replan_generation_;

  std::shared_ptr<DynamicVmaxSnapshot> committed(new DynamicVmaxSnapshot);
  committed->version = next_version_++;
  committed->v_max = action.filtered_v_max;
  committed->episode_id = action.episode_id;
  committed->step_index = action.step_index;
  committed->request_id = action.request_id;
  committed->source_header = action.header;
  const ros::Time minimum_apply_stamp =
      action.header.stamp + ros::Duration(0, 1);
  committed->apply_ros_stamp =
      apply_ros_time < minimum_apply_stamp ? minimum_apply_stamp
                                           : apply_ros_time;
  committed->force_replan_generation = force_replan_generation_;
  committed->force_replan_intent = force_replan;

  last_request_id_by_episode_[action.episode_id] = action.request_id;
  last_step_by_episode_[action.episode_id] = action.step_index;
  result.previous_v_max = previous->v_max;
  result.snapshot = committed;

  // Linearization point: after this release-store every later planner
  // admission can observe the complete immutable configuration.
  std::atomic_store_explicit(
      &latest_snapshot_,
      std::shared_ptr<const DynamicVmaxSnapshot>(committed),
      std::memory_order_release);
  result.status = DynamicVmaxCommitStatus::ACCEPTED;
  return result;
}

std::shared_ptr<const DynamicVmaxSnapshot>
DynamicVmaxSnapshotStore::latest() const
{
  return std::atomic_load_explicit(
      &latest_snapshot_, std::memory_order_acquire);
}

void DynamicVmaxSnapshotStore::shutdown()
{
  shutting_down_.store(true, std::memory_order_release);
}

DedicatedSpeedLimitChannel::~DedicatedSpeedLimitChannel()
{
  stop();
}

bool DedicatedSpeedLimitChannel::start(
    ros::NodeHandle &node, const std::string &action_topic,
    const std::string &applied_topic,
    const std::string &applied_stamped_topic,
    const DynamicSpeedLimitGate &gate, double initial_v_max,
    std::uint32_t queue_size)
{
  if (running() || !gate.validConfiguration() ||
      !gate.accepts(initial_v_max) || action_topic.empty() ||
      applied_topic.empty() || applied_stamped_topic.empty() ||
      queue_size == 0)
    return false;

  callback_queue_.enable();
  store_.reset(new DynamicVmaxSnapshotStore(gate, initial_v_max));
  applied_publisher_ =
      node.advertise<std_msgs::Float64>(applied_topic, 1, true);
  applied_stamped_publisher_ =
      node.advertise<learning_speed_rl::SpeedAppliedStamped>(
          applied_stamped_topic, queue_size, false);

  ros::SubscribeOptions options =
      ros::SubscribeOptions::create<learning_speed_rl::SpeedActionStamped>(
          action_topic, queue_size,
          boost::bind(&DedicatedSpeedLimitChannel::speedLimitCallback,
                      this, boost::placeholders::_1),
          ros::VoidPtr(), &callback_queue_);
  options.transport_hints = ros::TransportHints().tcpNoDelay();
  action_subscriber_ = node.subscribe(options);
  if (!action_subscriber_)
  {
    stop();
    return false;
  }

  running_.store(true, std::memory_order_release);
  spinner_.reset(new ros::AsyncSpinner(1, &callback_queue_));
  spinner_->start();

  std_msgs::Float64 initial;
  initial.data = initial_v_max;
  applied_publisher_.publish(initial);
  return true;
}

void DedicatedSpeedLimitChannel::stop()
{
  if (!running_.exchange(false, std::memory_order_acq_rel) && !store_)
    return;

  action_subscriber_.shutdown();
  if (spinner_)
  {
    spinner_->stop();
    spinner_.reset();
  }
  callback_queue_.disable();
  callback_queue_.clear();
  if (store_)
    store_->shutdown();
  applied_stamped_publisher_.shutdown();
  applied_publisher_.shutdown();
}

std::shared_ptr<const DynamicVmaxSnapshot>
DedicatedSpeedLimitChannel::latestSnapshot() const
{
  return store_ ? store_->latest()
                : std::shared_ptr<const DynamicVmaxSnapshot>();
}

void DedicatedSpeedLimitChannel::speedLimitCallback(
    const learning_speed_rl::SpeedActionStampedConstPtr &action)
{
  if (!running() || !store_ || !action)
    return;

  const DynamicVmaxCommitResult result =
      store_->commit(*action, ros::Time::now());
  switch (result.status)
  {
    case DynamicVmaxCommitStatus::ACCEPTED:
      publishApplied(result.snapshot);
      if (result.snapshot->force_replan_intent)
      {
        ROS_WARN("[EGO SPEED SNAPSHOT] committed version=%llu identity=%s/%llu/%llu "
                 "v_max=%.6f previous=%.6f force_generation=%llu.",
                 static_cast<unsigned long long>(result.snapshot->version),
                 result.snapshot->episode_id.c_str(),
                 static_cast<unsigned long long>(result.snapshot->step_index),
                 static_cast<unsigned long long>(result.snapshot->request_id),
                 result.snapshot->v_max, result.previous_v_max,
                 static_cast<unsigned long long>(
                     result.snapshot->force_replan_generation));
      }
      return;
    case DynamicVmaxCommitStatus::STALE_IDENTITY:
      ROS_ERROR("[EGO SPEED SNAPSHOT] rejected stale/duplicate identity "
                "episode=%s step=%llu request_id=%llu.",
                action->episode_id.c_str(),
                static_cast<unsigned long long>(action->step_index),
                static_cast<unsigned long long>(action->request_id));
      return;
    case DynamicVmaxCommitStatus::OUT_OF_RANGE:
      ROS_ERROR_THROTTLE(1.0,
                         "[EGO SPEED SNAPSHOT] rejected out-of-range filtered "
                         "v_max %.6f.",
                         action->filtered_v_max);
      return;
    case DynamicVmaxCommitStatus::INVALID_ACTION:
      ROS_ERROR_THROTTLE(
          1.0, "[EGO SPEED SNAPSHOT] rejected invalid stamped speed action.");
      return;
    case DynamicVmaxCommitStatus::SHUTTING_DOWN:
      return;
  }
}

void DedicatedSpeedLimitChannel::publishApplied(
    const std::shared_ptr<const DynamicVmaxSnapshot> &snapshot)
{
  if (!snapshot)
    return;

  std_msgs::Float64 scalar;
  scalar.data = snapshot->v_max;
  applied_publisher_.publish(scalar);

  learning_speed_rl::SpeedAppliedStamped applied;
  applied.header = snapshot->source_header;
  applied.header.stamp = snapshot->apply_ros_stamp;
  applied.version = "learning_speed_applied_v1.0";
  applied.episode_id = snapshot->episode_id;
  applied.step_index = snapshot->step_index;
  applied.request_id = snapshot->request_id;
  applied.applied_v_max = snapshot->v_max;
  applied_stamped_publisher_.publish(applied);
}

}  // namespace ego_planner
