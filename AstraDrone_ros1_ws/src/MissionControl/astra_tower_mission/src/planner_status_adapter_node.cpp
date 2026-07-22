#include <astra_custom_msgs/PlannerStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace astra_tower_mission {
namespace {

struct Point {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

double distance(const Point& a, const Point& b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) +
                   (a.y - b.y) * (a.y - b.y) +
                   (a.z - b.z) * (a.z - b.z));
}

class PlannerStatusAdapter {
 public:
  PlannerStatusAdapter()
      : node_(), private_node_("~") {
    private_node_.param<std::string>("command_topic", command_topic_,
                                     "/planning/pos_cmd");
    private_node_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    private_node_.param<std::string>("goal_topic", goal_topic_,
                                     "/planning/goal");
    private_node_.param<std::string>("cloud_topic", cloud_topic_,
                                     "/stage2/cloud_registered_filtered");
    private_node_.param<std::string>("occupancy_topic", occupancy_topic_,
                                     "/grid_map/occupancy_inflate");
    private_node_.param<std::string>("status_topic", status_topic_,
                                     "/planner/status");
    private_node_.param<std::string>("planning_frame", planning_frame_,
                                     "camera_init");
    private_node_.param("loop_rate", loop_rate_, 20.0);
    private_node_.param("input_timeout", input_timeout_, 0.7);
    private_node_.param("planning_timeout", planning_timeout_, 10.0);
    private_node_.param("emergency_stop_timeout", emergency_stop_timeout_,
                        3.0);
    private_node_.param("progress_window", progress_window_, 8.0);
    private_node_.param("progress_epsilon", progress_epsilon_, 0.15);
    private_node_.param("collision_radius", collision_radius_, 0.55);
    private_node_.param("cloud_timeout", cloud_timeout_, 0.7);
    command_sub_ = node_.subscribe(command_topic_, 50,
                                   &PlannerStatusAdapter::commandCallback, this);
    odom_sub_ = node_.subscribe(odom_topic_, 20,
                                &PlannerStatusAdapter::odomCallback, this);
    goal_sub_ = node_.subscribe(goal_topic_, 10,
                                &PlannerStatusAdapter::goalCallback, this);
    cloud_sub_ = node_.subscribe(cloud_topic_, 5,
                                 &PlannerStatusAdapter::cloudCallback, this);
    occupancy_sub_ = node_.subscribe(
        occupancy_topic_, 5, &PlannerStatusAdapter::occupancyCallback, this);
    status_pub_ = node_.advertise<astra_custom_msgs::PlannerStatus>(
        status_topic_, 10, true);
    timer_ = node_.createTimer(ros::Duration(1.0 / std::max(1.0, loop_rate_)),
                               &PlannerStatusAdapter::timerCallback, this);
  }

 private:
  void commandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    if (msg->header.frame_id != planning_frame_ || msg->header.stamp.isZero() ||
        !std::isfinite(msg->position.x) || !std::isfinite(msg->position.y) ||
        !std::isfinite(msg->position.z)) return;
    command_ = *msg;
    have_command_ = true;
    command_received_ = ros::Time::now();
    if (last_trajectory_id_ != msg->trajectory_id) {
      last_trajectory_id_ = msg->trajectory_id;
      last_trajectory_change_ = ros::Time::now();
      consecutive_failures_ = 0;
      last_plan_success_ = true;
      emergency_since_ = ros::Time(0);
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (msg->header.stamp.isZero() || msg->header.frame_id != planning_frame_) return;
    odom_ = *msg;
    have_odom_ = true;
    odom_received_ = ros::Time::now();
    current_ = {msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->pose.pose.position.z};
    if (have_goal_) {
      const Point goal{goal_.pose.position.x, goal_.pose.position.y,
                       goal_.pose.position.z};
      const double progress = distance(last_position_, goal) -
                               distance(current_, goal);
      if (last_progress_time_.isZero() || progress > progress_epsilon_) {
        last_progress_time_ = ros::Time::now();
        last_position_ = current_;
      }
    } else {
      last_position_ = current_;
    }
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (msg->header.frame_id != planning_frame_ || msg->header.stamp.isZero()) return;
    goal_ = *msg;
    have_goal_ = true;
    goal_received_ = ros::Time::now();
    target_id_ = "goal_" + std::to_string(++goal_sequence_);
    last_progress_time_ = ros::Time::now();
    if (have_odom_) last_position_ = current_;
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.stamp.isZero() || msg->header.frame_id != planning_frame_) return;
    points_.clear();
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      for (; x != x.end() && points_.size() < 200000U; ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          points_.push_back({*x, *y, *z});
        }
      }
    } catch (const std::exception&) {
      points_.clear();
    }
    cloud_received_ = ros::Time::now();
    have_cloud_ = !points_.empty();
  }

  void occupancyCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.stamp.isZero() || msg->header.frame_id != planning_frame_) return;
    occupancy_points_.clear();
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      for (; x != x.end() && occupancy_points_.size() < 300000U;
           ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          occupancy_points_.push_back({*x, *y, *z});
        }
      }
    } catch (const std::exception&) {
      occupancy_points_.clear();
    }
    occupancy_received_ = ros::Time::now();
    have_occupancy_ = true;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    astra_custom_msgs::PlannerStatus status;
    status.header.stamp = now;
    status.header.frame_id = have_odom_ ? odom_.header.frame_id : "";
    status.target_id = target_id_;
    status.trajectory_id = last_trajectory_id_;
    status.last_plan_success = last_plan_success_;
    status.consecutive_plan_failures = consecutive_failures_;
    status.status_timestamp = now;
    const bool map_fresh = have_cloud_ && have_occupancy_ &&
                           !cloud_received_.isZero() &&
                           !occupancy_received_.isZero() &&
                           now - cloud_received_ <= ros::Duration(cloud_timeout_) &&
                           now - occupancy_received_ <= ros::Duration(cloud_timeout_);
    const std::vector<Point>& map_points = occupancy_points_.empty()
                                               ? points_
                                               : occupancy_points_;
    const bool odom_fresh = have_odom_ && now - odom_received_ <=
                                           ros::Duration(input_timeout_);
    status.current_position_in_collision = false;
    if (map_fresh && odom_fresh) {
      for (const auto& point : map_points) {
        if (distance(current_, point) < collision_radius_) {
          status.current_position_in_collision = true;
          break;
        }
      }
    }
    status.goal_in_collision = false;
    if (map_fresh && have_goal_) {
      const Point goal{goal_.pose.position.x, goal_.pose.position.y,
                       goal_.pose.position.z};
      for (const auto& point : map_points) {
        if (distance(goal, point) < collision_radius_) {
          status.goal_in_collision = true;
          break;
        }
      }
    }
    status.emergency_stop_active = false;
    status.emergency_stop_duration = 0.0F;
    std::string reason = astra_custom_msgs::PlannerStatus::NONE;
    status.planner_state = "WAIT_TARGET";
    if (!map_fresh) {
      reason = astra_custom_msgs::PlannerStatus::MAP_STALE;
      status.planner_state = "MAP_STALE";
    } else if (status.current_position_in_collision) {
      reason = astra_custom_msgs::PlannerStatus::CURRENT_POSITION_IN_OCCUPANCY;
      status.planner_state = "EMERGENCY_STOP";
      status.emergency_stop_active = true;
    } else if (status.goal_in_collision) {
      reason = astra_custom_msgs::PlannerStatus::GOAL_IN_OCCUPANCY;
      status.planner_state = "GOAL_IN_OCCUPANCY";
      ++consecutive_failures_;
      last_plan_success_ = false;
    } else if (have_goal_ && have_command_ &&
               now - command_received_ <= ros::Duration(input_timeout_)) {
      const double command_speed = std::sqrt(
          command_.velocity.x * command_.velocity.x +
          command_.velocity.y * command_.velocity.y +
          command_.velocity.z * command_.velocity.z);
      const bool no_progress =
          command_speed < 0.05 && !last_progress_time_.isZero() &&
          now - last_progress_time_ >= ros::Duration(progress_window_);
      if (no_progress) {
        if (emergency_since_.isZero()) emergency_since_ = now;
        status.emergency_stop_active = true;
        status.planner_state = "EMERGENCY_STOP";
        status.emergency_stop_duration =
            static_cast<float>((now - emergency_since_).toSec());
        reason = status.emergency_stop_duration >= emergency_stop_timeout_
                     ? astra_custom_msgs::PlannerStatus::EMERGENCY_STOP_TIMEOUT
                     : astra_custom_msgs::PlannerStatus::REPLAN_FAILED;
        last_plan_success_ = false;
        ++consecutive_failures_;
      } else {
        status.planner_state = "EXEC_TRAJ";
        reason = astra_custom_msgs::PlannerStatus::NONE;
        last_plan_success_ = true;
        emergency_since_ = ros::Time(0);
      }
    } else if (have_goal_ &&
               now - goal_received_ >= ros::Duration(planning_timeout_)) {
      ++consecutive_failures_;
      last_plan_success_ = false;
      status.planner_state = "REPLAN_TRAJ";
      reason = astra_custom_msgs::PlannerStatus::NO_FEASIBLE_TRAJECTORY;
      if (!last_progress_time_.isZero() &&
          now - last_progress_time_ >= ros::Duration(progress_window_)) {
        if (emergency_since_.isZero()) emergency_since_ = now;
        status.emergency_stop_active = true;
        status.planner_state = "EMERGENCY_STOP";
        status.emergency_stop_duration =
            static_cast<float>((now - emergency_since_).toSec());
        reason = status.emergency_stop_duration >= emergency_stop_timeout_
                     ? astra_custom_msgs::PlannerStatus::EMERGENCY_STOP_TIMEOUT
                     : astra_custom_msgs::PlannerStatus::REPLAN_FAILED;
      }
    }
    status.failure_reason = reason;
    status_pub_.publish(status);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber command_sub_, odom_sub_, goal_sub_, cloud_sub_, occupancy_sub_;
  ros::Publisher status_pub_;
  ros::Timer timer_;
  std::string command_topic_, odom_topic_, goal_topic_, cloud_topic_, occupancy_topic_,
      status_topic_, planning_frame_;
  double loop_rate_{20.0};
  double input_timeout_{0.7};
  double planning_timeout_{10.0};
  double emergency_stop_timeout_{3.0};
  double progress_window_{8.0};
  double progress_epsilon_{0.15};
  double collision_radius_{0.55};
  double cloud_timeout_{0.7};
  quadrotor_msgs::PositionCommand command_;
  nav_msgs::Odometry odom_;
  geometry_msgs::PoseStamped goal_;
  bool have_command_{false}, have_odom_{false}, have_goal_{false},
      have_cloud_{false}, have_occupancy_{false};
  ros::Time command_received_, odom_received_, goal_received_, cloud_received_, occupancy_received_;
  ros::Time last_trajectory_change_, last_progress_time_, emergency_since_;
  std::uint32_t last_trajectory_id_{0}, goal_sequence_{0}, consecutive_failures_{0};
  bool last_plan_success_{false};
  std::string target_id_;
  Point current_, last_position_;
  std::vector<Point> points_, occupancy_points_;
};

}  // namespace
}  // namespace astra_tower_mission

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_status_adapter");
  astra_tower_mission::PlannerStatusAdapter adapter;
  ros::spin();
  return 0;
}
