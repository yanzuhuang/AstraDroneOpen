#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "ego_planner/Bspline.h"
#include "plan_manage/trajectory_cancellation_gate.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

#include <cmath>

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ego_planner::TrajectoryCancellationGate cancellation_gate_;

void cancelCallback(const std_msgs::EmptyConstPtr &)
{
  cancellation_gate_.noteCancel(
      ros::Time::now(), receive_traj_,
      receive_traj_ ? static_cast<std::uint32_t>(traj_id_) : 0U);
  receive_traj_ = false;
  traj_.clear();
  ROS_WARN("[Traj server]: active trajectory cancelled; PositionCommand publication stopped.");
}

void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!cancellation_gate_.noteGoal(msg->header.stamp))
  {
    ROS_WARN("[Traj server]: rejected goal with zero, stale, or non-increasing timestamp %.9f.",
             msg->header.stamp.toSec());
  }
}

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;
std::string command_frame_id_;

void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  if (!cancellation_gate_.acceptsTrajectory(
          msg->traj_id, msg->start_time))
  {
    ROS_WARN("[Traj server]: rejected cancelled or pre-goal B-spline trajectory %u (start %.9f).",
             static_cast<unsigned int>(msg->traj_id),
             msg->start_time.toSec());
    return;
  }
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  constexpr double MIN_YAW_DT = 1.0e-6;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  // On the first timer callback, time_last may be initialized from the same
  // simulated clock sample as time_now. Dividing a zero yaw error by that
  // zero interval produces NaN and permanently contaminates last_yaw_dot_.
  const double dt = (time_now - time_last).toSec();
  if (!std::isfinite(dt) || dt <= MIN_YAW_DT)
  {
    return std::make_pair(last_yaw_, 0.0);
  }

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = command_frame_id_;
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  std::string bspline_topic;
  std::string cancel_topic;
  std::string goal_topic;
  std::string position_command_topic;
  nh.param<std::string>("traj_server/bspline_topic", bspline_topic,
                        "planning/bspline");
  nh.param<std::string>("traj_server/cancel_topic", cancel_topic,
                        "planning/cancel");
  nh.param<std::string>("traj_server/goal_topic", goal_topic,
                        "planning/goal");
  nh.param<std::string>("traj_server/position_command_topic",
                        position_command_topic, "planning/pos_cmd");
  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param<std::string>("traj_server/frame_id", command_frame_id_, "world");

  if (bspline_topic.empty() || cancel_topic.empty() || goal_topic.empty() ||
      position_command_topic.empty())
  {
    ROS_FATAL("[Traj server]: topic parameters must not be empty.");
    return 1;
  }
  if (!std::isfinite(time_forward_) || time_forward_ < 0.0)
  {
    ROS_FATAL("[Traj server]: traj_server/time_forward must be finite and non-negative.");
    return 1;
  }
  if (command_frame_id_.empty())
  {
    ROS_FATAL("[Traj server]: traj_server/frame_id must not be empty.");
    return 1;
  }

  ros::Subscriber bspline_sub =
      node.subscribe(bspline_topic, 10, bsplineCallback);
  ros::Subscriber cancel_sub =
      node.subscribe(cancel_topic, 1, cancelCallback);
  ros::Subscriber goal_sub =
      node.subscribe(goal_topic, 1, goalCallback);

  pos_cmd_pub =
      node.advertise<quadrotor_msgs::PositionCommand>(
          position_command_topic, 50);

  ros::Timer cmd_timer =
      node.createTimer(ros::Duration(0.01), cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
