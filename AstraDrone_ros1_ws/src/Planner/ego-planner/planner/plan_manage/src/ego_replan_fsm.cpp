
#include <plan_manage/ego_replan_fsm.h>
#include <plan_manage/goal_height.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    flag_escape_emergency_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/manual_target_height", manual_target_height_, 1.0);
    nh.param("fsm/use_goal_height", use_goal_height_, false);
    nh.param("fsm/goal_velocity_timeout", goal_velocity_timeout_, 0.5);
    nh.param("fsm/goal_path_hint_timeout", goal_path_hint_timeout_, 0.5);
    nh.param("fsm/max_goal_velocity", max_goal_velocity_, 1.0);
    nh.param("fsm/replan_state_prediction_time",
             replan_state_prediction_time_, 0.10);
    nh.param<std::string>("fsm/status_topic", status_topic_, "/planner/status");
    nh.param<std::string>("fsm/cancel_topic", cancel_topic_, "/planning/cancel");
    nh.param<std::string>("fsm/status_frame_id", status_frame_id_, "camera_init");

    if (target_type_ != TARGET_TYPE::MANUAL_TARGET &&
        target_type_ != TARGET_TYPE::PRESET_TARGET)
    {
      ROS_FATAL("fsm/flight_type must be 1 (manual) or 2 (preset), got %d.", target_type_);
      ros::shutdown();
      return;
    }

    if (!std::isfinite(manual_target_height_) || manual_target_height_ <= 0.0)
    {
      ROS_WARN("Invalid fsm/manual_target_height=%.3f; using 1.0 m.", manual_target_height_);
      manual_target_height_ = 1.0;
    }
    if (!std::isfinite(goal_velocity_timeout_) ||
        goal_velocity_timeout_ <= 0.0 ||
        !std::isfinite(goal_path_hint_timeout_) ||
        goal_path_hint_timeout_ <= 0.0 ||
        !std::isfinite(max_goal_velocity_) || max_goal_velocity_ <= 0.0 ||
        !std::isfinite(replan_state_prediction_time_) ||
        replan_state_prediction_time_ < 0.0)
    {
      ROS_FATAL("Invalid goal velocity or replan prediction parameter.");
      ros::shutdown();
      return;
    }
    ROS_INFO("Manual waypoint height source: %s.",
             use_goal_height_ ? "incoming goal z" : "fsm/manual_target_height");

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    if (waypoint_num_ < 0 || waypoint_num_ > 50)
    {
      ROS_FATAL("fsm/waypoint_num must be in [0, 50], got %d.", waypoint_num_);
      ros::shutdown();
      return;
    }
    if (target_type_ == TARGET_TYPE::PRESET_TARGET && waypoint_num_ == 0)
    {
      ROS_FATAL("Preset flight requires at least one waypoint.");
      ros::shutdown();
      return;
    }
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
      if (!std::isfinite(waypoints_[i][0]) ||
          !std::isfinite(waypoints_[i][1]) ||
          !std::isfinite(waypoints_[i][2]))
      {
        ROS_FATAL("Waypoint %d contains NaN/Inf.", i);
        ros::shutdown();
        return;
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);
    status_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::statusCallback, this);

    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    cancel_sub_ = nh.subscribe(cancel_topic_, 1, &EGOReplanFSM::cancelCallback, this);
    goal_velocity_sub_ = nh.subscribe(
        "/planning/goal_velocity", 10,
        &EGOReplanFSM::goalVelocityCallback, this);
    goal_path_hint_sub_ = nh.subscribe(
        "/planning/goal_path_hint", 10,
        &EGOReplanFSM::goalPathHintCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);
    status_pub_ = nh.advertise<astra_custom_msgs::PlannerStatus>(status_topic_, 10, true);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];
    }
    end_pt_ = wps.back();
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    recordPlanningResult(success, astra_custom_msgs::PlannerStatus::NO_FEASIBLE_TRAJECTORY);

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!cancel_time_.isZero() &&
        (msg->header.stamp.isZero() || msg->header.stamp <= cancel_time_))
    {
      ROS_WARN("Ignoring waypoint generated before the latest trajectory cancellation.");
      return;
    }
    if (!have_odom_)
    {
      ROS_WARN("Ignoring waypoint until valid odometry is available.");
      return;
    }
    if (msg->poses.empty())
    {
      ROS_WARN("Ignoring empty waypoint path.");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    const auto &goal = msg->poses[0].pose.position;
    double target_height = 0.0;
    if (!std::isfinite(goal.x) || !std::isfinite(goal.y) ||
        !resolveManualGoalHeight(goal.z, manual_target_height_,
                                 use_goal_height_, &target_height))
    {
      ROS_ERROR("Ignoring waypoint with invalid coordinates or target height.");
      return;
    }

    cout << "Triggered!" << endl;
    target_id_ = "goal_" + std::to_string(++target_sequence_);
    status_tracker_.reset(astra_custom_msgs::PlannerStatus::NONE);
    trigger_ = true;
    init_pt_ = odom_pos_;

    const ros::Time now = ros::Time::now();
    end_vel_.setZero();
    if (have_goal_velocity_hint_ &&
        !goal_velocity_received_.isZero() &&
        now >= goal_velocity_received_ &&
        now - goal_velocity_received_ <=
            ros::Duration(goal_velocity_timeout_))
    {
      end_vel_ = goal_velocity_hint_;
    }
    // A velocity hint belongs to exactly one goal. Consuming it prevents the
    // final bridge-generated home hover from inheriting the previous EXIT
    // tangent if no explicit hint accompanies that goal.
    have_goal_velocity_hint_ = false;

    bool success = false;
    end_pt_ << goal.x, goal.y, target_height;
    std::vector<Eigen::Vector3d> path_waypoints;
    if (have_goal_path_hint_ &&
        !goal_path_hint_received_.isZero() &&
        now >= goal_path_hint_received_ &&
        now - goal_path_hint_received_ <=
            ros::Duration(goal_path_hint_timeout_) &&
        goal_path_hint_.poses.size() >= 2U)
    {
      const auto &hint_end = goal_path_hint_.poses.back().pose.position;
      const Eigen::Vector3d hint_end_point(
          hint_end.x, hint_end.y, hint_end.z);
      if ((hint_end_point - end_pt_).norm() <= 0.5)
      {
        for (const auto &pose : goal_path_hint_.poses)
        {
          const auto &point = pose.pose.position;
          Eigen::Vector3d waypoint(point.x, point.y, point.z);
          if (!waypoint.allFinite())
          {
            path_waypoints.clear();
            break;
          }
          if ((waypoint - odom_pos_).norm() > 0.20)
            path_waypoints.push_back(waypoint);
        }
        if (path_waypoints.empty() ||
            (path_waypoints.back() - end_pt_).norm() > 0.5)
          path_waypoints.clear();
        else
          path_waypoints.back() = end_pt_;
      }
    }
    have_goal_path_hint_ = false;
    if (!path_waypoints.empty())
    {
      success = planner_manager_->planGlobalTrajWaypoints(
          odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), path_waypoints,
          end_vel_, Eigen::Vector3d::Zero());
      ROS_INFO("EGO consumed global path hint with %zu points for current goal.",
               path_waypoints.size());
    }
    else
    {
      success = planner_manager_->planGlobalTraj(
          odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, end_vel_,
          Eigen::Vector3d::Zero());
    }
    recordPlanningResult(success, astra_custom_msgs::PlannerStatus::NO_FEASIBLE_TRAJECTORY);

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;
      ROS_INFO("EGO goal terminal velocity=(%.3f, %.3f, %.3f).",
               end_vel_.x(), end_vel_.y(), end_vel_.z());

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::goalVelocityCallback(
      const geometry_msgs::TwistStampedConstPtr &msg)
  {
    const auto &velocity = msg->twist.linear;
    if (msg->header.stamp.isZero() ||
        msg->header.frame_id != status_frame_id_ ||
        !std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
        !std::isfinite(velocity.z))
    {
      ROS_WARN("Ignoring invalid EGO goal velocity hint.");
      return;
    }
    goal_velocity_hint_ << velocity.x, velocity.y, velocity.z;
    const double speed = goal_velocity_hint_.norm();
    if (speed > max_goal_velocity_)
      goal_velocity_hint_ *= max_goal_velocity_ / speed;
    goal_velocity_received_ = ros::Time::now();
    have_goal_velocity_hint_ = true;
  }

  void EGOReplanFSM::goalPathHintCallback(
      const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses.size() < 2U ||
        msg->header.frame_id != status_frame_id_)
    {
      ROS_WARN_THROTTLE(
          1.0, "Ignoring invalid EGO global path hint (poses=%zu frame=%s).",
          msg->poses.size(), msg->header.frame_id.c_str());
      return;
    }
    goal_path_hint_ = *msg;
    goal_path_hint_received_ = ros::Time::now();
    have_goal_path_hint_ = true;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    const auto &position = msg->pose.pose.position;
    const auto &orientation = msg->pose.pose.orientation;
    const auto &velocity = msg->twist.twist.linear;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(velocity.x) ||
        !std::isfinite(velocity.y) || !std::isfinite(velocity.z) ||
        !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
        !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
    {
      ROS_ERROR_THROTTLE(1.0, "Ignoring odometry containing NaN/Inf.");
      return;
    }

    odom_pos_(0) = position.x;
    odom_pos_(1) = position.y;
    odom_pos_(2) = position.z;

    odom_vel_(0) = velocity.x;
    odom_vel_(1) = velocity.y;
    odom_vel_(2) = velocity.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = orientation.w;
    odom_orient_.x() = orientation.x;
    odom_orient_.y() = orientation.y;
    odom_orient_.z() = orientation.z;

    have_odom_ = true;
  }

  const char *EGOReplanFSM::stateName() const
  {
    static const char *names[] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ",
                                  "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    return names[static_cast<int>(exec_state_)];
  }

  void EGOReplanFSM::recordPlanningResult(bool success,
                                          const std::string &failure_reason)
  {
    status_tracker_.recordAttempt(
        success, failure_reason,
        static_cast<std::uint32_t>(
            std::max(0, planner_manager_->local_data_.traj_id_)));
  }

  void EGOReplanFSM::cancelCallback(const std_msgs::EmptyConstPtr &)
  {
    cancel_time_ = ros::Time::now();
    have_target_ = false;
    have_new_target_ = false;
    flag_escape_emergency_ = false;
    status_tracker_.reset(astra_custom_msgs::PlannerStatus::NONE);
    emergency_since_ = ros::Time(0);
    changeFSMExecState(WAIT_TARGET, "CANCEL");
    ROS_WARN("EGO FSM trajectory cancelled; waiting for a new target.");
  }

  void EGOReplanFSM::statusCallback(const ros::TimerEvent &)
  {
    const ros::Time now = ros::Time::now();
    astra_custom_msgs::PlannerStatus status;
    status.header.stamp = now;
    status.header.frame_id = status_frame_id_;
    status.planner_state = stateName();
    status.target_id = target_id_;
    status.trajectory_id = status_tracker_.trajectoryId();
    status.last_plan_success = status_tracker_.lastSuccess();
    status.consecutive_plan_failures = status_tracker_.consecutiveFailures();
    const bool have_map = planner_manager_ != nullptr &&
                          planner_manager_->grid_map_ != nullptr;
    status.goal_in_collision = have_map && have_target_ &&
        planner_manager_->grid_map_->getInflateOccupancy(end_pt_);
    status.current_position_in_collision = have_map && have_odom_ &&
        planner_manager_->grid_map_->getInflateOccupancy(odom_pos_);
    status.emergency_stop_active = exec_state_ == EMERGENCY_STOP;
    status.emergency_stop_duration = status.emergency_stop_active &&
        !emergency_since_.isZero()
        ? static_cast<float>((now - emergency_since_).toSec())
        : 0.0F;
    status.failure_reason = status_tracker_.failureReason().empty()
                                ? astra_custom_msgs::PlannerStatus::NONE
                                : status_tracker_.failureReason();
    if (status.current_position_in_collision)
      status.failure_reason = astra_custom_msgs::PlannerStatus::CURRENT_POSITION_IN_OCCUPANCY;
    else if (status.goal_in_collision)
      status.failure_reason = astra_custom_msgs::PlannerStatus::GOAL_IN_OCCUPANCY;
    status.status_timestamp = now;
    status_pub_.publish(status);
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    if (new_state == EMERGENCY_STOP && emergency_since_.isZero())
      emergency_since_ = ros::Time::now();
    else if (new_state != EMERGENCY_STOP)
      emergency_since_ = ros::Time(0);
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      previous_traj_replan_time_ = -1.0;

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    t_cur = std::max(0.0, std::min(
        info->duration_, t_cur + replan_state_prediction_time_));

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    previous_traj_replan_time_ = t_cur;

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            status_tracker_.setEventReason(
                astra_custom_msgs::PlannerStatus::REPLAN_FAILED);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(
            start_pt_, start_vel_, start_acc_, local_target_pt_,
            local_target_vel_, (have_new_target_ || flag_use_poly_init),
            flag_randomPolyTraj, previous_traj_replan_time_);
    have_new_target_ = false;

    recordPlanningResult(
        plan_success,
        exec_state_ == GEN_NEW_TRAJ
            ? astra_custom_msgs::PlannerStatus::NO_FEASIBLE_TRAJECTORY
            : astra_custom_msgs::PlannerStatus::REPLAN_FAILED);

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = end_vel_;
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

} // namespace ego_planner
