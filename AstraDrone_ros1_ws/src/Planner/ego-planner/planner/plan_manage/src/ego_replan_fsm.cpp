
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    status_tracker_.reset(astra_custom_msgs::PlannerStatus::NONE);

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/manual_target_height", manual_target_height_, 1.0);
    nh.param("fsm/use_goal_height", use_goal_height_, true);
    if (!std::isfinite(manual_target_height_) || manual_target_height_ <= 0.0)
    {
      ROS_FATAL("[EGO FSM] fsm/manual_target_height must be finite and positive.");
      manual_target_height_ = 1.0;
    }

    have_trigger_ = !flag_realworld_experiment_;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();
    if (planner_manager_->pp_.drone_id < 0)
    {
      ROS_FATAL("[EGO FSM] manager/drone_id must be non-negative.");
      return;
    }

    nh.param<std::string>("topics/odom", odom_topic_, "odom_world");
    nh.param<std::string>("topics/goal", waypoint_topic_, "/move_base_simple/goal");
    nh.param<std::string>("topics/cancel", cancel_topic_, "planning/cancel");
    nh.param<std::string>("topics/swarm_trajectories", swarm_trajectory_topic_,
                          "/swarm/trajectories");
    nh.param<std::string>("topics/status", status_topic_, "planner/status");
    nh.param<std::string>("status/frame_id", status_frame_id_, "camera_init");
    nh.param<std::string>("status/target_id", target_id_, "ego");
    nh.param<std::string>("swarm/common_frame", swarm_common_frame_, "world");
    double swarm_origin_x = 0.0;
    double swarm_origin_y = 0.0;
    double swarm_origin_z = 0.0;
    double swarm_origin_yaw = 0.0;
    nh.param("swarm/origin_x", swarm_origin_x, 0.0);
    nh.param("swarm/origin_y", swarm_origin_y, 0.0);
    nh.param("swarm/origin_z", swarm_origin_z, 0.0);
    nh.param("swarm/origin_yaw", swarm_origin_yaw, 0.0);
    swarm_frame_transform_ = SwarmFrameTransform(
        swarm_origin_x, swarm_origin_y, swarm_origin_z, swarm_origin_yaw);
    nh.param("swarm/trajectory_timeout", swarm_trajectory_timeout_, 3.0);
    if (!std::isfinite(swarm_trajectory_timeout_) || swarm_trajectory_timeout_ <= 0.0)
      swarm_trajectory_timeout_ = 3.0;
    if (odom_topic_.empty() || waypoint_topic_.empty() || cancel_topic_.empty() ||
        swarm_trajectory_topic_.empty() || status_topic_.empty() ||
        swarm_common_frame_.empty() || !swarm_frame_transform_.isFinite())
    {
      ROS_FATAL("[EGO FSM] invalid topic or common-frame transform parameters.");
      return;
    }

    /* callback */
    const std::string private_ns = nh.getNamespace();
    const std::size_t last_slash = private_ns.find_last_of('/');
    const std::string parent_ns =
        last_slash == std::string::npos || last_slash == 0
            ? std::string("/")
            : private_ns.substr(0, last_slash);
    ros::NodeHandle public_nh(parent_ns);
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);
    status_timer_ = nh.createTimer(ros::Duration(0.1), &EGOReplanFSM::statusCallback, this);

    odom_sub_ = nh.subscribe(odom_topic_, 1, &EGOReplanFSM::odometryCallback, this);

    // All vehicles share one absolute bus.  Namespaces still isolate odometry,
    // local B-spline and command topics, while the swarm chain is deliberately
    // common so every planner can observe the same safety envelope.
    swarm_trajs_sub_ = public_nh.subscribe(
        swarm_trajectory_topic_, 10, &EGOReplanFSM::swarmTrajsCallback, this,
        ros::TransportHints().tcpNoDelay());
    swarm_trajs_pub_ = public_nh.advertise<traj_utils::MultiBsplines>(
        swarm_trajectory_topic_, 10);

    broadcast_bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = nh.subscribe("planning/broadcast_bspline_to_planner", 100, &EGOReplanFSM::BroadcastBsplineCallback, this, ros::TransportHints().tcpNoDelay());

    bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/bspline", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    cancel_sub_ = public_nh.subscribe(cancel_topic_, 1, &EGOReplanFSM::cancelCallback, this);
    status_pub_ = public_nh.advertise<astra_custom_msgs::PlannerStatus>(
        status_topic_, 10, true);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = public_nh.subscribe(
          waypoint_topic_, 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 1 second.");
      int count = 0;
      while (ros::ok() && count++ < 1000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ROS_WARN("Waiting for trigger from [n3ctrl] from RC");

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::readGivenWps()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];

      // end_pt_ = wps_.back();
    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);

    // if (success)
    // {

    //   /*** display ***/
    //   constexpr double step_size_t = 0.1;
    //   int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    //   std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    //   for (int i = 0; i < i_end; i++)
    //   {
    //     gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    //   }

    //   end_vel_.setZero();
    //   have_target_ = true;
    //   have_new_target_ = true;

    //   /*** FSM ***/
    //   // if (exec_state_ == WAIT_TARGET)
    //   //changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    //   // trigger_ = true;
    //   // else if (exec_state_ == EXEC_TRAJ)
    //   //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

    //   // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
    //   ros::Duration(0.001).sleep();
    //   visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    //   ros::Duration(0.001).sleep();
    // }
    // else
    // {
    //   ROS_ERROR("Unable to generate global trajectory!");
    // }
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    const ros::Time now = ros::Time::now();
    if (msg->header.stamp.isZero() ||
        (!now.isZero() && (now - msg->header.stamp).toSec() > 5.0) ||
        !std::isfinite(msg->pose.position.x) ||
        !std::isfinite(msg->pose.position.y) ||
        !std::isfinite(msg->pose.position.z))
    {
      ROS_WARN_THROTTLE(1.0, "[EGO FSM] rejected stale, zero-stamped, or non-finite goal.");
      status_tracker_.setEventReason(astra_custom_msgs::PlannerStatus::MAP_STALE);
      return;
    }
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    have_trigger_ = true;
    init_pt_ = odom_pos_;

    const double goal_z = use_goal_height_ && msg->pose.position.z > 0.0
                              ? msg->pose.position.z
                              : manual_target_height_;
    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, goal_z);

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::cancelCallback(const std_msgs::EmptyConstPtr &)
  {
    have_cancel_ = true;
    cancel_time_ = ros::Time::now();
    have_target_ = false;
    have_new_target_ = false;
    have_trigger_ = false;
    planner_manager_->local_data_.start_time_ = ros::Time(0);
    status_tracker_.setEventReason("CANCELLED");
    changeFSMExecState(WAIT_TARGET, "CANCEL");
    ROS_WARN("[EGO FSM] active target cancelled; waiting for a fresh stamped goal.");
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    if (!std::isfinite(msg->pose.pose.position.x) ||
        !std::isfinite(msg->pose.pose.position.y) ||
        !std::isfinite(msg->pose.pose.position.z) ||
        !std::isfinite(msg->twist.twist.linear.x) ||
        !std::isfinite(msg->twist.twist.linear.y) ||
        !std::isfinite(msg->twist.twist.linear.z))
    {
      have_odom_ = false;
      status_tracker_.setEventReason("INVALID_ODOM");
      return;
    }
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg)
  {
    if (!msg || msg->drone_id < 0 ||
        msg->frame_id != swarm_common_frame_ || msg->order != 3 ||
        msg->pos_pts.size() < 4 ||
        msg->knots.size() < msg->pos_pts.size() + msg->order + 1 ||
        msg->start_time.isZero())
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[EGO FSM] rejected malformed or non-common-frame B-spline.");
      return;
    }
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    {
      ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs",
                msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      return;
    }

    traj_utils::Bspline local_msg = *msg;
    local_msg.frame_id = status_frame_id_;
    for (auto &point : local_msg.pos_pts)
      point = swarm_frame_transform_.commonToLocal(point);

    /* Fill up the buffer */
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(local_msg.pos_pts[0].x, local_msg.pos_pts[0].y, local_msg.pos_pts[0].z);
    Eigen::Vector3d cp1(local_msg.pos_pts[1].x, local_msg.pos_pts[1].y, local_msg.pos_pts[1].z);
    Eigen::Vector3d cp2(local_msg.pos_pts[2].x, local_msg.pos_pts[2].y, local_msg.pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, local_msg.pos_pts.size());
    Eigen::VectorXd knots(local_msg.knots.size());
    for (size_t j = 0; j < local_msg.knots.size(); ++j)
    {
      knots(j) = local_msg.knots[j];
    }
    for (size_t j = 0; j < local_msg.pos_pts.size(); ++j)
    {
      pos_pts(0, j) = local_msg.pos_pts[j].x;
      pos_pts(1, j) = local_msg.pos_pts[j].y;
      pos_pts(2, j) = local_msg.pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    if (local_msg.order % 2)
    {
      double cutback = (double)local_msg.order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = local_msg.knots[local_msg.knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)local_msg.order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (local_msg.knots[local_msg.knots.size() - floor(cutback)] + local_msg.knots[local_msg.knots.size() - ceil(cutback)]) / 2;
    }

    UniformBspline pos_traj(pos_pts, local_msg.order, local_msg.knots[1] - local_msg.knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = local_msg.start_time;
    // planner_manager_->swarm_trajs_buf_[id].start_time_ = ros::Time::now(); // Un-reliable time sync

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
    if (static_cast<int>(id) == planner_manager_->pp_.drone_id - 1)
      have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg)
  {

    if (!msg || msg->drone_id_from < 0 ||
        msg->traj.empty() ||
        static_cast<std::size_t>(msg->drone_id_from + 1) != msg->traj.size())
    {
      ROS_WARN_THROTTLE(1.0, "[EGO FSM] rejected malformed swarm trajectory chain.");
      return;
    }
    const ros::Time now = ros::Time::now();
    for (std::size_t index = 0; index < msg->traj.size(); ++index)
    {
      const auto &remote = msg->traj[index];
      if (remote.order != 3 || remote.pos_pts.size() < 4 ||
          remote.knots.size() < remote.pos_pts.size() + remote.order + 1 ||
          remote.start_time.isZero() || remote.drone_id != static_cast<int>(index) ||
          remote.frame_id != swarm_common_frame_ ||
          (!now.isZero() &&
           ((remote.start_time - now).toSec() > swarm_trajectory_timeout_ ||
            (now - remote.start_time).toSec() >
                remote.knots.back() + swarm_trajectory_timeout_)))
      {
        ROS_WARN_THROTTLE(1.0, "[EGO FSM] rejected invalid or future swarm trajectory.");
        return;
      }
    }

    if (multi_bspline_msgs_buf_.traj.size() < msg->traj.size())
      multi_bspline_msgs_buf_.traj.resize(msg->traj.size());
    for (std::size_t index = 0; index < msg->traj.size(); ++index)
    {
      const auto &remote = msg->traj[index];
      auto &stored = multi_bspline_msgs_buf_.traj[index];
      if (stored.start_time.isZero() ||
          remote.start_time >= stored.start_time)
        stored = remote;
    }
    multi_bspline_msgs_buf_.drone_id_from =
        std::max(multi_bspline_msgs_buf_.drone_id_from,
                 msg->drone_id_from);

    // cout << "\033[45;33mmulti_bspline_msgs_buf.drone_id_from=" << multi_bspline_msgs_buf_.drone_id_from << " multi_bspline_msgs_buf_.traj.size()=" << multi_bspline_msgs_buf_.traj.size() << "\033[0m" << endl;

    if (!have_odom_)
    {
      ROS_ERROR("swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      ROS_ERROR("Wrong trajectory size! msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      ROS_ERROR("Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    if (planner_manager_->swarm_trajs_buf_.size() < msg->traj.size())
      planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    for (size_t i = 0; i < msg->traj.size(); i++)
    {
      traj_utils::Bspline local_msg = msg->traj[i];
      local_msg.frame_id = status_frame_id_;
      for (auto &point : local_msg.pos_pts)
        point = swarm_frame_transform_.commonToLocal(point);

      Eigen::Vector3d cp0(local_msg.pos_pts[0].x, local_msg.pos_pts[0].y, local_msg.pos_pts[0].z);
      Eigen::Vector3d cp1(local_msg.pos_pts[1].x, local_msg.pos_pts[1].y, local_msg.pos_pts[1].z);
      Eigen::Vector3d cp2(local_msg.pos_pts[2].x, local_msg.pos_pts[2].y, local_msg.pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      Eigen::MatrixXd pos_pts(3, local_msg.pos_pts.size());
      Eigen::VectorXd knots(local_msg.knots.size());
      for (size_t j = 0; j < local_msg.knots.size(); ++j)
      {
        knots(j) = local_msg.knots[j];
      }
      for (size_t j = 0; j < local_msg.pos_pts.size(); ++j)
      {
        pos_pts(0, j) = local_msg.pos_pts[j].x;
        pos_pts(1, j) = local_msg.pos_pts[j].y;
        pos_pts(2, j) = local_msg.pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      if (local_msg.order % 2)
      {
        double cutback = (double)local_msg.order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = local_msg.knots[local_msg.knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)local_msg.order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (local_msg.knots[local_msg.knots.size() - floor(cutback)] + local_msg.knots[local_msg.knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, local_msg.order, local_msg.knots[1] - local_msg.knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = local_msg.start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
        // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      // return;
      else
      {
        // if ( planner_manager_->pp_.drone_id <= 0 )
        // {
        //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        // }
        // else
        // {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
        // }
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      // cout << "id=" << planner_manager_->pp_.drone_id << " have_recv_pre_agent_=" << have_recv_pre_agent_ << endl;
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            ROS_ERROR("Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          ROS_ERROR("No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
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
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

          if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          {
            wp_id_ = 0;
            planNextWaypoint(wps_[wp_id_]);
          }

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
          // return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
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
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
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

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 2.0 * planner_manager_->getSwarmClearance();
    double t_cur_global = ros::Time::now().toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      occ |= map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t));

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.toSec();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
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

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;
    recordPlanningResult(plan_and_refine_success,
                         plan_and_refine_success ? "" : astra_custom_msgs::PlannerStatus::NO_FEASIBLE_TRAJECTORY);

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::Bspline bspline;
      bspline.drone_id = planner_manager_->pp_.drone_id;
      bspline.frame_id = status_frame_id_;
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
      // cout << knots.transpose() << endl;
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_.publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::Bspline bspline;
    bspline.frame_id = swarm_common_frame_;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(swarm_frame_transform_.localToCommon(pt));
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    // cout << knots.transpose() << endl;
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    const std::size_t required_size =
        static_cast<std::size_t>(planner_manager_->pp_.drone_id + 1);
    if (multi_bspline_msgs_buf_.traj.size() < required_size)
      multi_bspline_msgs_buf_.traj.resize(required_size);
    multi_bspline_msgs_buf_.drone_id_from =
        std::max(multi_bspline_msgs_buf_.drone_id_from,
                 planner_manager_->pp_.drone_id);
    multi_bspline_msgs_buf_.traj[planner_manager_->pp_.drone_id] = bspline;
    // Publish the complete chain after every successful replan.  The upstream
    // implementation only refreshed this on startup, leaving stale collision
    // envelopes during normal replanning.
    swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);

    broadcast_bspline_pub_.publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::Bspline bspline;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.frame_id = status_frame_id_;
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
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
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
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
  }

  const char *EGOReplanFSM::stateName() const
  {
    static const char *const names[] = {
        "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ",
        "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    const int index = static_cast<int>(exec_state_);
    return index >= 0 && index < 7 ? names[index] : "UNKNOWN";
  }

  void EGOReplanFSM::recordPlanningResult(bool success,
                                           const std::string &failure_reason)
  {
    status_tracker_.recordAttempt(
        success, failure_reason,
        static_cast<std::uint32_t>(planner_manager_->local_data_.traj_id_));
  }

  void EGOReplanFSM::statusCallback(const ros::TimerEvent &)
  {
    if (!status_pub_)
      return;

    astra_custom_msgs::PlannerStatus status;
    status.header.stamp = ros::Time::now();
    status.header.frame_id = status_frame_id_;
    status.planner_state = stateName();
    status.target_id = target_id_;
    status.trajectory_id = status_tracker_.trajectoryId();
    status.last_plan_success = status_tracker_.lastSuccess();
    status.consecutive_plan_failures = status_tracker_.consecutiveFailures();
    status.goal_in_collision = false;
    status.current_position_in_collision =
        planner_manager_->grid_map_ && have_odom_ &&
        planner_manager_->grid_map_->getInflateOccupancy(odom_pos_);
    status.emergency_stop_active = exec_state_ == EMERGENCY_STOP;
    status.emergency_stop_duration =
        status.emergency_stop_active && !cancel_time_.isZero()
            ? static_cast<float>((ros::Time::now() - cancel_time_).toSec())
            : 0.0f;
    status.failure_reason = status_tracker_.failureReason();
    status.status_timestamp = status.header.stamp;
    status_pub_.publish(status);
  }

} // namespace ego_planner
