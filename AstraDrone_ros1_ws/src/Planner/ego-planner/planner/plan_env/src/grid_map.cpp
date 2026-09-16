#include "plan_env/grid_map.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_.reset(new ros::NodeHandle(nh));
  ros::NodeHandle& node = *node_;

  /* get parameter */
  double x_size, y_size, z_size;
  node.param("grid_map/resolution", mp_.resolution_, -1.0);
  node.param("grid_map/map_size_x", x_size, -1.0);
  node.param("grid_map/map_size_y", y_size, -1.0);
  node.param("grid_map/map_size_z", z_size, -1.0);
  node.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  node.param("grid_map/fx", mp_.fx_, -1.0);
  node.param("grid_map/fy", mp_.fy_, -1.0);
  node.param("grid_map/cx", mp_.cx_, -1.0);
  node.param("grid_map/cy", mp_.cy_, -1.0);

  node.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node.param("grid_map/p_min", mp_.p_min_, 0.12);
  node.param("grid_map/p_max", mp_.p_max_, 0.97);
  node.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);
  node.param("grid_map/cloud_odom_timeout", mp_.cloud_odom_timeout_, 0.15);
  node.param("grid_map/cloud_sensor_origin_x", mp_.cloud_sensor_origin_body_(0), 0.0);
  node.param("grid_map/cloud_sensor_origin_y", mp_.cloud_sensor_origin_body_(1), 0.0);
  node.param("grid_map/cloud_sensor_origin_z", mp_.cloud_sensor_origin_body_(2), 0.13);

  node.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  node.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);
  node.param("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_, -0.1);
  node.param("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_, -0.1);

  node.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node.param("grid_map/pose_type", mp_.pose_type_, 1);

  node.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
  node.param("grid_map/ground_height", mp_.ground_height_, 1.0);

  node.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);

  if( mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  if (mp_.resolution_ <= 0.0 || (mp_.local_update_range_.array() <= 0.0).any() ||
      mp_.min_ray_length_ < 0.0 || mp_.max_ray_length_ <= mp_.min_ray_length_ ||
      mp_.cloud_odom_timeout_ <= 0.0) {
    throw std::runtime_error("invalid GridMap resolution/range/ray/origin parameters");
  }

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  initializeMapBuffers();

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node, "grid_map/depth", 50));
  extrinsic_sub_ = node.subscribe<nav_msgs::Odometry>(
      "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this); //sub

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node, "grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  }

  // use odometry and point cloud
  indep_cloud_sub_ =
      node.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::cloudCallback, this);
  indep_odom_sub_ =
      node.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::odomCallback, this);

  occ_timer_ = node.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node.createTimer(ros::Duration(0.11), &GridMap::visCallback, this);

  map_pub_ = node.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);
  local_inflated_obstacles_pub_ =
      node.advertise<sensor_msgs::PointCloud2>("grid_map/inflated_cloud", 2);
  map_free_pub_ = node.advertise<sensor_msgs::PointCloud2>("grid_map/known_free", 2);
  map_unknown_pub_ = node.advertise<sensor_msgs::PointCloud2>("grid_map/unknown", 2);
  mapping_stats_pub_ = node.advertise<std_msgs::String>("grid_map/mapping_stats", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_.fromSec(0);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  ROS_INFO("GridMap PointCloud2 mapper: raycast log-odds, bounded multi-frame local map, "
           "3D inflation, binary planning occupancy, three-state sensing semantics");

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::initializeMapBuffers()
{
  const int buffer_size =
      mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_.assign(
      buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_.assign(buffer_size, 0);
  md_.occupancy_buffer_inflate_count_.assign(buffer_size, 0);
  md_.count_hit_and_miss_.assign(buffer_size, 0);
  md_.count_hit_.assign(buffer_size, 0);
  md_.flag_rayend_.assign(buffer_size, 0);
  md_.flag_traverse_.assign(buffer_size, 0);
  md_.raycast_num_ = 0;
  md_.proj_points_.clear();
  md_.proj_points_hit_.clear();
  md_.proj_points_.reserve(20000);
  md_.proj_points_hit_.reserve(20000);
  md_.proj_points_cnt = 0;
  md_.sensor_origin_history_.clear();
  md_.local_window_initialized_ = false;
  md_.last_cloud_stamp_ = ros::Time(0);
  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = Eigen::Vector3i::Zero();
  md_.previous_local_bound_min_ = Eigen::Vector3i::Zero();
  md_.previous_local_bound_max_ = Eigen::Vector3i::Zero();
  md_.last_input_point_count_ = 0;
  md_.last_endpoint_voxel_count_ = 0;
  md_.last_raycast_voxel_count_ = 0;
  md_.last_raw_occupied_count_ = 0;
  md_.last_known_free_count_ = 0;
  md_.last_unknown_count_ = 0;
  md_.last_inflated_count_ = 0;
  md_.last_raycast_wall_ms_ = 0.0;
  md_.last_inflation_wall_ms_ = 0.0;
  md_.last_callback_wall_ms_ = 0.0;
}

bool GridMap::isSweptSegmentFree(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    SweptCollisionResult* result)
{
  SweptCollisionResult local_result;
  SweptCollisionResult& out = result ? *result : local_result;
  out = SweptCollisionResult();

  if (!start.allFinite() || !end.allFinite() ||
      !isInMap(start) || !isInMap(end)) {
    out.collision = true;
    out.outside_map = true;
    out.reason = BlockReason::OUT_OF_MAP;
    return false;
  }

  Eigen::Vector3i current, finish;
  posToIndex(start, current);
  posToIndex(end, finish);

  const auto blocked = [&](const Eigen::Vector3i& id) {
    ++out.visited_voxels;
    if (!isInMap(id)) {
      out.collision = true;
      out.outside_map = true;
      out.first_blocked_index = id;
      out.reason = BlockReason::OUT_OF_MAP;
      return true;
    }
    if (md_.occupancy_buffer_inflate_[toAddress(id)] != 0 ||
        getVoxelState(id) == VoxelState::OCCUPIED) {
      out.collision = true;
      out.first_blocked_index = id;
      out.reason = BlockReason::OCCUPIED;
      return true;
    }
    return false;
  };

  if (blocked(current))
    return false;
  if (current == finish)
    return true;

  const Eigen::Vector3d start_grid =
      (start - mp_.map_origin_) * mp_.resolution_inv_;
  const Eigen::Vector3d end_grid =
      (end - mp_.map_origin_) * mp_.resolution_inv_;
  const Eigen::Vector3d delta = end_grid - start_grid;
  Eigen::Vector3i step = Eigen::Vector3i::Zero();
  Eigen::Vector3d t_max = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d t_delta = t_max;
  for (int axis = 0; axis < 3; ++axis) {
    if (delta(axis) > 0.0) {
      step(axis) = 1;
      t_max(axis) =
          (static_cast<double>(current(axis) + 1) - start_grid(axis)) /
          delta(axis);
      t_delta(axis) = 1.0 / delta(axis);
    } else if (delta(axis) < 0.0) {
      step(axis) = -1;
      t_max(axis) =
          (start_grid(axis) - static_cast<double>(current(axis))) /
          -delta(axis);
      t_delta(axis) = 1.0 / -delta(axis);
    }
  }

  // Conservative 3-D supercover traversal.  When a line crosses a voxel
  // edge or corner, every adjacent voxel touched at that same parametric
  // instant is checked.  This is the corner-cut behavior required by the
  // 26-neighbor A* and uses the map's own voxel resolution/origin.
  constexpr double tie_epsilon = 1.0e-12;
  while (current != finish) {
    const double next_t = t_max.minCoeff();
    if (!std::isfinite(next_t) || next_t > 1.0 + tie_epsilon) {
      out.collision = true;
      out.outside_map = true;
      out.reason = BlockReason::OUT_OF_MAP;
      return false;
    }

    int tied_mask = 0;
    for (int axis = 0; axis < 3; ++axis)
      if (std::abs(t_max(axis) - next_t) <= tie_epsilon)
        tied_mask |= 1 << axis;

    for (int subset = 1; subset < 8; ++subset) {
      if ((subset & tied_mask) != subset)
        continue;
      Eigen::Vector3i candidate = current;
      for (int axis = 0; axis < 3; ++axis)
        if (subset & (1 << axis))
          candidate(axis) += step(axis);
      if (blocked(candidate))
        return false;
    }

    for (int axis = 0; axis < 3; ++axis) {
      if (!(tied_mask & (1 << axis)))
        continue;
      current(axis) += step(axis);
      t_max(axis) += t_delta(axis);
    }
  }
  return true;
}

bool GridMap::isSweptPolylineFree(
    const std::vector<Eigen::Vector3d>& points,
    SweptCollisionResult* result)
{
  SweptCollisionResult aggregate;
  if (points.empty()) {
    aggregate.collision = true;
    aggregate.outside_map = true;
    aggregate.reason = BlockReason::OUT_OF_MAP;
    if (result)
      *result = aggregate;
    return false;
  }
  if (points.size() == 1) {
    const bool free = !getPlanningOccupancy(points.front());
    aggregate.collision = !free;
    aggregate.outside_map = !isInMap(points.front());
    aggregate.reason = aggregate.outside_map
                           ? BlockReason::OUT_OF_MAP
                           : (free ? BlockReason::NONE : BlockReason::OCCUPIED);
    aggregate.visited_voxels = 1;
    if (result)
      *result = aggregate;
    return free;
  }

  for (size_t index = 1; index < points.size(); ++index) {
    SweptCollisionResult segment;
    if (!isSweptSegmentFree(points[index - 1], points[index], &segment)) {
      aggregate.collision = true;
      aggregate.outside_map = segment.outside_map;
      aggregate.visited_voxels += segment.visited_voxels;
      aggregate.first_blocked_index = segment.first_blocked_index;
      aggregate.reason = segment.reason;
      if (result)
        *result = aggregate;
      return false;
    }
    aggregate.visited_voxels += segment.visited_voxels;
  }
  if (result)
    *result = aggregate;
  return true;
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset raw evidence and keep incremental inflation references coherent */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        const Eigen::Vector3i id(x, y, z);
        const int address = toAddress(id);
        if (md_.occupancy_buffer_[address] > mp_.min_occupancy_log_)
          updateInflationForVoxel(id, -1);
        md_.occupancy_buffer_[address] =
            mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.count_hit_[address] = 0;
        md_.count_hit_and_miss_[address] = 0;
      }
}

void GridMap::clearForEnvironmentReset()
{
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(),
            mp_.clamp_min_log_ - mp_.unknown_flag_);
  std::fill(md_.occupancy_buffer_inflate_.begin(),
            md_.occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.occupancy_buffer_inflate_count_.begin(),
            md_.occupancy_buffer_inflate_count_.end(), 0);
  std::fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
  std::fill(md_.count_hit_and_miss_.begin(),
            md_.count_hit_and_miss_.end(), 0);
  std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), 0);
  std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), 0);
  std::queue<Eigen::Vector3i> empty_cache;
  md_.cache_voxel_.swap(empty_cache);
  md_.raycast_num_ = 0;
  md_.proj_points_cnt = 0;
  md_.proj_points_.clear();
  md_.proj_points_hit_.clear();
  md_.sensor_origin_history_.clear();
  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_cloud_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.last_occ_update_time_ = ros::Time(0);
  md_.last_cloud_stamp_ = ros::Time(0);
  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = Eigen::Vector3i::Zero();
  md_.previous_local_bound_min_ = Eigen::Vector3i::Zero();
  md_.previous_local_bound_max_ = Eigen::Vector3i::Zero();
  md_.local_window_initialized_ = false;
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return INVALID_IDX;
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  md_.proj_points_cnt = 0;
  md_.proj_points_hit_.clear();

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;
  const size_t maximum_projected_points =
      static_cast<size_t>((rows / std::max(1, skip_pix) + 1) *
                          (cols / std::max(1, skip_pix) + 1) * 2);
  if (md_.proj_points_.size() < maximum_projected_points)
    md_.proj_points_.resize(maximum_projected_points);

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v+=skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u+=skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  if (md_.proj_points_cnt == 0)
    return;

  const ros::WallTime raycast_begin = ros::WallTime::now();
  if (++md_.raycast_num_ == 0) {
    std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), 0);
    std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), 0);
    md_.raycast_num_ = 1;
  }

  int vox_idx;
  double length;
  size_t traversed_voxels = 0;

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;
  const bool explicit_endpoint_semantics =
      md_.proj_points_hit_.size() == static_cast<size_t>(md_.proj_points_cnt);

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];
    bool endpoint_is_hit = explicit_endpoint_semantics && md_.proj_points_hit_[i] != 0;

    if (!explicit_endpoint_semantics) {
      if (!isInMap(pt_w)) {
        pt_w = closetPointInMap(pt_w, md_.camera_pos_);
      } else {
        endpoint_is_hit = true;
      }
      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ +
               md_.camera_pos_;
        endpoint_is_hit = false;
      }
    }

    vox_idx = setCacheOccupancy(pt_w, endpoint_is_hit ? 1 : 0);

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    if (!raycaster.setInput(pt_w / mp_.resolution_,
                            md_.camera_pos_ / mp_.resolution_))
      continue;

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      if (!isInMap(tmp))
        continue;

      vox_idx = setCacheOccupancy(tmp, 0);
      ++traversed_voxels;

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    // A measured endpoint is stronger evidence than any number of rays that
    // traverse the same voxel in this scan. Cross-scan clearing still happens
    // through repeated MISS log-odds updates when the endpoint disappears.
    const double log_odds_update =
        md_.count_hit_[idx_ctns] > 0 ?
            mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    const bool was_occupied =
        md_.occupancy_buffer_[idx_ctns] > mp_.min_occupancy_log_;

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) &&
                    idx(1) >= min_id(1) && idx(1) <= max_id(1) &&
                    idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      if (was_occupied)
        updateInflationForVoxel(idx, -1);
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      continue;
    }

    if (log_odds_update >= 0 &&
        md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_max_log_;
    } else if (log_odds_update <= 0 &&
               md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    } else {
      md_.occupancy_buffer_[idx_ctns] = std::min(
          std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update,
                   mp_.clamp_min_log_),
          mp_.clamp_max_log_);
    }

    const bool is_occupied =
        md_.occupancy_buffer_[idx_ctns] > mp_.min_occupancy_log_;
    if (is_occupied != was_occupied)
      updateInflationForVoxel(idx, is_occupied ? 1 : -1);
  }

  md_.last_raycast_voxel_count_ = traversed_voxels;
  md_.last_raycast_wall_ms_ =
      (ros::WallTime::now() - raycast_begin).toSec() * 1000.0;
}

void GridMap::updateInflationForVoxel(
    const Eigen::Vector3i& occupied_id, int delta)
{
  if (delta != 1 && delta != -1)
    return;
  const int inf_step =
      static_cast<int>(std::ceil(mp_.obstacles_inflation_ / mp_.resolution_));
  vector<Eigen::Vector3i> inflated_ids(
      static_cast<size_t>(std::pow(2 * inf_step + 1, 3)));
  inflatePoint(occupied_id, inf_step, inflated_ids);
  for (const Eigen::Vector3i& inflated_id : inflated_ids) {
    if (!isInMap(inflated_id))
      continue;
    const int address = toAddress(inflated_id);
    uint16_t& count = md_.occupancy_buffer_inflate_count_[address];
    if (delta > 0) {
      if (count < std::numeric_limits<uint16_t>::max())
        ++count;
    } else if (count > 0) {
      --count;
    }
    md_.occupancy_buffer_inflate_[address] = count > 0 ? 1 : 0;
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearExitedLocalWindow(
    const Eigen::Vector3i& old_min, const Eigen::Vector3i& old_max,
    const Eigen::Vector3i& new_min, const Eigen::Vector3i& new_max)
{
  for (int x = old_min(0); x <= old_max(0); ++x)
    for (int y = old_min(1); y <= old_max(1); ++y)
      for (int z = old_min(2); z <= old_max(2); ++z) {
        const bool still_local =
            x >= new_min(0) && x <= new_max(0) &&
            y >= new_min(1) && y <= new_max(1) &&
            z >= new_min(2) && z <= new_max(2);
        if (still_local)
          continue;
        const int address = toAddress(x, y, z);
        if (md_.occupancy_buffer_[address] > mp_.min_occupancy_log_)
          updateInflationForVoxel(Eigen::Vector3i(x, y, z), -1);
        md_.occupancy_buffer_[address] =
            mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
}

void GridMap::updateLocalMapWindow(const Eigen::Vector3d& sensor_origin)
{
  Eigen::Vector3i new_min, new_max;
  posToIndex(sensor_origin - mp_.local_update_range_, new_min);
  posToIndex(sensor_origin + mp_.local_update_range_, new_max);
  boundIndex(new_min);
  boundIndex(new_max);

  if (md_.local_window_initialized_) {
    md_.previous_local_bound_min_ = md_.local_bound_min_;
    md_.previous_local_bound_max_ = md_.local_bound_max_;
    if (new_min != md_.local_bound_min_ || new_max != md_.local_bound_max_)
      clearExitedLocalWindow(md_.local_bound_min_, md_.local_bound_max_,
                             new_min, new_max);
  } else {
    md_.previous_local_bound_min_ = new_min;
    md_.previous_local_bound_max_ = new_max;
    md_.local_window_initialized_ = true;
  }
  md_.local_bound_min_ = new_min;
  md_.local_bound_max_ = new_max;
}

void GridMap::clearAndInflateLocalMap()
{
  const ros::WallTime inflation_begin = ros::WallTime::now();
  size_t raw_occupied = 0;
  size_t known_free = 0;
  size_t unknown = 0;
  size_t inflated = 0;

  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        const Eigen::Vector3i raw_id(x, y, z);
        const VoxelState state = getVoxelState(raw_id);
        inflated += md_.occupancy_buffer_inflate_[toAddress(raw_id)] != 0;
        if (state == VoxelState::UNKNOWN) {
          ++unknown;
          continue;
        }
        if (state == VoxelState::KNOWN_FREE) {
          ++known_free;
        } else {
          ++raw_occupied;
        }
      }

  if (mp_.virtual_ceil_height_ > -0.5) {
    const int ceil_id = static_cast<int>(std::floor(
        (mp_.virtual_ceil_height_ - mp_.map_origin_(2)) *
        mp_.resolution_inv_)) - 1;
    if (ceil_id >= 0 && ceil_id < mp_.map_voxel_num_(2)) {
      for (int x = md_.previous_local_bound_min_(0);
           x <= md_.previous_local_bound_max_(0); ++x)
        for (int y = md_.previous_local_bound_min_(1);
             y <= md_.previous_local_bound_max_(1); ++y) {
          const int address = toAddress(x, y, ceil_id);
          md_.occupancy_buffer_inflate_[address] =
              md_.occupancy_buffer_inflate_count_[address] > 0 ? 1 : 0;
        }
      for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
          md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
    }
  }

  md_.last_raw_occupied_count_ = raw_occupied;
  md_.last_known_free_count_ = known_free;
  md_.last_unknown_count_ = unknown;
  md_.last_inflated_count_ = inflated;
  md_.last_inflation_wall_ms_ =
      (ros::WallTime::now() - inflation_begin).toSec() * 1000.0;
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{
  // Also emit an empty debug frame after reset, so RViz can discard the old view.
  publishLocalInflatedObstacles();
  if (!md_.local_window_initialized_)
    return;
  publishMapInflate(true);
  publishMap();
  publishVoxelStateMap(VoxelState::KNOWN_FREE, map_free_pub_);
  publishVoxelStateMap(VoxelState::UNKNOWN, map_unknown_pub_);
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (md_.last_occ_update_time_.toSec() < 1.0 ) md_.last_occ_update_time_ = ros::Time::now();
  
  if (!md_.occ_need_update_)
  {
    if ( md_.flag_use_depth_fusion && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_ )
    {
      ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f", 
        ros::Time::now().toSec(), md_.last_occ_update_time_.toSec(), mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return;
  }
  md_.last_occ_update_time_ = ros::Time::now();

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  updateLocalMapWindow(md_.camera_pos_);
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.has_first_depth_)
    return;

  const Eigen::Vector3d body_position(
      odom->pose.pose.position.x,
      odom->pose.pose.position.y,
      odom->pose.pose.position.z);
  const Eigen::Quaterniond body_orientation(
      odom->pose.pose.orientation.w,
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z);
  if (!body_position.allFinite() || !std::isfinite(body_orientation.norm()) ||
      body_orientation.norm() < 1.0e-6)
    return;

  md_.camera_pos_ = body_position +
      body_orientation.normalized() * mp_.cloud_sensor_origin_body_;
  if (!isInMap(md_.camera_pos_))
    return;

  MappingData::StampedSensorOrigin sample;
  sample.stamp = odom->header.stamp.isZero() ? ros::Time::now() : odom->header.stamp;
  sample.position = md_.camera_pos_;
  if (md_.sensor_origin_history_.empty() ||
      sample.stamp > md_.sensor_origin_history_.back().stamp) {
    md_.sensor_origin_history_.push_back(sample);
    while (md_.sensor_origin_history_.size() > 200)
      md_.sensor_origin_history_.pop_front();
  }

  md_.has_odom_ = true;
}

bool GridMap::selectSensorOrigin(
    const ros::Time& cloud_stamp, Eigen::Vector3d& sensor_origin) const
{
  if (!md_.has_odom_ || md_.sensor_origin_history_.empty())
    return false;

  if (cloud_stamp.isZero()) {
    sensor_origin = md_.sensor_origin_history_.back().position;
    return true;
  }

  for (auto it = md_.sensor_origin_history_.rbegin();
       it != md_.sensor_origin_history_.rend(); ++it) {
    if (it->stamp > cloud_stamp)
      continue;
    const double lag = (cloud_stamp - it->stamp).toSec();
    if (lag > mp_.cloud_odom_timeout_)
      return false;
    sensor_origin = it->position;
    return true;
  }
  return false;
}

void GridMap::processPointCloud(
    const vector<Eigen::Vector3d>& points,
    const Eigen::Vector3d& sensor_origin)
{
  if (points.empty() || !sensor_origin.allFinite() || !isInMap(sensor_origin))
    return;

  const ros::WallTime callback_begin = ros::WallTime::now();
  md_.camera_pos_ = sensor_origin;
  updateLocalMapWindow(sensor_origin);
  md_.proj_points_.clear();
  md_.proj_points_hit_.clear();
  md_.proj_points_cnt = 0;
  md_.last_input_point_count_ = points.size();

  std::unordered_map<int, size_t> endpoint_voxels;
  endpoint_voxels.reserve(points.size());

  for (const Eigen::Vector3d& point : points) {
    if (!point.allFinite())
      continue;

    const Eigen::Vector3d delta = point - sensor_origin;
    const double range = delta.norm();
    if (!std::isfinite(range) || range < mp_.min_ray_length_ || range < 1.0e-9)
      continue;

    double scale = 1.0;
    for (int axis = 0; axis < 3; ++axis) {
      if (std::fabs(delta(axis)) > mp_.local_update_range_(axis))
        scale = std::min(scale,
                         mp_.local_update_range_(axis) / std::fabs(delta(axis)));
      if (delta(axis) > 0.0) {
        scale = std::min(
            scale,
            (mp_.map_max_boundary_(axis) - 1.0e-4 - sensor_origin(axis)) /
                delta(axis));
      } else if (delta(axis) < 0.0) {
        scale = std::min(
            scale,
            (mp_.map_min_boundary_(axis) + 1.0e-4 - sensor_origin(axis)) /
                delta(axis));
      }
    }
    if (range > mp_.max_ray_length_)
      scale = std::min(scale, mp_.max_ray_length_ / range);
    if (!std::isfinite(scale) || scale <= 0.0)
      continue;
    scale = std::min(1.0, scale);

    const Eigen::Vector3d endpoint = sensor_origin + scale * delta;
    if (!isInMap(endpoint))
      continue;
    const bool endpoint_is_hit = scale >= 1.0 - 1.0e-9;

    Eigen::Vector3i endpoint_id;
    posToIndex(endpoint, endpoint_id);
    const int address = toAddress(endpoint_id);
    const auto found = endpoint_voxels.find(address);
    if (found == endpoint_voxels.end()) {
      endpoint_voxels[address] = md_.proj_points_.size();
      md_.proj_points_.push_back(endpoint);
      md_.proj_points_hit_.push_back(endpoint_is_hit ? 1 : 0);
    } else if (endpoint_is_hit) {
      md_.proj_points_hit_[found->second] = 1;
    }
  }

  md_.proj_points_cnt = static_cast<int>(md_.proj_points_.size());
  md_.last_endpoint_voxel_count_ = md_.proj_points_.size();
  if (md_.proj_points_cnt == 0)
    return;

  raycastProcess();
  if (md_.local_updated_)
    clearAndInflateLocalMap();
  md_.local_updated_ = false;
  md_.last_occ_update_time_ = ros::Time::now();
  md_.last_callback_wall_ms_ =
      (ros::WallTime::now() - callback_begin).toSec() * 1000.0;
  publishMappingStats();
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  Eigen::Vector3d sensor_origin;
  if (!selectSensorOrigin(img->header.stamp, sensor_origin)) {
    ROS_WARN_THROTTLE(1.0,
                      "GridMap rejected PointCloud2: no causal sensor origin within %.3f s",
                      mp_.cloud_odom_timeout_);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);
  md_.last_cloud_stamp_ = img->header.stamp;
  vector<Eigen::Vector3d> points;
  points.reserve(latest_cloud.points.size());
  for (const pcl::PointXYZ& point : latest_cloud.points)
    points.emplace_back(point.x, point.y, point.z);

  md_.has_cloud_ = !points.empty();
  processPointCloud(points, sensor_origin);
}

void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = md_.last_cloud_stamp_;
  map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = md_.last_cloud_stamp_;
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

pcl::PointCloud<pcl::PointXYZ> GridMap::makeLocalInflatedObstacleCloud()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = mp_.frame_id_;
  cloud.height = 1;
  cloud.is_dense = true;
  if (!md_.local_window_initialized_)
    return cloud;

  // Reuse the authoritative moving window, without local_map_margin. Read only:
  // the planning map still includes the virtual ceiling and its hard constraints.
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        const int address = toAddress(x, y, z);
        // A virtual-ceiling-only voxel has no observed obstacle contributing to
        // its inflation count. Keep real obstacle inflation even on that plane.
        if (md_.occupancy_buffer_inflate_[address] == 0 ||
            md_.occupancy_buffer_inflate_count_[address] == 0)
          continue;
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos.z() > mp_.visualization_truncate_height_)
          continue;
        cloud.push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
      }
  cloud.width = cloud.points.size();
  return cloud;
}

void GridMap::publishLocalInflatedObstacles()
{
  if (local_inflated_obstacles_pub_.getNumSubscribers() == 0)
    return;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(makeLocalInflatedObstacleCloud(), cloud_msg);
  cloud_msg.header.stamp = md_.last_cloud_stamp_;
  local_inflated_obstacles_pub_.publish(cloud_msg);
}

void GridMap::publishVoxelStateMap(
    VoxelState state, const ros::Publisher& publisher)
{
  if (publisher.getNumSubscribers() <= 0 || !md_.local_window_initialized_)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ point;
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        const Eigen::Vector3i id(x, y, z);
        if (getVoxelState(id) != state)
          continue;
        Eigen::Vector3d position;
        indexToPos(id, position);
        if (position(2) > mp_.visualization_truncate_height_)
          continue;
        point.x = position(0);
        point.y = position(1);
        point.z = position(2);
        cloud.push_back(point);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.stamp = md_.last_cloud_stamp_;
  publisher.publish(message);
}

void GridMap::publishMappingStats()
{
  if (mapping_stats_pub_.getNumSubscribers() <= 0)
    return;

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6)
         << "{\"schema_version\":\"ego_grid_map_pointcloud_mapping_v1.0\""
         << ",\"input_points\":" << md_.last_input_point_count_
         << ",\"endpoint_voxels\":" << md_.last_endpoint_voxel_count_
         << ",\"raycast_voxel_visits\":" << md_.last_raycast_voxel_count_
         << ",\"raw_occupied_voxels\":" << md_.last_raw_occupied_count_
         << ",\"known_free_voxels\":" << md_.last_known_free_count_
         << ",\"unknown_voxels\":" << md_.last_unknown_count_
         << ",\"inflated_voxels\":" << md_.last_inflated_count_
         << ",\"raycast_wall_ms\":" << md_.last_raycast_wall_ms_
         << ",\"inflation_wall_ms\":" << md_.last_inflation_wall_ms_
         << ",\"callback_wall_ms\":" << md_.last_callback_wall_ms_
         << ",\"cloud_stamp\":" << md_.last_cloud_stamp_.toSec()
         << ",\"sensor_origin\":[" << md_.camera_pos_(0) << ","
         << md_.camera_pos_(1) << "," << md_.camera_pos_(2) << "]"
         << ",\"planning_contract\":\"inflated_occupied_blocks_unknown_searchable\"}";
  std_msgs::String message;
  message.data = stream.str();
  mapping_stats_pub_.publish(message);
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
  md_.flag_use_depth_fusion = true;
}
