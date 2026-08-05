#include "astra_swarm_perception/self_cloud_filter.h"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <boost/bind.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace astra_swarm_perception {
namespace {

diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                   const std::string& value) {
  diagnostic_msgs::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

template <typename Value>
diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                   const Value& value) {
  std::ostringstream stream;
  stream << std::setprecision(9) << value;
  return keyValue(key, stream.str());
}

std::size_t pointCount(const sensor_msgs::PointCloud2& cloud) {
  return static_cast<std::size_t>(cloud.width) * cloud.height;
}

}  // namespace

class SelfCloudFilterNode {
 public:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry>;

  SelfCloudFilterNode() : node_(), private_node_("~") {
    std::string input_topic, odom_topic, output_topic, before_topic,
        after_topic, diagnostic_topic, inflated_topic;
    private_node_.param<std::string>("input_topic", input_topic,
                                     "cloud_registered_peer_filtered");
    private_node_.param<std::string>("odom_topic", odom_topic, "Odometry");
    private_node_.param<std::string>("output_topic", output_topic,
                                     "cloud_registered_self_filtered");
    private_node_.param<std::string>("before_topic", before_topic,
                                     "stage5/cloud_before_self");
    private_node_.param<std::string>("after_topic", after_topic,
                                     "stage5/cloud_after_self");
    private_node_.param<std::string>("diagnostic_topic", diagnostic_topic,
                                     "stage5/cloud_filter_diagnostics");
    private_node_.param<std::string>("inflated_occupancy_topic", inflated_topic,
                                     "stage3/occupancy_inflate");
    private_node_.param("synchronization_slop", synchronization_slop_, 0.03);
    private_node_.param("occupancy_collision_radius", collision_radius_, 0.55);
    if (input_topic.empty() || odom_topic.empty() || output_topic.empty() ||
        before_topic.empty() || after_topic.empty() || diagnostic_topic.empty() ||
        inflated_topic.empty() || input_topic == output_topic ||
        !std::isfinite(synchronization_slop_) || synchronization_slop_ <= 0.0 ||
        synchronization_slop_ > 0.2 || !std::isfinite(collision_radius_) ||
        std::abs(collision_radius_ - 0.55) > 1.0e-9) {
      throw std::runtime_error("invalid self cloud filter topic/safety configuration");
    }
    loadConfiguration();

    output_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(output_topic, 2);
    before_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(before_topic, 2);
    after_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(after_topic, 2);
    diagnostic_publisher_ =
        node_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostic_topic, 10);
    occupancy_subscriber_ = node_.subscribe(
        inflated_topic, 2, &SelfCloudFilterNode::occupancyCallback, this);
    cloud_subscriber_.subscribe(node_, input_topic, 5);
    odom_subscriber_.subscribe(node_, odom_topic, 20);
    synchronizer_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(30), cloud_subscriber_, odom_subscriber_));
    synchronizer_->setMaxIntervalDuration(ros::Duration(synchronization_slop_));
    synchronizer_->registerCallback(
        boost::bind(&SelfCloudFilterNode::cloudCallback, this, _1, _2));

    ROS_WARN("[SELF_FILTER] enabled with %zu boxes and %zu cylinders; "
             "input=%s output=%s odom=%s",
             configuration_.boxes.size(), configuration_.cylinders.size(),
             input_topic.c_str(), output_topic.c_str(), odom_topic.c_str());
  }

 private:
  std::vector<double> requiredVector(const std::string& name) {
    std::vector<double> values;
    if (!private_node_.getParam(name, values) || values.empty()) {
      throw std::runtime_error("missing/non-empty required parameter: " + name);
    }
    return values;
  }

  void loadConfiguration() {
    const std::vector<double> translation =
        requiredVector("self_exclusion/lidar_to_imu_translation");
    const std::vector<double> rotation =
        requiredVector("self_exclusion/lidar_to_imu_rotation");
    if (translation.size() != 3U || rotation.size() != 9U) {
      throw std::runtime_error(
          "self_exclusion extrinsic must contain 3 translation and 9 rotation values");
    }
    configuration_.lidar_to_imu_translation =
        {translation[0], translation[1], translation[2]};
    std::copy(rotation.begin(), rotation.end(),
              configuration_.lidar_to_imu_rotation.begin());

    const auto box_x = requiredVector("self_exclusion/boxes/x");
    const auto box_y = requiredVector("self_exclusion/boxes/y");
    const auto box_z = requiredVector("self_exclusion/boxes/z");
    const auto box_size_x = requiredVector("self_exclusion/boxes/size_x");
    const auto box_size_y = requiredVector("self_exclusion/boxes/size_y");
    const auto box_size_z = requiredVector("self_exclusion/boxes/size_z");
    const auto box_yaw = requiredVector("self_exclusion/boxes/yaw");
    const std::size_t box_count = box_x.size();
    if (box_y.size() != box_count || box_z.size() != box_count ||
        box_size_x.size() != box_count || box_size_y.size() != box_count ||
        box_size_z.size() != box_count || box_yaw.size() != box_count) {
      throw std::runtime_error("self_exclusion box arrays must have equal length");
    }
    for (std::size_t index = 0U; index < box_count; ++index) {
      BoxVolume box;
      box.center = {box_x[index], box_y[index], box_z[index]};
      box.size = {box_size_x[index], box_size_y[index], box_size_z[index]};
      box.yaw = box_yaw[index];
      configuration_.boxes.push_back(box);
    }

    const auto cylinder_x = requiredVector("self_exclusion/cylinders/x");
    const auto cylinder_y = requiredVector("self_exclusion/cylinders/y");
    const auto cylinder_z = requiredVector("self_exclusion/cylinders/z");
    const auto cylinder_radius =
        requiredVector("self_exclusion/cylinders/radius");
    const auto cylinder_z_min =
        requiredVector("self_exclusion/cylinders/z_min");
    const auto cylinder_z_max =
        requiredVector("self_exclusion/cylinders/z_max");
    const std::size_t cylinder_count = cylinder_x.size();
    if (cylinder_y.size() != cylinder_count ||
        cylinder_z.size() != cylinder_count ||
        cylinder_radius.size() != cylinder_count ||
        cylinder_z_min.size() != cylinder_count ||
        cylinder_z_max.size() != cylinder_count) {
      throw std::runtime_error(
          "self_exclusion cylinder arrays must have equal length");
    }
    for (std::size_t index = 0U; index < cylinder_count; ++index) {
      CylinderVolume cylinder;
      cylinder.center =
          {cylinder_x[index], cylinder_y[index], cylinder_z[index]};
      cylinder.radius = cylinder_radius[index];
      cylinder.z_min = cylinder_z_min[index];
      cylinder.z_max = cylinder_z_max[index];
      configuration_.cylinders.push_back(cylinder);
    }
    std::string reason;
    if (!validateConfiguration(configuration_, &reason)) {
      throw std::runtime_error("invalid self-exclusion geometry: " + reason);
    }
  }

  void publishDiagnostic(const std_msgs::Header& header,
                         const std::string& name,
                         const std::vector<diagnostic_msgs::KeyValue>& values,
                         std::uint8_t level = diagnostic_msgs::DiagnosticStatus::OK,
                         const std::string& message = "OK") {
    diagnostic_msgs::DiagnosticArray array;
    array.header = header;
    diagnostic_msgs::DiagnosticStatus status;
    status.level = level;
    status.name = ros::this_node::getNamespace() + "/" + name;
    status.hardware_id = ros::this_node::getName();
    status.message = message;
    status.values = values;
    array.status.push_back(status);
    diagnostic_publisher_.publish(array);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                     const nav_msgs::OdometryConstPtr& odometry) {
    latest_pose_ = odometry->pose.pose;
    latest_odom_stamp_ = odometry->header.stamp;
    have_latest_pose_ = true;
    if (cloud->header.frame_id != odometry->header.frame_id) {
      ROS_ERROR_THROTTLE(1.0,
                         "[SELF_FILTER] cloud frame '%s' != odom frame '%s'",
                         cloud->header.frame_id.c_str(),
                         odometry->header.frame_id.c_str());
      publishDiagnostic(
          cloud->header, "self_filter", {}, diagnostic_msgs::DiagnosticStatus::ERROR,
          "cloud/odometry frame mismatch");
      return;
    }
    sensor_msgs::PointCloud2 filtered;
    FilterStatistics statistics;
    std::string reason;
    if (!filterSelfCloud(*cloud, odometry->pose.pose, configuration_, &filtered,
                         &statistics, &reason)) {
      ROS_ERROR_THROTTLE(1.0, "[SELF_FILTER] %s", reason.c_str());
      publishDiagnostic(cloud->header, "self_filter", {},
                        diagnostic_msgs::DiagnosticStatus::ERROR, reason);
      return;
    }
    before_publisher_.publish(cloud);
    after_publisher_.publish(filtered);
    output_publisher_.publish(filtered);
    publishDiagnostic(
        cloud->header, "self_filter",
        {keyValue("input_points", statistics.input_points),
         keyValue("self_removed_points", statistics.self_removed_points),
         keyValue("output_points", statistics.output_points),
         keyValue("nearest_before_m", statistics.have_nearest_before
                                          ? statistics.nearest_before
                                          : std::numeric_limits<double>::quiet_NaN()),
         keyValue("nearest_after_m", statistics.have_nearest_after
                                         ? statistics.nearest_after
                                         : std::numeric_limits<double>::quiet_NaN()),
         keyValue("cloud_odom_stamp_delta_s",
                  std::abs((cloud->header.stamp - odometry->header.stamp).toSec()))});
    ROS_INFO_THROTTLE(
        1.0,
        "[SELF_FILTER] input=%zu self_removed=%zu output=%zu "
        "nearest_before=%.3f nearest_after=%.3f",
        statistics.input_points, statistics.self_removed_points,
        statistics.output_points,
        statistics.have_nearest_before ? statistics.nearest_before : -1.0,
        statistics.have_nearest_after ? statistics.nearest_after : -1.0);
  }

  void occupancyCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!have_latest_pose_) return;
    if (cloud->header.frame_id.empty()) return;
    double nearest_squared = std::numeric_limits<double>::infinity();
    std::size_t inside = 0U;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      const double radius_squared = collision_radius_ * collision_radius_;
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        const double dx = *x - latest_pose_.position.x;
        const double dy = *y - latest_pose_.position.y;
        const double dz = *z - latest_pose_.position.z;
        const double squared = dx * dx + dy * dy + dz * dz;
        nearest_squared = std::min(nearest_squared, squared);
        if (squared < radius_squared) ++inside;
      }
    } catch (const std::runtime_error& error) {
      publishDiagnostic(cloud->header, "inflated_occupancy", {},
                        diagnostic_msgs::DiagnosticStatus::ERROR, error.what());
      return;
    }
    const double nearest = std::isfinite(nearest_squared)
                               ? std::sqrt(nearest_squared)
                               : std::numeric_limits<double>::quiet_NaN();
    publishDiagnostic(
        cloud->header, "inflated_occupancy",
        {keyValue("inflated_points", pointCount(*cloud)),
         keyValue("nearest_inflated_to_uav_m", nearest),
         keyValue("inflated_points_within_0_55_m", inside),
         keyValue("occupancy_odom_stamp_delta_s",
                  std::abs((cloud->header.stamp - latest_odom_stamp_).toSec()))},
        inside == 0U ? diagnostic_msgs::DiagnosticStatus::OK
                     : diagnostic_msgs::DiagnosticStatus::ERROR,
        inside == 0U ? "clear" : "inflated occupancy inside 0.55 m");
  }

  ros::NodeHandle node_, private_node_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_subscriber_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_subscriber_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  ros::Subscriber occupancy_subscriber_;
  ros::Publisher output_publisher_, before_publisher_, after_publisher_;
  ros::Publisher diagnostic_publisher_;
  SelfMaskConfiguration configuration_;
  geometry_msgs::Pose latest_pose_;
  ros::Time latest_odom_stamp_;
  bool have_latest_pose_{false};
  double synchronization_slop_{0.03};
  double collision_radius_{0.55};
};

}  // namespace astra_swarm_perception

int main(int argc, char** argv) {
  ros::init(argc, argv, "self_cloud_filter");
  try {
    astra_swarm_perception::SelfCloudFilterNode node;
    ros::spin();
  } catch (const std::exception& error) {
    std::cerr << "[SELF_FILTER] " << error.what() << std::endl;
    ROS_FATAL("[SELF_FILTER] %s", error.what());
    return 1;
  }
  return 0;
}
