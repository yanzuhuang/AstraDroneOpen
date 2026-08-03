#include "ego_gazebo_bridge/cloud_filter.h"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace ego_gazebo_bridge {

class Stage2CloudFilterNode {
 public:
  Stage2CloudFilterNode() : node_(), private_node_("~") {
    private_node_.param<std::string>("input_topic", input_topic_,
                                     "/cloud_registered");
    private_node_.param<std::string>("output_topic", output_topic_,
                                     "/stage2/cloud_registered_filtered");
    private_node_.param<std::string>("frame_id", frame_id_, "camera_init");
    private_node_.param("minimum_z", minimum_z_, 0.2);
    private_node_.param<std::string>("diagnostic_topic", diagnostic_topic_,
                                     "stage5/cloud_filter_diagnostics");
    if (input_topic_.empty() || output_topic_.empty() || frame_id_.empty() ||
        input_topic_ == output_topic_ || !std::isfinite(minimum_z_)) {
      throw std::runtime_error("invalid Stage 2 cloud filter configuration");
    }
    publisher_ = node_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    diagnostic_publisher_ = node_.advertise<diagnostic_msgs::DiagnosticArray>(
        diagnostic_topic_, 10);
    subscriber_ = node_.subscribe(input_topic_, 1,
                                  &Stage2CloudFilterNode::callback, this);
  }

 private:
  void callback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    if (message->header.frame_id != frame_id_) {
      ROS_ERROR_THROTTLE(1.0,
                         "[STAGE2_CLOUD] frame '%s' must equal '%s'",
                         message->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    sensor_msgs::PointCloud2 filtered;
    std::string reason;
    if (!filterCloudByMinimumZ(*message, minimum_z_, &filtered, &reason)) {
      ROS_ERROR_THROTTLE(1.0, "[STAGE2_CLOUD] %s", reason.c_str());
      return;
    }
    if (filtered.width == 0) {
      ROS_WARN_THROTTLE(2.0,
                        "[STAGE2_CLOUD] no points remain above %.2f m",
                        minimum_z_);
    }
    publisher_.publish(filtered);
    std::size_t ground_removed = 0U;
    std::size_t nonfinite_removed = 0U;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) ||
            !std::isfinite(*z)) {
          ++nonfinite_removed;
        } else if (*z < minimum_z_) {
          ++ground_removed;
        }
      }
    } catch (const std::runtime_error& error) {
      ROS_ERROR_THROTTLE(1.0, "[STAGE5_GROUND_FILTER] %s", error.what());
    }
    diagnostic_msgs::DiagnosticArray array;
    array.header = message->header;
    diagnostic_msgs::DiagnosticStatus status;
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.name = ros::this_node::getNamespace() + "/ground_filter";
    status.hardware_id = ros::this_node::getName();
    status.message = "OK";
    const auto add = [&status](const std::string& key, std::size_t value) {
      diagnostic_msgs::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      status.values.push_back(item);
    };
    add("input_points",
        static_cast<std::size_t>(message->width) * message->height);
    add("ground_removed_points", ground_removed);
    add("nonfinite_removed_points", nonfinite_removed);
    add("final_output_points", filtered.width);
    array.status.push_back(status);
    diagnostic_publisher_.publish(array);
    ROS_INFO_THROTTLE(1.0,
                      "[STAGE5_GROUND_FILTER] input=%u ground_removed=%zu "
                      "nonfinite_removed=%zu final_output=%u",
                      message->width * message->height, ground_removed,
                      nonfinite_removed, filtered.width);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_, diagnostic_publisher_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  std::string diagnostic_topic_;
  double minimum_z_{0.2};
};

}  // namespace ego_gazebo_bridge

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_cloud_filter");
  try {
    ego_gazebo_bridge::Stage2CloudFilterNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[STAGE2_CLOUD] %s", exception.what());
    return 1;
  }
  return 0;
}
