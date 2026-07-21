#include "ego_gazebo_bridge/cloud_filter.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

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
    if (input_topic_.empty() || output_topic_.empty() || frame_id_.empty() ||
        input_topic_ == output_topic_ || !std::isfinite(minimum_z_)) {
      throw std::runtime_error("invalid Stage 2 cloud filter configuration");
    }
    publisher_ = node_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
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
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
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
