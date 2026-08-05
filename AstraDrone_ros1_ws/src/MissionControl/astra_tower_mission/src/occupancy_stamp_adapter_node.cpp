#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

namespace astra_tower_mission {

class OccupancyStampAdapter {
 public:
  OccupancyStampAdapter() : private_node_("~") {
    private_node_.param<std::string>("input_topic", input_topic_,
                                     "/grid_map/occupancy_inflate");
    private_node_.param<std::string>("output_topic", output_topic_,
                                     "/stage3/occupancy_inflate");
    private_node_.param<std::string>("planning_frame", planning_frame_,
                                     "camera_init");
    publisher_ = node_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
    subscriber_ = node_.subscribe(input_topic_, 10,
                                  &OccupancyStampAdapter::callback, this);
    ROS_INFO("[OCCUPANCY_STAMP_ADAPTER] %s -> %s (zero stamps use receipt sim time)",
             input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void callback(const sensor_msgs::PointCloud2::ConstPtr& input) {
    if (input->header.frame_id != planning_frame_) {
      ROS_WARN_THROTTLE(5.0,
                        "[OCCUPANCY_STAMP_ADAPTER] reject occupancy frame '%s', expected '%s'",
                        input->header.frame_id.c_str(), planning_frame_.c_str());
      return;
    }
    sensor_msgs::PointCloud2 output = *input;
    if (output.header.stamp.isZero()) {
      const ros::Time receipt = ros::Time::now();
      if (receipt.isZero()) {
        ROS_WARN_THROTTLE(5.0,
                          "[OCCUPANCY_STAMP_ADAPTER] simulation time is zero; occupancy withheld");
        return;
      }
      output.header.stamp = receipt;
    }
    publisher_.publish(output);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string input_topic_, output_topic_, planning_frame_;
};

}  // namespace astra_tower_mission

int main(int argc, char** argv) {
  ros::init(argc, argv, "occupancy_stamp_adapter");
  astra_tower_mission::OccupancyStampAdapter adapter;
  ros::spin();
  return 0;
}
