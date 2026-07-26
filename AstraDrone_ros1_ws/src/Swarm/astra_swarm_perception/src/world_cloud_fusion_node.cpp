#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

class WorldCloudFusion {
 public:
  WorldCloudFusion() : nh_(), pnh_("~") {
    pnh_.param("maximum_age", maximum_age_, 1.0);
    pnh_.param("uav1_home_x", homes_[0][0], 0.0);
    pnh_.param("uav1_home_y", homes_[0][1], 0.0);
    pnh_.param("uav1_home_z", homes_[0][2], 0.0);
    pnh_.param("uav2_home_x", homes_[1][0], 2.0);
    pnh_.param("uav2_home_y", homes_[1][1], 0.0);
    pnh_.param("uav2_home_z", homes_[1][2], 0.0);
    std::string first, second, output;
    pnh_.param<std::string>("uav1_cloud_topic", first,
                            "/uav1/cloud_registered_peer_filtered");
    pnh_.param<std::string>("uav2_cloud_topic", second,
                            "/uav2/cloud_registered_peer_filtered");
    pnh_.param<std::string>("output_topic", output,
                            "/swarm/visualization/cloud_world");
    first_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        first, 1, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) {
          clouds_[0] = msg;
          received_[0] = ros::Time::now();
          publish();
        });
    second_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        second, 1, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) {
          clouds_[1] = msg;
          received_[1] = ros::Time::now();
          publish();
        });
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output, 1);
  }

 private:
  void publish() {
    const ros::Time now = ros::Time::now();
    if (!clouds_[0] || !clouds_[1] ||
        now - received_[0] > ros::Duration(maximum_age_) ||
        now - received_[1] > ros::Duration(maximum_age_)) {
      return;
    }
    std::vector<std::array<float, 3>> points;
    for (std::size_t vehicle = 0; vehicle < 2U; ++vehicle) {
      sensor_msgs::PointCloud2ConstIterator<float> x(*clouds_[vehicle], "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*clouds_[vehicle], "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*clouds_[vehicle], "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          points.push_back({
              static_cast<float>(*x + homes_[vehicle][0]),
              static_cast<float>(*y + homes_[vehicle][1]),
              static_cast<float>(*z + homes_[vehicle][2])});
        }
      }
    }
    sensor_msgs::PointCloud2 output;
    output.header.stamp = now;
    output.header.frame_id = "world";
    output.height = 1;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> z(output, "z");
    for (const auto& point : points) {
      *x = point[0]; *y = point[1]; *z = point[2];
      ++x; ++y; ++z;
    }
    publisher_.publish(output);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber first_sub_, second_sub_;
  ros::Publisher publisher_;
  std::array<sensor_msgs::PointCloud2::ConstPtr, 2> clouds_;
  std::array<ros::Time, 2> received_;
  std::array<std::array<double, 3>, 2> homes_{{{{0.0, 0.0, 0.0}},
                                                {{2.0, 0.0, 0.0}}}};
  double maximum_age_{1.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "world_cloud_fusion");
  WorldCloudFusion node;
  ros::spin();
  return 0;
}
