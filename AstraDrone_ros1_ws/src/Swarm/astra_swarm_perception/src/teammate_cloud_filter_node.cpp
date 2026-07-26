#include <astra_swarm_msgs/SwarmState.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <string>

class TeammateCloudFilter {
 public:
  TeammateCloudFilter() : nh_(), pnh_("~") {
    pnh_.param("home_x", home_x_, 0.0);
    pnh_.param("home_y", home_y_, 0.0);
    pnh_.param("home_z", home_z_, 0.0);
    pnh_.param("safety_envelope_radius", radius_, 1.2);
    pnh_.param("peer_timeout", timeout_, 1.0);
    std::string input, output, peer;
    pnh_.param<std::string>("input_topic", input, "cloud_registered");
    pnh_.param<std::string>("output_topic", output,
                            "cloud_registered_peer_filtered");
    pnh_.param<std::string>("peer_state_topic", peer,
                            "/uav2/swarm/state");
    cloud_sub_ = nh_.subscribe(input, 1, &TeammateCloudFilter::cloudCb, this);
    peer_sub_ = nh_.subscribe(peer, 5, &TeammateCloudFilter::peerCb, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output, 1);
  }

 private:
  void peerCb(const astra_swarm_msgs::SwarmState::ConstPtr& message) {
    peer_x_ = message->pose.position.x - home_x_;
    peer_y_ = message->pose.position.y - home_y_;
    peer_z_ = message->pose.position.z - home_z_;
    peer_received_ = ros::Time::now();
    have_peer_ = message->heartbeat_ok && message->localization_valid;
  }

  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& message) {
    if (!have_peer_ ||
        ros::Time::now() - peer_received_ > ros::Duration(timeout_)) {
      cloud_pub_.publish(message);
      return;
    }
    sensor_msgs::PointCloud2 output;
    output.header = message->header;
    output.height = 1;
    output.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(message->width * message->height);
    sensor_msgs::PointCloud2ConstIterator<float> in_x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(*message, "z");
    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
    std::size_t kept = 0U;
    const double radius_squared = radius_ * radius_;
    for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z) {
      if (!std::isfinite(*in_x) || !std::isfinite(*in_y) ||
          !std::isfinite(*in_z)) {
        continue;
      }
      const double dx = *in_x - peer_x_;
      const double dy = *in_y - peer_y_;
      const double dz = *in_z - peer_z_;
      if (dx * dx + dy * dy + dz * dz <= radius_squared) continue;
      *out_x = *in_x;
      *out_y = *in_y;
      *out_z = *in_z;
      ++out_x;
      ++out_y;
      ++out_z;
      ++kept;
    }
    modifier.resize(kept);
    output.width = static_cast<std::uint32_t>(kept);
    cloud_pub_.publish(output);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber cloud_sub_, peer_sub_;
  ros::Publisher cloud_pub_;
  double home_x_{0.0}, home_y_{0.0}, home_z_{0.0};
  double radius_{1.2}, timeout_{1.0};
  double peer_x_{0.0}, peer_y_{0.0}, peer_z_{0.0};
  bool have_peer_{false};
  ros::Time peer_received_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "teammate_cloud_filter");
  TeammateCloudFilter node;
  ros::spin();
  return 0;
}
