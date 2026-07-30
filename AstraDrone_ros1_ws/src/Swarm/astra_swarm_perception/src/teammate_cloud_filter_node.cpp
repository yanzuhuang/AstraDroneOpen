#include <astra_swarm_msgs/SwarmState.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <string>
#include <vector>

class TeammateCloudFilter {
 public:
  TeammateCloudFilter() : nh_(), pnh_("~") {
    pnh_.param("home_x", home_x_, 0.0);
    pnh_.param("home_y", home_y_, 0.0);
    pnh_.param("home_z", home_z_, 0.0);
    pnh_.param("safety_envelope_radius", radius_, 1.2);
    pnh_.param("peer_timeout", timeout_, 1.0);
    std::string input, output;
    pnh_.param<std::string>("input_topic", input, "cloud_registered");
    pnh_.param<std::string>("output_topic", output,
                            "cloud_registered_peer_filtered");
    cloud_sub_ = nh_.subscribe(input, 1, &TeammateCloudFilter::cloudCb, this);
    std::vector<std::string> peer_topics;
    XmlRpc::XmlRpcValue configured_topics;
    if (pnh_.getParam("peer_state_topics", configured_topics) &&
        configured_topics.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int index = 0; index < configured_topics.size(); ++index) {
        if (configured_topics[index].getType() ==
            XmlRpc::XmlRpcValue::TypeString) {
          peer_topics.push_back(
              static_cast<std::string>(configured_topics[index]));
        }
      }
    }
    if (peer_topics.empty()) {
      std::string peer;
      pnh_.param<std::string>("peer_state_topic", peer,
                              "/uav2/swarm/state");
      peer_topics.push_back(peer);
    }
    peers_.resize(peer_topics.size());
    for (std::size_t index = 0; index < peer_topics.size(); ++index) {
      peer_subs_.push_back(nh_.subscribe<astra_swarm_msgs::SwarmState>(
          peer_topics[index], 5,
          boost::bind(&TeammateCloudFilter::peerCb, this, _1, index)));
    }
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output, 1);
  }

 private:
  struct Peer {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    bool valid{false};
    ros::Time received;
  };

  void peerCb(const astra_swarm_msgs::SwarmState::ConstPtr& message,
              std::size_t index) {
    if (index >= peers_.size()) return;
    Peer& peer = peers_[index];
    peer.x = message->pose.position.x - home_x_;
    peer.y = message->pose.position.y - home_y_;
    peer.z = message->pose.position.z - home_z_;
    peer.received = ros::Time::now();
    peer.valid = message->heartbeat_ok && message->localization_valid;
  }

  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& message) {
    const ros::Time now = ros::Time::now();
    bool have_live_peer = false;
    for (const Peer& peer : peers_) {
      have_live_peer |= (
          peer.valid && now - peer.received <= ros::Duration(timeout_));
    }
    if (!have_live_peer) {
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
      bool inside_peer_envelope = false;
      for (const Peer& peer : peers_) {
        if (!peer.valid ||
            now - peer.received > ros::Duration(timeout_)) {
          continue;
        }
        const double dx = *in_x - peer.x;
        const double dy = *in_y - peer.y;
        const double dz = *in_z - peer.z;
        if (dx * dx + dy * dy + dz * dz <= radius_squared) {
          inside_peer_envelope = true;
          break;
        }
      }
      if (inside_peer_envelope) continue;
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
  ros::Subscriber cloud_sub_;
  std::vector<ros::Subscriber> peer_subs_;
  ros::Publisher cloud_pub_;
  double home_x_{0.0}, home_y_{0.0}, home_z_{0.0};
  double radius_{1.2}, timeout_{1.0};
  std::vector<Peer> peers_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "teammate_cloud_filter");
  TeammateCloudFilter node;
  ros::spin();
  return 0;
}
