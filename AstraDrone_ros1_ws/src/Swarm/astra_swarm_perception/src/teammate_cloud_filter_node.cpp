#include <astra_swarm_msgs/SwarmState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <stdexcept>
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
    loadStaticCylinders();
    std::string input, output;
    pnh_.param<std::string>("input_topic", input, "cloud_registered");
    pnh_.param<std::string>("output_topic", output,
                            "cloud_registered_peer_filtered");
    std::string diagnostic_topic;
    pnh_.param<std::string>("diagnostic_topic", diagnostic_topic,
                            "stage5/cloud_filter_diagnostics");
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
    diagnostic_pub_ =
        nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostic_topic, 10);
  }

 private:
  struct StaticPoint {
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
  };

  struct Peer {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    bool valid{false};
    ros::Time received;
  };

  void loadStaticCylinders() {
    std::vector<double> xs, ys, z_mins, z_maxs, radii;
    const bool any =
        pnh_.getParam("static_cylinders/x", xs) ||
        pnh_.getParam("static_cylinders/y", ys) ||
        pnh_.getParam("static_cylinders/z_min", z_mins) ||
        pnh_.getParam("static_cylinders/z_max", z_maxs) ||
        pnh_.getParam("static_cylinders/radius", radii);
    if (!any) return;
    pnh_.getParam("static_cylinders/x", xs);
    pnh_.getParam("static_cylinders/y", ys);
    pnh_.getParam("static_cylinders/z_min", z_mins);
    pnh_.getParam("static_cylinders/z_max", z_maxs);
    pnh_.getParam("static_cylinders/radius", radii);
    if (xs.empty() || xs.size() != ys.size() ||
        xs.size() != z_mins.size() || xs.size() != z_maxs.size() ||
        xs.size() != radii.size()) {
      throw std::runtime_error(
          "static_cylinders arrays must have equal nonzero length");
    }
    double resolution = 0.35;
    pnh_.param("static_cylinders/resolution", resolution, resolution);
    if (!std::isfinite(resolution) || resolution < 0.1 ||
        resolution > 1.0) {
      throw std::runtime_error(
          "static_cylinders/resolution must be in [0.1, 1.0]");
    }
    for (std::size_t index = 0; index < xs.size(); ++index) {
      if (!std::isfinite(xs[index]) || !std::isfinite(ys[index]) ||
          !std::isfinite(z_mins[index]) ||
          !std::isfinite(z_maxs[index]) ||
          !std::isfinite(radii[index]) || radii[index] <= 0.0 ||
          z_maxs[index] <= z_mins[index]) {
        throw std::runtime_error("invalid static cylinder geometry");
      }
      const double radius_squared = radii[index] * radii[index];
      for (double z = z_mins[index];
           z <= z_maxs[index] + 0.5 * resolution; z += resolution) {
        for (double dx = -radii[index];
             dx <= radii[index] + 0.5 * resolution; dx += resolution) {
          for (double dy = -radii[index];
               dy <= radii[index] + 0.5 * resolution; dy += resolution) {
            if (dx * dx + dy * dy > radius_squared) continue;
            StaticPoint point;
            point.x = static_cast<float>(xs[index] + dx);
            point.y = static_cast<float>(ys[index] + dy);
            point.z = static_cast<float>(z);
            static_points_.push_back(point);
          }
        }
      }
    }
    ROS_WARN("[SWARM_PERCEPTION] augmenting every live map cloud with %zu "
             "points representing configured measured static geometry",
             static_points_.size());
  }

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
    if (!have_live_peer && static_points_.empty()) {
      cloud_pub_.publish(message);
      publishDiagnostic(message->header,
                        static_cast<std::size_t>(message->width) * message->height,
                        0U, 0U,
                        static_cast<std::size_t>(message->width) * message->height);
      return;
    }
    sensor_msgs::PointCloud2 output;
    output.header = message->header;
    output.height = 1;
    output.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(
        message->width * message->height + static_points_.size());
    sensor_msgs::PointCloud2ConstIterator<float> in_x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(*message, "z");
    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
    std::size_t kept = 0U;
    std::size_t peer_removed = 0U;
    std::size_t nonfinite_removed = 0U;
    const double radius_squared = radius_ * radius_;
    for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z) {
      if (!std::isfinite(*in_x) || !std::isfinite(*in_y) ||
          !std::isfinite(*in_z)) {
        ++nonfinite_removed;
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
      if (inside_peer_envelope) {
        ++peer_removed;
        continue;
      }
      *out_x = *in_x;
      *out_y = *in_y;
      *out_z = *in_z;
      ++out_x;
      ++out_y;
      ++out_z;
      ++kept;
    }
    for (const StaticPoint& point : static_points_) {
      *out_x = point.x;
      *out_y = point.y;
      *out_z = point.z;
      ++out_x;
      ++out_y;
      ++out_z;
      ++kept;
    }
    modifier.resize(kept);
    output.width = static_cast<std::uint32_t>(kept);
    cloud_pub_.publish(output);
    publishDiagnostic(message->header,
                      static_cast<std::size_t>(message->width) * message->height,
                      peer_removed, nonfinite_removed, kept);
    ROS_INFO_THROTTLE(1.0,
                      "[PEER_FILTER] input=%u peer_removed=%zu "
                      "nonfinite_removed=%zu static_added=%zu output=%zu",
                      message->width * message->height, peer_removed,
                      nonfinite_removed, static_points_.size(), kept);
  }

  void publishDiagnostic(const std_msgs::Header& header, std::size_t input,
                         std::size_t peer_removed,
                         std::size_t nonfinite_removed,
                         std::size_t output) {
    diagnostic_msgs::DiagnosticArray array;
    array.header = header;
    diagnostic_msgs::DiagnosticStatus status;
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.name = ros::this_node::getNamespace() + "/peer_filter";
    status.hardware_id = ros::this_node::getName();
    status.message = "OK";
    const auto add = [&status](const std::string& key, std::size_t value) {
      diagnostic_msgs::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      status.values.push_back(item);
    };
    add("input_points", input);
    add("peer_removed_points", peer_removed);
    add("nonfinite_removed_points", nonfinite_removed);
    add("static_points_added", static_points_.size());
    add("output_points", output);
    array.status.push_back(status);
    diagnostic_pub_.publish(array);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber cloud_sub_;
  std::vector<ros::Subscriber> peer_subs_;
  ros::Publisher cloud_pub_, diagnostic_pub_;
  double home_x_{0.0}, home_y_{0.0}, home_z_{0.0};
  double radius_{1.2}, timeout_{1.0};
  std::vector<Peer> peers_;
  std::vector<StaticPoint> static_points_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "teammate_cloud_filter");
  TeammateCloudFilter node;
  ros::spin();
  return 0;
}
