#ifndef OFFBOARD_TRAJECTORY_REFERENCE_H_
#define OFFBOARD_TRAJECTORY_REFERENCE_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace offboard {

// A position and unit tangent sampled from a closed, planar trajectory.
struct TrajectoryPoint {
    double x;
    double y;
    double tangent_x;
    double tangent_y;
};

// Builds a cumulative arc-length table once, then converts travelled metres
// into a reference point.  This keeps reference speed independent of loop rate.
class TrajectoryReference {
public:
    TrajectoryReference(const std::string& type,
                        double center_x,
                        double center_y,
                        double radius,
                        double side_length,
                        double ellipse_a,
                        double ellipse_b,
                        bool clockwise,
                        int sample_count)
        : type_(type),
          center_x_(center_x),
          center_y_(center_y),
          radius_(radius),
          side_length_(side_length),
          ellipse_a_(ellipse_a),
          ellipse_b_(ellipse_b),
          direction_(clockwise ? -1.0 : 1.0),
          length_(0.0) {
        sample_count = std::max(sample_count, 100);
        if (type_ == "square" && sample_count % 4 != 0) {
            sample_count += 4 - sample_count % 4;
        }

        samples_.reserve(static_cast<std::size_t>(sample_count + 1));
        cumulative_length_.reserve(static_cast<std::size_t>(sample_count + 1));
        for (int i = 0; i <= sample_count; ++i) {
            const double u = static_cast<double>(i) / sample_count;
            const RawPoint point = rawPoint(u);
            samples_.push_back(point);
            if (i == 0) {
                cumulative_length_.push_back(0.0);
                continue;
            }
            const double dx = point.x - samples_[i - 1].x;
            const double dy = point.y - samples_[i - 1].y;
            length_ += std::sqrt(dx * dx + dy * dy);
            cumulative_length_.push_back(length_);
        }
    }

    double length() const { return length_; }
    const std::string& type() const { return type_; }

    TrajectoryPoint sample(double travelled_metres) const {
        double local_s = 0.0;
        if (length_ > 0.0) {
            local_s = std::fmod(std::max(0.0, travelled_metres), length_);
            if (local_s < 0.0) {
                local_s += length_;
            }
        }

        const std::vector<double>::const_iterator upper = std::upper_bound(
            cumulative_length_.begin(), cumulative_length_.end(), local_s);
        std::size_t upper_index = static_cast<std::size_t>(
            std::distance(cumulative_length_.begin(), upper));
        upper_index = std::max<std::size_t>(1, upper_index);
        upper_index = std::min(upper_index, samples_.size() - 1);
        const std::size_t lower_index = upper_index - 1;

        const double segment_length =
            cumulative_length_[upper_index] - cumulative_length_[lower_index];
        const double ratio = segment_length > 1e-9
            ? (local_s - cumulative_length_[lower_index]) / segment_length
            : 0.0;
        const RawPoint& a = samples_[lower_index];
        const RawPoint& b = samples_[upper_index];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double norm = std::sqrt(dx * dx + dy * dy);

        TrajectoryPoint result;
        result.x = a.x + ratio * dx;
        result.y = a.y + ratio * dy;
        result.tangent_x = norm > 1e-9 ? dx / norm : 1.0;
        result.tangent_y = norm > 1e-9 ? dy / norm : 0.0;
        return result;
    }

    // An ideal square has discontinuous tangent at each corner.  Reduce the
    // reference speed smoothly near corners so the vehicle has time to turn.
    double speedScale(double travelled_metres,
                      double slowdown_distance,
                      double minimum_ratio) const {
        if (type_ != "square" || slowdown_distance <= 0.0 || length_ <= 0.0) {
            return 1.0;
        }
        const double edge_length = length_ / 4.0;
        const double local = std::fmod(std::max(0.0, travelled_metres), edge_length);
        const double corner_distance = std::min(local, edge_length - local);
        if (corner_distance >= slowdown_distance) {
            return 1.0;
        }
        const double blend = corner_distance / slowdown_distance;
        return minimum_ratio + (1.0 - minimum_ratio) * blend;
    }

private:
    struct RawPoint {
        double x;
        double y;
    };

    RawPoint rawPoint(double u) const {
        const double pi = 3.14159265358979323846;
        if (type_ == "circle") {
            const double angle = direction_ * 2.0 * pi * u;
            return RawPoint{center_x_ + radius_ * std::cos(angle),
                            center_y_ + radius_ * std::sin(angle)};
        }
        if (type_ == "ellipse") {
            const double angle = direction_ * 2.0 * pi * u;
            return RawPoint{center_x_ + ellipse_a_ * std::cos(angle),
                            center_y_ + ellipse_b_ * std::sin(angle)};
        }
        if (type_ == "figure8") {
            const double angle = direction_ * 2.0 * pi * u;
            return RawPoint{center_x_ + ellipse_a_ * std::sin(angle),
                            center_y_ + ellipse_b_ * std::sin(2.0 * angle)};
        }

        // Counter-clockwise square starts at its lower-right corner.  Reversing
        // the vertex order gives the clockwise version of the same path.
        const double half = side_length_ * 0.5;
        const double vertices_ccw[5][2] = {
            {half, -half}, {half, half}, {-half, half},
            {-half, -half}, {half, -half}};
        const double vertices_cw[5][2] = {
            {half, -half}, {-half, -half}, {-half, half},
            {half, half}, {half, -half}};
        const double (*vertices)[2] = direction_ > 0.0
            ? vertices_ccw : vertices_cw;
        const double scaled = std::min(u * 4.0, 4.0);
        const int edge = std::min(static_cast<int>(scaled), 3);
        const double edge_u = scaled - edge;
        return RawPoint{
            center_x_ + vertices[edge][0] +
                edge_u * (vertices[edge + 1][0] - vertices[edge][0]),
            center_y_ + vertices[edge][1] +
                edge_u * (vertices[edge + 1][1] - vertices[edge][1])};
    }

    std::string type_;
    double center_x_;
    double center_y_;
    double radius_;
    double side_length_;
    double ellipse_a_;
    double ellipse_b_;
    double direction_;
    double length_;
    std::vector<RawPoint> samples_;
    std::vector<double> cumulative_length_;
};

inline double unwrapAngle(double angle, double previous_angle) {
    return previous_angle + std::atan2(std::sin(angle - previous_angle),
                                       std::cos(angle - previous_angle));
}

}  // namespace offboard

#endif  // OFFBOARD_TRAJECTORY_REFERENCE_H_
