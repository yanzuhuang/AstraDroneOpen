#ifndef BSPLINE_OPT_SHORT_SEGMENT_GUIDANCE_H
#define BSPLINE_OPT_SHORT_SEGMENT_GUIDANCE_H

#include <Eigen/Eigen>

#include <cmath>
#include <limits>
#include <vector>

namespace ego_planner
{

enum class ShortSegmentGuidanceSource
{
  NONE,
  MIDPOINT_INTERSECTION,
  NEAREST_PROJECTED_PATH
};

struct ShortSegmentGuidance
{
  Eigen::Vector3d base_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d safe_path_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d direction{Eigen::Vector3d::Zero()};
  ShortSegmentGuidanceSource source{ShortSegmentGuidanceSource::NONE};
};

inline bool computeShortSegmentGuidance(
    const Eigen::Vector3d& segment_start,
    const Eigen::Vector3d& segment_end,
    const std::vector<Eigen::Vector3d>& safe_path,
    ShortSegmentGuidance& guidance)
{
  guidance = ShortSegmentGuidance();
  if (!segment_start.allFinite() || !segment_end.allFinite() ||
      safe_path.size() < 2)
    return false;
  for (const Eigen::Vector3d& point : safe_path)
    if (!point.allFinite())
      return false;

  const Eigen::Vector3d segment_law = segment_end - segment_start;
  const double segment_norm = segment_law.norm();
  if (!std::isfinite(segment_norm) || segment_norm <= 1.0e-9)
    return false;
  const Eigen::Vector3d tangent = segment_law / segment_norm;
  const Eigen::Vector3d midpoint = 0.5 * (segment_start + segment_end);

  // Preserve the midpoint-plane intersection as the first-choice geometry.
  // Ties are resolved by path order, so identical input is deterministic.
  double best_intersection_rank = std::numeric_limits<double>::infinity();
  Eigen::Vector3d best_intersection = Eigen::Vector3d::Zero();
  bool have_intersection = false;
  const double path_middle = 0.5 * static_cast<double>(safe_path.size() - 1);
  for (std::size_t index = 1; index < safe_path.size(); ++index) {
    const double before = (safe_path[index - 1] - midpoint).dot(segment_law);
    const double after = (safe_path[index] - midpoint).dot(segment_law);
    const double denominator = before - after;
    if (before * after > 0.0 || std::abs(denominator) <= 1.0e-12)
      continue;
    const double alpha = before / denominator;
    if (alpha < 0.0 || alpha > 1.0)
      continue;
    const Eigen::Vector3d intersection =
        safe_path[index - 1] +
        alpha * (safe_path[index] - safe_path[index - 1]);
    if ((intersection - midpoint).norm() <= 0.01)
      continue;
    const double rank = std::abs(
        static_cast<double>(index) - 0.5 - path_middle);
    if (!have_intersection || rank < best_intersection_rank) {
      have_intersection = true;
      best_intersection_rank = rank;
      best_intersection = intersection;
    }
  }

  if (have_intersection) {
    guidance.base_point = segment_start;
    guidance.safe_path_point = best_intersection;
    guidance.direction = (best_intersection - midpoint).normalized();
    guidance.source = ShortSegmentGuidanceSource::MIDPOINT_INTERSECTION;
    return guidance.direction.allFinite();
  }

  // If the plane intersection is absent or geometrically coincident, project
  // the collision midpoint onto every A* polyline edge.  Prefer the candidate
  // with the closest longitudinal relation to the short segment, then the
  // nearest Euclidean point.  A non-zero perpendicular component is required:
  // a collinear/degenerate path cannot define an away-from-collision side and
  // therefore fails closed.
  bool have_projection = false;
  double best_longitudinal = std::numeric_limits<double>::infinity();
  double best_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector3d best_projection = Eigen::Vector3d::Zero();
  for (std::size_t index = 1; index < safe_path.size(); ++index) {
    const Eigen::Vector3d edge = safe_path[index] - safe_path[index - 1];
    const double edge_norm_sq = edge.squaredNorm();
    if (!std::isfinite(edge_norm_sq) || edge_norm_sq <= 1.0e-18)
      continue;
    double alpha = (midpoint - safe_path[index - 1]).dot(edge) / edge_norm_sq;
    alpha = std::max(0.0, std::min(1.0, alpha));
    const Eigen::Vector3d projected = safe_path[index - 1] + alpha * edge;
    const Eigen::Vector3d offset = projected - midpoint;
    const Eigen::Vector3d perpendicular = offset - offset.dot(tangent) * tangent;
    const double perpendicular_norm = perpendicular.norm();
    const double distance = offset.norm();
    if (!std::isfinite(distance) || distance <= 1.0e-9 ||
        perpendicular_norm <= 1.0e-9)
      continue;
    const double longitudinal = std::abs(offset.dot(tangent));
    if (!have_projection ||
        longitudinal < best_longitudinal - 1.0e-12 ||
        (std::abs(longitudinal - best_longitudinal) <= 1.0e-12 &&
         distance < best_distance - 1.0e-12)) {
      have_projection = true;
      best_longitudinal = longitudinal;
      best_distance = distance;
      best_projection = projected;
    }
  }

  if (!have_projection)
    return false;
  guidance.base_point = segment_start;
  guidance.safe_path_point = best_projection;
  guidance.direction = (best_projection - midpoint).normalized();
  guidance.source = ShortSegmentGuidanceSource::NEAREST_PROJECTED_PATH;
  return guidance.direction.allFinite() &&
         std::abs(guidance.direction.norm() - 1.0) <= 1.0e-9;
}

}  // namespace ego_planner

#endif
