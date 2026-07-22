#include "astra_tower_mission/stage3_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace astra_tower_mission {
namespace {

double distance2d(double ax, double ay, double bx, double by) {
  return std::hypot(ax - bx, ay - by);
}

double distance3d(double ax, double ay, double az, double bx, double by,
                 double bz) {
  return std::sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by) +
                   (az - bz) * (az - bz));
}

std::string indexedId(int sector, std::size_t index) {
  std::ostringstream stream;
  stream << "s" << sector << "_c" << index;
  return stream.str();
}

double obstacleClearance(const geometry_msgs::Point& point,
                         const StaticObstacle& obstacle,
                         double inflation) {
  if (point.z < obstacle.z_min || point.z > obstacle.z_max) {
    return std::numeric_limits<double>::infinity();
  }
  return distance2d(point.x, point.y, obstacle.x, obstacle.y) -
         obstacle.radius - inflation;
}

}  // namespace

const char* sectorStateName(SectorState state) {
  switch (state) {
    case SectorState::kPending: return "PENDING";
    case SectorState::kEvaluating: return "EVALUATING";
    case SectorState::kTargetLocked: return "TARGET_LOCKED";
    case SectorState::kNavigating: return "NAVIGATING";
    case SectorState::kHolding: return "HOLDING";
    case SectorState::kCovered: return "COVERED";
    case SectorState::kRelocating: return "RELOCATING";
    case SectorState::kRecovering: return "RECOVERING";
    case SectorState::kFailed: return "FAILED";
  }
  return "UNKNOWN";
}

std::vector<Sector> buildInspectionSectors(
    const RouteConfig& route, int sector_count, int layer_count,
    double angle_half_width_deg, double radius_half_width,
    double height_half_width, const std::vector<CandidateOffset>& offsets) {
  if (sector_count < 1 || layer_count != 1 || offsets.empty()) return {};
  std::vector<Sector> sectors;
  sectors.reserve(static_cast<std::size_t>(sector_count));
  const double step = 2.0 * kPi / sector_count;
  const double angle_half_width = angle_half_width_deg * kPi / 180.0;
  for (int index = 0; index < sector_count; ++index) {
    Sector sector;
    sector.sector_id = index;
    sector.layer_id = 0;
    sector.nominal_angle_rad = normalizeAngle(route.start_angle_rad +
                                               (route.direction ==
                                                        OrbitDirection::kCounterClockwise
                                                    ? 1.0
                                                    : -1.0) *
                                                   index * step);
    sector.nominal_radius = route.radius;
    sector.nominal_height = route.height;
    sector.center_x = route.center_x;
    sector.center_y = route.center_y;
    sector.tower_collision_radius = route.tower_collision_radius;
    sector.min_angle_rad = sector.nominal_angle_rad - angle_half_width;
    sector.max_angle_rad = sector.nominal_angle_rad + angle_half_width;
    sector.min_radius = route.radius - radius_half_width;
    sector.max_radius = route.radius + radius_half_width;
    sector.min_height = std::max(route.minimum_height,
                                 route.height - height_half_width);
    sector.max_height = std::min(route.maximum_height,
                                 route.height + height_half_width);
    for (std::size_t offset_index = 0; offset_index < offsets.size();
         ++offset_index) {
      const auto& offset = offsets[offset_index];
      const double theta = normalizeAngle(sector.nominal_angle_rad +
                                           offset.angle_deg * kPi / 180.0);
      CandidatePoint candidate;
      candidate.id = indexedId(index, offset_index);
      candidate.sector_id = index;
      candidate.layer_id = 0;
      const double radius = route.radius + offset.radius_m;
      candidate.x = route.center_x + radius * std::cos(theta);
      candidate.y = route.center_y + radius * std::sin(theta);
      candidate.z = route.height + offset.height_m;
      candidate.yaw = normalizeAngle(
          std::atan2(route.center_y - candidate.y,
                     route.center_x - candidate.x) -
          route.camera_yaw_offset_rad);
      sector.candidates.push_back(candidate);
    }
    sectors.push_back(sector);
  }
  return sectors;
}

bool pointInObstacle(const geometry_msgs::Point& point,
                     const StaticObstacle& obstacle, double inflation) {
  return obstacleClearance(point, obstacle, inflation) < 0.0;
}

bool lineCorridorSafe(const geometry_msgs::Point& from,
                      const geometry_msgs::Point& to,
                      const std::vector<geometry_msgs::Point>& cloud_points,
                      const std::vector<StaticObstacle>& obstacles,
                      double inflation, double sample_step) {
  const double length = distance3d(from.x, from.y, from.z, to.x, to.y, to.z);
  const std::size_t samples = static_cast<std::size_t>(std::max(
      1.0, std::ceil(length / std::max(0.05, sample_step))));
  for (std::size_t sample = 0; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / samples;
    geometry_msgs::Point point;
    point.x = from.x + ratio * (to.x - from.x);
    point.y = from.y + ratio * (to.y - from.y);
    point.z = from.z + ratio * (to.z - from.z);
    for (const auto& obstacle : obstacles) {
      if (pointInObstacle(point, obstacle, inflation)) return false;
    }
    for (const auto& cloud : cloud_points) {
      if (distance3d(point.x, point.y, point.z, cloud.x, cloud.y, cloud.z) <
          inflation) {
        return false;
      }
    }
  }
  return true;
}

bool evaluateCandidate(CandidatePoint* candidate, const Sector& sector,
                       const geometry_msgs::Point& current_position,
                       const std::vector<geometry_msgs::Point>& cloud_points,
                       const std::vector<StaticObstacle>& obstacles,
                       bool map_fresh, const CandidateFilterConfig& config,
                       const CandidatePoint* previous_target) {
  if (candidate == nullptr) return false;
  candidate->accepted = false;
  candidate->rejection_reason.clear();
  candidate->score = -1.0e9;
  candidate->clearance = std::numeric_limits<double>::infinity();
  geometry_msgs::Point target;
  target.x = candidate->x;
  target.y = candidate->y;
  target.z = candidate->z;
  const auto reject = [candidate](const std::string& reason) {
    candidate->rejection_reason = reason;
    return false;
  };
  if (!map_fresh) return reject("MAP_STALE");
  if (!std::isfinite(candidate->x) || !std::isfinite(candidate->y) ||
      !std::isfinite(candidate->z) || candidate->z < sector.min_height ||
      candidate->z > sector.max_height) {
    return reject("OUT_OF_BOUNDS");
  }
  const double tower_clearance =
      distance2d(candidate->x, candidate->y, sector.center_x, sector.center_y) -
      sector.tower_collision_radius - config.tower_extra_clearance;
  candidate->clearance = tower_clearance;
  if (tower_clearance < config.minimum_clearance) {
    return reject("TOWER_KEEP_OUT");
  }
  const double candidate_radius =
      distance2d(candidate->x, candidate->y, sector.center_x, sector.center_y);
  const double candidate_angle = normalizeAngle(
      std::atan2(candidate->y - sector.center_y, candidate->x - sector.center_x));
  const double angle_delta = normalizeAngle(candidate_angle -
                                             sector.nominal_angle_rad);
  if (candidate_radius < sector.min_radius || candidate_radius > sector.max_radius ||
      std::abs(angle_delta) >
          std::max(std::abs(normalizeAngle(sector.max_angle_rad -
                                           sector.nominal_angle_rad)),
                   std::abs(normalizeAngle(sector.min_angle_rad -
                                           sector.nominal_angle_rad)))) {
    return reject("SECTOR_BOUNDARY");
  }
  for (const auto& obstacle : obstacles) {
    const double clearance = obstacleClearance(target, obstacle,
                                               config.cloud_inflation);
    candidate->clearance = std::min(candidate->clearance, clearance);
    if (clearance < config.minimum_clearance) {
      return reject("KNOWN_OBSTACLE_CLEARANCE");
    }
  }
  for (const auto& cloud : cloud_points) {
    const double clearance = distance3d(target.x, target.y, target.z,
                                         cloud.x, cloud.y, cloud.z) -
                             config.cloud_inflation;
    candidate->clearance = std::min(candidate->clearance, clearance);
    if (clearance < config.minimum_clearance) {
      return reject("OCCUPANCY_OR_CLEARANCE");
    }
  }
  if (!lineCorridorSafe(current_position, target, cloud_points, obstacles,
                        config.minimum_clearance + config.cloud_inflation,
                        config.corridor_sample_step)) {
    return reject("NO_LOCAL_CORRIDOR");
  }
  candidate->unknown_ratio = 0.0;
  if (candidate->unknown_ratio > config.unknown_ratio_limit) {
    return reject("UNKNOWN_REGION");
  }
  if (!std::isfinite(candidate->clearance)) candidate->clearance = 100.0;
  const double nominal_distance =
      distance3d(candidate->x, candidate->y, candidate->z, 0.0, 0.0, 0.0);
  const double current_distance = distance3d(candidate->x, candidate->y,
                                              candidate->z,
                                              current_position.x,
                                              current_position.y,
                                              current_position.z);
  const double nominal_error = std::abs(candidate->z - sector.nominal_height);
  const double continuity_error = previous_target == nullptr
                                      ? 0.0
                                      : distance3d(candidate->x, candidate->y,
                                                   candidate->z,
                                                   previous_target->x,
                                                   previous_target->y,
                                                   previous_target->z);
  candidate->score = config.score_clearance_weight * candidate->clearance -
                     config.score_nominal_weight * nominal_error -
                     config.score_distance_weight * current_distance -
                     config.score_continuity_weight * continuity_error -
                     config.score_unknown_weight * candidate->unknown_ratio -
                     0.001 * nominal_distance;
  candidate->accepted = true;
  return true;
}

int chooseBestCandidate(const Sector& sector, const CandidatePoint* locked,
                        double replacement_margin) {
  int best = -1;
  double best_score = -1.0e9;
  for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
    const auto& candidate = sector.candidates[index];
    if (candidate.accepted && candidate.score > best_score) {
      best = static_cast<int>(index);
      best_score = candidate.score;
    }
  }
  if (locked == nullptr || best < 0) return best;
  if (locked->accepted && locked->score + replacement_margin >= best_score) {
    for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
      if (sector.candidates[index].id == locked->id &&
          sector.candidates[index].accepted) {
        return static_cast<int>(index);
      }
    }
  }
  return best;
}

RecoveryTargets makeRecoveryTargets(const RouteConfig& route,
                                    const geometry_msgs::Point& current,
                                    const Sector& sector,
                                    const RecoveryConfig& config) {
  RecoveryTargets targets;
  const double dx = current.x - route.center_x;
  const double dy = current.y - route.center_y;
  const double current_radius = std::max(0.1, std::hypot(dx, dy));
  const double ux = dx / current_radius;
  const double uy = dy / current_radius;
  const double sign = config.direction == OrbitDirection::kCounterClockwise
                          ? 1.0
                          : -1.0;
  const double tangent_x = -uy * sign;
  const double tangent_y = ux * sign;
  const double recovery_radius = std::min(
      config.maximum_radius, current_radius + std::max(0.1, config.radial_step));
  auto make = [&](const std::string& id, double x, double y) {
    CandidatePoint point;
    point.id = id;
    point.sector_id = sector.sector_id;
    point.layer_id = sector.layer_id;
    point.x = x;
    point.y = y;
    point.z = config.recovery_height;
    point.yaw = normalizeAngle(std::atan2(route.center_y - y,
                                          route.center_x - x));
    return point;
  };
  targets.r1 = make("recovery_r1", route.center_x + ux * recovery_radius,
                    route.center_y + uy * recovery_radius);
  targets.r2 = make("recovery_r2", targets.r1.x + tangent_x * config.tangent_step,
                    targets.r1.y + tangent_y * config.tangent_step);
  const double target_radius = std::max(route.radius, sector.nominal_radius);
  const double entry_angle = sector.nominal_angle_rad;
  targets.reentry = make(
      "recovery_reentry", route.center_x + target_radius * std::cos(entry_angle),
      route.center_y + target_radius * std::sin(entry_angle));
  targets.reentry.z = sector.nominal_height;
  return targets;
}

}  // namespace astra_tower_mission
