#include "astra_tower_mission/inspection_candidate_planner.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <utility>

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

double pointSegmentDistance(const geometry_msgs::Point& point,
                            const geometry_msgs::Point& start,
                            const geometry_msgs::Point& finish) {
  const double vx = finish.x - start.x;
  const double vy = finish.y - start.y;
  const double vz = finish.z - start.z;
  const double length_squared = vx * vx + vy * vy + vz * vz;
  if (length_squared <= 1.0e-12) {
    return distance3d(point.x, point.y, point.z,
                      start.x, start.y, start.z);
  }
  const double projection = std::max(
      0.0, std::min(1.0,
                    ((point.x - start.x) * vx +
                     (point.y - start.y) * vy +
                     (point.z - start.z) * vz) /
                        length_squared));
  return distance3d(point.x, point.y, point.z,
                    start.x + projection * vx,
                    start.y + projection * vy,
                    start.z + projection * vz);
}

std::string indexedId(int sector, std::size_t index) {
  std::ostringstream stream;
  stream << "s" << sector << "_c" << index;
  return stream.str();
}

double obstacleClearance(const geometry_msgs::Point& point,
                         const StaticObstacle& obstacle,
                         double inflation) {
  if (obstacle.half_extent_x > 0.0 && obstacle.half_extent_y > 0.0 &&
      std::isfinite(obstacle.z_min) && std::isfinite(obstacle.z_max)) {
    const double cosine = std::cos(obstacle.yaw);
    const double sine = std::sin(obstacle.yaw);
    const double dx = point.x - obstacle.x;
    const double dy = point.y - obstacle.y;
    const double local_x = cosine * dx + sine * dy;
    const double local_y = -sine * dx + cosine * dy;
    const double center_z = 0.5 * (obstacle.z_min + obstacle.z_max);
    const double half_extent_z = 0.5 * (obstacle.z_max - obstacle.z_min);
    const double qx = std::abs(local_x) - obstacle.half_extent_x;
    const double qy = std::abs(local_y) - obstacle.half_extent_y;
    const double qz = std::abs(point.z - center_z) - half_extent_z;
    const double outside = std::sqrt(
        std::max(qx, 0.0) * std::max(qx, 0.0) +
        std::max(qy, 0.0) * std::max(qy, 0.0) +
        std::max(qz, 0.0) * std::max(qz, 0.0));
    const double inside = std::min(std::max(qx, std::max(qy, qz)), 0.0);
    return outside + inside - inflation;
  }
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

double mappedTaskClearance(double minimum_clearance,
                           bool map_points_are_inflated,
                           double map_additional_clearance) {
  return map_points_are_inflated ? map_additional_clearance
                                 : minimum_clearance;
}

double mappedTaskClearance(const CandidateFilterConfig& config) {
  return mappedTaskClearance(config.minimum_clearance,
                             config.map_points_are_inflated,
                             config.map_additional_clearance);
}

double mappedTaskClearance(const EntryGateConfig& config) {
  return mappedTaskClearance(config.minimum_clearance,
                             config.map_points_are_inflated,
                             config.map_additional_clearance);
}

bool mappedEndpointClear(
    const CandidatePoint& target,
    const std::vector<geometry_msgs::Point>& map_points,
    double clearance) {
  if (!std::isfinite(target.x) || !std::isfinite(target.y) ||
      !std::isfinite(target.z) || !std::isfinite(clearance) ||
      clearance < 0.0) {
    return false;
  }
  for (const auto& occupied : map_points) {
    if (distance3d(target.x, target.y, target.z,
                   occupied.x, occupied.y, occupied.z) < clearance) {
      return false;
    }
  }
  return true;
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
    // Standard waypoint numbers are fixed counter-clockwise in the map:
    // waypoint 1 is start_angle_rad, waypoint 2 is one positive angular
    // step later, and so on. Flight direction only reorders these fixed
    // waypoints; it must not change their map angle or number.
    sector.nominal_angle_rad =
        normalizeAngle(route.start_angle_rad + index * step);
    sector.nominal_radius = route.radius;
    sector.nominal_height = route.height;
    sector.center_x = route.center_x;
    sector.center_y = route.center_y;
    sector.tower_collision_radius = route.tower_collision_radius;
    sector.minimum_tower_clearance = route.minimum_safety_distance;
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

std::string hardStaticEndpointBlockage(
    const CandidatePoint& target,
    const std::vector<StaticObstacle>& obstacles,
    const std::string& tower_obstacle_id,
    double minimum_clearance,
    bool known_obstacle_is_hard_constraint) {
  geometry_msgs::Point point;
  point.x = target.x;
  point.y = target.y;
  point.z = target.z;
  for (const auto& obstacle : obstacles) {
    const bool tower_body = obstacle.id == tower_obstacle_id;
    if ((tower_body || known_obstacle_is_hard_constraint) &&
        pointInObstacle(point, obstacle, minimum_clearance)) {
      return obstacle.id;
    }
  }
  return std::string();
}

CorridorCheckResult lineCorridorCheck(
    const geometry_msgs::Point& from,
    const geometry_msgs::Point& to,
    const std::vector<geometry_msgs::Point>& cloud_points,
    const std::vector<StaticObstacle>& obstacles,
    double inflation,
    double sample_step) {
  CorridorCheckResult result;
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
      if (pointInObstacle(point, obstacle, inflation)) {
        result.safe = false;
        result.has_blocking_point = true;
        result.blocking_point = point;
        result.blocking_point_to_corridor_distance = 0.0;
        return result;
      }
    }
    for (const auto& cloud : cloud_points) {
      if (distance3d(point.x, point.y, point.z, cloud.x, cloud.y, cloud.z) <
          inflation) {
        result.safe = false;
        result.has_blocking_point = true;
        result.blocking_point = cloud;
        result.blocking_point_to_corridor_distance =
            pointSegmentDistance(cloud, from, to);
        return result;
      }
    }
  }
  return result;
}

bool lineCorridorSafe(const geometry_msgs::Point& from,
                      const geometry_msgs::Point& to,
                      const std::vector<geometry_msgs::Point>& cloud_points,
                      const std::vector<StaticObstacle>& obstacles,
                      double inflation, double sample_step) {
  return lineCorridorCheck(from, to, cloud_points, obstacles, inflation,
                           sample_step)
      .safe;
}

bool evaluateCandidate(CandidatePoint* candidate, const Sector& sector,
                       const geometry_msgs::Point& current_position,
                       const std::vector<geometry_msgs::Point>& cloud_points,
                       const std::vector<StaticObstacle>& obstacles,
                       bool map_fresh, const CandidateFilterConfig& config,
                       const CandidatePoint* previous_target,
                       double unknown_ratio) {
  if (candidate == nullptr) return false;
  candidate->accepted = false;
  candidate->target_invalid = false;
  candidate->straight_corridor_blocked = false;
  candidate->planner_unreachable = false;
  candidate->rejection_reason.clear();
  candidate->risk_reason.clear();
  candidate->score = -1.0e9;
  candidate->clearance = std::numeric_limits<double>::infinity();
  geometry_msgs::Point target;
  target.x = candidate->x;
  target.y = candidate->y;
  target.z = candidate->z;
  const auto reject = [candidate](const std::string& reason) {
    candidate->target_invalid = true;
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
  if (tower_clearance < sector.minimum_tower_clearance) {
    return reject("TOWER_KEEP_OUT");
  }
  const double candidate_radius =
      distance2d(candidate->x, candidate->y, sector.center_x, sector.center_y);
  const double candidate_angle = normalizeAngle(
      std::atan2(candidate->y - sector.center_y, candidate->x - sector.center_x));
  const double angle_delta = normalizeAngle(candidate_angle -
                                             sector.nominal_angle_rad);
  // Candidate coordinates are generated with sin/cos and then converted back
  // with hypot/atan2.  Keep the configured boundary inclusive despite the
  // resulting sub-nanometre round-off (notably the 16.5 m outer candidate).
  constexpr double kBoundaryEpsilon = 1.0e-6;
  if (candidate_radius < sector.min_radius - kBoundaryEpsilon ||
      candidate_radius > sector.max_radius + kBoundaryEpsilon ||
      std::abs(angle_delta) > kBoundaryEpsilon +
          std::max(std::abs(normalizeAngle(sector.max_angle_rad -
                                           sector.nominal_angle_rad)),
                   std::abs(normalizeAngle(sector.min_angle_rad -
                                           sector.nominal_angle_rad)))) {
    return reject("SECTOR_BOUNDARY");
  }
  for (const auto& obstacle : obstacles) {
    // StaticObstacle already describes the audited physical/coarse envelope.
    // Apply minimum_clearance exactly once to the audited physical/coarse
    // envelope supplied by the producer.
    const double clearance = obstacleClearance(target, obstacle, 0.0);
    if (config.known_obstacle_is_hard_constraint) {
      candidate->clearance = std::min(candidate->clearance, clearance);
    }
    if (clearance < config.minimum_clearance &&
        config.known_obstacle_is_hard_constraint) {
      return reject("KNOWN_OBSTACLE_CLEARANCE");
    }
    if (clearance < config.minimum_clearance &&
        candidate->risk_reason.empty()) {
      candidate->risk_reason = "COARSE_KNOWN_OBSTACLE_OVERLAP";
    }
  }
  const double map_required_clearance = mappedTaskClearance(config);
  for (const auto& cloud : cloud_points) {
    const double clearance = distance3d(target.x, target.y, target.z,
                                         cloud.x, cloud.y, cloud.z);
    candidate->clearance = std::min(candidate->clearance, clearance);
    if (clearance < map_required_clearance) {
      return reject("OCCUPANCY_OR_CLEARANCE");
    }
  }
  static const std::vector<geometry_msgs::Point> no_map_points;
  static const std::vector<StaticObstacle> no_static_obstacles;
  const bool static_corridor_safe = lineCorridorSafe(
      current_position, target, no_map_points, obstacles,
      config.minimum_clearance,
      config.corridor_sample_step);
  const bool map_corridor_safe = lineCorridorSafe(
      current_position, target, cloud_points, no_static_obstacles,
      map_required_clearance,
      config.corridor_sample_step);
  // Known world geometry is deterministic and already carries its audited
  // physical envelope.  Letting a fixed sector leg cross it and hoping for a
  // late local detour can place the vehicle inside an inflated cell before
  // the next task update.  Reject only this known-static corridor here;
  // live-map corridor occupancy remains a soft EGO replanning hint below.
  if (!static_corridor_safe &&
      config.known_obstacle_corridor_is_hard_constraint) {
    candidate->straight_corridor_blocked = true;
    return reject("KNOWN_OBSTACLE_CORRIDOR");
  }
  candidate->straight_corridor_blocked =
      !static_corridor_safe || !map_corridor_safe;
  if (candidate->straight_corridor_blocked) {
    if (candidate->risk_reason.empty()) {
      candidate->risk_reason = "STRAIGHT_CORRIDOR_BLOCKED";
    } else {
      candidate->risk_reason += ";STRAIGHT_CORRIDOR_BLOCKED";
    }
  }
  candidate->unknown_ratio = unknown_ratio;
  if (candidate->unknown_ratio > config.unknown_ratio_limit) {
    if (config.unknown_is_hard_constraint) {
      return reject("UNKNOWN_REGION");
    }
    if (candidate->risk_reason.empty()) {
      candidate->risk_reason = "UNKNOWN_REGION_DIAGNOSTIC";
    } else {
      candidate->risk_reason += ";UNKNOWN_REGION_DIAGNOSTIC";
    }
  }
  if (!std::isfinite(candidate->clearance)) candidate->clearance = 100.0;
  const double angle_error =
      std::abs(normalizeAngle(candidate_angle - sector.nominal_angle_rad));
  const double radius_error =
      std::abs(candidate_radius - sector.nominal_radius);
  const double height_error =
      std::abs(candidate->z - sector.nominal_height);
  candidate->nominal_deviation = distance3d(
      candidate->x, candidate->y, candidate->z,
      sector.center_x + sector.nominal_radius *
                            std::cos(sector.nominal_angle_rad),
      sector.center_y + sector.nominal_radius *
                            std::sin(sector.nominal_angle_rad),
      sector.nominal_height);
  candidate->height_deviation = height_error;
  candidate->observation_deviation =
      radius_error + sector.nominal_radius * angle_error;
  const double current_distance = distance3d(candidate->x, candidate->y,
                                              candidate->z,
                                              current_position.x,
                                              current_position.y,
                                              current_position.z);
  const double continuity_error = previous_target == nullptr
                                      ? 0.0
                                      : distance3d(candidate->x, candidate->y,
                                                   candidate->z,
                                                   previous_target->x,
                                                   previous_target->y,
                                                   previous_target->z);
  candidate->continuity_error = continuity_error;
  candidate->route_distance = current_distance;
  const double epsilon = 1.0e-6;
  const double small_angle =
      config.small_angle_offset_deg * kPi / 180.0 + epsilon;
  if (angle_error < epsilon && radius_error < epsilon &&
      height_error < epsilon) {
    candidate->priority = 0;
  } else if (height_error < epsilon && radius_error < epsilon &&
             angle_error <= small_angle) {
    candidate->priority = 1;
  } else if (height_error < epsilon && angle_error < epsilon &&
             radius_error <= config.small_radius_offset_m + epsilon) {
    candidate->priority = 2;
  } else if (height_error > epsilon &&
             height_error <= config.small_height_offset_m + epsilon) {
    candidate->priority = 3;
  } else if (height_error < epsilon) {
    candidate->priority = 4;
  } else {
    candidate->priority = 5;
  }
  candidate->score = config.score_clearance_weight * candidate->clearance -
                     config.score_radius_weight * radius_error -
                     config.score_sector_weight *
                         sector.nominal_radius * angle_error -
                     config.score_height_weight * height_error -
                     config.score_distance_weight * current_distance -
                     config.score_continuity_weight * continuity_error -
                     config.score_unknown_weight * candidate->unknown_ratio -
                     (candidate->straight_corridor_blocked
                          ? config.blocked_corridor_penalty
                          : 0.0);
  candidate->accepted = true;
  return true;
}

int chooseBestCandidate(const Sector& sector, const CandidatePoint* locked,
                        double replacement_margin,
                        bool prefer_clear_straight_corridor) {
  const bool have_clear_straight_corridor =
      prefer_clear_straight_corridor &&
      std::any_of(sector.candidates.begin(), sector.candidates.end(),
                  [](const CandidatePoint& candidate) {
                    return candidate.accepted &&
                           !candidate.straight_corridor_blocked;
                  });
  const auto eligible = [have_clear_straight_corridor](
                            const CandidatePoint& candidate) {
    return candidate.accepted &&
           (!have_clear_straight_corridor ||
            !candidate.straight_corridor_blocked);
  };
  // Candidate generation places the exact nominal point first, but identify
  // it geometrically so configuration ordering cannot silently change the
  // safety policy. A usable nominal target P normally wins.  Stage 5 may
  // additionally require a clear straight corridor to win over a nominal or
  // high-score soft-risk candidate; if no such candidate exists, the
  // original EGO-detour candidate set remains available.
  for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
    const auto& candidate = sector.candidates[index];
    const double radius = distance2d(candidate.x, candidate.y,
                                     sector.center_x, sector.center_y);
    const double angle = std::atan2(candidate.y - sector.center_y,
                                    candidate.x - sector.center_x);
    if (eligible(candidate) &&
        std::abs(radius - sector.nominal_radius) < 1.0e-6 &&
        std::abs(normalizeAngle(angle - sector.nominal_angle_rad)) < 1.0e-6 &&
        std::abs(candidate.z - sector.nominal_height) < 1.0e-6) {
      return static_cast<int>(index);
    }
  }
  int best = -1;
  for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
    const auto& candidate = sector.candidates[index];
    if (!eligible(candidate)) continue;
    if (best < 0 || candidate.score > sector.candidates[best].score + 1.0e-9 ||
        (std::abs(candidate.score - sector.candidates[best].score) <= 1.0e-9 &&
         std::make_tuple(candidate.observation_deviation,
                         candidate.height_deviation,
                         candidate.route_distance, candidate.id) <
             std::make_tuple(sector.candidates[best].observation_deviation,
                             sector.candidates[best].height_deviation,
                             sector.candidates[best].route_distance,
                             sector.candidates[best].id))) {
      best = static_cast<int>(index);
    }
  }
  if (locked == nullptr || best < 0) return best;
  if (eligible(*locked) &&
      locked->score + replacement_margin >= sector.candidates[best].score) {
    for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
      if (sector.candidates[index].id == locked->id &&
          eligible(sector.candidates[index])) {
        return static_cast<int>(index);
      }
    }
  }
  return best;
}

int chooseBestCandidateByRadiusTier(
    const Sector& sector, const CandidatePoint* locked,
    double replacement_margin, bool prefer_clear_straight_corridor,
    double radius_step) {
  if (!std::isfinite(radius_step) || radius_step <= 0.0) return -1;

  constexpr double kRadiusEpsilon = 1.0e-6;
  constexpr double kAngleEpsilon = 1.0e-6;
  const auto radius_tier = [&sector, radius_step](
                               const CandidatePoint& candidate) {
    const double radius = distance2d(candidate.x, candidate.y,
                                     sector.center_x, sector.center_y);
    const double raw_tier = (radius - sector.nominal_radius) / radius_step;
    const int tier = static_cast<int>(std::lround(raw_tier));
    if (tier < 0 || tier > 2 ||
        std::abs(raw_tier - static_cast<double>(tier)) > 1.0e-6) {
      return -1;
    }
    return tier;
  };

  for (int tier = 0; tier <= 2; ++tier) {
    const bool have_clear_straight_corridor =
        prefer_clear_straight_corridor &&
        std::any_of(sector.candidates.begin(), sector.candidates.end(),
                    [&](const CandidatePoint& candidate) {
                      return candidate.accepted &&
                             radius_tier(candidate) == tier &&
                             !candidate.straight_corridor_blocked;
                    });
    const auto eligible = [&](const CandidatePoint& candidate) {
      return candidate.accepted && radius_tier(candidate) == tier &&
             (!have_clear_straight_corridor ||
              !candidate.straight_corridor_blocked);
    };
    if (!std::any_of(sector.candidates.begin(), sector.candidates.end(),
                     eligible)) {
      continue;
    }

    // A still-safe lock in the current minimum tier is the most stable
    // choice after a HOLD/map refresh.  A lower radius tier becoming safe
    // still wins because it is examined before the lock's tier.
    if (locked != nullptr && eligible(*locked)) {
      for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
        if (sector.candidates[index].id == locked->id &&
            eligible(sector.candidates[index])) {
          return static_cast<int>(index);
        }
      }
    }

    // Within a radius tier, retain the established exact-angle preference.
    for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
      const CandidatePoint& candidate = sector.candidates[index];
      const double angle = std::atan2(candidate.y - sector.center_y,
                                      candidate.x - sector.center_x);
      if (eligible(candidate) &&
          std::abs(normalizeAngle(angle - sector.nominal_angle_rad)) <
              kAngleEpsilon &&
          std::abs(candidate.z - sector.nominal_height) < kRadiusEpsilon) {
        return static_cast<int>(index);
      }
    }

    int best = -1;
    for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
      const CandidatePoint& candidate = sector.candidates[index];
      if (!eligible(candidate)) continue;
      if (best < 0 ||
          candidate.score > sector.candidates[best].score + 1.0e-9 ||
          (std::abs(candidate.score - sector.candidates[best].score) <=
               1.0e-9 &&
           std::make_tuple(candidate.observation_deviation,
                           candidate.height_deviation,
                           candidate.route_distance, candidate.id) <
               std::make_tuple(
                   sector.candidates[best].observation_deviation,
                   sector.candidates[best].height_deviation,
                   sector.candidates[best].route_distance,
                   sector.candidates[best].id))) {
        best = static_cast<int>(index);
      }
    }
    if (best < 0 || locked == nullptr || !eligible(*locked)) return best;
    if (locked->score + replacement_margin >=
        sector.candidates[best].score) {
      for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
        if (sector.candidates[index].id == locked->id &&
            eligible(sector.candidates[index])) {
          return static_cast<int>(index);
        }
      }
    }
    return best;
  }
  return -1;
}

double directedOrbitTargetProgress(double target_angle,
                                   double orbit_start_angle,
                                   OrbitDirection direction,
                                   bool closing_lap) {
  double progress = direction == OrbitDirection::kCounterClockwise
                        ? normalizeAngle(target_angle - orbit_start_angle)
                        : normalizeAngle(orbit_start_angle - target_angle);
  if (progress < 0.0) progress += 2.0 * kPi;
  if (closing_lap && progress < kPi / 2.0) progress += 2.0 * kPi;
  return progress;
}

bool orbitTargetAtOrAhead(double target_angle, double orbit_start_angle,
                          OrbitDirection direction, double minimum_progress,
                          bool closing_lap, double epsilon) {
  if (!std::isfinite(target_angle) || !std::isfinite(orbit_start_angle) ||
      !std::isfinite(minimum_progress) || !std::isfinite(epsilon) ||
      epsilon < 0.0) {
    return false;
  }
  return directedOrbitTargetProgress(target_angle, orbit_start_angle,
                                     direction, closing_lap) +
             epsilon >=
         minimum_progress;
}

std::vector<CandidatePoint> buildEntryGateCandidates(
    const RouteConfig& route, int entry_sector_user, double minimum_radius,
    double maximum_radius, double preferred_radius,
    double angular_sample_step_deg, double radial_sample_step,
    double inspection_height) {
  std::vector<CandidatePoint> candidates;
  if (entry_sector_user < 1 || entry_sector_user > 8 ||
      !std::isfinite(minimum_radius) || !std::isfinite(maximum_radius) ||
      !std::isfinite(preferred_radius) ||
      !std::isfinite(angular_sample_step_deg) ||
      !std::isfinite(radial_sample_step) ||
      !std::isfinite(inspection_height) ||
      minimum_radius <= 0.0 || maximum_radius < minimum_radius ||
      preferred_radius < minimum_radius ||
      preferred_radius > maximum_radius ||
      angular_sample_step_deg <= 0.0 ||
      angular_sample_step_deg > 22.5 || radial_sample_step <= 0.0) {
    return candidates;
  }

  const auto append_unique = [](double value, std::vector<double>* values) {
    const auto duplicate = std::find_if(
        values->begin(), values->end(), [value](double existing) {
          return std::abs(existing - value) <= 1.0e-9;
        });
    if (duplicate == values->end()) values->push_back(value);
  };

  std::vector<double> angle_offsets_deg{0.0};
  for (double offset = angular_sample_step_deg;
       offset < 22.5 + 1.0e-9; offset += angular_sample_step_deg) {
    append_unique(-std::min(offset, 22.5), &angle_offsets_deg);
    append_unique(std::min(offset, 22.5), &angle_offsets_deg);
  }
  // Always include both legal sector boundaries. They retain the selected
  // sector id and never cause a search in either neighboring sector.
  append_unique(-22.5, &angle_offsets_deg);
  append_unique(22.5, &angle_offsets_deg);

  std::vector<double> radii{preferred_radius};
  for (double offset = radial_sample_step;
       preferred_radius - offset >= minimum_radius - 1.0e-9 ||
       preferred_radius + offset <= maximum_radius + 1.0e-9;
       offset += radial_sample_step) {
    if (preferred_radius - offset >= minimum_radius - 1.0e-9) {
      append_unique(std::max(minimum_radius, preferred_radius - offset),
                    &radii);
    }
    if (preferred_radius + offset <= maximum_radius + 1.0e-9) {
      append_unique(std::min(maximum_radius, preferred_radius + offset),
                    &radii);
    }
  }
  append_unique(minimum_radius, &radii);
  append_unique(maximum_radius, &radii);

  candidates.reserve(angle_offsets_deg.size() * radii.size());
  const double sector_center =
      entryGateSectorCenterAngleRad(entry_sector_user);
  std::size_t index = 0;
  for (double angle_offset : angle_offsets_deg) {
    for (double radius : radii) {
      const double angle = normalizeAngle(
          sector_center + angle_offset * kPi / 180.0);
      CandidatePoint candidate;
      std::ostringstream id;
      id << "ENTRY_GATE_s" << entry_sector_user << "_c" << index++;
      candidate.id = id.str();
      candidate.sector_id = entry_sector_user - 1;
      candidate.layer_id = 0;
      candidate.x = route.center_x + radius * std::cos(angle);
      candidate.y = route.center_y + radius * std::sin(angle);
      candidate.z = inspection_height;
      candidate.yaw = normalizeAngle(
          std::atan2(route.center_y - candidate.y,
                     route.center_x - candidate.x) -
          route.camera_yaw_offset_rad);
      candidate.require_arrival_yaw = true;
      candidate.face_tower = true;
      // ENTRY_GATE preference is lexicographic: the sector center is used
      // whenever it is safe, and angular relocation is considered only after
      // all candidates at a smaller center offset have been rejected.
      candidate.priority = static_cast<int>(
          std::lround(std::abs(angle_offset) * 1000.0));
      candidates.push_back(candidate);
    }
  }
  return candidates;
}

CandidatePoint buildFixedEntryGate(const RouteConfig& route,
                                   int sector_count,
                                   int entry_sector_user,
                                   double gate_radius,
                                   double gate_height) {
  CandidatePoint candidate;
  if (sector_count != 8 || entry_sector_user < 1 ||
      entry_sector_user > sector_count || !std::isfinite(gate_radius) ||
      !std::isfinite(gate_height)) {
    candidate.target_invalid = true;
    candidate.rejection_reason = "INVALID_FIXED_ENTRY_GATE";
    return candidate;
  }
  const double angle = entryGateSectorCenterAngleRad(entry_sector_user);
  std::ostringstream id;
  id << "ENTRY_GATE_s" << entry_sector_user;
  candidate.id = id.str();
  candidate.sector_id = entry_sector_user - 1;
  candidate.layer_id = 0;
  candidate.x = route.center_x + gate_radius * std::cos(angle);
  candidate.y = route.center_y + gate_radius * std::sin(angle);
  candidate.z = gate_height;
  candidate.yaw = normalizeAngle(
      std::atan2(route.center_y - candidate.y,
                 route.center_x - candidate.x) -
      route.camera_yaw_offset_rad);
  candidate.require_arrival_yaw = true;
  candidate.face_tower = true;
  return candidate;
}

double entryGateSectorCenterAngleRad(int entry_sector_user) {
  if (entry_sector_user < 1 || entry_sector_user > 8) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return normalizeAngle(
      static_cast<double>(entry_sector_user - 1) * kPi / 4.0);
}

bool entryGatePointInSector(const CandidatePoint& candidate,
                            const RouteConfig& route,
                            int entry_sector_user,
                            double tolerance_rad) {
  const double center = entryGateSectorCenterAngleRad(entry_sector_user);
  if (!std::isfinite(center) || !std::isfinite(candidate.x) ||
      !std::isfinite(candidate.y) || !std::isfinite(tolerance_rad) ||
      tolerance_rad < 0.0) {
    return false;
  }
  const double angle = std::atan2(candidate.y - route.center_y,
                                  candidate.x - route.center_x);
  return std::abs(normalizeAngle(angle - center)) <=
         kPi / 8.0 + tolerance_rad;
}

bool evaluateEntryGateCandidate(
    CandidatePoint* candidate, const RouteConfig& route,
    const geometry_msgs::Point& current_position,
    const geometry_msgs::Point& home_position,
    const std::vector<geometry_msgs::Point>& map_points,
    const std::vector<StaticObstacle>& obstacles, bool map_fresh,
    const EntryGateConfig& config, double unknown_ratio,
    double unknown_ratio_limit,
    const CandidatePoint* first_inspection_target) {
  if (candidate == nullptr) return false;
  candidate->accepted = false;
  candidate->target_invalid = false;
  candidate->straight_corridor_blocked = false;
  candidate->rejection_reason.clear();
  candidate->risk_reason.clear();
  candidate->score = -1.0e9;
  candidate->clearance = std::numeric_limits<double>::infinity();
  const auto reject = [candidate](const std::string& reason) {
    candidate->target_invalid = true;
    candidate->rejection_reason = reason;
    return false;
  };
  if (!map_fresh) return reject("MAP_STALE");
  if (!std::isfinite(candidate->x) || !std::isfinite(candidate->y) ||
      !std::isfinite(candidate->z) ||
      candidate->z < config.minimum_height ||
      candidate->z > config.maximum_height ||
      std::abs(candidate->z - config.inspection_height) > 1.0e-6) {
    return reject("OUT_OF_BOUNDS");
  }

  const double radius = distance2d(candidate->x, candidate->y,
                                    route.center_x, route.center_y);
  constexpr double kBoundaryTolerance = 1.0e-6;
  if (radius < config.minimum_radius - kBoundaryTolerance ||
      radius > config.maximum_radius + kBoundaryTolerance) {
    return reject("ENTRY_GATE_RADIUS");
  }
  const double home_distance = distance2d(candidate->x, candidate->y,
                                           home_position.x, home_position.y);
  if (home_distance > config.maximum_horizontal_distance) {
    return reject("TASK_BOUNDARY");
  }
  const double tower_clearance = radius - route.tower_collision_radius;
  candidate->clearance = tower_clearance;
  if (tower_clearance < route.minimum_safety_distance) {
    return reject("TOWER_KEEP_OUT");
  }

  geometry_msgs::Point target;
  target.x = candidate->x;
  target.y = candidate->y;
  target.z = candidate->z;
  for (const auto& obstacle : obstacles) {
    const double clearance = obstacleClearance(target, obstacle, 0.0);
    candidate->clearance = std::min(candidate->clearance, clearance);
    if (clearance < config.minimum_clearance) {
      return reject("KNOWN_OBSTACLE_CLEARANCE");
    }
  }
  const double map_required_clearance = mappedTaskClearance(config);
  for (const auto& map_point : map_points) {
    const double clearance = distance3d(target.x, target.y, target.z,
                                         map_point.x, map_point.y,
                                         map_point.z);
    candidate->clearance = std::min(candidate->clearance, clearance);
    if (clearance < map_required_clearance) {
      return reject("OCCUPANCY_OR_CLEARANCE");
    }
  }
  candidate->unknown_ratio = unknown_ratio;
  if (unknown_ratio > unknown_ratio_limit) {
    return reject("UNKNOWN_REGION");
  }

  // A straight line is only a risk hint here.  The rolling goals below are
  // deliberately handed to EGO one at a time so EGO can bend around a
  // partially observed obstacle instead of the task layer bypassing it.
  static const std::vector<geometry_msgs::Point> no_map_points;
  static const std::vector<StaticObstacle> no_static_obstacles;
  const bool static_corridor_safe = lineCorridorSafe(
      current_position, target, no_map_points, obstacles,
      config.minimum_clearance,
      config.corridor_sample_step);
  const bool map_corridor_safe = lineCorridorSafe(
      current_position, target, map_points, no_static_obstacles,
      map_required_clearance,
      config.corridor_sample_step);
  candidate->straight_corridor_blocked =
      !static_corridor_safe || !map_corridor_safe;
  if (candidate->straight_corridor_blocked) {
    candidate->risk_reason = "STRAIGHT_CORRIDOR_BLOCKED";
  }
  if (!std::isfinite(candidate->clearance)) candidate->clearance = 100.0;
  const double preferred_radius_error =
      std::abs(radius - config.preferred_radius);
  const double sector_center =
      static_cast<double>(candidate->sector_id) * kPi / 4.0;
  const double sector_center_error =
      std::abs(normalizeAngle(
          std::atan2(candidate->y - route.center_y,
                     candidate->x - route.center_x) -
          sector_center));
  const double first_waypoint_distance =
      first_inspection_target == nullptr
          ? 0.0
          : distance2d(candidate->x, candidate->y,
                       first_inspection_target->x,
                       first_inspection_target->y);
  candidate->score =
      config.clearance_weight * candidate->clearance -
      config.distance_weight * distance3d(
          candidate->x, candidate->y, candidate->z, current_position.x,
          current_position.y, current_position.z) -
      config.sector_center_weight * radius * sector_center_error -
      config.preferred_radius_weight * preferred_radius_error -
      config.first_waypoint_weight * first_waypoint_distance -
      (candidate->straight_corridor_blocked
           ? config.blocked_corridor_penalty
           : 0.0);
  candidate->accepted = true;
  return true;
}

int chooseBestEntryGateCandidate(
    const std::vector<CandidatePoint>& candidates) {
  int best = -1;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (!candidates[index].accepted) continue;
    if (best < 0 ||
        candidates[index].priority < candidates[best].priority ||
        (candidates[index].priority == candidates[best].priority &&
         candidates[index].score > candidates[best].score)) {
      best = static_cast<int>(index);
    }
  }
  return best;
}

std::vector<CandidatePoint> buildRollingApproachGoals(
    const geometry_msgs::Point& start, const CandidatePoint& entry_gate,
    double maximum_segment_length) {
  const double distance = distance3d(
      start.x, start.y, start.z, entry_gate.x, entry_gate.y, entry_gate.z);
  const int segments = std::max(
      1, static_cast<int>(std::ceil(
             distance / std::max(0.5, maximum_segment_length))));
  std::vector<CandidatePoint> goals;
  goals.reserve(static_cast<std::size_t>(segments));
  for (int index = 1; index <= segments; ++index) {
    const double ratio = static_cast<double>(index) / segments;
    CandidatePoint goal = entry_gate;
    std::ostringstream id;
    if (index == segments) {
      goal.id = entry_gate.id;
    } else {
      id << "ENTRY_GATE_ROLL_" << index;
      goal.id = id.str();
    }
    goal.sector_id = -1;
    goal.x = start.x + ratio * (entry_gate.x - start.x);
    goal.y = start.y + ratio * (entry_gate.y - start.y);
    goal.z = start.z + ratio * (entry_gate.z - start.z);
    // ENTRY_GATE is transit, not inspection. Its yaw follows horizontal
    // motion; tower-facing yaw begins only after a sector target is issued.
    goal.require_arrival_yaw = false;
    goal.face_tower = false;
    goals.push_back(goal);
  }
  return goals;
}

std::vector<CandidatePoint> buildVerticalClimbGoals(
    const CandidatePoint& staging_point, double entry_height,
    double height_step) {
  if (!std::isfinite(staging_point.x) || !std::isfinite(staging_point.y) ||
      !std::isfinite(staging_point.z) || !std::isfinite(entry_height) ||
      !std::isfinite(height_step) || height_step <= 0.0 ||
      entry_height <= staging_point.z) {
    return {};
  }
  const int segments = std::max(
      1, static_cast<int>(std::ceil(
             (entry_height - staging_point.z) / height_step)));
  std::vector<CandidatePoint> goals;
  goals.reserve(static_cast<std::size_t>(segments));
  for (int index = 1; index <= segments; ++index) {
    CandidatePoint goal = staging_point;
    goal.id = index == segments
                  ? "SAFE_ALTITUDE_HOLD"
                  : "STAGING_CLIMB_" + std::to_string(index);
    goal.z = staging_point.z +
             (entry_height - staging_point.z) *
                 static_cast<double>(index) / segments;
    // A fixed-XY vertical climb has no meaningful horizontal bearing. Small
    // EGO XY corrections can legitimately rotate the velocity-facing yaw, so
    // yaw must not gate safe-altitude arrival. ENTRY_GATE yaw is evaluated
    // and locked only after the map dwell that follows this target.
    goal.require_arrival_yaw = false;
    goals.push_back(goal);
  }
  return goals;
}

std::vector<CandidatePoint> buildVerticalGoalsAtHeights(
    const CandidatePoint& reference,
    const std::vector<double>& target_heights,
    const std::string& id_prefix) {
  if (!std::isfinite(reference.x) || !std::isfinite(reference.y) ||
      !std::isfinite(reference.z) || target_heights.empty()) {
    return {};
  }
  std::vector<CandidatePoint> goals;
  goals.reserve(target_heights.size());
  double previous_height = reference.z;
  for (std::size_t index = 0; index < target_heights.size(); ++index) {
    const double height = target_heights[index];
    if (!std::isfinite(height) ||
        std::abs(height - previous_height) < 1.0e-6) {
      return {};
    }
    CandidatePoint goal = reference;
    std::ostringstream id;
    id << id_prefix << '_' << index << "_Z" << height;
    goal.id = id.str();
    goal.z = height;
    goals.push_back(goal);
    previous_height = height;
  }
  return goals;
}

CandidatePoint buildHomeOverheadGoal(const geometry_msgs::Point& home,
                                     double inspection_height,
                                     const std::string& id) {
  CandidatePoint overhead;
  if (!std::isfinite(home.x) || !std::isfinite(home.y) ||
      !std::isfinite(home.z) || !std::isfinite(inspection_height) ||
      id.empty()) {
    return overhead;
  }
  overhead.id = id;
  overhead.sector_id = -1;
  overhead.x = home.x;
  overhead.y = home.y;
  overhead.z = inspection_height;
  overhead.require_arrival_yaw = false;
  overhead.face_tower = false;
  return overhead;
}

std::vector<double> deriveDescendingIntermediateHeights(
    double current_height, double final_height,
    const std::vector<double>& configured_heights) {
  if (!std::isfinite(current_height) || !std::isfinite(final_height) ||
      current_height <= final_height) {
    return {};
  }
  std::vector<double> heights;
  heights.reserve(configured_heights.size());
  for (double height : configured_heights) {
    if (std::isfinite(height) && height < current_height - 1.0e-6 &&
        height > final_height + 1.0e-6) {
      heights.push_back(height);
    }
  }
  std::sort(heights.begin(), heights.end(), std::greater<double>());
  heights.erase(std::unique(
                    heights.begin(), heights.end(),
                    [](double first, double second) {
                      return std::abs(first - second) < 1.0e-6;
                    }),
                heights.end());
  return heights;
}

std::vector<double> deriveInspectionHeights(
    double inspection_top_height,
    const std::vector<double>& layer_offsets) {
  if (!std::isfinite(inspection_top_height) || layer_offsets.empty()) {
    return {};
  }
  if (!std::isfinite(layer_offsets.front()) ||
      std::abs(layer_offsets.front()) > 1.0e-9) {
    return {};
  }
  std::vector<double> heights;
  heights.reserve(layer_offsets.size());
  for (double offset : layer_offsets) {
    const double height = inspection_top_height + offset;
    if (!std::isfinite(offset) || !std::isfinite(height)) {
      return {};
    }
    heights.push_back(height);
  }
  return heights;
}

std::vector<CandidatePoint> buildLayerTransitionGoals(
    const CandidatePoint& from,
    const CandidatePoint& to,
    double maximum_vertical_step,
    double same_xy_tolerance,
    const std::string& id_prefix) {
  if (!std::isfinite(from.x) || !std::isfinite(from.y) ||
      !std::isfinite(from.z) || !std::isfinite(to.x) ||
      !std::isfinite(to.y) || !std::isfinite(to.z) ||
      !std::isfinite(maximum_vertical_step) ||
      !std::isfinite(same_xy_tolerance) ||
      maximum_vertical_step <= 0.0 || same_xy_tolerance < 0.0 ||
      std::abs(to.z - from.z) < 1.0e-6) {
    return {};
  }
  const bool same_xy =
      std::hypot(to.x - from.x, to.y - from.y) <= same_xy_tolerance;
  const std::size_t segment_count = static_cast<std::size_t>(
      std::max(1.0, std::ceil(std::abs(to.z - from.z) /
                              maximum_vertical_step)));
  std::vector<CandidatePoint> goals;
  goals.reserve(segment_count);
  for (std::size_t index = 1U; index <= segment_count; ++index) {
    const double ratio =
        static_cast<double>(index) / static_cast<double>(segment_count);
    CandidatePoint goal = to;
    std::ostringstream id;
    id << id_prefix << '_' << (index - 1U) << "_L" << to.layer_id
       << "_Z" << (from.z + (to.z - from.z) * ratio);
    goal.id = id.str();
    goal.x = same_xy ? from.x : from.x + (to.x - from.x) * ratio;
    goal.y = same_xy ? from.y : from.y + (to.y - from.y) * ratio;
    goal.z = from.z + (to.z - from.z) * ratio;
    goals.push_back(goal);
  }
  return goals;
}

std::vector<int> buildLayerVisitSequence(std::size_t layer_count,
                                         int planned_cycles) {
  if (layer_count == 0U || planned_cycles < 1) return {};
  std::vector<int> sequence;
  sequence.reserve(layer_count * static_cast<std::size_t>(planned_cycles));
  for (int cycle = 0; cycle < planned_cycles; ++cycle) {
    for (std::size_t layer = 0U; layer < layer_count; ++layer) {
      sequence.push_back(static_cast<int>(layer));
    }
  }
  return sequence;
}

std::vector<std::size_t> buildClosedLapVisitSequence(
    std::size_t waypoint_count, int inspection_laps) {
  if (waypoint_count == 0U || inspection_laps < 1) return {};
  std::vector<std::size_t> visits;
  visits.reserve(1U +
                 static_cast<std::size_t>(inspection_laps) * waypoint_count);
  visits.push_back(0U);
  for (int lap = 0; lap < inspection_laps; ++lap) {
    for (std::size_t waypoint = 1U; waypoint < waypoint_count; ++waypoint) {
      visits.push_back(waypoint);
    }
    visits.push_back(0U);
  }
  return visits;
}

double directedAngularDifference(double entry_angle_rad,
                                 double waypoint_angle_rad,
                                 OrbitDirection direction) {
  const double signed_difference =
      direction == OrbitDirection::kCounterClockwise
          ? waypoint_angle_rad - entry_angle_rad
          : entry_angle_rad - waypoint_angle_rad;
  double wrapped = std::fmod(signed_difference, 2.0 * kPi);
  if (wrapped < 0.0) wrapped += 2.0 * kPi;
  if (wrapped >= 2.0 * kPi - 1.0e-12) wrapped = 0.0;
  return wrapped;
}

std::vector<std::size_t> directionalSectorOrder(
    double entry_angle_rad, const std::vector<Sector>& sectors,
    OrbitDirection direction) {
  std::vector<std::size_t> order(sectors.size());
  for (std::size_t index = 0; index < sectors.size(); ++index) {
    order[index] = index;
  }
  std::stable_sort(
      order.begin(), order.end(),
      [&sectors, entry_angle_rad, direction](std::size_t lhs,
                                             std::size_t rhs) {
        const double lhs_difference = directedAngularDifference(
            entry_angle_rad, sectors[lhs].nominal_angle_rad, direction);
        const double rhs_difference = directedAngularDifference(
            entry_angle_rad, sectors[rhs].nominal_angle_rad, direction);
        if (std::abs(lhs_difference - rhs_difference) > 1.0e-12) {
          return lhs_difference < rhs_difference;
        }
        return sectors[lhs].sector_id < sectors[rhs].sector_id;
      });
  return order;
}

std::pair<int, int> findAcceptedCandidateById(
    const std::vector<Sector>& sectors,
    const std::string& candidate_id) {
  if (candidate_id.empty()) return {-1, -1};
  for (std::size_t sector_index = 0; sector_index < sectors.size();
       ++sector_index) {
    const auto& candidates = sectors[sector_index].candidates;
    for (std::size_t candidate_index = 0;
         candidate_index < candidates.size(); ++candidate_index) {
      if (candidates[candidate_index].id == candidate_id &&
          candidates[candidate_index].accepted) {
        return {static_cast<int>(sector_index),
                static_cast<int>(candidate_index)};
      }
    }
  }
  return {-1, -1};
}

bool plannerMapContainsOrbitEnvelope(
    double tower_center_x,
    double tower_center_y,
    double maximum_orbit_radius,
    double planning_horizon,
    double map_size_x,
    double map_size_y,
    std::string* reason) {
  const bool finite = std::isfinite(tower_center_x) &&
                      std::isfinite(tower_center_y) &&
                      std::isfinite(maximum_orbit_radius) &&
                      std::isfinite(planning_horizon) &&
                      std::isfinite(map_size_x) && std::isfinite(map_size_y);
  if (!finite || maximum_orbit_radius <= 0.0 || planning_horizon < 0.0 ||
      map_size_x <= 0.0 || map_size_y <= 0.0) {
    if (reason != nullptr) *reason = "non-finite or non-positive map contract";
    return false;
  }

  const double required_radius = maximum_orbit_radius + planning_horizon;
  const double half_x = 0.5 * map_size_x;
  const double half_y = 0.5 * map_size_y;
  const double min_x = tower_center_x - required_radius;
  const double max_x = tower_center_x + required_radius;
  const double min_y = tower_center_y - required_radius;
  const double max_y = tower_center_y + required_radius;
  const bool contained = min_x > -half_x + 1.0e-4 &&
                         max_x < half_x - 1.0e-4 &&
                         min_y > -half_y + 1.0e-4 &&
                         max_y < half_y - 1.0e-4;
  if (reason != nullptr) {
    std::ostringstream stream;
    stream << "required_xy=[" << min_x << ',' << max_x << "]x["
           << min_y << ',' << max_y << "] map_xy=[" << -half_x << ','
           << half_x << "]x[" << -half_y << ',' << half_y << ']';
    *reason = stream.str();
  }
  return contained;
}

void rotateSectorsToNearest(const geometry_msgs::Point& current,
                            std::vector<Sector>* sectors) {
  if (sectors == nullptr || sectors->empty()) return;
  auto nearest = sectors->begin();
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (auto iterator = sectors->begin(); iterator != sectors->end(); ++iterator) {
    const double x = iterator->center_x + iterator->nominal_radius *
                                            std::cos(iterator->nominal_angle_rad);
    const double y = iterator->center_y + iterator->nominal_radius *
                                            std::sin(iterator->nominal_angle_rad);
    const double distance = distance3d(current.x, current.y, current.z,
                                       x, y, iterator->nominal_height);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = iterator;
    }
  }
  std::rotate(sectors->begin(), nearest, sectors->end());
}

int nearestSectorIndexWithAcceptedCandidate(
    const geometry_msgs::Point& current,
    const std::vector<Sector>& sectors) {
  int selected = -1;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < sectors.size(); ++index) {
    const Sector& sector = sectors[index];
    const bool has_accepted =
        std::any_of(sector.candidates.begin(), sector.candidates.end(),
                    [](const CandidatePoint& candidate) {
                      return candidate.accepted;
                    });
    if (!has_accepted) continue;
    const double x = sector.center_x +
                     sector.nominal_radius * std::cos(sector.nominal_angle_rad);
    const double y = sector.center_y +
                     sector.nominal_radius * std::sin(sector.nominal_angle_rad);
    const double distance = distance3d(current.x, current.y, current.z, x, y,
                                       sector.nominal_height);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      selected = static_cast<int>(index);
    }
  }
  return selected;
}

RecoveryTargets makeRecoveryTargets(const RouteConfig& route,
                                    const geometry_msgs::Point& current,
                                    const Sector& sector,
                                    const RecoveryConfig& config,
                                    const CandidatePoint* locked_target) {
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
  if (locked_target != nullptr) {
    targets.reentry = *locked_target;
    targets.reentry.id = "recovery_reentry_" + locked_target->id;
  } else {
    const double target_radius = std::max(route.radius, sector.nominal_radius);
    const double entry_angle = sector.nominal_angle_rad;
    targets.reentry = make(
        "recovery_reentry", route.center_x + target_radius * std::cos(entry_angle),
        route.center_y + target_radius * std::sin(entry_angle));
    targets.reentry.z = sector.nominal_height;
  }
  return targets;
}

RecoveryAssessment assessRecoveryTargets(
    const geometry_msgs::Point& current,
    const RecoveryTargets& targets,
    const std::vector<geometry_msgs::Point>& cloud_points,
    const std::vector<StaticObstacle>& obstacles,
    double map_clearance,
    double static_clearance,
    double sample_step) {
  RecoveryAssessment result;
  const CandidatePoint candidates[] = {targets.r1, targets.r2, targets.reentry};
  for (const auto& candidate : candidates) {
    geometry_msgs::Point point;
    point.x = candidate.x;
    point.y = candidate.y;
    point.z = candidate.z;
    for (const auto& obstacle : obstacles) {
      if (pointInObstacle(point, obstacle, static_clearance)) return result;
    }
    for (const auto& cloud : cloud_points) {
      if (distance3d(point.x, point.y, point.z,
                     cloud.x, cloud.y, cloud.z) < map_clearance) {
        return result;
      }
    }
  }
  result.endpoints_safe = true;
  geometry_msgs::Point points[4];
  points[0] = current;
  points[1].x = targets.r1.x; points[1].y = targets.r1.y; points[1].z = targets.r1.z;
  points[2].x = targets.r2.x; points[2].y = targets.r2.y; points[2].z = targets.r2.z;
  points[3].x = targets.reentry.x; points[3].y = targets.reentry.y;
  points[3].z = targets.reentry.z;
  double length = 0.0;
  for (int index = 0; index < 3; ++index) {
    static const std::vector<geometry_msgs::Point> no_map_points;
    static const std::vector<StaticObstacle> no_static_obstacles;
    const bool map_safe = lineCorridorSafe(
        points[index], points[index + 1], cloud_points,
        no_static_obstacles, map_clearance, sample_step);
    const bool static_safe = lineCorridorSafe(
        points[index], points[index + 1], no_map_points,
        obstacles, static_clearance, sample_step);
    if (!map_safe || !static_safe) {
      ++result.blocked_corridors;
    }
    length += distance3d(points[index].x, points[index].y, points[index].z,
                         points[index + 1].x, points[index + 1].y,
                         points[index + 1].z);
  }
  // A blocked straight segment is a risk hint, never an unreachable verdict.
  result.score = -100.0 * result.blocked_corridors - length;
  return result;
}

bool recoveryTargetsStayInSector(const RecoveryTargets& targets,
                                 const Sector& sector,
                                 double maximum_descent) {
  if (!std::isfinite(maximum_descent) || maximum_descent < 0.0) {
    return false;
  }
  const CandidatePoint points[] = {
      targets.r1, targets.r2, targets.reentry};
  const double angle_half_width = std::max(
      std::abs(normalizeAngle(sector.max_angle_rad -
                              sector.nominal_angle_rad)),
      std::abs(normalizeAngle(sector.min_angle_rad -
                              sector.nominal_angle_rad)));
  for (const auto& point : points) {
    const double radius =
        distance2d(point.x, point.y, sector.center_x, sector.center_y);
    const double angle = std::atan2(point.y - sector.center_y,
                                    point.x - sector.center_x);
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || radius < sector.min_radius ||
        radius > sector.max_radius ||
        std::abs(normalizeAngle(angle - sector.nominal_angle_rad)) >
            angle_half_width + 1.0e-9 ||
        point.z > sector.nominal_height + 1.0e-9 ||
        point.z < sector.nominal_height - maximum_descent - 1.0e-9) {
      return false;
    }
  }
  return true;
}

bool returnOrLandingTimedOut(bool landing_active,
                             double return_elapsed,
                             double landing_elapsed,
                             double return_timeout,
                             double landing_timeout) {
  return landing_active ? landing_elapsed >= landing_timeout
                        : return_elapsed >= return_timeout;
}

std::vector<CandidatePoint> buildSafeReturnEgressGoals(
    const RouteConfig& route, const geometry_msgs::Point& current,
    const geometry_msgs::Point& home,
    const std::vector<StaticObstacle>& obstacles,
    const ReturnEgressConfig& config) {
  if (!std::isfinite(config.orbit_radius) ||
      !std::isfinite(config.transit_height) ||
      !std::isfinite(config.maximum_angle_step_rad) ||
      config.orbit_radius <= route.tower_collision_radius ||
      config.transit_height < route.minimum_height ||
      config.transit_height > route.maximum_height ||
      config.maximum_angle_step_rad <= 0.0 ||
      config.maximum_angle_step_rad > kPi ||
      config.obstacle_inflation < 0.0 ||
      config.corridor_sample_step <= 0.0 ||
      config.minimum_goal_separation <= 0.0) {
    return {};
  }

  const double current_angle =
      std::atan2(current.y - route.center_y, current.x - route.center_x);
  const double home_angle =
      std::atan2(home.y - route.center_y,
                 home.x - route.center_x);
  const double ccw_delta = std::fmod(
      home_angle - current_angle + 2.0 * kPi, 2.0 * kPi);
  const double cw_delta = ccw_delta > 0.0 ? ccw_delta - 2.0 * kPi : 0.0;

  const auto make_route = [&](double angular_delta,
                              const char* direction) {
    std::vector<CandidatePoint> goals;
    CandidatePoint radial;
    radial.id = std::string("RETURN_EGRESS_") + direction + "_RADIAL";
    radial.sector_id = -1;
    radial.x = route.center_x +
               config.orbit_radius * std::cos(current_angle);
    radial.y = route.center_y +
               config.orbit_radius * std::sin(current_angle);
    radial.z = config.transit_height;
    radial.yaw = current_angle;
    radial.require_arrival_yaw = false;
    radial.face_tower = false;
    if (distance3d(current.x, current.y, current.z,
                   radial.x, radial.y, radial.z) >
        config.minimum_goal_separation) {
      goals.push_back(radial);
    }

    const int arc_segments = std::max(
        1, static_cast<int>(std::ceil(
               std::abs(angular_delta) / config.maximum_angle_step_rad)));
    for (int index = 1; index <= arc_segments; ++index) {
      const double ratio = static_cast<double>(index) / arc_segments;
      const double angle = current_angle + ratio * angular_delta;
      CandidatePoint goal;
      std::ostringstream id;
      if (index == arc_segments) {
        id << "RETURN_GATE_" << direction;
      } else {
        id << "RETURN_EGRESS_" << direction << "_ARC_" << index;
      }
      goal.id = id.str();
      goal.sector_id = -1;
      goal.x = route.center_x + config.orbit_radius * std::cos(angle);
      goal.y = route.center_y + config.orbit_radius * std::sin(angle);
      goal.z = config.transit_height;
      goal.yaw = angle;
      goal.require_arrival_yaw = false;
      goal.face_tower = false;
      goals.push_back(goal);
    }
    // Keep the final horizontal leg above the known crane, then let the
    // bridge publish only a vertical home goal. This avoids coupling a long
    // horizontal crossing with the descent from transit height.
    CandidatePoint overhead;
    overhead.id = "RETURN_HOME_OVERHEAD";
    overhead.sector_id = -1;
    overhead.x = home.x;
    overhead.y = home.y;
    overhead.z = config.transit_height;
    overhead.yaw = home_angle;
    overhead.require_arrival_yaw = false;
    overhead.face_tower = false;
    goals.push_back(overhead);
    return goals;
  };

  const auto route_is_safe =
      [&](const std::vector<CandidatePoint>& goals) {
        geometry_msgs::Point from = current;
        for (const CandidatePoint& goal : goals) {
          geometry_msgs::Point to;
          to.x = goal.x;
          to.y = goal.y;
          to.z = goal.z;
          if (!lineCorridorSafe(from, to, {}, obstacles,
                                config.obstacle_inflation,
                                config.corridor_sample_step)) {
            return false;
          }
          from = to;
        }
        // The conservative crane OBB intentionally overlaps the known-safe
        // launch pad at low altitude. The last task goal is directly above
        // home; the bridge/EGO vertical descent and live occupancy checks own
        // the final segment, as they do during the validated normal return.
        return true;
      };

  const auto route_length =
      [&](const std::vector<CandidatePoint>& goals) {
        geometry_msgs::Point from = current;
        double length = 0.0;
        for (const CandidatePoint& goal : goals) {
          length += distance3d(from.x, from.y, from.z,
                               goal.x, goal.y, goal.z);
          from.x = goal.x;
          from.y = goal.y;
          from.z = goal.z;
        }
        length += distance3d(from.x, from.y, from.z,
                             home.x, home.y, home.z);
        return length;
      };

  const std::vector<CandidatePoint> ccw = make_route(ccw_delta, "CCW");
  const std::vector<CandidatePoint> cw = make_route(cw_delta, "CW");
  const bool ccw_safe = route_is_safe(ccw);
  const bool cw_safe = route_is_safe(cw);
  if (!ccw_safe && !cw_safe) return {};
  if (ccw_safe && (!cw_safe || route_length(ccw) <= route_length(cw))) {
    return ccw;
  }
  return cw;
}

bool returnLandingNearHome(const geometry_msgs::Point& landed_position,
                           const geometry_msgs::Point& home_position,
                           double horizontal_tolerance) {
  if (!std::isfinite(landed_position.x) ||
      !std::isfinite(landed_position.y) ||
      !std::isfinite(home_position.x) ||
      !std::isfinite(home_position.y) ||
      !std::isfinite(horizontal_tolerance) ||
      horizontal_tolerance <= 0.0) {
    return false;
  }
  return std::hypot(landed_position.x - home_position.x,
                    landed_position.y - home_position.y) <=
         horizontal_tolerance;
}

}  // namespace astra_tower_mission
