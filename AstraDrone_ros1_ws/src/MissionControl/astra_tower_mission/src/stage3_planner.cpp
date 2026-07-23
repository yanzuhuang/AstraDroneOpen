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
  candidate->straight_corridor_blocked = false;
  candidate->rejection_reason.clear();
  candidate->risk_reason.clear();
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
  candidate->straight_corridor_blocked = !lineCorridorSafe(
      current_position, target, cloud_points, obstacles,
      config.minimum_clearance + config.cloud_inflation,
      config.corridor_sample_step);
  if (candidate->straight_corridor_blocked) {
    candidate->risk_reason = "STRAIGHT_CORRIDOR_BLOCKED";
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
                     (candidate->straight_corridor_blocked
                          ? config.blocked_corridor_penalty
                          : 0.0) -
                     0.001 * nominal_distance;
  candidate->accepted = true;
  return true;
}

int chooseBestCandidate(const Sector& sector, const CandidatePoint* locked,
                        double replacement_margin) {
  // Candidate generation places the exact nominal point first, but identify
  // it geometrically so configuration ordering cannot silently change the
  // safety policy. A usable nominal target P always wins; relocation is only
  // allowed after P is rejected by a hard endpoint check.
  for (std::size_t index = 0; index < sector.candidates.size(); ++index) {
    const auto& candidate = sector.candidates[index];
    const double radius = distance2d(candidate.x, candidate.y,
                                     sector.center_x, sector.center_y);
    const double angle = std::atan2(candidate.y - sector.center_y,
                                    candidate.x - sector.center_x);
    if (candidate.accepted &&
        std::abs(radius - sector.nominal_radius) < 1.0e-6 &&
        std::abs(normalizeAngle(angle - sector.nominal_angle_rad)) < 1.0e-6 &&
        std::abs(candidate.z - sector.nominal_height) < 1.0e-6) {
      return static_cast<int>(index);
    }
  }
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

std::vector<CandidatePoint> buildEntryGateCandidates(
    const RouteConfig& route, const std::vector<double>& angle_offsets_deg,
    const std::vector<double>& radius_offsets_m, double inspection_height) {
  std::vector<CandidatePoint> candidates;
  candidates.reserve(angle_offsets_deg.size() * radius_offsets_m.size());
  std::size_t index = 0;
  for (double angle_offset : angle_offsets_deg) {
    for (double radius_offset : radius_offsets_m) {
      const double angle = normalizeAngle(
          route.start_angle_rad + angle_offset * kPi / 180.0);
      const double radius = route.radius + radius_offset;
      CandidatePoint candidate;
      std::ostringstream id;
      id << "ENTRY_GATE_a" << index++;
      candidate.id = id.str();
      candidate.sector_id = -1;
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
      candidates.push_back(candidate);
    }
  }
  return candidates;
}

bool evaluateEntryGateCandidate(
    CandidatePoint* candidate, const RouteConfig& route,
    const geometry_msgs::Point& current_position,
    const geometry_msgs::Point& home_position,
    const std::vector<geometry_msgs::Point>& map_points,
    const std::vector<StaticObstacle>& obstacles, bool map_fresh,
    const EntryGateConfig& config) {
  if (candidate == nullptr) return false;
  candidate->accepted = false;
  candidate->straight_corridor_blocked = false;
  candidate->rejection_reason.clear();
  candidate->risk_reason.clear();
  candidate->score = -1.0e9;
  candidate->clearance = std::numeric_limits<double>::infinity();
  const auto reject = [candidate](const std::string& reason) {
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
  if (radius < config.minimum_radius || radius > config.maximum_radius) {
    return reject("ENTRY_GATE_RADIUS");
  }
  const double home_distance = distance2d(candidate->x, candidate->y,
                                           home_position.x, home_position.y);
  if (home_distance > config.maximum_horizontal_distance) {
    return reject("TASK_BOUNDARY");
  }
  const double tower_clearance =
      radius - route.tower_collision_radius - config.cloud_inflation;
  candidate->clearance = tower_clearance;
  if (tower_clearance < config.minimum_clearance) {
    return reject("TOWER_KEEP_OUT");
  }

  geometry_msgs::Point target;
  target.x = candidate->x;
  target.y = candidate->y;
  target.z = candidate->z;
  for (const auto& obstacle : obstacles) {
    const double clearance = obstacleClearance(
        target, obstacle, config.cloud_inflation);
    candidate->clearance = std::min(candidate->clearance, clearance);
    if (clearance < config.minimum_clearance) {
      return reject("KNOWN_OBSTACLE_CLEARANCE");
    }
  }
  for (const auto& map_point : map_points) {
    const double clearance = distance3d(target.x, target.y, target.z,
                                         map_point.x, map_point.y,
                                         map_point.z) -
                             config.cloud_inflation;
    candidate->clearance = std::min(candidate->clearance, clearance);
    if (clearance < config.minimum_clearance) {
      return reject("OCCUPANCY_OR_CLEARANCE");
    }
  }

  // A straight line is only a risk hint here.  The rolling goals below are
  // deliberately handed to EGO one at a time so EGO can bend around a
  // partially observed obstacle instead of the task layer bypassing it.
  candidate->straight_corridor_blocked = !lineCorridorSafe(
      current_position, target, map_points, obstacles,
      config.minimum_clearance + config.cloud_inflation,
      config.corridor_sample_step);
  if (candidate->straight_corridor_blocked) {
    candidate->risk_reason = "STRAIGHT_CORRIDOR_BLOCKED";
  }
  if (!std::isfinite(candidate->clearance)) candidate->clearance = 100.0;
  candidate->score =
      config.clearance_weight * candidate->clearance -
      config.distance_weight * distance3d(
          candidate->x, candidate->y, candidate->z, current_position.x,
          current_position.y, current_position.z) -
      (candidate->straight_corridor_blocked
           ? config.blocked_corridor_penalty
           : 0.0);
  candidate->accepted = true;
  return true;
}

int chooseBestEntryGateCandidate(
    const std::vector<CandidatePoint>& candidates) {
  int best = -1;
  double best_score = -1.0e9;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (candidates[index].accepted && candidates[index].score > best_score) {
      best = static_cast<int>(index);
      best_score = candidates[index].score;
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
    goal.require_arrival_yaw = index == segments;
    goal.face_tower = true;
    goals.push_back(goal);
  }
  return goals;
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
    double inflation,
    double sample_step) {
  RecoveryAssessment result;
  const CandidatePoint candidates[] = {targets.r1, targets.r2, targets.reentry};
  for (const auto& candidate : candidates) {
    geometry_msgs::Point point;
    point.x = candidate.x;
    point.y = candidate.y;
    point.z = candidate.z;
    for (const auto& obstacle : obstacles) {
      if (pointInObstacle(point, obstacle, inflation)) return result;
    }
    for (const auto& cloud : cloud_points) {
      if (distance3d(point.x, point.y, point.z,
                     cloud.x, cloud.y, cloud.z) < inflation) {
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
    if (!lineCorridorSafe(points[index], points[index + 1], cloud_points,
                          obstacles, inflation, sample_step)) {
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

}  // namespace astra_tower_mission
