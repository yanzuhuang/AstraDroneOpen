#ifndef ASTRA_TOWER_MISSION_STAGE3_PLANNER_H_
#define ASTRA_TOWER_MISSION_STAGE3_PLANNER_H_

#include "astra_tower_mission/tower_route.h"

#include <geometry_msgs/Point.h>

#include <string>
#include <vector>

namespace astra_tower_mission {

enum class SectorState {
  kPending,
  kEvaluating,
  kTargetLocked,
  kNavigating,
  kHolding,
  kCovered,
  kRelocating,
  kRecovering,
  kFailed,
};

const char* sectorStateName(SectorState state);

struct StaticObstacle {
  std::string id;
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double radius{0.0};
  double z_min{-1.0e9};
  double z_max{1.0e9};
};

struct CandidateOffset {
  double angle_deg{0.0};
  double radius_m{0.0};
  double height_m{0.0};
};

struct CandidatePoint {
  std::string id;
  int sector_id{0};
  int layer_id{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  bool accepted{false};
  std::string rejection_reason;
  double clearance{0.0};
  double score{-1.0e9};
  double unknown_ratio{0.0};
};

struct Sector {
  int sector_id{0};
  int layer_id{0};
  double nominal_angle_rad{0.0};
  double nominal_radius{0.0};
  double nominal_height{0.0};
  double center_x{0.0};
  double center_y{0.0};
  double tower_collision_radius{0.0};
  double min_angle_rad{0.0};
  double max_angle_rad{0.0};
  double min_radius{0.0};
  double max_radius{0.0};
  double min_height{0.0};
  double max_height{0.0};
  std::vector<CandidatePoint> candidates;
  int locked_index{-1};
  SectorState state{SectorState::kPending};
  int recovery_count{0};
  std::string failure_reason;
};

struct CandidateFilterConfig {
  double minimum_clearance{2.0};
  double map_timeout{0.5};
  double cloud_inflation{0.4};
  double unknown_ratio_limit{0.25};
  double corridor_sample_step{0.5};
  double tower_extra_clearance{0.0};
  double score_clearance_weight{1.0};
  double score_nominal_weight{1.0};
  double score_distance_weight{0.25};
  double score_continuity_weight{0.5};
  double score_unknown_weight{1.0};
};

struct RecoveryConfig {
  double radial_step{3.0};
  double tangent_step{4.0};
  double maximum_radius{24.0};
  double recovery_height{30.0};
  OrbitDirection direction{OrbitDirection::kCounterClockwise};
};

struct RecoveryTargets {
  CandidatePoint r1;
  CandidatePoint r2;
  CandidatePoint reentry;
};

std::vector<Sector> buildInspectionSectors(const RouteConfig& route,
                                            int sector_count,
                                            int layer_count,
                                            double angle_half_width_deg,
                                            double radius_half_width,
                                            double height_half_width,
                                            const std::vector<CandidateOffset>& offsets);

bool evaluateCandidate(CandidatePoint* candidate,
                       const Sector& sector,
                       const geometry_msgs::Point& current_position,
                       const std::vector<geometry_msgs::Point>& cloud_points,
                       const std::vector<StaticObstacle>& obstacles,
                       bool map_fresh,
                       const CandidateFilterConfig& config,
                        const CandidatePoint* previous_target = nullptr);

int chooseBestCandidate(const Sector& sector,
                        const CandidatePoint* locked_target,
                        double replacement_margin);

RecoveryTargets makeRecoveryTargets(const RouteConfig& route,
                                    const geometry_msgs::Point& current,
                                    const Sector& sector,
                                    const RecoveryConfig& config);

bool pointInObstacle(const geometry_msgs::Point& point,
                     const StaticObstacle& obstacle,
                     double inflation);
bool lineCorridorSafe(const geometry_msgs::Point& from,
                      const geometry_msgs::Point& to,
                      const std::vector<geometry_msgs::Point>& cloud_points,
                      const std::vector<StaticObstacle>& obstacles,
                      double inflation,
                      double sample_step);

}  // namespace astra_tower_mission

#endif  // ASTRA_TOWER_MISSION_STAGE3_PLANNER_H_
