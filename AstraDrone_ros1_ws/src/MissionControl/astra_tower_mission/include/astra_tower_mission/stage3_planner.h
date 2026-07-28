#ifndef ASTRA_TOWER_MISSION_STAGE3_PLANNER_H_
#define ASTRA_TOWER_MISSION_STAGE3_PLANNER_H_

#include "astra_tower_mission/low_altitude_path_planner.h"
#include "astra_tower_mission/tower_route.h"

#include <geometry_msgs/Point.h>

#include <string>
#include <tuple>
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
  // Positive half extents select a yaw-oriented 3-D box.  Otherwise the
  // legacy vertical cylinder defined by radius/z_min/z_max is used.
  double half_extent_x{0.0};
  double half_extent_y{0.0};
  double yaw{0.0};
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
  bool require_arrival_yaw{true};
  bool face_tower{true};
  bool accepted{false};
  bool target_invalid{false};
  bool straight_corridor_blocked{false};
  bool planner_unreachable{false};
  std::string rejection_reason;
  std::string risk_reason;
  double clearance{0.0};
  double score{-1.0e9};
  double unknown_ratio{0.0};
  int priority{99};
  double nominal_deviation{1.0e9};
  double height_deviation{1.0e9};
  double observation_deviation{1.0e9};
  double continuity_error{1.0e9};
  double route_distance{1.0e9};
};

// ENTRY_GATE is a first-class task waypoint at inspection height.  It is
// deliberately kept outside the sector vector so sector ids and coverage
// accounting remain the original eight-sector contract.
struct EntryGateConfig {
  double inspection_height{30.0};
  double minimum_height{2.0};
  double maximum_height{45.0};
  double preferred_radius{16.0};
  double minimum_clearance{2.0};
  double cloud_inflation{0.4};
  bool map_points_are_inflated{false};
  double map_additional_clearance{0.0};
  double corridor_sample_step{0.5};
  double minimum_radius{16.0};
  double maximum_radius{24.0};
  double maximum_horizontal_distance{60.0};
  double angular_sample_step_deg{7.5};
  double radial_sample_step{1.0};
  double clearance_weight{0.2};
  double distance_weight{0.1};
  double sector_center_weight{4.0};
  double preferred_radius_weight{4.0};
  double first_waypoint_weight{0.8};
  double blocked_corridor_penalty{8.0};
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
  double minimum_tower_clearance{2.0};
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
  bool map_points_are_inflated{false};
  double map_additional_clearance{0.0};
  double unknown_ratio_limit{0.25};
  bool unknown_is_hard_constraint{true};
  bool known_obstacle_is_hard_constraint{true};
  double corridor_sample_step{0.5};
  double tower_extra_clearance{0.0};
  double score_clearance_weight{0.2};
  double score_radius_weight{6.0};
  double score_sector_weight{4.0};
  double score_height_weight{14.0};
  double score_distance_weight{0.1};
  double score_continuity_weight{0.2};
  double score_unknown_weight{1.0};
  double blocked_corridor_penalty{6.0};
  double small_angle_offset_deg{5.0};
  double small_radius_offset_m{2.0};
  double small_height_offset_m{1.0};
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

struct RecoveryAssessment {
  bool endpoints_safe{false};
  int blocked_corridors{0};
  double score{-1.0e9};
};

struct ReturnEgressConfig {
  double orbit_radius{24.0};
  double transit_height{38.0};
  double maximum_angle_step_rad{kPi / 6.0};
  double obstacle_inflation{2.0};
  double corridor_sample_step{0.5};
  double minimum_goal_separation{0.5};
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
                       const CandidatePoint* previous_target = nullptr,
                       double unknown_ratio = 0.0);

int chooseBestCandidate(const Sector& sector,
                        const CandidatePoint* locked_target,
                        double replacement_margin);

std::vector<CandidatePoint> buildEntryGateCandidates(
    const RouteConfig& route,
    int entry_sector_user,
    double minimum_radius,
    double maximum_radius,
    double preferred_radius,
    double angular_sample_step_deg,
    double radial_sample_step,
    double inspection_height);

CandidatePoint buildFixedEntryGate(const RouteConfig& route,
                                   int sector_count,
                                   int entry_sector_user,
                                   double gate_radius,
                                   double gate_height);

double entryGateSectorCenterAngleRad(int entry_sector_user);

bool entryGatePointInSector(const CandidatePoint& candidate,
                            const RouteConfig& route,
                            int entry_sector_user,
                            double tolerance_rad = 1.0e-9);

bool evaluateEntryGateCandidate(
    CandidatePoint* candidate,
    const RouteConfig& route,
    const geometry_msgs::Point& current_position,
    const geometry_msgs::Point& home_position,
    const std::vector<geometry_msgs::Point>& map_points,
    const std::vector<StaticObstacle>& obstacles,
    bool map_fresh,
    const EntryGateConfig& config,
    double unknown_ratio = 0.0,
    double unknown_ratio_limit = 1.0,
    const CandidatePoint* first_inspection_target = nullptr);

int chooseBestEntryGateCandidate(const std::vector<CandidatePoint>& candidates);

std::vector<CandidatePoint> buildRollingApproachGoals(
    const geometry_msgs::Point& start,
    const CandidatePoint& entry_gate,
    double maximum_segment_length);

std::vector<CandidatePoint> buildVerticalClimbGoals(
    const CandidatePoint& staging_point,
    double entry_height,
    double height_step);

std::vector<CandidatePoint> buildVerticalGoalsAtHeights(
    const CandidatePoint& reference,
    const std::vector<double>& target_heights,
    const std::string& id_prefix);

std::vector<double> deriveInspectionHeights(
    double inspection_top_height,
    const std::vector<double>& layer_offsets);

std::vector<CandidatePoint> buildLayerTransitionGoals(
    const CandidatePoint& from,
    const CandidatePoint& to,
    double maximum_vertical_step,
    double same_xy_tolerance,
    const std::string& id_prefix);

std::vector<int> buildLayerVisitSequence(std::size_t layer_count,
                                         int planned_cycles);

std::vector<std::size_t> buildClosedLapVisitSequence(
    std::size_t waypoint_count,
    int inspection_laps);

double directedAngularDifference(double entry_angle_rad,
                                 double waypoint_angle_rad,
                                 OrbitDirection direction);

std::vector<std::size_t> directionalSectorOrder(
    double entry_angle_rad,
    const std::vector<Sector>& sectors,
    OrbitDirection direction);

void rotateSectorsToNearest(const geometry_msgs::Point& current,
                            std::vector<Sector>* sectors);

int nearestSectorIndexWithAcceptedCandidate(
    const geometry_msgs::Point& current,
    const std::vector<Sector>& sectors);

RecoveryTargets makeRecoveryTargets(const RouteConfig& route,
                                    const geometry_msgs::Point& current,
                                    const Sector& sector,
                                    const RecoveryConfig& config,
                                    const CandidatePoint* locked_target = nullptr);

RecoveryAssessment assessRecoveryTargets(
    const geometry_msgs::Point& current,
    const RecoveryTargets& targets,
    const std::vector<geometry_msgs::Point>& cloud_points,
    const std::vector<StaticObstacle>& obstacles,
    double inflation,
    double sample_step);

bool recoveryTargetsStayInSector(const RecoveryTargets& targets,
                                 const Sector& sector,
                                 double maximum_descent);

bool returnOrLandingTimedOut(bool landing_active,
                             double return_elapsed,
                             double landing_elapsed,
                             double return_timeout,
                             double landing_timeout);

std::vector<CandidatePoint> buildSafeReturnEgressGoals(
    const RouteConfig& route,
    const geometry_msgs::Point& current,
    const geometry_msgs::Point& home,
    const std::vector<StaticObstacle>& obstacles,
    const ReturnEgressConfig& config);

bool returnLandingNearHome(const geometry_msgs::Point& landed_position,
                           const geometry_msgs::Point& home_position,
                           double horizontal_tolerance);

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
