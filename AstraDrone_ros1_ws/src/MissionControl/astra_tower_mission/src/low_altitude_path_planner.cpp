#include "astra_tower_mission/low_altitude_path_planner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace astra_tower_mission {
namespace {

constexpr int kDirectionCount = 8;
constexpr int kStartDirection = kDirectionCount;
constexpr int kStateDirectionCount = kDirectionCount + 1;
constexpr double kPi = 3.14159265358979323846;

const std::array<int, kDirectionCount> kDx{{1, 1, 0, -1, -1, -1, 0, 1}};
const std::array<int, kDirectionCount> kDy{{0, 1, 1, 1, 0, -1, -1, -1}};

bool finitePoint(const geometry_msgs::Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

double distanceToSegment(double x, double y,
                         const geometry_msgs::Point& start,
                         const geometry_msgs::Point& goal) {
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double denominator = dx * dx + dy * dy;
  if (denominator <= 1.0e-12) return std::hypot(x - goal.x, y - goal.y);
  const double ratio = std::max(
      0.0, std::min(1.0, ((x - start.x) * dx + (y - start.y) * dy) /
                              denominator));
  return std::hypot(x - (start.x + ratio * dx),
                    y - (start.y + ratio * dy));
}

double turnAngle(int previous_direction, int next_direction) {
  if (previous_direction == kStartDirection) return 0.0;
  int delta = std::abs(previous_direction - next_direction);
  delta = std::min(delta, kDirectionCount - delta);
  return delta * kPi / 4.0;
}

struct QueueItem {
  double priority{0.0};
  std::size_t index{0U};
};

struct QueueItemCompare {
  bool operator()(const QueueItem& left, const QueueItem& right) const {
    return left.priority > right.priority;
  }
};

}  // namespace

LevelPathResult planLevelPath(
    const geometry_msgs::Point& start,
    const geometry_msgs::Point& goal,
    const std::vector<geometry_msgs::Point>& occupied_points,
    const LevelPathConfig& config) {
  LevelPathResult result;
  const LevelPathCostWeights& weights = config.cost;
  if (!finitePoint(start) || !finitePoint(goal) ||
      !std::isfinite(config.altitude) ||
      !std::isfinite(config.vertical_half_extent) ||
      !std::isfinite(config.additional_clearance) ||
      !std::isfinite(config.resolution) ||
      !std::isfinite(config.boundary_margin) ||
      !std::isfinite(config.maximum_segment_length) ||
      !std::isfinite(config.tower_x) ||
      !std::isfinite(config.tower_y) ||
      !std::isfinite(config.tower_keep_out_radius) ||
      !std::isfinite(config.preferred_tower_radius) ||
      !std::isfinite(weights.path_length) ||
      !std::isfinite(weights.goal_deviation) ||
      !std::isfinite(weights.tower_distance) ||
      !std::isfinite(weights.turn) ||
      !std::isfinite(weights.obstacle_clearance) ||
      config.vertical_half_extent <= 0.0 ||
      config.additional_clearance < 0.0 || config.resolution <= 0.0 ||
      config.boundary_margin <= 0.0 ||
      config.maximum_segment_length <= 0.0 ||
      config.maximum_cell_count == 0U ||
      config.tower_keep_out_radius < 0.0 ||
      config.preferred_tower_radius < config.tower_keep_out_radius ||
      weights.path_length <= 0.0 || weights.goal_deviation < 0.0 ||
      weights.tower_distance < 0.0 || weights.turn < 0.0 ||
      weights.obstacle_clearance < 0.0) {
    result.reason = "INVALID_LEVEL_PATH_CONFIG";
    return result;
  }

  const double minimum_x =
      std::min(start.x, goal.x) - config.boundary_margin;
  const double maximum_x =
      std::max(start.x, goal.x) + config.boundary_margin;
  const double minimum_y =
      std::min(start.y, goal.y) - config.boundary_margin;
  const double maximum_y =
      std::max(start.y, goal.y) + config.boundary_margin;
  const int width =
      static_cast<int>(std::ceil((maximum_x - minimum_x) /
                                 config.resolution)) +
      1;
  const int height =
      static_cast<int>(std::ceil((maximum_y - minimum_y) /
                                 config.resolution)) +
      1;
  if (width < 3 || height < 3 ||
      static_cast<std::size_t>(width) >
          config.maximum_cell_count / static_cast<std::size_t>(height)) {
    result.reason = "LEVEL_PATH_GRID_TOO_LARGE";
    return result;
  }
  const std::size_t cell_count =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (cell_count > config.maximum_cell_count) {
    result.reason = "LEVEL_PATH_GRID_TOO_LARGE";
    return result;
  }

  const auto indexOf = [width](int x, int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x);
  };
  const auto gridX = [&](double x) {
    return static_cast<int>(
        std::lround((x - minimum_x) / config.resolution));
  };
  const auto gridY = [&](double y) {
    return static_cast<int>(
        std::lround((y - minimum_y) / config.resolution));
  };
  const auto worldX = [&](int x) {
    return minimum_x + x * config.resolution;
  };
  const auto worldY = [&](int y) {
    return minimum_y + y * config.resolution;
  };
  const auto inBounds = [width, height](int x, int y) {
    return x >= 0 && x < width && y >= 0 && y < height;
  };

  // Safety is a hard mask. The input occupancy has already been expanded by
  // EGO for the vehicle body; this dilation adds only the configured
  // operational margin.
  std::vector<std::uint8_t> blocked(cell_count, 0U);
  const int inflation_cells = static_cast<int>(
      std::ceil(config.additional_clearance / config.resolution));
  for (const auto& point : occupied_points) {
    if (!finitePoint(point) ||
        std::abs(point.z - config.altitude) >
            config.vertical_half_extent) {
      continue;
    }
    const int center_x = gridX(point.x);
    const int center_y = gridY(point.y);
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        if (dx * dx + dy * dy >
            inflation_cells * inflation_cells) {
          continue;
        }
        const int x = center_x + dx;
        const int y = center_y + dy;
        if (inBounds(x, y)) blocked[indexOf(x, y)] = 1U;
      }
    }
  }
  if (config.use_tower_constraint) {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (std::hypot(worldX(x) - config.tower_x,
                       worldY(y) - config.tower_y) <
            config.tower_keep_out_radius) {
          blocked[indexOf(x, y)] = 1U;
        }
      }
    }
  }
  result.occupied_cell_count = static_cast<std::size_t>(
      std::count(blocked.begin(), blocked.end(), std::uint8_t{1U}));

  const int start_x = gridX(start.x);
  const int start_y = gridY(start.y);
  const int goal_x = gridX(goal.x);
  const int goal_y = gridY(goal.y);
  if (!inBounds(start_x, start_y) || !inBounds(goal_x, goal_y)) {
    result.reason = "LEVEL_PATH_ENDPOINT_OUTSIDE_GRID";
    return result;
  }
  const std::size_t start_cell = indexOf(start_x, start_y);
  const std::size_t goal_cell = indexOf(goal_x, goal_y);
  if (blocked[start_cell] != 0U) {
    result.reason = "LEVEL_PATH_START_OCCUPIED";
    return result;
  }
  if (blocked[goal_cell] != 0U) {
    result.reason = "LEVEL_PATH_GOAL_OCCUPIED";
    return result;
  }

  // Distance to the hard mask is used only to compare already-safe paths.
  // It cannot relax the blocked mask above.
  std::vector<double> clearance_cells(
      cell_count, std::numeric_limits<double>::infinity());
  std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemCompare>
      clearance_queue;
  for (std::size_t index = 0U; index < cell_count; ++index) {
    if (blocked[index] != 0U) {
      clearance_cells[index] = 0.0;
      clearance_queue.push({0.0, index});
    }
  }
  while (!clearance_queue.empty()) {
    const QueueItem current = clearance_queue.top();
    clearance_queue.pop();
    if (current.priority > clearance_cells[current.index] + 1.0e-12) {
      continue;
    }
    const int x =
        static_cast<int>(current.index % static_cast<std::size_t>(width));
    const int y =
        static_cast<int>(current.index / static_cast<std::size_t>(width));
    for (int direction = 0; direction < kDirectionCount; ++direction) {
      const int next_x = x + kDx[direction];
      const int next_y = y + kDy[direction];
      if (!inBounds(next_x, next_y)) continue;
      const std::size_t next = indexOf(next_x, next_y);
      const double step =
          kDx[direction] != 0 && kDy[direction] != 0 ? std::sqrt(2.0) : 1.0;
      const double candidate = current.priority + step;
      if (candidate + 1.0e-12 < clearance_cells[next]) {
        clearance_cells[next] = candidate;
        clearance_queue.push({candidate, next});
      }
    }
  }

  const std::size_t state_count =
      cell_count * static_cast<std::size_t>(kStateDirectionCount);
  const auto stateOf = [](std::size_t cell, int direction) {
    return cell * static_cast<std::size_t>(kStateDirectionCount) +
           static_cast<std::size_t>(direction);
  };
  const auto cellOf = [](std::size_t state) {
    return state / static_cast<std::size_t>(kStateDirectionCount);
  };
  const auto directionOf = [](std::size_t state) {
    return static_cast<int>(
        state % static_cast<std::size_t>(kStateDirectionCount));
  };
  const auto heuristic = [&](int x, int y) {
    return weights.path_length * config.resolution *
           std::hypot(static_cast<double>(goal_x - x),
                      static_cast<double>(goal_y - y));
  };
  const auto cellPolicyCost = [&](int x, int y, std::size_t cell,
                                  double step_length) {
    const double wx = worldX(x);
    const double wy = worldY(y);
    const double goal_deviation = distanceToSegment(wx, wy, start, goal);
    double tower_deviation = 0.0;
    if (config.use_tower_constraint) {
      tower_deviation =
          std::abs(std::hypot(wx - config.tower_x, wy - config.tower_y) -
                   config.preferred_tower_radius);
    }
    const double clearance =
        clearance_cells[cell] * config.resolution;
    const double clearance_penalty =
        std::isfinite(clearance)
            ? 1.0 / std::max(clearance, config.resolution)
            : 0.0;
    return step_length *
           (weights.goal_deviation * goal_deviation +
            weights.tower_distance * tower_deviation +
            weights.obstacle_clearance * clearance_penalty);
  };

  std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemCompare> open;
  std::vector<double> cost(
      state_count, std::numeric_limits<double>::infinity());
  std::vector<std::int64_t> parent(state_count, -1);
  std::vector<std::uint8_t> closed(state_count, 0U);
  const std::size_t start_state = stateOf(start_cell, kStartDirection);
  cost[start_state] = 0.0;
  open.push({heuristic(start_x, start_y), start_state});
  std::size_t goal_state = state_count;
  while (!open.empty()) {
    const QueueItem current = open.top();
    open.pop();
    if (closed[current.index] != 0U) continue;
    closed[current.index] = 1U;
    const std::size_t current_cell = cellOf(current.index);
    if (current_cell == goal_cell) {
      goal_state = current.index;
      break;
    }
    const int current_direction = directionOf(current.index);
    const int current_x =
        static_cast<int>(current_cell % static_cast<std::size_t>(width));
    const int current_y =
        static_cast<int>(current_cell / static_cast<std::size_t>(width));
    for (int direction = 0; direction < kDirectionCount; ++direction) {
      const int next_x = current_x + kDx[direction];
      const int next_y = current_y + kDy[direction];
      if (!inBounds(next_x, next_y)) continue;
      const std::size_t next_cell = indexOf(next_x, next_y);
      if (blocked[next_cell] != 0U) continue;
      const bool diagonal =
          kDx[direction] != 0 && kDy[direction] != 0;
      if (diagonal &&
          (blocked[indexOf(current_x + kDx[direction], current_y)] != 0U ||
           blocked[indexOf(current_x, current_y + kDy[direction])] != 0U)) {
        continue;
      }
      const double step_length =
          config.resolution * (diagonal ? std::sqrt(2.0) : 1.0);
      const double next_cost =
          cost[current.index] + weights.path_length * step_length +
          weights.turn * turnAngle(current_direction, direction) +
          cellPolicyCost(next_x, next_y, next_cell, step_length);
      const std::size_t next_state = stateOf(next_cell, direction);
      if (next_cost + 1.0e-12 >= cost[next_state]) continue;
      cost[next_state] = next_cost;
      parent[next_state] = static_cast<std::int64_t>(current.index);
      open.push({next_cost + heuristic(next_x, next_y), next_state});
    }
  }
  if (goal_state == state_count) {
    result.reason = "NO_LEVEL_PATH";
    return result;
  }

  std::vector<std::pair<int, int>> cells;
  for (std::int64_t state = static_cast<std::int64_t>(goal_state);
       state >= 0;) {
    const std::size_t cell = cellOf(static_cast<std::size_t>(state));
    cells.emplace_back(
        static_cast<int>(cell % static_cast<std::size_t>(width)),
        static_cast<int>(cell / static_cast<std::size_t>(width)));
    if (static_cast<std::size_t>(state) == start_state) break;
    state = parent[static_cast<std::size_t>(state)];
  }
  if (cells.empty() ||
      cells.back() != std::make_pair(start_x, start_y)) {
    result.reason = "LEVEL_PATH_PARENT_CHAIN_BROKEN";
    return result;
  }
  std::reverse(cells.begin(), cells.end());

  // Preserve the scored route. Only collinear grid runs are compressed, so
  // simplification cannot silently switch to the opposite side of an
  // obstacle or erase the tower-distance/clearance decision.
  std::vector<std::pair<int, int>> bends;
  bends.push_back(cells.front());
  int previous_dx = 0;
  int previous_dy = 0;
  for (std::size_t index = 1U; index < cells.size(); ++index) {
    const int dx = cells[index].first - cells[index - 1U].first;
    const int dy = cells[index].second - cells[index - 1U].second;
    if (index > 1U && (dx != previous_dx || dy != previous_dy)) {
      bends.push_back(cells[index - 1U]);
    }
    previous_dx = dx;
    previous_dy = dy;
  }
  if (bends.back() != cells.back()) bends.push_back(cells.back());

  result.points.push_back(start);
  geometry_msgs::Point previous = start;
  double minimum_clearance = std::numeric_limits<double>::infinity();
  for (std::size_t bend_index = 1U; bend_index < bends.size(); ++bend_index) {
    geometry_msgs::Point bend;
    const bool final_bend = bend_index + 1U == bends.size();
    bend.x = final_bend ? goal.x : worldX(bends[bend_index].first);
    bend.y = final_bend ? goal.y : worldY(bends[bend_index].second);
    bend.z = config.altitude;
    const double distance =
        std::hypot(bend.x - previous.x, bend.y - previous.y);
    const int segments = std::max(
        1, static_cast<int>(
               std::ceil(distance / config.maximum_segment_length)));
    for (int segment = 1; segment <= segments; ++segment) {
      const double ratio = static_cast<double>(segment) / segments;
      geometry_msgs::Point waypoint;
      waypoint.x = previous.x + ratio * (bend.x - previous.x);
      waypoint.y = previous.y + ratio * (bend.y - previous.y);
      waypoint.z = config.altitude;
      result.points.push_back(waypoint);
    }
    result.path_length += distance;
    previous = bend;
  }
  for (const auto& cell : cells) {
    minimum_clearance = std::min(
        minimum_clearance,
        clearance_cells[indexOf(cell.first, cell.second)] *
            config.resolution);
  }
  result.minimum_clearance =
      std::isfinite(minimum_clearance) ? minimum_clearance : 100.0;
  result.score = cost[goal_state];
  result.reachable = result.points.size() >= 2U;
  result.reason = result.reachable ? "LEVEL_PATH_AVAILABLE"
                                   : "LEVEL_PATH_EMPTY";
  return result;
}

}  // namespace astra_tower_mission
