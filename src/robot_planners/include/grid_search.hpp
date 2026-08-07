// Shared grid-search core, A* + Dijkstra planners.
//
// Same 8-connected grid, only the heuristic differs:
//   * A*       h(n) = max(|dx|, |dy|) * (sqrt(2)-1) + min(|dx|,|dy|)  (octile)
//   * Dijkstra h(n) = 0
//
// Instrumented (nodes_expanded, path_length_cells) for benchmarking.

#ifndef ROBOT_PLANNERS__GRID_SEARCH_HPP_
#define ROBOT_PLANNERS__GRID_SEARCH_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace robot_planners
{

struct GridSearchResult
{
  bool success{false};
  std::vector<std::pair<int, int>> path;  // cell coords, not world
  std::size_t nodes_expanded{0};
  double path_length_cells{0.0};
};

// cardinal 1.0, diagonal sqrt(2)
inline double step_cost(int dx, int dy)
{
  return (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
}

// admissible for 8-connected grids
inline double octile_heuristic(int dx, int dy)
{
  const int adx = std::abs(dx);
  const int ady = std::abs(dy);
  return static_cast<double>(std::max(adx, ady)) +
    (std::sqrt(2.0) - 1.0) * static_cast<double>(std::min(adx, ady));
}

// use_heuristic true -> A*, false -> Dijkstra.
// cost_at: 0 free, >0 cost, <0 lethal/unknown (untraversable).
// Stops on goal pop, optimal either way.
template<typename CostFn>
GridSearchResult grid_search(
  int width, int height,
  int start_x, int start_y,
  int goal_x, int goal_y,
  CostFn cost_at,
  bool use_heuristic)
{
  GridSearchResult result;

  if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
    goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height)
  {
    return result;
  }

  const auto index = [width](int x, int y) { return y * width + x; };

  // dense vector beats unordered_map at these grid sizes (few thousand cells)
  std::vector<double> g_score(static_cast<std::size_t>(width * height), -1.0);
  std::vector<std::pair<int, int>> came_from(
    static_cast<std::size_t>(width * height), {-1, -1});

  using Node = std::pair<double, int>;  // (f, cell index)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

  const int start_idx = index(start_x, start_y);
  g_score[start_idx] = 0.0;
  open.emplace(0.0, start_idx);

  constexpr int kDirs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
  };

  while (!open.empty()) {
    const auto [f, current] = open.top();
    open.pop();

    const int cx = current % width;
    const int cy = current / width;

    if (cx == goal_x && cy == goal_y) {
      result.success = true;
      break;
    }

    // stale entry: g not stored per-entry, recompute h and compare to f
    double h_current = 0.0;
    if (use_heuristic) {
      h_current = octile_heuristic(goal_x - cx, goal_y - cy);
    }
    if (g_score[current] < 0.0 || f > g_score[current] + h_current + 1e-9) {
      continue;
    }

    ++result.nodes_expanded;

    for (const auto & dir : kDirs) {
      const int nx = cx + dir[0];
      const int ny = cy + dir[1];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      const double cell_cost = cost_at(nx, ny);
      if (cell_cost < 0.0) {
        continue;  // lethal/unknown
      }

      const int nidx = index(nx, ny);
      const double tentative = g_score[current] +
        step_cost(dir[0], dir[1]) * (1.0 + cell_cost);

      if (g_score[nidx] < 0.0 || tentative < g_score[nidx] - 1e-9) {
        g_score[nidx] = tentative;
        came_from[nidx] = {cx, cy};
        double h = 0.0;
        if (use_heuristic) {
          h = octile_heuristic(goal_x - nx, goal_y - ny);
        }
        open.emplace(tentative + h, nidx);
      }
    }
  }

  if (!result.success) {
    return result;
  }

  // walk back goal -> start
  int cx = goal_x;
  int cy = goal_y;
  while (cx != start_x || cy != start_y) {
    result.path.emplace_back(cx, cy);
    const auto prev = came_from[index(cx, cy)];
    if (prev.first < 0) {
      result.success = false;
      result.path.clear();
      return result;
    }
    cx = prev.first;
    cy = prev.second;
  }
  result.path.emplace_back(start_x, start_y);
  std::reverse(result.path.begin(), result.path.end());

  for (std::size_t i = 1; i < result.path.size(); ++i) {
    const auto & a = result.path[i - 1];
    const auto & b = result.path[i];
    result.path_length_cells += step_cost(b.first - a.first, b.second - a.second);
  }

  return result;
}

}  // namespace robot_planners

#endif  // ROBOT_PLANNERS__GRID_SEARCH_HPP_
