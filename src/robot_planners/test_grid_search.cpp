#include <cmath>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "robot_planners/grid_search.hpp"

namespace
{

struct FreeGrid
{
  int width;
  int height;
  double operator()(int, int) const { return 0.0; }
};

// Grid with a vertical wall of lethal cells at x = wall_x, leaving a
// gap at row gap_y so the goal stays reachable but requires a detour
struct WallGrid
{
  int width;
  int height;
  int wall_x;
  int gap_y;
  double operator()(int x, int y) const
  {
    return (x == wall_x && y != gap_y) ? -1.0 : 0.0;
  }
};

}  // namespace

TEST(GridSearchTest, AstarFindsShortestPathOnFreeGrid)
{
  FreeGrid grid{10, 10};
  auto result = robot_planners::grid_search(10, 10, 0, 0, 9, 9, grid, true);
  ASSERT_TRUE(result.success);
  EXPECT_GT(result.nodes_expanded, 0u);
  // Octile heuristic should make the path straight diagonal
  EXPECT_NEAR(result.path_length_cells, 9.0 * std::sqrt(2.0), 1e-6);
  EXPECT_EQ(result.path.front(), std::make_pair(0, 0));
  EXPECT_EQ(result.path.back(), std::make_pair(9, 9));
}

TEST(GridSearchTest, DijkstraFindsPathOfSameLength)
{
  FreeGrid grid{10, 10};
  auto astar = robot_planners::grid_search(10, 10, 0, 0, 9, 9, grid, true);
  auto dijkstra = robot_planners::grid_search(10, 10, 0, 0, 9, 9, grid, false);
  ASSERT_TRUE(astar.success);
  ASSERT_TRUE(dijkstra.success);
  EXPECT_NEAR(astar.path_length_cells, dijkstra.path_length_cells, 1e-6);
  // Dijkstra expands at least as many nodes (often far more)
  EXPECT_GE(dijkstra.nodes_expanded, astar.nodes_expanded);
}

TEST(GridSearchTest, WallForcesDetour)
{
  // Full-height wall at x == 5 with a single gap at y == 0.  The gap is
  // off the direct row (y == 5) so any valid path must detour around
  // the wall and is therefore longer than the straight line
  WallGrid grid{11, 11, 5, 0};
  auto result = robot_planners::grid_search(11, 11, 0, 5, 10, 5, grid, true);
  ASSERT_TRUE(result.success);
  EXPECT_GT(result.path_length_cells, 10.0 + 1e-6);
}

TEST(GridSearchTest, UnreachableGoalFails)
{
  struct BoxedGrid
  {
    int width;
    int height;
    int box_x;
    int box_y;
    double operator()(int x, int y) const
    {
      const bool inside =
        x == box_x - 1 || x == box_x + 1 || y == box_y - 1 || y == box_y + 1;
      return inside ? -1.0 : 0.0;
    }
  };
  // 7x7 box around (3,3) with a lethal ring at distance 1
  BoxedGrid grid{7, 7, 3, 3};
  auto result = robot_planners::grid_search(7, 7, 0, 0, 3, 3, grid, true);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.path.empty());
}

TEST(GridSearchTest, OutOfBoundsFailsGracefully)
{
  FreeGrid grid{5, 5};
  auto result = robot_planners::grid_search(5, 5, -1, 0, 4, 4, grid, true);
  EXPECT_FALSE(result.success);
  result = robot_planners::grid_search(5, 5, 0, 0, 5, 4, grid, true);
  EXPECT_FALSE(result.success);
}
