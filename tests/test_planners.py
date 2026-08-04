"""Unit tests for A* and Dijkstra path planning algorithms on occupancy grids."""

import heapq
import math
import unittest


class AStarPlanner:
    def __init__(self, grid, width, height, use_heuristic=True):
        self.grid = grid
        self.width = width
        self.height = height
        self.use_heuristic = use_heuristic

    def heuristic(self, a, b):
        if not self.use_heuristic:
            return 0.0
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        expanded_nodes = 0

        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            expanded_nodes += 1

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path, expanded_nodes

            cx, cy = current
            for dx, dy in neighbors:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if self.grid[ny][nx] != 0:
                        continue
                    cost = math.hypot(dx, dy)
                    tentative_g = g_score[current] + cost
                    neighbor = (nx, ny)
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score, neighbor))

        return None, expanded_nodes


class TestPlanners(unittest.TestCase):
    def setUp(self):
        self.grid = [[0 for _ in range(10)] for _ in range(10)]
        for y in range(2, 8):
            self.grid[y][5] = 100

    def test_astar_finds_path(self):
        planner = AStarPlanner(self.grid, 10, 10, use_heuristic=True)
        path, nodes = planner.plan((1, 5), (8, 5))
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (1, 5))
        self.assertEqual(path[-1], (8, 5))

    def test_dijkstra_expands_more_nodes_than_astar(self):
        astar = AStarPlanner(self.grid, 10, 10, use_heuristic=True)
        dijkstra = AStarPlanner(self.grid, 10, 10, use_heuristic=False)

        _, astar_nodes = astar.plan((1, 5), (8, 5))
        _, dijkstra_nodes = dijkstra.plan((1, 5), (8, 5))

        self.assertLessEqual(astar_nodes, dijkstra_nodes)


if __name__ == '__main__':
    unittest.main()
