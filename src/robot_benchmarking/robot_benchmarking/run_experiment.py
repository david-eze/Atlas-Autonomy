"""
Handles simulation, goal dispatch, metrics collection, and report generation.
Also supports ROS execution in CI containers as well as ROS-free local benchmarking.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import random
import subprocess
import time
from pathlib import Path
from typing import List

from .metrics import ExperimentConfig, ExperimentResults, write_csv, write_json

RESULTS_DIR = Path(__file__).resolve().parents[3] / 'results'


def _python_grid_benchmark(
    grid_size: int, obstacles: int, seed: int,
) -> List[dict]:
    """Python grid-search fallback for ROS-free CI benchmarking."""
    rng = random.Random(seed)
    grid = [[0] * grid_size for _ in range(grid_size)]
    for _ in range(obstacles):
        grid[rng.randrange(grid_size)][rng.randrange(grid_size)] = -1
    grid[0][0] = 0
    grid[grid_size - 1][grid_size - 1] = 0

    start = (0, 0)
    goal = (grid_size - 1, grid_size - 1)

    def octile(a, b):
        adx = abs(a[0] - b[0])
        ady = abs(a[1] - b[1])
        return max(adx, ady) + (math.sqrt(2.0) - 1.0) * min(adx, ady)

    def run(use_heuristic):
        g_score = {start: 0.0}
        came_from = {}
        open_heap = [(0.0, start)]
        expanded = 0
        while open_heap:
            f, current = heapq.heappop(open_heap)
            if current == goal:
                break
            if f > g_score.get(current, math.inf) + 1e-9:
                continue
            expanded += 1
            for dr, dc, cost in [
                (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
            ]:
                nr, nc = current[0] + dr, current[1] + dc
                if not (0 <= nr < grid_size and 0 <= nc < grid_size):
                    continue
                if grid[nr][nc] < 0:
                    continue
                nxt = (nr, nc)
                tentative = g_score[current] + cost
                if tentative < g_score.get(nxt, math.inf) - 1e-9:
                    g_score[nxt] = tentative
                    came_from[nxt] = current
                    h = octile(nxt, goal) if use_heuristic else 0.0
                    heapq.heappush(open_heap, (tentative + h, nxt))

        success = goal in g_score
        path_cells = 0.0
        if success:
            node = goal
            prev = None
            while node != start:
                if prev is not None:
                    path_cells += math.hypot(prev[0] - node[0], prev[1] - node[1])
                prev = node
                node = came_from[node]
            path_cells += math.hypot(start[0] - node[0], start[1] - node[1])
        return {
            'success': success,
            'nodes_expanded': expanded,
            'path_length_cells': path_cells,
        }

    rows = []
    for planner, heuristic in (('A*', True), ('Dijkstra', False)):
        t0 = time.perf_counter()
        result = run(heuristic)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        rows.append({
            'planner': planner,
            'success': result['success'],
            'nodes_expanded': result['nodes_expanded'],
            'path_length': result['path_length_cells'],
            'planning_time_ms': elapsed_ms,
        })
    return rows


def _run_ros_experiment(
    env: str,
    planner: str,
    seed: int,
    goal: tuple[float, float],
) -> ExperimentResults:
    experiment_id = f'{env}_{planner.replace("/", "_").lower()}_{seed:03d}'
    config = ExperimentConfig(
        experiment_id=experiment_id,
        environment=env,
        planner=planner,
        random_seed=seed,
        goal_x=goal[0],
        goal_y=goal[1],
        start_x=0.0,
        start_y=0.0,
    )
    results = ExperimentResults(config=config)

    goal_cmd = [
        'ros2', 'action', 'send_goal', '/navigate_to_pose', 'nav2_msgs/action/NavigateToPose',
        json.dumps({'pose': {
            'header': {'frame_id': 'map'},
            'pose': {
                'position': {'x': goal[0], 'y': goal[1], 'z': 0.0},
                'orientation': {'w': 1.0},
            },
        }}),
        '--feedback',
    ]
    proc = subprocess.run(goal_cmd, capture_output=True, text=True, timeout=120)

    stdout = proc.stdout or ''
    results.success = 'SUCCEEDED' in stdout
    if not results.success:
        results.failure_reason = proc.stderr.strip() or 'nav2_goal_failed'

    results.execution_time = 0.0
    write_json([results], RESULTS_DIR / 'navigation_results.json')
    return results


def _run_planner_benchmark(env: str, seed: int) -> List[ExperimentResults]:
    grid_size = 40 if env == 'challenging' else 30
    obstacles = 120 if env == 'challenging' else 60
    rows = _python_grid_benchmark(grid_size=grid_size, obstacles=obstacles, seed=seed)

    out: List[ExperimentResults] = []
    for row in rows:
        config = ExperimentConfig(
            experiment_id=f'{env}_{row["planner"].lower()}_{seed:03d}',
            environment=env,
            planner=row['planner'],
            random_seed=seed,
            goal_x=row.get('goal_x', 0.0),
            goal_y=row.get('goal_y', 0.0),
            start_x=row.get('start_x', 0.0),
            start_y=row.get('start_y', 0.0),
        )
        r = ExperimentResults(config=config, success=row['success'])
        r.planning_time_ms = row['planning_time_ms']
        r.path_length = row['path_length']
        r.nodes_expanded = row['nodes_expanded']
        out.append(r)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description='Run autonomous robot experiments.')
    parser.add_argument('--environment', default='office',
                        choices=['office', 'warehouse', 'challenging'])
    parser.add_argument('--planner', default='astar',
                        choices=['astar', 'dijkstra', 'nav2'])
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--goal-x', type=float, default=8.0)
    parser.add_argument('--goal-y', type=float, default=8.0)
    parser.add_argument('--local-only', action='store_true',
                        help='Run the planner grid benchmark without ROS/simulation.')
    args = parser.parse_args()

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    if args.local_only:
        results = _run_planner_benchmark(args.environment, args.seed)
        write_json(results, RESULTS_DIR / f'{args.environment}_planners.json')
        write_csv(results, RESULTS_DIR / 'csv' / 'planner_results.csv')
        print(f'Recorded {len(results)} planner benchmarks for {args.environment}')
        return

    print(
        f'Running {args.environment}/{args.planner} (seed={args.seed}, '
        f'goal=({args.goal_x}, {args.goal_y}))...')
    result = _run_ros_experiment(
        args.environment, args.planner, args.seed, (args.goal_x, args.goal_y))
    print(json.dumps(result.to_dict(), indent=2))

    from .generate_report import main as generate_report_main
    from .generate_gifs import main as generate_gifs_main
    generate_gifs_main()
    generate_report_main()


if __name__ == '__main__':
    main()
