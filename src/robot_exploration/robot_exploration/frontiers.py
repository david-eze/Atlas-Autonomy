"""Frontier detection and scoring (pure Python, no ROS deps so it's easy to test).

A frontier is a free cell next to unknown space. We cluster adjacent
frontier cells together and score each cluster on info gain, distance
from the robot, distance to obstacles, and a history penalty so we
don't keep circling back to the same spot.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

UNKNOWN = -1
OCCUPIED_THRESHOLD = 50


@dataclass(frozen=True)
class Frontier:
    """A connected cluster of frontier cells."""

    centroid: Tuple[int, int]         # (row, col)
    cells: Tuple[Tuple[int, int], ...]
    score: float = 0.0
    info_gain: int = 0                # unknown neighbours across the cluster


def find_frontier_clusters(grid: np.ndarray) -> List[Frontier]:
    """Find connected frontier clusters in an occupancy grid.

    A cell counts as frontier if it's free and has at least one
    4-connected unknown neighbour. We flood-fill the frontier cells
    into clusters afterward.
    """
    unknown_mask = grid == UNKNOWN
    free_mask = (grid != UNKNOWN) & (grid < OCCUPIED_THRESHOLD)
    height, width = grid.shape

    frontier_cells = []
    for r in range(height):
        for c in range(width):
            if not free_mask[r, c]:
                continue
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width and unknown_mask[nr, nc]:
                    frontier_cells.append((r, c))
                    break

    if not frontier_cells:
        return []

    cell_set = set(frontier_cells)
    visited = set()
    clusters: List[List[Tuple[int, int]]] = []

    for cell in frontier_cells:
        if cell in visited:
            continue
        stack = [cell]
        cluster = []
        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)
            cluster.append(current)
            r, c = current
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nxt = (r + dr, c + dc)
                if nxt in cell_set and nxt not in visited:
                    stack.append(nxt)
        if cluster:
            clusters.append(cluster)

    frontiers = []
    for cluster in clusters:
        rows = np.array([cell[0] for cell in cluster])
        cols = np.array([cell[1] for cell in cluster])
        centroid = (int(round(rows.mean())), int(round(cols.mean())))
        gain = 0
        for r, c in cluster:
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width and unknown_mask[nr, nc]:
                    gain += 1
        frontiers.append(Frontier(
            centroid=centroid,
            cells=tuple(cluster),
            info_gain=gain,
        ))
    return frontiers


def score_frontiers(
    frontiers: Sequence[Frontier],
    robot_cell: Tuple[int, int],
    grid: np.ndarray,
    history: Dict[Tuple[int, int], float] | None = None,
    weights: Dict[str, float] | None = None,
) -> List[Frontier]:
    """Rank frontier clusters, best first.

    score = w_gain * normalised_info_gain
          - w_dist  * normalised_robot_distance
          - w_obs   * normalised_obstacle_proximity
          - w_hist  * normalised_history_penalty

    Obstacle proximity knocks down frontiers sitting close to occupied
    cells, since those are usually tight passages the diff-drive base
    can't reliably get through.
    """
    if not frontiers:
        return []

    w = weights or {
        'gain': 1.0, 'dist': 1.0, 'obs': 1.0, 'history': 1.0,
    }
    height, width = grid.shape

    robot_r, robot_c = robot_cell
    gains = np.array([f.info_gain for f in frontiers], dtype=float)
    distances = np.array([
        np.hypot(f.centroid[0] - robot_r, f.centroid[1] - robot_c)
        for f in frontiers
    ], dtype=float)

    occupied_mask = grid >= OCCUPIED_THRESHOLD
    # Chamfer distance to nearest occupied cell (cheaper than a real
    # distance transform, close enough for scoring purposes).
    obstacle_dist = np.full((height, width), np.inf, dtype=float)
    obstacle_dist[occupied_mask] = 0.0
    for r in range(height):
        for c in range(width):
            if obstacle_dist[r, c] != 0.0:
                candidates = []
                if r > 0:
                    candidates.append(obstacle_dist[r - 1, c] + 1.0)
                if c > 0:
                    candidates.append(obstacle_dist[r, c - 1] + 1.0)
                if r > 0 and c > 0:
                    candidates.append(obstacle_dist[r - 1, c - 1] + 1.5)
                if r > 0 and c < width - 1:
                    candidates.append(obstacle_dist[r - 1, c + 1] + 1.5)
                if candidates:
                    obstacle_dist[r, c] = min(obstacle_dist[r, c], min(candidates))
    for r in range(height - 1, -1, -1):
        for c in range(width - 1, -1, -1):
            candidates = [obstacle_dist[r, c]]
            if r < height - 1:
                candidates.append(obstacle_dist[r + 1, c] + 1.0)
            if c < width - 1:
                candidates.append(obstacle_dist[r, c + 1] + 1.0)
            if r < height - 1 and c < width - 1:
                candidates.append(obstacle_dist[r + 1, c + 1] + 1.5)
            if r < height - 1 and c > 0:
                candidates.append(obstacle_dist[r + 1, c - 1] + 1.5)
            obstacle_dist[r, c] = min(candidates)

    proximity = np.array([
        obstacle_dist[f.centroid[0], f.centroid[1]] for f in frontiers
    ], dtype=float)
    proximity = np.where(np.isinf(proximity), 50.0, proximity)

    hist_penalty = np.array([
        history.get(f.centroid, 0.0) for f in frontiers
    ], dtype=float)

    def normalize(values: np.ndarray) -> np.ndarray:
        span = values.max() - values.min()
        if span < 1e-9:
            return np.zeros_like(values)
        return (values - values.min()) / span

    scores = (
        w['gain'] * normalize(gains)
        - w['dist'] * normalize(distances)
        - w['obs'] * normalize(1.0 / (1.0 + proximity))
        - w['hist'] * normalize(hist_penalty)
    )

    scored = []
    for f, s in zip(frontiers, scores):
        scored.append(Frontier(
            centroid=f.centroid,
            cells=f.cells,
            score=float(s),
            info_gain=f.info_gain,
        ))
    scored.sort(key=lambda f: f.score, reverse=True)
    return scored
