"""Pure-python map quality metrics.

Operates on an occupancy grid (NumPy array, -1 unknown, 0..100 occupied)
and returns coverage, free/occupied/unknown fractions, frontier cell
count and a quality score. No ROS imports so it is unit-testable on host.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np

# Normalisation dtype: signed, so -1 (unknown) round-trips exactly for any
# input dtype. Without this, an unsigned grid (e.g. uint8) would wrap -1 to
# 255 and silently defeat "unknown" detection.
_GRID_DTYPE = np.int16


def map_statistics(
    grid: np.ndarray,
    unknown: int = -1,
    occupied_threshold: int = 50,
) -> Dict[str, float]:
    """Compute coverage statistics for an occupancy grid.

    Occupied cells are those with value >= occupied_threshold; anything
    else non-negative is free space. Unknown cells are excluded from both
    masks so the three fractions always sum to 1. The grid is normalised
    to a signed integer dtype so negative unknown markers are reliable
    regardless of the caller's dtype.
    """
    grid = np.asarray(grid, dtype=_GRID_DTYPE)
    if grid.ndim != 2:
        raise ValueError(f'Expected a 2-D occupancy grid, got shape {grid.shape}')

    total = grid.size
    if total == 0:
        return {
            'coverage': 0.0,
            'free_fraction': 0.0,
            'occupied_fraction': 0.0,
            'unknown_fraction': 1.0,
            'frontier_count': 0.0,
        }

    unknown_mask = grid == unknown
    occupied_mask = (grid >= occupied_threshold) & ~unknown_mask
    free_mask = ~unknown_mask & ~occupied_mask

    covered = total - int(np.count_nonzero(unknown_mask))
    stats = {
        'coverage': float(covered) / total,
        'free_fraction': float(np.count_nonzero(free_mask)) / total,
        'occupied_fraction': float(np.count_nonzero(occupied_mask)) / total,
        'unknown_fraction': float(np.count_nonzero(unknown_mask)) / total,
        'frontier_count': float(len(find_frontier_cells(grid, unknown, occupied_threshold))),
    }
    return stats


def find_frontier_cells(
    grid: np.ndarray,
    unknown: int = -1,
    occupied_threshold: int = 50,
) -> List[Tuple[int, int]]:
    """Return (row, col) coordinates of frontier cells.

    A frontier cell is a free cell adjacent (4-connected) to at least one
    unknown cell. Fully vectorised with NumPy (no per-cell Python loop),
    so it is safe to call on every map tick even for large grids.
    """
    grid = np.asarray(grid, dtype=_GRID_DTYPE)
    if grid.ndim != 2:
        raise ValueError(f'Expected a 2-D occupancy grid, got shape {grid.shape}')
    if grid.size == 0:
        return []

    unknown_mask = grid == unknown
    free_mask = (grid != unknown) & (grid < occupied_threshold)

    if not free_mask.any():
        return []

    # 4-connected unknown-neighbour test, vectorised via a padded shift.
    # A border of False is added so map-edge cells never see out-of-bounds
    # neighbours.
    unknown_padded = np.pad(unknown_mask, 1, constant_values=False)
    neighbor_unknown = (
        unknown_padded[:-2, 1:-1]      # up
        | unknown_padded[2:, 1:-1]     # down
        | unknown_padded[1:-1, :-2]    # left
        | unknown_padded[1:-1, 2:]     # right
    )
    frontier_mask = free_mask & neighbor_unknown
    rows, cols = np.nonzero(frontier_mask)
    return list(zip(rows.tolist(), cols.tolist()))


def quality_score(stats: Dict[str, float]) -> float:
    """A single 0..1 score favouring high coverage with low unknown area.

    Coverage is defined as ``1 - unknown_fraction`` by construction, so the
    two factors in the product are not independent: the score is effectively
    ``coverage^2``. It ranks maps monotonically with coverage while
    compressing low values, which is the desired stop-exploring behaviour.
    Used by the exploration controller to decide when to stop exploring.
    """
    return stats['coverage'] * (1.0 - stats['unknown_fraction'])
