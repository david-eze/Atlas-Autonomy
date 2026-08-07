"""Deterministic safety-check core (pure Python, unit-testable).

Independent of Nav2, the AI stack, and the mission layer on purpose.
Takes raw LiDAR ranges and a commanded velocity, decides if it's safe,
and if not, overrides it. This is the only thing that can veto motion,
and the rule set is simple enough to audit by reading it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class SafetyConfig:
    """Safety thresholds. All distances in metres, velocities in m/s."""

    min_obstacle_distance: float = 0.25
    # linear velocity scales down as the closest obstacle nears this;
    # below hard_stop_distance the robot stops outright
    hard_stop_distance: float = 0.20
    max_linear_velocity: float = 1.2
    max_angular_velocity: float = 1.5
    max_scan_age: float = 0.5


@dataclass
class SafetyDecision:
    """Result of evaluating a commanded velocity against the scan."""

    safe: bool
    override_linear: float
    override_angular: float
    min_distance: float
    reason: str


def closest_obstacle(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    max_range: float,
) -> Tuple[float, float]:
    """Return (min_distance, angle_of_min) in the robot frame.

    Angles follow the standard LaserScan convention: 0 is forward,
    positive is counter-clockwise (left).
    """
    min_dist = float('inf')
    min_angle = 0.0
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        if r < min_dist:
            min_dist = r
            min_angle = angle_min + i * angle_increment
    if math.isinf(min_dist):
        return max_range, 0.0
    return min_dist, min_angle


def evaluate_command(
    linear: float,
    angular: float,
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    max_range: float,
    config: SafetyConfig,
) -> SafetyDecision:
    """Check a commanded twist against the current scan.

    Returns a decision with an override velocity. When unsafe, the
    override is zero (hard stop). When the obstacle is between the hard
    stop and the soft threshold, the linear velocity is scaled down
    proportionally so the robot slows as it approaches.
    """
    min_dist, _ = closest_obstacle(ranges, angle_min, angle_increment, max_range)

    if min_dist < config.hard_stop_distance:
        return SafetyDecision(
            safe=False,
            override_linear=0.0,
            override_angular=0.0,
            min_distance=min_dist,
            reason='hard_stop',
        )

    if min_dist < config.min_obstacle_distance:
        # scales 0 at hard_stop_distance up to the commanded value at
        # min_obstacle_distance; angular is left alone so it can steer away
        t = (min_dist - config.hard_stop_distance) / (
            config.min_obstacle_distance - config.hard_stop_distance)
        scaled_linear = linear * max(0.0, min(1.0, t))
        return SafetyDecision(
            safe=False,
            override_linear=scaled_linear,
            override_angular=angular,
            min_distance=min_dist,
            reason='slow_down',
        )

    clamped_linear = max(-config.max_linear_velocity,
                         min(config.max_linear_velocity, linear))
    clamped_angular = max(-config.max_angular_velocity,
                          min(config.max_angular_velocity, angular))
    return SafetyDecision(
        safe=True,
        override_linear=clamped_linear,
        override_angular=clamped_angular,
        min_distance=min_dist,
        reason='ok',
    )
