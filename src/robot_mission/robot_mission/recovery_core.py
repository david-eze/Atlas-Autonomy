"""Recovery decision core (pure Python, unit-testable).

Analyses a navigation failure and selects an appropriate recovery
action, rather than blindly repeating the same behaviour. The decision
is based on the failure type, sensor health, and obstacle persistence.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict


class FailureType(Enum):
    BLOCKED_PATH = 'blocked_path'
    LOCALIZATION_UNCERTAIN = 'localization_uncertain'
    PLANNER_FAILURE = 'planner_failure'
    CONTROLLER_FAILURE = 'controller_failure'
    SENSOR_DEGRADED = 'sensor_degraded'
    UNKNOWN = 'unknown'


class RecoveryAction(Enum):
    STOP = 'stop'
    ROTATE_IN_PLACE = 'rotate_in_place'
    CLEAR_COSTMAP = 'clear_costmap'
    REPLAN = 'replan'
    BACKUP = 'backup'
    WAIT = 'wait'
    DEGRADE_SENSORS = 'degrade_sensors'
    ABORT = 'abort'


@dataclass
class FailureContext:
    """Information available at the time of a navigation failure."""

    failure_type: FailureType
    obstacle_persistence_count: int = 0
    localization_covariance: float = 0.0
    sensor_health: Dict[str, bool] = field(default_factory=dict)
    replan_count: int = 0
    recovery_count: int = 0


@dataclass
class RecoveryDecision:
    """The selected recovery action and its rationale."""

    action: RecoveryAction
    reason: str
    log_entry: str


def decide_recovery(ctx: FailureContext) -> RecoveryDecision:
    """Map a failure context to a recovery action.

    The decision logic is deliberately explicit and auditable:
      * Localization uncertainty -> stop, rotate in place, then replan.
      * Blocked path -> clear costmap, then replan; if the obstacle
        persists across many attempts, back up and try an alternative.
      * Planner failure -> clear costmap and replan.
      * Controller failure -> stop and wait (may be a transient).
      * Sensor degraded -> degrade gracefully to remaining sensors.
      * Repeated failures -> abort rather than loop forever.
    """
    if ctx.recovery_count >= 5:
        return RecoveryDecision(
            RecoveryAction.ABORT,
            'recovery_count_exceeded',
            f'Aborting after {ctx.recovery_count} recovery attempts')

    if ctx.failure_type == FailureType.LOCALIZATION_UNCERTAIN:
        return RecoveryDecision(
            RecoveryAction.ROTATE_IN_PLACE,
            'localization_covariance_too_high',
            f'Localization covariance {ctx.localization_covariance:.3f} '
            'exceeds threshold; rotating to re-acquire')

    if ctx.failure_type == FailureType.BLOCKED_PATH:
        if ctx.obstacle_persistence_count >= 3:
            return RecoveryDecision(
                RecoveryAction.BACKUP,
                'obstacle_persistent',
                f'Obstacle persisted for {ctx.obstacle_persistence_count} '
                'attempts; backing up to find an alternative route')
        return RecoveryDecision(
            RecoveryAction.CLEAR_COSTMAP,
            'path_blocked',
            'Path blocked; clearing costmap and replanning')

    if ctx.failure_type == FailureType.PLANNER_FAILURE:
        return RecoveryDecision(
            RecoveryAction.REPLAN,
            'planner_failed',
            'Planner failed; clearing costmap and replanning')

    if ctx.failure_type == FailureType.CONTROLLER_FAILURE:
        return RecoveryDecision(
            RecoveryAction.WAIT,
            'controller_failure',
            'Controller failure; waiting for transient to clear')

    if ctx.failure_type == FailureType.SENSOR_DEGRADED:
        return RecoveryDecision(
            RecoveryAction.DEGRADE_SENSORS,
            'sensor_degraded',
            'Sensor degraded; continuing with remaining sensors')

    return RecoveryDecision(
        RecoveryAction.STOP,
        'unknown_failure',
        'Unknown failure; stopping for safety')
