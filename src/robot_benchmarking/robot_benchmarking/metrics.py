"""
The ExperimentRecorder subscribes to odometry, the goal, and safety
topics, then writes a structured mission log with provenance metadata.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass
class ExperimentConfig:
    experiment_id: str
    environment: str
    planner: str
    random_seed: int
    goal_x: float
    goal_y: float
    start_x: float
    start_y: float


@dataclass
class ExperimentResults:
    config: ExperimentConfig
    success: bool = False
    path_length: float = 0.0
    execution_time: float = 0.0
    planning_time_ms: float = 0.0
    avg_velocity: float = 0.0
    min_obstacle_distance: float = 0.0
    replans: int = 0
    recoveries: int = 0
    localisation_error: float = 0.0
    failure_reason: str = ''
    trajectory: List[Tuple[float, float, float]] = field(default_factory=list)
    ground_truth: List[Tuple[float, float, float]] = field(default_factory=list)
    map_coverage: float = 0.0
    exploration_time: float = 0.0
    inference_latency_ms: float = 0.0
    nodes_expanded: int = 0
    timestamp: str = field(default_factory=lambda: datetime.utcnow().isoformat())

    def to_dict(self) -> Dict:
        return {
            'experiment_id': self.config.experiment_id,
            'environment': self.config.environment,
            'planner': self.config.planner,
            'random_seed': self.config.random_seed,
            'goal_x': self.config.goal_x,
            'goal_y': self.config.goal_y,
            'start_x': self.config.start_x,
            'start_y': self.config.start_y,
            'success': self.success,
            'path_length': round(self.path_length, 3),
            'execution_time': round(self.execution_time, 3),
            'planning_time_ms': round(self.planning_time_ms, 3),
            'avg_velocity': round(self.avg_velocity, 3),
            'min_obstacle_distance': round(self.min_obstacle_distance, 3),
            'replans': self.replans,
            'recoveries': self.recoveries,
            'localisation_error': round(self.localisation_error, 4),
            'failure_reason': self.failure_reason,
            'map_coverage': round(self.map_coverage, 4),
            'exploration_time': round(self.exploration_time, 3),
            'inference_latency_ms': round(self.inference_latency_ms, 2),
            'nodes_expanded': self.nodes_expanded,
            'timestamp': self.timestamp,
        }

    def compute_trajectory_stats(self) -> None:
        """Finalise metrics computed from the recorded trajectory."""
        if not self.trajectory:
            return
        distances = 0.0
        prev = self.trajectory[0]
        for x, y, t in self.trajectory[1:]:
            distances += math.hypot(x - prev[0], y - prev[1])
            prev = (x, y, t)
        self.path_length = distances
        self.execution_time = max(0.001, self.trajectory[-1][2] - self.trajectory[0][2])
        self.avg_velocity = distances / self.execution_time


def write_csv(results: List[ExperimentResults], path: Path) -> None:
    if not results:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].to_dict().keys()))
        writer.writeheader()
        for r in results:
            writer.writerow(r.to_dict())


def write_json(results: List[ExperimentResults], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump([r.to_dict() for r in results], f, indent=2)


def load_results(path: Path) -> List[Dict]:
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)
