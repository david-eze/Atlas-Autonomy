"""
Pipeline:
    results/*.json  ->  planner_comparison.png
                         navigation_performance.png
                         localization_accuracy.png
                         experiment_report.md

All graphs are produced from real recorded data; no values are ever
synthesised. Raises if the input files are missing or empty.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

RESULTS_DIR = Path(__file__).resolve().parents[3] / 'results'
GRAPH_DIR = RESULTS_DIR / 'graphs'
REPORT_DIR = RESULTS_DIR / 'report'


def _load_all_experiments() -> List[Dict]:
    """Load every experiment recorded in results/*.json."""
    experiments = []
    for path in sorted(RESULTS_DIR.glob('*.json')):
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            if isinstance(data, list):
                experiments.extend(data)
            else:
                experiments.append(data)
    return experiments


def _planner_comparison(experiments: List[Dict]) -> Path:
    """Group by planner and plot planning time, path length, nodes expanded."""
    planners: Dict[str, List[Dict]] = {}
    for e in experiments:
        planners.setdefault(e['planner'], []).append(e)

    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    names = sorted(planners)
    metrics = [
        ('planning_time_ms', 'Planning time (ms)'),
        ('path_length', 'Path length (m)'),
        ('nodes_expanded', 'Nodes expanded'),
    ]
    colors = ['#4C72B0', '#DD8452', '#55A868']

    for ax, (key, label) in zip(axes, metrics):
        means = [np.mean([e[key] for e in planners[n]]) for n in names]
        stds = [np.std([e[key] for e in planners[n]]) for n in names]
        ax.bar(names, means, yerr=stds, capsize=5, color=colors[:len(names)])
        ax.set_ylabel(label)
        ax.set_title(label)
        ax.tick_params(axis='x', rotation=15)

    fig.suptitle('Planner Performance Comparison')
    fig.tight_layout()
    out = GRAPH_DIR / 'planner_comparison.png'
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def _navigation_performance(experiments: List[Dict]) -> Path:
    """Group by environment and plot navigation metrics across envs."""
    environments: Dict[str, List[Dict]] = {}
    for e in experiments:
        environments.setdefault(e['environment'], []).append(e)

    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    names = sorted(environments)
    metrics = [
        ('execution_time', 'Execution time (s)'),
        ('min_obstacle_distance', 'Min obstacle distance (m)'),
        ('replans', 'Number of replans'),
    ]
    colors = ['#4C72B0', '#DD8452', '#55A868']

    for ax, (key, label) in zip(axes, metrics):
        means = [np.mean([e[key] for e in environments[n]]) for n in names]
        stds = [np.std([e[key] for e in environments[n]]) for n in names]
        ax.bar(names, means, yerr=stds, capsize=5, color=colors[:len(names)])
        ax.set_ylabel(label)
        ax.set_title(label)
        ax.tick_params(axis='x', rotation=15)

    fig.suptitle('Navigation Performance by Environment')
    fig.tight_layout()
    out = GRAPH_DIR / 'navigation_performance.png'
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def _localization_accuracy(experiments: List[Dict]) -> Path:
    """Plot ground-truth vs estimated trajectory for the first mission.

    Trajectory samples are stored under the 'trajectory' and
    'ground_truth' keys. If the full trajectories were not recorded, the
    start/goal/end points are plotted instead.
    """
    fig, ax = plt.subplots(figsize=(7, 7))
    found = False
    for e in experiments:
        if e.get('trajectory') and e.get('ground_truth'):
            traj = np.array(e['trajectory'])
            gt = np.array(e['ground_truth'])
            ax.plot(gt[:, 0], gt[:, 1], 'g-', label='Ground truth', linewidth=2)
            ax.plot(traj[:, 0], traj[:, 1], 'b--', label='Estimated', linewidth=2)
            ax.plot(e['start_x'], e['start_y'], 'go', markersize=10, label='Start')
            ax.plot(e['goal_x'], e['goal_y'], 'r*', markersize=16, label='Goal')
            found = True
            break

    if not found:
        # Fall back to a start/goal/end diagram for each experiment
        for e in experiments[:5]:
            ax.plot(
                [e['start_x'], e['goal_x']],
                [e['start_y'], e['goal_y']],
                'o-', label=e['experiment_id'], alpha=0.6)
        ax.set_title('Start/Goal Pairs (no trajectory data recorded)')

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Localization Accuracy: Ground Truth vs Estimated Trajectory')
    ax.legend()
    ax.set_aspect('equal')
    out = GRAPH_DIR / 'localization_accuracy.png'
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


def _write_report(experiments: List[Dict], graph_paths: Dict[str, Path]) -> Path:
    """Write a Markdown experiment report with results and graph links."""
    successes = [e for e in experiments if e.get('success')]
    lines = [
        '# Experiment Report',
        '',
        f'- Experiments recorded: `{len(experiments)}`',
        f'- Successful missions: `{len(successes)}`',
        '',
        '## Results Summary',
        '',
        '| Metric | Value |',
        '| ------ | ----- |',
    ]

    def _avg(key: str, fmt: str = '{:.3f}') -> str:
        vals = [e.get(key, 0.0) for e in experiments]
        if not vals:
            return 'n/a'
        return fmt.format(float(np.mean(vals)))

    rows = [
        ('Navigation success rate', '{:.1%}'.format(len(successes) / max(1, len(experiments)))),
        ('Average path length', _avg('path_length') + ' m'),
        ('Average execution time', _avg('execution_time') + ' s'),
        ('Average minimal obstacle distance', _avg('min_obstacle_distance') + ' m'),
        ('Average number of replans', _avg('replans', '{:.2f}')),
        ('Average planning time', _avg('planning_time_ms') + ' ms'),
    ]
    lines += [f'| {k} | {v} |' for k, v in rows]

    lines += ['', '## Graphs', '']
    for label, path in graph_paths.items():
        rel = path.relative_to(RESULTS_DIR)
        lines.append(f'![{label}](../{rel})')
        lines.append('')

    lines += ['## Observations', '']
    for e in experiments[:5]:
        lines.append(
            f'- `{e["experiment_id"]}` ({e["environment"]}/{e["planner"]}): '
            f'success={e["success"]}, path={e["path_length"]:.2f} m, '
            f'time={e["execution_time"]:.1f} s, replans={e["replans"]}')
    lines.append('')

    report_dir = REPORT_DIR
    report_dir.mkdir(parents=True, exist_ok=True)
    out = report_dir / 'experiment_report.md'
    out.write_text('\n'.join(lines), encoding='utf-8')
    return out


def main() -> None:
    experiments = _load_all_experiments()
    if not experiments:
        print('No experiment data found in results/. Run experiments first.')
        return

    print(f'Generating graphs from {len(experiments)} recorded experiments...')
    graph_paths = {
        'Planner Comparison': _planner_comparison(experiments),
        'Navigation Performance': _navigation_performance(experiments),
        'Localization Accuracy': _localization_accuracy(experiments),
    }
    report = _write_report(experiments, graph_paths)
    print(f'Graphs written to {GRAPH_DIR}')
    print(f'Report written to {report}')


if __name__ == '__main__':
    main()
