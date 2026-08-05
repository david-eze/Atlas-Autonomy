# Testing Strategy & Automated Verification

## Test Hierarchy

```
         [End-to-End Simulation Integration]
         [ROS 2 Action & Topic Integration ]
         [Unit Tests: Kinematics, TF2, Planners]
```

## Unit Test Coverage (`tests/`)

| Module                | Test File                 | Target Functionality                                              |
| --------------------- | ------------------------- | ----------------------------------------------------------------- |
| Kinematics            | `test_kinematics.py`      | Differential-drive forward and inverse velocity conversion        |
| Coordinate Transforms | `test_tf2_transforms.py`  | 2D rigid-body transformation matrix calculations                  |
| Planners              | `test_planners.py`        | A* and Dijkstra grid-search correctness and node expansion counts |
| Frontier Exploration  | `test_frontiers.py`       | Frontier cell detection and multi-criteria scoring                |
| Safety Layer          | `test_safety_monitor.py`  | Obstacle distance threshold checks and velocity command override  |
| Semantic Navigation   | `test_semantic_nav.py`    | Command parsing and map coordinate lookup                         |
| TF Diagnostics        | `test_tf2_diagnostics.py` | TF edge presence and tree connectivity checks                     |

## Executing Test Suite

```bash
py -m unittest discover -s tests -p "test_*.py"
```

Or using the ROS 2 colcon test tools:

```bash
colcon test --event-handlers console_direct+
```
