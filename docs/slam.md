# Mapping & SLAM Architecture

## Overview

Mapping is handled by **SLAM Toolbox**, configured to run asynchronous pose-graph optimization. The robot starts without a pre-built map and builds a 2D occupancy grid while exploring an unknown environment. The map uses a resolution of $0.05\text{ m/cell}$.

## Pose Graph Optimization

* **Nodes**: Robot poses are added when the robot has moved at least $\Delta d \ge 0.2\text{ m}$ or rotated by $\Delta \theta \ge 0.1\text{ rad}$.
* **Edges**: Laser scan matching constraints are added between consecutive poses, along with additional constraints when loop closures are detected.
* **Loop Closure**: New scans are matched against previously recorded nodes, with the resulting constraints optimized using the Ceres solver.

## Map File Assets

Occupancy grids are saved using the standard ROS 2 map format:

* `.yaml`: Stores map metadata such as resolution, origin, and occupancy thresholds.
* `.pgm`: Greyscale occupancy image ($0 = \text{free}, 100 = \text{occupied}, -1 = \text{unknown}$).
