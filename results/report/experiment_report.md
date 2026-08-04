# Experiment Report

- Experiments recorded: `6`
- Successful missions: `6`

## Results Summary

| Metric | Value |
| ------ | ----- |
| Navigation success rate | 100.0% |
| Average path length | 12.273 m |
| Average execution time | 27.250 s |
| Average minimal obstacle distance | 0.515 m |
| Average number of replans | 1.17 |
| Average planning time | 26.600 ms |

## Graphs

![Planner Comparison](../graphs\planner_comparison.png)

![Navigation Performance](../graphs\navigation_performance.png)

![Localization Accuracy](../graphs\localization_accuracy.png)

## Observations

- `office_astar_001` (office/A*): success=True, path=11.32 m, time=24.5 s, replans=1
- `office_dijkstra_001` (office/Dijkstra): success=True, path=11.35 m, time=24.8 s, replans=1
- `office_nav2_001` (office/Nav2 Default): success=True, path=11.45 m, time=25.1 s, replans=2
- `warehouse_astar_001` (warehouse/A*): success=True, path=12.80 m, time=28.2 s, replans=0
- `warehouse_dijkstra_001` (warehouse/Dijkstra): success=True, path=12.82 m, time=28.5 s, replans=0
