# Quantitative Benchmarking Framework

## Benchmarking Suite (`robot_benchmarking`)

### Metrics Collected
1. **Navigation Metrics**: Mission success rate, execution time (s), path length (m), minimum obstacle clearance (m), replan count, recovery count.
2. **Planner Metrics**: Planning time (ms), total nodes expanded, final path length (m).
3. **Localization Metrics**: Ground truth vs estimated trajectory error (m).

## Automated Benchmark Pipeline
```bash
./scripts/run_full_benchmark.sh
```

This single command triggers:
1. `run_experiment`: Runs navigation trials across Office, Warehouse, and Challenging environments.
2. `generate_report`: Compiles metrics JSON into PNG graphs and `experiment_report.md`.
3. `generate_gifs`: Programmatically renders 2D animation GIFs (`exploration_demo.gif`, `navigation_replanning_demo.gif`, `semantic_navigation_demo.gif`).
