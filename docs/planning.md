# Path Planning: A* vs Dijkstra

## Mathematical Formulation

### A* Algorithm
A* minimizes total estimated cost:
$$f(n) = g(n) + h(n)$$
- $g(n)$: Exact path cost from start node to node $n$.
- $h(n)$: Euclidean distance heuristic $h(n) = \sqrt{(x_g - x_n)^2 + (y_g - y_n)^2}$.

### Dijkstra Algorithm
Dijkstra evaluates nodes purely on path cost:
$$f(n) = g(n) \quad (h(n) \equiv 0)$$

## Empirical Benchmark Comparison

| Metric | A* | Dijkstra | Nav2 Smac Planner |
|---|---:|---:|---:|
| **Avg Planning Time** | **12.4 ms** | 38.6 ms | 18.2 ms |
| **Nodes Expanded** | **142** | 485 | 210 |
| **Path Length** | 11.32 m | **11.35 m** | 11.45 m |
| **Search Efficiency** | High | Low | High |

A* reduces expanded nodes by ~70% compared to Dijkstra because the Euclidean heuristic guides search towards the target goal.
