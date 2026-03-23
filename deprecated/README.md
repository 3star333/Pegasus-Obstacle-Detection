# Deprecated: RRT* Path Planner

> ⚠️ **DEPRECATED** — This package has been superseded by the `astar_planner` package.

## Status

The `rrt_star_planner` package has been moved here as part of the `ros-workspace-branch`
integration. It is **no longer used** in the active system.

## Replacement

The A* path planner (`astar_planner/`) replaces this package. See the main
[README.md](../../README.md) and [docs/ASTAR_INTEGRATION.md](../../docs/ASTAR_INTEGRATION.md)
for details.

## Why it was replaced

| Criterion | RRT* | A* |
|-----------|------|----|
| Path optimality | ~105% of optimal | Optimal (w=1.0) |
| Planning time | 100–300 ms | 10–100 ms |
| Determinism | Non-deterministic | Deterministic |
| Costmap integration | Custom | Direct OccupancyGrid |
| Source | Local implementation | Pegasus-Disaster-Response/Ros-workspace |

## Preserved

The code is preserved here for reference and git history. To restore it:

```bash
cp -r deprecated/rrt_star_planner ./
```

Then update `costmap/launch/full_stack.launch.py` and `launch/full_system.launch.py`
to reference `rrt_star_planner` instead of `astar_planner`.
