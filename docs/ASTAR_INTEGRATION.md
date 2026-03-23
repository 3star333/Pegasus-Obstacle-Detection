# A* Path Planning Integration

## Overview

This document describes the integration of the A\* pathfinding algorithm from
[Pegasus-Disaster-Response/Ros-workspace](https://github.com/Pegasus-Disaster-Response/Ros-workspace)
into the Pegasus Obstacle Detection stack, replacing the previous RRT\* planner.

**Branch:** `ros-workspace-branch`
**Source commit:** c752b073 (March 4, 2026 — "A\* implementation")
**Source path:** `src/pegasus_ros/pegasus_autonomy/global_planner_node.py`

---

## Rationale: A\* vs RRT\*

| Criterion | RRT\* (previous) | A\* (current) |
|-----------|-----------------|---------------|
| Path optimality | ~105% of optimal | Optimal (w=1.0) |
| Planning time | 100–300 ms | 10–100 ms |
| Determinism | Non-deterministic | Deterministic |
| Costmap integration | Custom occupancy check | Direct `OccupancyGrid` |
| Grid connectivity | Continuous state-space | 8-connected grid |
| Replanning | Event-driven | Periodic + event-driven |
| Source | Local implementation | Pegasus-Disaster-Response/Ros-workspace |

The A\* planner is better suited to the existing `costmap_node` which already
publishes a `nav_msgs/OccupancyGrid` — A\* operates directly on this grid
without any conversion overhead.

---

## Algorithm Details

The core A\* implementation (`astar_planner/astar_algorithm.py`) is a
**weighted A\*** on a 2D occupancy grid:

- **Grid:** `nav_msgs/OccupancyGrid` from `costmap_node` (60×60 m, 0.5 m/cell)
- **Connectivity:** 8-connected (diagonal movement enabled by default)
- **Heuristic:** Octile distance — admissible for 8-connected grids
- **Heuristic weight:** Configurable (`heuristic_weight`, default 1.0 = optimal)
- **Cost function:**
  - Free cell (cost=0): move cost = 1.0 (cardinal) or √2 (diagonal)
  - Inflated cell (cost 1–89): move cost scaled by `cost_penalty_factor`
  - Unknown cell (cost=-1): move cost × 1.5 (mild penalty)
  - Lethal cell (cost≥70): impassable
- **Iteration cap:** `max_iterations` (default 10,000) prevents infinite loops
  in disconnected maps

---

## Package Structure

```
astar_planner/
├── astar_planner/
│   ├── __init__.py
│   ├── astar_algorithm.py     # Core A* search (from Ros-workspace)
│   ├── astar_node.py          # ROS 2 node wrapping the algorithm
│   └── costmap_interface.py   # OccupancyGrid → numpy bridge
├── config/
│   └── astar_params.yaml      # Tunable parameters
├── launch/
│   └── astar_planner.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Topic Interfaces

### Subscribed (preserved from RRT\*)

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/costmap/grid` | `nav_msgs/OccupancyGrid` | `costmap_node` | 2D costmap for planning |
| `/obstacle_detected` | `std_msgs/Bool` | `obstacle_detector` | Triggers replan |
| `/obstacle_distance` | `std_msgs/Float32` | `obstacle_detector` | Closest obstacle distance |
| `/avoidance_command` | `std_msgs/String` | `obstacle_detector` | Zone command → replan |
| `/obstacle_velocity` | `geometry_msgs/Vector3Stamped` | `obstacle_detector` | Kalman velocity |
| `/vehicle/pose` | `geometry_msgs/PoseStamped` | UAV | Current position |
| `/planning/goal` | `geometry_msgs/PoseStamped` | Mission planner | Navigation goal |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/planned_path` | `nav_msgs/Path` | A\*-generated global path |
| `/astar_planner/status` | `std_msgs/String` | JSON status string |

---

## Configuration

Key parameters in `astar_planner/config/astar_params.yaml`:

```yaml
astar_planner:
  ros__parameters:
    heuristic_weight: 1.0         # 1.0=optimal, 1.2=slightly greedy
    max_iterations: 10000         # raise for larger environments
    planning_frequency: 2.0       # Hz
    lethal_cost_threshold: 70     # must match costmap_lethal_cost
    cost_penalty_factor: 15.0     # higher → paths avoid inflated zones
    goal_tolerance_m: 3.0         # metres
    safety_margin: 3.0            # metres (15ft wingspan UAV)
```

The `lethal_cost_threshold` (default 70) must match the costmap's lethal cost setting.
This value aligns with the `costmap_lethal_cost` parameter in
`costmap/config/costmap_params.yaml` to ensure consistent obstacle treatment.

---

## Preserved Capabilities

The following features from the original system are **fully preserved**:

- ✅ `obstacle_detection/` — LiDAR point cloud processing
- ✅ `KalmanObstacleTracker` — per-obstacle Kalman filter (constant-velocity 3D model)
- ✅ Zone-based avoidance (CRITICAL / DANGER / WARNING / CAUTION / SAFE)
- ✅ `/avoidance_command` topic — zone commands still published
- ✅ `/obstacle_velocity` topic — Kalman velocity still published
- ✅ `costmap/` — OccupancyGrid costmap with zone-based inflation
- ✅ `integrated_planning/` — 3D Hybrid A\* for local planning (unchanged)

---

## Migration Guide from RRT\*

If you need to revert to RRT\*:

1. Copy `deprecated/rrt_star_planner/` back to the repository root:
   ```bash
   cp -r deprecated/rrt_star_planner ./
   ```

2. Update `costmap/launch/full_stack.launch.py` — replace `astar_planner` node
   with `rrt_star_planner` node.

3. Update `launch/full_system.launch.py` — replace `astar_planner` node.

4. The `/planned_path` topic output format is identical (`nav_msgs/Path`),
   so downstream consumers require no changes.

---

## Testing

Run the pure-Python A\* unit tests (no ROS required):

```bash
cd astar_planner
python -m pytest tests/ -v
```

Full system test (requires ROS 2 + Gazebo):

```bash
ros2 launch launch/full_system.launch.py
ros2 topic echo /planned_path
ros2 topic echo /astar_planner/status
```
