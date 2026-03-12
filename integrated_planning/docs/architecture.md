# Integrated Planning System – Architecture

## Overview

The integrated planning system bridges the existing **obstacle detection**
pipeline with **3D autonomous path planning** for the Pegasus UAV.

```
┌─────────────────────────────────────────────────────────────┐
│                    Pegasus UAV System                        │
│                                                             │
│  ┌──────────┐    ┌─────────────────┐    ┌───────────────┐  │
│  │  LiDAR   │───►│ obstacle_       │───►│ obstacle_map_ │  │
│  │ VLP-16   │    │ detector.py     │    │ bridge.py     │  │
│  └──────────┘    └────────┬────────┘    └───────┬───────┘  │
│                           │                     │           │
│               /obstacle_detected          Updates           │
│               /obstacle_distance          VoxelMap          │
│               /avoidance_zone                │              │
│                           │                  │              │
│                           ▼                  ▼              │
│                  ┌─────────────────────────────────┐        │
│                  │   integrated_planner_node.py    │        │
│                  │                                 │        │
│                  │  ┌──────────────────────────┐  │        │
│                  │  │  HybridAStarPlanner3D    │  │        │
│                  │  │  (5-DOF, HOVER + CRUISE) │  │        │
│                  │  └──────────────────────────┘  │        │
│                  │  ┌──────────────────────────┐  │        │
│                  │  │       VoxelMap3D          │  │        │
│                  │  │  (3D occupancy grid)      │  │        │
│                  │  └──────────────────────────┘  │        │
│                  └────────────────┬────────────────┘        │
│                                   │                         │
│                            /planned_path                    │
│                                   │                         │
│                                   ▼                         │
│                          ┌────────────────┐                 │
│                          │  PX4 Autopilot │                 │
│                          └────────────────┘                 │
└─────────────────────────────────────────────────────────────┘
```

## Components

### 1. Obstacle Detector (`obstacle_detection` package)

Existing node that processes LiDAR point clouds and publishes:

| Topic | Type | Description |
|-------|------|-------------|
| `/obstacle_detected` | `std_msgs/Bool` | Obstacle presence flag |
| `/obstacle_distance` | `std_msgs/Float32` | Nearest obstacle distance (m) |
| `/avoidance_zone` | `std_msgs/String` | Zone name (CRITICAL/DANGER/…) |
| `/lidar/points` | `sensor_msgs/PointCloud2` | Raw LiDAR scan |
| `/obstacle_position` | `geometry_msgs/Vector3Stamped` | Obstacle 3D position |

### 2. ObstacleMapBridge (`integrated_planning` package)

Converts obstacle data into voxel occupancy:

```
/lidar/points ──────────────► pointcloud_callback()
                                  │
                                  ▼ pointcloud2_to_array()
                                  │
                                  ▼ VoxelMap.update_from_pointcloud()

/obstacle_position ─────────► obstacle_position_callback()
                                  │
                                  ▼ VoxelMap.add_sphere_obstacle()

Timer (10 Hz) ──────────────► publish_map_visualization()
                                  │
                                  ▼ /voxel_map_viz (MarkerArray)
```

### 3. VoxelMap (`integrated_planning.maps`)

Dense 3D occupancy grid:

```
VoxelMap(size_x=100, size_y=100, size_z=50, resolution=1.0)
    │
    ├── _grid: np.ndarray[uint8, (nx, ny, nz)]
    │
    ├── add_box_obstacle(x, y, z, sx, sy, sz)
    ├── add_cylinder_obstacle(x, y, z, radius, height)
    ├── add_sphere_obstacle(x, y, z, radius)
    │
    ├── is_occupied_xyz(x, y, z) → bool
    ├── aabb_collision(x, y, z, yaw, pitch, vehicle) → bool
    │
    └── update_from_pointcloud(points: ndarray)
```

### 4. HybridAStarPlanner3D (`integrated_planning.planners`)

5-DOF Hybrid A* with dual flight modes:

```
State: (x, y, z, yaw, pitch)

HOVER mode:
    26-direction holonomic movement
    step = vehicle.hover_step (default 1.0 m)

CRUISE mode:
    Non-holonomic forward flight
    step = vehicle.cruise_step (default 2.0 m)
    Steer angles: ±arcsin(step / 2r)

Cost function:
    f(n) = g(n) + h(n)
    g(n) = Σ(step_dist + W_STEER·|Δyaw| + W_PITCH·|pitch|
              + W_PITCH_CHANGE·|Δpitch| + W_MODE_SWITCH·[mode change]
              + W_ALTITUDE_FLOOR·[z < min_alt+1])
    h(n) = Euclidean3D(n, goal)
```

### 5. IntegratedPlannerNode (`integrated_planning.ros_integration`)

Central coordination node:

```
Subscriptions → State updates → Zone-based action selection → Plan/Replan

/obstacle_detected ──► obstacle_detected flag
/obstacle_distance ──► obstacle_distance (m)
                           │
                           ▼ if dist ≤ emergency_replan_distance
                           └──► emergency_replan() [20k iterations]
/avoidance_zone ────► zone string
                           │
                           ├── CRITICAL → emergency_hover()
                           ├── DANGER   → emergency_replan()
                           └── WARNING  → replan() [80k iterations]
/goal_pose ─────────► goal_pose tuple → replan()
/current_pose ──────► current_pose tuple

Timer (1 Hz) ───────► replan_timer_callback()
                           └── if obstacle_detected → replan()
```

## Data Flows

### Normal Operation

```
1. UAV receives /goal_pose
2. IntegratedPlannerNode stores goal, calls replan()
3. HybridAStarPlanner3D.plan(current_pose, goal) runs
4. Smoothed path published to /planned_path (nav_msgs/Path)
5. PX4 follows waypoints
```

### Obstacle Avoidance

```
1. LiDAR detects obstacle
2. obstacle_detector publishes /obstacle_detected=True
3. ObstacleMapBridge marks voxels in VoxelMap
4. IntegratedPlannerNode receives /obstacle_detected
5. Calls replan() with updated map
6. New path avoids obstacle, published to /planned_path
```

### Emergency Avoidance

```
1. Obstacle enters DANGER/CRITICAL zone
2. /avoidance_zone = "CRITICAL"
3. IntegratedPlannerNode.emergency_hover() called
4. UAV ascends 5 m and hovers
5. Replanning attempted with reduced iteration budget
```

## Configuration

All parameters are ROS2-configurable via `config/planner_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_size_x/y/z` | 100/100/50 m | Voxel map dimensions |
| `map_resolution` | 1.0 m | Voxel side length |
| `vehicle_min_altitude` | 5.0 m | FAA minimum AGL |
| `vehicle_max_altitude` | 120.0 m | FAA Part 107 ceiling |
| `max_iterations` | 80 000 | Normal A* budget |
| `emergency_replan_distance` | 3.0 m | Emergency trigger |
| `preferred_mode` | `"hover"` | Default flight mode |

## Dependencies

### Python (≥ 3.8)
- `numpy ≥ 1.21.0`
- `scipy ≥ 1.7.0` (optional, advanced features)

### ROS2 (Humble / Iron / Jazzy)
- `rclpy`
- `std_msgs`, `sensor_msgs`, `nav_msgs`
- `geometry_msgs`, `visualization_msgs`

## Performance

| Metric | Value |
|--------|-------|
| Normal planning time | < 0.5 s (80k iterations) |
| Emergency planning time | < 0.1 s (20k iterations) |
| Map update rate | 10 Hz |
| Replan check period | 1 Hz |
