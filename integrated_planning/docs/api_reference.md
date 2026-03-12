# Integrated Planning System – API Reference

## Module: `integrated_planning`

Top-level package exposing the main public classes.

```python
from integrated_planning import HybridAStarPlanner3D, VehicleConfig, FlightMode, VoxelMap
```

---

## `integrated_planning.maps.voxel_map_3d`

### `class VoxelMap`

3D occupancy grid backed by a numpy `uint8` array.

```python
VoxelMap(size_x, size_y, size_z, resolution=1.0)
```

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `size_x` | `float` | Map X extent in metres |
| `size_y` | `float` | Map Y extent in metres |
| `size_z` | `float` | Map Z extent in metres |
| `resolution` | `float` | Voxel side length in metres (default 1.0) |

**Attributes:**

| Name | Type | Description |
|------|------|-------------|
| `nx, ny, nz` | `int` | Number of voxels per axis |
| `size_x, size_y, size_z` | `float` | Map dimensions |
| `resolution` | `float` | Voxel side length |

#### Methods

##### `world_to_voxel(x, y, z) → Tuple[int, int, int]`

Convert world coordinates to voxel indices.

##### `voxel_to_world(ix, iy, iz) → Tuple[float, float, float]`

Convert voxel indices to world coordinates (voxel centre).

##### `add_box_obstacle(x, y, z, size_x, size_y, size_z)`

Mark an axis-aligned rectangular box as occupied.
- `(x, y, z)` is the minimum corner.
- Extents are `(size_x, size_y, size_z)`.

##### `add_cylinder_obstacle(x, y, z, radius, height)`

Mark a vertical cylinder as occupied.
- `(x, y)` is the horizontal centre, `z` is the base altitude.

##### `add_sphere_obstacle(x, y, z, radius)`

Mark a sphere as occupied.

##### `clear_map()`

Reset all voxels to free.

##### `is_occupied_voxel(ix, iy, iz) → bool`

Return `True` if voxel `(ix, iy, iz)` is occupied.
Out-of-bounds indices return `True` (conservative).

##### `is_occupied_xyz(x, y, z) → bool`

Return `True` if world point `(x, y, z)` is occupied.

##### `aabb_collision(x, y, z, yaw, pitch, vehicle) → bool`

Check for collision using a conservative AABB sweep of the vehicle footprint.

**Parameters:**
- `x, y, z` – vehicle centre
- `yaw` – heading in radians
- `pitch` – pitch angle in radians
- `vehicle` – `VehicleConfig` instance

##### `update_from_pointcloud(points: np.ndarray)`

Mark voxels from an `(N, 3)` float array of XYZ points.
- Points outside the map are silently ignored.
- NaN / Inf points are filtered out.

##### `get_occupied_voxels() → List[Tuple[int, int, int]]`

Return list of all occupied voxel index tuples.

##### `get_occupancy_rate() → float`

Return fraction of occupied voxels in `[0.0, 1.0]`.

##### `save_map(filename: str)`

Save map to a compressed `.npz` file.

##### `classmethod load_map(filename: str) → VoxelMap`

Load a map from a `.npz` file created by `save_map`.

---

## `integrated_planning.planners.hybrid_astar_3d`

### `class FlightMode(IntEnum)`

```python
class FlightMode(IntEnum):
    HOVER = 1   # 26-direction holonomic movement
    CRUISE = 2  # Non-holonomic forward flight
```

---

### `class VehicleConfig`

Dataclass describing UAV dimensions and constraints.

```python
@dataclass
class VehicleConfig:
    length: float = 1.8
    width: float = 1.2
    height: float = 0.6
    rotor_diameter: float = 0.5
    min_altitude: float = 5.0
    max_altitude: float = 120.0
    min_turn_radius: float = 3.0
    hover_step: float = 1.0
    cruise_step: float = 2.0
    inflation: float = 0.35
```

---

### `class Node3D`

Internal search node (not typically used directly).

| Attribute | Type | Description |
|-----------|------|-------------|
| `x, y, z` | `float` | Position |
| `yaw, pitch` | `float` | Orientation (radians) |
| `parent` | `Optional[str]` | Parent node key |
| `g` | `float` | Cost-to-come |
| `h` | `float` | Heuristic cost-to-go |
| `f` | `float` | Total cost (g + h) |
| `mode` | `FlightMode` | Current flight mode |
| `key` | `str` | Discrete grid key (property) |

---

### `class HybridAStarPlanner3D(BasePlanner)`

3D Hybrid A* path planner.

```python
HybridAStarPlanner3D(voxel_map, vehicle=None, preferred_mode=FlightMode.HOVER)
```

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `voxel_map` | `VoxelMap` | Shared occupancy map |
| `vehicle` | `VehicleConfig` | Vehicle constraints (default `VehicleConfig()`) |
| `preferred_mode` | `FlightMode` | Initial flight mode |

#### Methods

##### `plan(start, goal, max_iter=80000) → Optional[List[Tuple]]`

Find a collision-free path from `start` to `goal`.

**Parameters:**
- `start` – `(x, y, z, yaw, pitch)` tuple
- `goal` – `(x, y, z, yaw, pitch)` tuple
- `max_iter` – Maximum A* expansions (use ~20 000 for emergency replanning)

**Returns:** Ordered list of `(x, y, z, yaw, pitch)` tuples, or `None`.

##### `validate_pose(pose) → bool`

Return `True` if pose altitude is within `[min_altitude, max_altitude]`.

##### `static smooth_path(path, iterations=150) → List[Tuple]`

Smooth a raw path using iterative gradient descent.
- Start and goal poses are preserved.
- Only XYZ coordinates are smoothed; yaw/pitch are taken from the original path.

##### `_get_motion_primitives(mode) → List[Tuple]`

Return motion primitives for the given `FlightMode`.
- HOVER: 26 unit steps scaled by `vehicle.hover_step`
- CRUISE: 9 forward primitives (straight, bank left/right × climb/level/dive)

---

## `integrated_planning.planners.base_planner`

### `class BasePlanner(ABC)`

Abstract interface for all planners.

#### Abstract Methods

- `plan(start, goal) → Optional[List[Tuple]]`
- `validate_pose(pose) → bool`

#### Utility Methods

- `static path_length(path) → float` – Total 3D arc-length
- `static validate_path(path, max_segment_length=10.0) → bool` – Sanity check
- `static euclidean_distance_3d(a, b) → float` – 3D distance

---

## `integrated_planning.ros_integration.obstacle_map_bridge`

### `class ObstacleMapBridge(Node)`

ROS2 node that bridges obstacle detection data to a `VoxelMap`.

```python
# Launched via ROS2
ros2 run integrated_planning obstacle_map_bridge
```

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `map_size_x` | `float` | 100.0 | Map X extent |
| `map_size_y` | `float` | 100.0 | Map Y extent |
| `map_size_z` | `float` | 50.0 | Map Z extent |
| `map_resolution` | `float` | 1.0 | Voxel resolution |
| `obstacle_inflation` | `float` | 0.5 | Sphere obstacle radius |
| `update_rate` | `float` | 10.0 | Visualization publish rate (Hz) |

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar/points` | `sensor_msgs/PointCloud2` | LiDAR scan |
| `/obstacle_position` | `geometry_msgs/Vector3Stamped` | Obstacle position |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/voxel_map_viz` | `visualization_msgs/MarkerArray` | Occupied voxels for RViz2 |

#### Key Methods

- `pointcloud_callback(msg)` – Process PointCloud2 → update VoxelMap
- `obstacle_position_callback(msg)` – Add sphere at obstacle position
- `publish_map_visualization()` – Publish MarkerArray
- `static pointcloud2_to_array(msg) → np.ndarray` – Convert ROS msg to `(N, 3)` array

---

## `integrated_planning.ros_integration.integrated_planner_node`

### `class IntegratedPlannerNode(Node)`

Main integration node connecting obstacle detection with path planning.

```python
# Launched via ROS2
ros2 run integrated_planning integrated_planner_node
```

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `map_size_x/y/z` | `float` | 100/100/50 | Map dimensions |
| `map_resolution` | `float` | 1.0 | Voxel resolution |
| `vehicle_length/width/height` | `float` | 1.8/1.2/0.6 | Vehicle size |
| `vehicle_inflation` | `float` | 0.35 | Safety margin |
| `vehicle_min/max_altitude` | `float` | 5.0/120.0 | Altitude limits |
| `max_iterations` | `int` | 80 000 | Normal A* budget |
| `preferred_mode` | `str` | `"hover"` | Flight mode |
| `replan_on_obstacle` | `bool` | `true` | Auto-replan on detection |
| `emergency_replan_distance` | `float` | 3.0 | Emergency trigger (m) |

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/obstacle_detected` | `std_msgs/Bool` | Obstacle flag |
| `/obstacle_distance` | `std_msgs/Float32` | Distance (m) |
| `/avoidance_zone` | `std_msgs/String` | Zone name |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal |
| `/current_pose` | `geometry_msgs/PoseStamped` | Current UAV pose |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/planned_path` | `nav_msgs/Path` | Waypoints for PX4 |
| `/planner_status` | `std_msgs/String` | Status messages |

#### Key Methods

| Method | Description |
|--------|-------------|
| `replan()` | Normal replanning (80k iterations) |
| `emergency_replan()` | Fast replanning (20k iterations) |
| `emergency_hover()` | Publish ascend + hover path |
| `publish_path(waypoints)` | Convert and publish `nav_msgs/Path` |
| `publish_status(status)` | Publish status string |
| `static quaternion_to_yaw(quat)` | Extract yaw from quaternion |
| `static yaw_to_quaternion(yaw)` | Build quaternion from yaw |

---

## Cost Weights

Defined as module-level constants in `hybrid_astar_3d.py`:

| Constant | Value | Purpose |
|----------|-------|---------|
| `W_STEER` | 1.2 | Penalise lateral steering |
| `W_PITCH` | 1.0 | Penalise pitch angle |
| `W_PITCH_CHANGE` | 2.0 | Penalise rapid pitch changes (comfort) |
| `W_MODE_SWITCH` | 3.0 | Penalise mode transitions |
| `W_ALTITUDE_FLOOR` | 50.0 | Heavy penalty near minimum altitude |
