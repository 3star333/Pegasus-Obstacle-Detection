# PX4 ROS 2 Obstacle Detection & A* Path Planning

> ⚠️ **Branch: `ros-workspace-branch`** — This branch integrates the A* pathfinding
> algorithm from [Pegasus-Disaster-Response/Ros-workspace](https://github.com/Pegasus-Disaster-Response/Ros-workspace)
> (commit c752b073, March 4 2026), replacing the previous RRT* planner.  All obstacle
> detection, Kalman tracking, and zone-based avoidance capabilities are fully preserved.
>
> The RRT* planner code is archived at [`deprecated/rrt_star_planner/`](deprecated/rrt_star_planner/).

A modular ROS 2 package for real-time LiDAR-based obstacle detection and **A\* path planning** designed for large fixed-wing/VTOL UAVs (15ft wingspan). This implementation is simulation-tested and hardware-ready.

##  Mission Goal

**Detect obstacles using LiDAR and generate flyable collision-free paths for large fixed-wing/VTOL aircraft with kinodynamic constraints.**

##  Architecture Overview

### **A\* Global Planner + Zone-Based Avoidance** (Current)

The system now uses weighted A\* (sourced from Pegasus-Disaster-Response/Ros-workspace)
for global path planning, combined with the existing Kalman-tracking zone-based avoidance
for local reactive control.

**Key Benefits:**
-  **Deterministic** — same inputs always produce the same path
-  **Optimal paths** (w=1.0) or configurable speed/quality trade-off (w>1.0)
-  **Direct costmap integration** — reads `nav_msgs/OccupancyGrid` natively
-  **Fast** — 10–100 ms for typical environments (vs 100–300 ms for RRT*)
-  **Preserves all obstacle detection** — Kalman tracking and zone commands still active

**System flow:**
```
LiDAR → obstacle_detector → /avoidance_command, /obstacle_velocity
                          → /costmap/update_trigger → costmap_node → /costmap/grid
                          → astar_planner           → /planned_path
```

### ~~RRT* with Kinodynamic Constraints~~ (Deprecated)

The previous RRT* planner has been moved to `deprecated/rrt_star_planner/`.
See [docs/ASTAR_INTEGRATION.md](docs/ASTAR_INTEGRATION.md) for the migration rationale.

##  Features

### Obstacle Detection
- Real-time PointCloud2 processing
- Configurable detection parameters (distance, FOV, height)
- Obstacle inflation radius for structural clearance (wing tips, props)
- Configured for 15ft wingspan UAV with 80m danger distance

### Path Planning (A\*)
- Weighted A\* on 2D OccupancyGrid (sourced from Pegasus-Disaster-Response/Ros-workspace)
- 8-connected grid with diagonal movement
- Configurable heuristic weight (1.0 = optimal, >1.0 = faster/greedy)
- Costmap-driven obstacle avoidance with configurable lethal threshold
- 2 Hz replanning, triggered by obstacle detection or path deviation
- Compatible with all existing `/planned_path` consumers

##  Project Structure

```
peg/
├── obstacle_detection/          # Obstacle detection ROS 2 package
│   ├── obstacle_detection/
│   │   ├── __init__.py
│   │   └── obstacle_detector.py # Core detection node
│   ├── config/
│   │   └── params.yaml          # Detection parameters
│   ├── launch/
│   │   └── detection.launch.py  # Launch file
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
├── astar_planner/               # A* path planner ROS 2 package (replaces RRT*)
│   ├── astar_planner/
│   │   ├── __init__.py
│   │   ├── astar_node.py        # Main A* ROS 2 node
│   │   ├── astar_algorithm.py   # Core A* implementation (from Ros-workspace)
│   │   └── costmap_interface.py # OccupancyGrid → numpy bridge
│   ├── config/
│   │   └── astar_params.yaml    # A* configuration
│   ├── launch/
│   │   └── astar_planner.launch.py
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
├── costmap/                     # Costmap ROS 2 package
├── integrated_planning/         # Integrated planning module
├── deprecated/
│   ├── README.md                # Deprecation notice
│   └── rrt_star_planner/        # Archived RRT* planner (no longer active)
├── launch/
│   └── full_system.launch.py    # Full stack launch file
├── config/
│   └── integrated_system.yaml   # Unified system configuration
├── context/
│   └── Autonomy Trade Study.xlsx
├── docs/
│   ├── ASTAR_INTEGRATION.md     # A* integration guide (new)
│   ├── RRT_star_tuning.md       # Historical RRT* tuning notes
│   ├── architecture_information.md
│   └── testing_guide.md
├── README.md
└── requirements.txt
```

### Prerequisites

- ROS 2 Humble (or later)
- PX4 Autopilot + Gazebo simulation
- Python 3.8+

### Installation

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone <your-repo-url>
   ```

2. **Install dependencies:**
   ```bash
   cd ~/ros2_ws/src/peg
   pip install -r requirements.txt
   ```

3. **Build packages:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select obstacle_detection costmap astar_planner integrated_planning
   source install/setup.bash
   ```

### Running the Node

1. **Launch PX4 + Gazebo simulation:**
   ```bash
   cd ~/PX4-Autopilot
   make px4_sitl gazebo-classic
   ```

2. **Start the full stack (A* + obstacle detection):**
   ```bash
   ros2 launch full_system.launch.py
   ```

   Or launch individual components:
   ```bash
   ros2 launch obstacle_detection detection.launch.py
   ros2 launch costmap costmap.launch.py
   ros2 launch astar_planner astar_planner.launch.py
   ```

3. **Visualize in RViz2:**
   ```bash
   rviz2
   ```

##  Configuration

### Obstacle Detection Parameters

Edit [obstacle_detection/config/params.yaml](obstacle_detection/config/params.yaml) for obstacle detection:

```yaml
obstacle_detector:
  ros__parameters:
    danger_distance: 80.0      # Distance threshold (meters) - configured for large UAV
    detection_width: 9.0       # Horizontal width ±Y (meters) - accounts for 15ft wingspan
    detection_height: 6.0      # Vertical height ±Z (meters)
    min_distance: 2.0          # Ignore points closer than this (meters)
    obstacle_inflation: 3.0    # Safety margin around obstacles (meters)
    lidar_topic: "/lidar/points"
```

**UAV Specifications:**
- Wingspan: 15 ft (4.6m)
- Length: 14.5 ft (4.4m)
- Height: 4 ft (1.2m)
- Payload: 6ft × 0.75ft × 0.75ft box

### Path Planning Parameters

Edit [astar_planner/config/astar_params.yaml](astar_planner/config/astar_params.yaml) for A* planner:

```yaml
astar_planner:
  ros__parameters:
    grid_resolution: 0.5       # metres per cell
    heuristic_weight: 1.0      # 1.0=optimal, >1.0=faster/greedy
    max_iterations: 10000      # hard cap on node expansions
    planning_frequency: 2.0    # Hz replanning rate
    lethal_cost_threshold: 70  # cells >= this are impassable
    cost_penalty_factor: 15.0  # penalty for traversing inflated cells
    goal_tolerance_m: 3.0      # goal acceptance radius (metres)
    safety_margin: 3.0         # minimum clearance (metres, 15ft wingspan)
    min_altitude_m: 5.0
    max_altitude_m: 120.0
```

##  Topics

**Subscribed by obstacle_detector:**
- `/lidar/points` (sensor_msgs/msg/PointCloud2) - LiDAR point cloud data

**Published by obstacle_detector:**
- `/obstacle_detected` (std_msgs/msg/Bool) - True when obstacle within danger distance
- `/obstacle_distance` (std_msgs/msg/Float32) - Distance to nearest obstacle in meters
- `/avoidance_command` (std_msgs/msg/String) - Zone command (EMERGENCY_HOVER / HARD_AVOID / REROUTE / ADJUST_HEADING / NORMAL_FLIGHT)
- `/obstacle_velocity` (geometry_msgs/msg/Vector3Stamped) - Kalman-estimated obstacle velocity

**Published by costmap_node:**
- `/costmap/grid` (nav_msgs/msg/OccupancyGrid) - 2D costmap for A* planning

**Published by astar_planner:**
- `/planned_path` (nav_msgs/msg/Path) - A*-generated global path (replaces RRT* path)
- `/astar_planner/status` (std_msgs/msg/String) - JSON planner status

##  Testing in Simulation

1. **Verify LiDAR topic:**
   ```bash
   ros2 topic list | grep cloud
   ros2 topic info /lidar/points
   ```

2. **Place obstacles in Gazebo** and monitor detection:
   ```bash
   ros2 topic echo /obstacle_detected
   ros2 topic echo /obstacle_distance
   ```

##  Hardware Deployment

1. **Update LiDAR driver:**
   - Replace Gazebo LiDAR with real sensor driver (Velodyne, Ouster, Livox, etc.)
   - Update `lidar_topic` parameter if needed

2. **Verify UAV dimensions:**
   - Confirm wingspan, length, and height match parameters
   - Adjust `obstacle_inflation` based on structural clearance requirements
   - Update `min_turn_radius` based on flight testing

3. **Tune for real-world conditions:**
   - Adjust `danger_distance` for flight speed and reaction time
   - Reduce `detection_height` to filter ground noise
   - Increase `obstacle_clearance` in cluttered environments

4. **No code changes required** in core detection/planning logic

##  Development Checklist

See [docs/CHECKLIST.md](docs/CHECKLIST.md) for the complete phase-by-phase implementation guide.

##  Integration with PX4

The `/obstacle_detected` and `/obstacle_distance` topics can be consumed by:
- The path planning team's system to trigger rerouting
- PX4's `ObstacleDistance` UORB message for onboard collision prevention
- Custom action logic (hold position, climb, return-to-home)

**Design principle:** Perception is decoupled from planning. This package is planning-agnostic.

> See [docs/ARCHITECTURE_DECISION.md](docs/ARCHITECTURE_DECISION.md) for the full architecture trade study and rationale.

##  Known Limitations

- Forward-facing detection only (X > 0)
- Fixed detection box (not velocity-adaptive)
- No ground plane removal (assumes level flight)
- Basic minimum-distance logic (no clustering)

##  Future Enhancements

- [ ] DBSCAN clustering for multiple obstacle tracking
- [ ] RANSAC ground plane removal
- [ ] Dynamic danger zone scaling based on airspeed
- [ ] Stereo camera depth fusion

##  Acknowledgments

Built following PX4 + ROS 2 best practices for simulation-to-real continuity.

A\* pathfinding algorithm sourced from
[Pegasus-Disaster-Response/Ros-workspace](https://github.com/Pegasus-Disaster-Response/Ros-workspace)
(commit c752b073, March 4 2026) by Team Pegasus — Cal Poly Pomona.
