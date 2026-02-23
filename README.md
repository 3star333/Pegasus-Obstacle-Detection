# PX4 ROS 2 Obstacle Detection & Path Planning

A modular ROS 2 package for real-time LiDAR-based obstacle detection and RRT* path planning designed for large fixed-wing/VTOL UAVs (15ft wingspan). This implementation is simulation-tested and hardware-ready.

##  Mission Goal

**Detect obstacles using LiDAR and generate flyable collision-free paths for large fixed-wing/VTOL aircraft with kinodynamic constraints.**

##  Architecture Overview

### **RRT* with Kinodynamic Constraints** (Recommended)

Based on trade study analysis, this architecture provides optimal balance for 15ft wingspan UAVs:

**Key Benefits:**
-  **Resource Usage: 30-40% CPU/Memory** (vs 50-75% for hybrid approach)
-  **Respects turn radius & flight dynamics** (critical for large aircraft)
-  **Planning horizon: 100m** (appropriate for forward flight speeds)
-  **Smooth, flyable trajectories** (not aggressive zigzags)
-  **Handles 3m obstacle inflation** (accounts for 15ft wingspan + safety)

**Trade-offs:**
- Path optimality: ~105% (slight overhead vs pure A*, but paths are flyable)
- Planning latency: 100-300ms (acceptable for 1-2 Hz replanning)

##  Features

### Obstacle Detection
- Real-time PointCloud2 processing
- Configurable detection parameters (distance, FOV, height)
- Obstacle inflation radius for structural clearance (wing tips, props)
- Configured for 15ft wingspan UAV with 80m danger distance

### Path Planning (RRT*)
- Kinodynamic constraints (turn radius, climb angle)
- Collision-free path generation with 3m safety margin
- Path smoothing and optimization
- Anytime planning (can terminate early if needed)
- 1-2 Hz replanning suitable for fixed-wing dynamics

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
├── rrt_star_planner/            # Path planning ROS 2 package (separate team)
│   ├── rrt_star_planner/
│   │   └── rrt_star_planner.py  # RRT* kinodynamic planner
│   ├── config/
│   │   └── rrt_star_params.yaml
│   ├── launch/
│   │   └── rrt_star_planner.launch.py
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
├── context/
│   └── Autonomy Trade Study.xlsx
├── docs/
│   ├── ARCHITECTURE_DECISION.md
│   ├── RRT_STAR_TUNING_GUIDE.md
│   ├── CHECKLIST.md
│   └── GETTING_STARTED.md
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
   # Build obstacle detection only
   colcon build --packages-select obstacle_detection
   
   # Build path planner separately (separate team)
   colcon build --packages-select rrt_star_planner
   
   source install/setup.bash
   ```

### Running the Node

1. **Launch PX4 + Gazebo simulation:**
   ```bash
   cd ~/PX4-Autopilot
   make px4_sitl gazebo-classic
   ```

2. **Start the obstacle detector:**
   ```bash
   ros2 run obstacle_detection obstacle_detector
   ```

3. **Visualize in RViz2:**
   ```bash
   rviz2
   ```

> **Note:** Path planning is handled by the separate `rrt_star_planner` package.

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

Edit [obstacle_detection/config/rrt_star_params.yaml](obstacle_detection/config/rrt_star_params.yaml) for RRT* planner:

```yaml
rrt_star_planner:
  ros__parameters:
    max_iterations: 2000       # Planning iterations
    step_size: 5.0             # Tree extension step (meters)
    goal_tolerance: 3.0        # Goal acceptance radius (meters)
    search_radius: 15.0        # RRT* rewiring radius (meters)
    planning_horizon: 100.0    # Look-ahead distance (meters)
    min_turn_radius: 20.0      # Kinodynamic constraint (meters)
    obstacle_clearance: 3.0    # Safety margin (meters)
    max_climb_angle: 20.0      # Vertical constraint (degrees)
    replan_rate: 1.0           # Replanning frequency (Hz)
```

##  Topics

**Subscribed:**
- `/lidar/points` (sensor_msgs/msg/PointCloud2) - LiDAR point cloud data

**Published:**
- `/obstacle_detected` (std_msgs/msg/Bool) - True when obstacle within danger distance
- `/obstacle_distance` (std_msgs/msg/Float32) - Distance to nearest obstacle in meters (inflation-adjusted)

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
