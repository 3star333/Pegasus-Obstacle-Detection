# Getting Started Guide

This guide walks you through your first run of the obstacle detection system.

## Prerequisites Check

Before starting, verify you have:
- [ ] ROS 2 Humble (or later) installed
- [ ] PX4 Autopilot cloned and built
- [ ] Gazebo Classic installed
- [ ] Python 3.8+ with pip

## Step-by-Step Setup

### 1. Verify PX4 Simulation Works

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```

You should see a UAV spawn in Gazebo. Press Ctrl+C to stop.

### 2. Check LiDAR Configuration

Start PX4 simulation, then in a new terminal:

```bash
ros2 topic list | grep cloud
```

You should see a topic like `/lidar/points` or similar. Note the exact name.

```bash
ros2 topic info /lidar/points
```

Verify it shows: `Type: sensor_msgs/msg/PointCloud2`

### 3. Build the Obstacle Detection Package

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Clone this repository
git clone <your-repo-url> peg
cd peg

# Install Python dependencies
pip install -r requirements.txt

# Build the package
cd ~/ros2_ws
colcon build --packages-select obstacle_detection
source install/setup.bash
```

### 4. Update Configuration (if needed)

If your LiDAR topic name is different, edit the config file:

```bash
nano ~/ros2_ws/src/peg/obstacle_detection/config/params.yaml
```

Change `lidar_topic` to match your topic name.

### 5. Launch Everything

Terminal 1 - Start PX4 + Gazebo:
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```

Terminal 2 - Start obstacle detector:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch obstacle_detection detection.launch.py
```

Terminal 3 - Monitor detections:
```bash
ros2 topic echo /obstacle_detected
```

Terminal 4 - Monitor distances:
```bash
ros2 topic echo /obstacle_distance
```

### 6. Test Detection

In Gazebo:
1. Insert a simple object (Insert tab → Box or Cylinder)
2. Place it in front of the UAV (within 5 meters)
3. Watch Terminal 3 for detection changes

### 7. Visualize in RViz2

```bash
rviz2
```

1. Set Fixed Frame to `base_link` or your UAV frame
2. Add → PointCloud2 → Topic: `/lidar/points`
3. Verify points appear in front of the UAV

## Troubleshooting

### No point cloud data
- Check that LiDAR is configured in PX4 model
- Verify topic name matches configuration
- Check `ros2 topic hz /lidar/points` for update rate

### Node crashes on startup
- Verify all dependencies installed: `pip install numpy`
- Check ROS 2 workspace is sourced: `source ~/ros2_ws/install/setup.bash`

### False detections
- Ground plane may be visible - adjust `min_distance` parameter
- Reduce `detection_height` to filter low points

### No detections when obstacle present
- Check obstacle is within detection box (X > 0.3, |Y| < 2.0, |Z| < 1.5)
- Verify `danger_distance` is large enough
- Use RViz2 to visualize point cloud

## Next Steps

Once basic detection works:
1. Follow [docs/CHECKLIST.md](docs/CHECKLIST.md) for complete validation
2. Experiment with parameter tuning
3. Test multiple obstacle scenarios
4. Prepare for hardware deployment

## Need Help?

Open an issue with:
- ROS 2 version
- PX4 version
- Error messages
- Steps to reproduce
