# RRT* Kinodynamic Planner Configuration Guide

Quick reference for tuning RRT* planner parameters for different scenarios and missions.

## Parameter Overview

| Parameter | Default | Range | Impact |
|-----------|---------|-------|--------|
| `max_iterations` | 2000 | 500-5000 | Planning quality vs computation time |
| `step_size` | 5.0m | 2.0-10.0m | Tree granularity and path smoothness |
| `goal_tolerance` | 3.0m | 1.0-5.0m | Goal acceptance radius |
| `search_radius` | 15.0m | 5.0-30.0m | Path optimization quality |
| `planning_horizon` | 100.0m | 50.0-200.0m | Look-ahead distance |
| `min_turn_radius` | 20.0m | 10.0-40.0m | Turn sharpness constraint |
| `obstacle_clearance` | 3.0m | 2.0-5.0m | Safety margin |
| `max_climb_angle` | 20.0° | 10.0-30.0° | Vertical maneuver limit |
| `replan_rate` | 1.0 Hz | 0.5-5.0 Hz | Replanning frequency |

## Scenario-Based Configurations

### 1. Open Outdoor Navigation (Sparse Obstacles)

**Use Case:** Long-distance waypoint navigation with few obstacles

```yaml
max_iterations: 1000          # Fewer needed for sparse environments
step_size: 8.0               # Larger steps ok in open space
goal_tolerance: 5.0          # Relaxed tolerance for waypoints
search_radius: 20.0          # More optimization possible
planning_horizon: 150.0      # Look farther ahead
min_turn_radius: 25.0        # Can use gentler turns
obstacle_clearance: 3.0      # Standard clearance
max_climb_angle: 15.0        # Gentle climbs for efficiency
replan_rate: 0.5             # Less frequent replanning needed
```

**Expected Performance:**
- Planning time: ~50-150ms
- Path quality: 102-105% optimal
- Resource usage: 25-30%

---

### 2. Cluttered Environment (Dense Obstacles)

**Use Case:** Urban navigation, forest flight, complex terrain

```yaml
max_iterations: 3000          # More iterations for complex space
step_size: 3.0               # Smaller steps for precision
goal_tolerance: 2.0          # Tighter goal reaching
search_radius: 12.0          # Focused optimization
planning_horizon: 80.0       # Shorter horizon in clutter
min_turn_radius: 18.0        # Sharper turns if needed
obstacle_clearance: 4.0      # Extra safety in clutter
max_climb_angle: 25.0        # More vertical freedom
replan_rate: 2.0             # More frequent replanning
```

**Expected Performance:**
- Planning time: ~200-400ms
- Path quality: 105-110% optimal
- Resource usage: 35-45%

---

### 3. High-Speed Flight (20+ m/s)

**Use Case:** Fast transit, time-critical missions

```yaml
max_iterations: 1500          # Balance speed and quality
step_size: 10.0              # Large steps for long paths
goal_tolerance: 5.0          # Relaxed for speed
search_radius: 25.0          # Smooth long paths
planning_horizon: 200.0      # Long horizon for reaction time
min_turn_radius: 35.0        # Wider turns at high speed
obstacle_clearance: 5.0      # Extra margin for speed
max_climb_angle: 15.0        # Gentle maneuvers
replan_rate: 1.5             # Faster replanning
```

**Expected Performance:**
- Planning time: ~100-250ms
- Path quality: 108-112% optimal
- Resource usage: 30-35%

**Note:** `danger_distance` should be increased to 120-150m for high-speed flight.

---

### 4. Precision Landing/Docking

**Use Case:** Final approach to landing zone, docking maneuver

```yaml
max_iterations: 2500          # High quality needed
step_size: 2.0               # Very fine granularity
goal_tolerance: 1.0          # Precise goal reaching
search_radius: 10.0          # Local optimization
planning_horizon: 50.0       # Short horizon for precision
min_turn_radius: 15.0        # Tighter turns at low speed
obstacle_clearance: 2.5      # Can reduce at low speed
max_climb_angle: 30.0        # More vertical freedom
replan_rate: 3.0             # High replanning rate
```

**Expected Performance:**
- Planning time: ~150-350ms
- Path quality: 102-106% optimal
- Resource usage: 35-40%

---

### 5. Emergency/Avoidance Mode

**Use Case:** Sudden obstacle detection, collision avoidance

```yaml
max_iterations: 800           # Fast planning prioritized
step_size: 5.0               # Standard steps
goal_tolerance: 4.0          # Relaxed goal
search_radius: 10.0          # Less optimization
planning_horizon: 60.0       # Near-term planning
min_turn_radius: 15.0        # Sharp evasive turns
obstacle_clearance: 4.0      # Extra safety
max_climb_angle: 30.0        # Max vertical freedom
replan_rate: 5.0             # Continuous replanning
```

**Expected Performance:**
- Planning time: ~50-120ms (fast!)
- Path quality: 110-120% optimal (sub-optimal but safe)
- Resource usage: 30-40%

**Trigger:** When obstacle detected within 2× `min_turn_radius` distance.

---

## Parameter Tuning Guidelines

### Increasing Planning Quality
1. ↑ `max_iterations` (2000 → 3000)
2. ↑ `search_radius` (15.0 → 20.0)
3. ↓ `step_size` (5.0 → 3.0)

**Cost:** Longer planning time, higher CPU usage

### Reducing Computation Time
1. ↓ `max_iterations` (2000 → 1000)
2. ↓ `search_radius` (15.0 → 10.0)
3. ↑ `step_size` (5.0 → 7.0)

**Cost:** Less optimal paths, may struggle in tight spaces

### Improving Path Smoothness
1. ↑ `search_radius` (15.0 → 20.0)
2. ↑ `min_turn_radius` (20.0 → 25.0)
3. ↓ `max_climb_angle` (20.0 → 15.0)

**Cost:** Requires more maneuvering space

### Handling Tighter Spaces
1. ↓ `min_turn_radius` (20.0 → 15.0)
2. ↓ `step_size` (5.0 → 3.0)
3. ↓ `obstacle_clearance` (3.0 → 2.5) *only if safe*

**Cost:** Less smooth paths, higher computational load

---

## Adaptive Parameter Strategies

### Speed-Based Adaptation

```python
# Pseudo-code for adaptive parameters
if vehicle_speed < 10 m/s:
    min_turn_radius = 15.0
    planning_horizon = 60.0
    obstacle_clearance = 2.5
elif vehicle_speed < 20 m/s:
    min_turn_radius = 20.0
    planning_horizon = 100.0
    obstacle_clearance = 3.0
else:  # High speed
    min_turn_radius = 35.0
    planning_horizon = 150.0
    obstacle_clearance = 5.0
```

### Environment-Based Adaptation

```python
# Pseudo-code based on obstacle density
obstacle_density = count_obstacles_in_horizon() / planning_volume

if obstacle_density > 0.1:  # Cluttered
    max_iterations = 3000
    step_size = 3.0
    replan_rate = 2.0
elif obstacle_density > 0.05:  # Moderate
    max_iterations = 2000
    step_size = 5.0
    replan_rate = 1.0
else:  # Sparse
    max_iterations = 1000
    step_size = 8.0
    replan_rate = 0.5
```

---

## Debugging Tips

### Problem: Planning takes too long (>500ms)

**Solutions:**
- Reduce `max_iterations` to 1000-1500
- Increase `step_size` to 7-8m
- Reduce `search_radius` to 10-12m
- Check for excessive obstacle points in point cloud

### Problem: Paths collide with obstacles

**Solutions:**
- Increase `obstacle_clearance` to 4-5m
- Verify LiDAR point cloud accuracy
- Check `obstacle_inflation` in obstacle detector (should be 3.0m)
- Reduce `step_size` for finer collision checking

### Problem: No path found

**Solutions:**
- Increase `max_iterations` to 3000-5000
- Increase `planning_horizon` (may need intermediate goal)
- Reduce `min_turn_radius` if kinematically safe
- Check if goal is reachable with current constraints
- Increase `goal_tolerance` for waypoint navigation

### Problem: Paths too jagged/not smooth

**Solutions:**
- Increase `search_radius` to 20-25m
- Increase `min_turn_radius` to 25-30m
- Reduce `max_climb_angle` to 15°
- Implement additional path smoothing post-processing

### Problem: High CPU usage (>50%)

**Solutions:**
- Reduce `max_iterations` to 1500
- Increase `step_size` to 6-8m
- Reduce `replan_rate` to 0.5-1.0 Hz
- Downsample LiDAR point cloud before planning
- Use spatial hashing for collision detection

---

## Testing Recommendations

### Phase 1: Simulation (Gazebo)
1. Start with default parameters
2. Add simple obstacles (walls, boxes)
3. Verify paths are collision-free and smooth
4. Measure planning time and resource usage

### Phase 2: Complex Simulation
1. Add dense obstacle fields
2. Test different configurations (cluttered, open, mixed)
3. Validate kinodynamic constraints (turn radius, climb angle)
4. Test emergency scenarios

### Phase 3: Hardware Testing
1. Start in open environment with default params
2. Validate obstacle detection accuracy
3. Tune `obstacle_clearance` based on real sensor noise
4. Test at various speeds
5. Gradually increase environment complexity

### Phase 4: Mission Testing
1. Full mission profiles with waypoints
2. Different weather conditions
3. Day/night operations (LiDAR performance)
4. Edge cases (narrow gaps, overpasses, etc.)

---

## Performance Benchmarks

Target performance metrics for validation:

| Metric | Target | Acceptable | Poor |
|--------|--------|------------|------|
| Planning Time | <200ms | 200-400ms | >400ms |
| Path Quality | <110% | 110-115% | >115% |
| CPU Usage | <35% | 35-45% | >45% |
| Success Rate | >95% | 90-95% | <90% |
| Clearance Violations | 0 | 0 | >0 |

**Note:** Targets may vary based on scenario and mission criticality.

---

## Contact & Support

For questions or assistance with parameter tuning:
- Review [ARCHITECTURE_DECISION.md](ARCHITECTURE_DECISION.md) for design rationale
- Check [CHECKLIST.md](CHECKLIST.md) for implementation status
- Refer to code documentation in `rrt_star_planner.py`
