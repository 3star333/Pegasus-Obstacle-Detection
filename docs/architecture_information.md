# Architecture Decision: RRT* Kinodynamic Path Planning for 15ft Wingspan UAV

**Date:** February 22, 2026  
**Status:** Accepted  
**Author:** Autonomy Team

## Context

The project initially considered a **Hybrid Architecture (Global Graph + Local Optimizer)** based on a comprehensive trade study evaluation. However, after analyzing the UAV specifications (15ft wingspan, 14.5ft length, 4ft height), a critical reassessment was needed.

## UAV Specifications

### Physical Dimensions
- **Wingspan:** 15 ft (4.6m)
- **Length:** 14.5 ft (4.4m) including propeller protrusion
- **Height:** ~4 ft (1.2m) - subject to landing gear configuration
- **Payload:** 6ft × 0.75ft × 0.75ft box

### Derived Clearance Requirements
- **Minimum corridor width:** 10-12m (wingspan + 2× safety margin)
- **Obstacle inflation radius:** 3m (2× wing radius for safety)
- **Structural clearance zones:** Wing tips, forward motors/props

## Trade Study Results Summary

| Architecture | Resource Usage | Path Optimality | Planning Latency | Replanning Rate |
|-------------|----------------|-----------------|------------------|----------------|
| **Hybrid (Global+Local)** | 50-75% | 100-103% | 200-600ms | 1.5-3.0 Hz |
| **RRT* Kinodynamic** | 30-40% | ~105% | 100-300ms | 1-2 Hz |
| **Reactive Local (DWA)** | 10-20% | 110-120% | <50ms | 20-50 Hz |
| **Sparse RRT*** | 30-45% | 102-110% | 100-300ms | N/A |

## Decision

**Selected: RRT* with Kinodynamic Constraints**

### Rationale

#### 1. Resource Efficiency (Critical)
- **30-40% CPU/Memory usage** vs 50-75% for Hybrid
- **40% reduction** in computational load
- Leaves headroom for other critical systems (flight control, communication, payload)
- Thermal and power constraints met

#### 2. Kinodynamic Suitability (Essential)
- **Respects turn radius constraints** (~20m minimum for fixed-wing/VTOL)
- **Climb angle limitations** (20° maximum)
- Generates **smooth, flyable trajectories** (not aggressive zigzags)
- Natural integration with PX4 position setpoints

#### 3. Planning Performance (Adequate)
- **100m planning horizon** appropriate for forward flight
- **1-2 Hz replanning rate** sufficient for fixed-wing dynamics (doesn't need 20Hz like multirotor)
- **100-300ms latency** acceptable given aircraft response time
- Early termination capability for time-critical scenarios

#### 4. Obstacle Handling (Superior)
- **3m safety margin** built into collision checking
- Handles **large obstacle inflation** (critical for 15ft wingspan)
- Natural representation of 3D obstacle space
- Probabilistic completeness guarantees

#### 5. Implementation Complexity (Manageable)
- Well-documented algorithm (Karaman & Frazzoli, 2011)
- Moderate complexity vs Hybrid approach
- Clear parameter tuning guidelines
- Proven in similar aerospace applications

### Trade-offs Accepted

1. **Path optimality:** ~105% vs 100-103% for Hybrid
   - **Acceptable:** 5% path length increase is negligible compared to flyability gains
   - Smooth paths reduce control effort and energy consumption

2. **Replanning rate:** 1-2 Hz vs 1.5-3.0 Hz for Hybrid
   - **Acceptable:** Fixed-wing aircraft have slower dynamics than multirotors
   - 1 Hz is sufficient for obstacle avoidance at typical cruise speeds (15-30 m/s)

## Alternatives Considered and Rejected

### ❌ Reactive Local Planners (DWA/VFH+)
**Why rejected:**
- Designed for highly maneuverable multirotors
- Planning horizon too short (<10m) for 15ft aircraft
- Assumes instantaneous direction changes (not possible for fixed-wing)
- Can generate unflyable paths with sharp turns

**When suitable:** Small, agile multirotors in cluttered indoor environments

### ❌ Pure Hybrid (Global Graph + Local Optimizer)
**Why rejected:**
- **Resource usage too high:** 50-75% is concerning for UAV applications
- Added complexity doesn't provide clear benefit for this use case
- Marginal optimality improvement (100-103% vs 105%) doesn't justify cost
- Local optimizer may conflict with kinodynamic constraints

**When suitable:** Ground robots with abundant compute, highly structured environments

### ❌ Corridor-Based A*
**Why rejected:**
- Requires pre-computed or known environment structure
- Not suitable for unknown/dynamic environments
- Limited 3D maneuverability
- Better for waypoint-based navigation with known corridors

**When suitable:** Outdoor navigation in semi-structured spaces (roads, valleys)

## Implementation Parameters

### Obstacle Detection (Updated for 15ft UAV)
```yaml
danger_distance: 80.0      # meters - reaction time for forward flight
detection_width: 9.0       # meters - accounts for wingspan + margin
detection_height: 6.0      # meters - vertical clearance
min_distance: 2.0          # meters - ignore too-close points
obstacle_inflation: 3.0    # meters - structural clearance
```

### RRT* Planner Configuration
```yaml
max_iterations: 2000       # Planning budget
step_size: 5.0            # meters - larger for big aircraft
goal_tolerance: 3.0       # meters - acceptance radius
search_radius: 15.0       # meters - rewiring radius
planning_horizon: 100.0   # meters - look-ahead distance
min_turn_radius: 20.0     # meters - kinodynamic constraint
obstacle_clearance: 3.0   # meters - safety margin
max_climb_angle: 20.0     # degrees - vertical constraint
replan_rate: 1.0          # Hz - fixed-wing replanning rate
```

## Validation Criteria

The RRT* implementation will be considered successful if:

1. **Resource Usage:** CPU/Memory stays below 40% during nominal operations
2. **Path Quality:** Paths are flyable (respect turn radius and climb angle)
3. **Safety:** All paths maintain ≥3m clearance from obstacles
4. **Performance:** Planning completes within 300ms for 95% of cases
5. **Reliability:** Finds valid path when one exists (within horizon) in >90% of scenarios

## Future Considerations

### Short-term Enhancements
- Path smoothing optimization (Bézier curves, B-splines)
- Informed RRT* (goal-biased sampling)
- Multi-resolution planning (coarse then fine)

### Long-term Research
- Wind-aware trajectory optimization
- Dynamic obstacle prediction and avoidance
- Multi-agent coordination (formation flight)
- Learning-based cost function tuning

## References

1. Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. *International Journal of Robotics Research*.

2. LaValle, S. M., & Kuffner, J. J. (2001). Randomized kinodynamic planning. *International Journal of Robotics Research*.

3. PX4 Autopilot Documentation: Obstacle Avoidance. https://docs.px4.io/main/en/computer_vision/obstacle_avoidance.html

4. Project Trade Study: Autonomy Trade Study Evaluation Matrix (Hybrid Architecture)

## Revision History

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2026-02-22 | 1.0 | Initial decision document | Autonomy Team |
