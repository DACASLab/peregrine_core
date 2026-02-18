# trajectory_manager

**Package Type:** ROS2 Node Package with Plugin Architecture  
**Dependencies:** peregrine_interfaces, pluginlib, rclcpp, Eigen3  

---

## Overview

`trajectory_manager` handles **trajectory generation, storage, and evaluation**. It provides setpoints to the controller at the required rate, supporting various trajectory types through plugins.

---

## Responsibilities

1. **Load trajectory generator plugins** (polynomial, B-spline, waypoint, etc.)
2. **Store and manage trajectories** - accept new, queue, replace, abort
3. **Evaluate trajectories** - sample at controller rate, provide setpoints
4. **Track progress** - report completion percentage, ETA
5. **Handle constraints** - respect collision avoidance constraints from multi_agent_coordinator

---

## Architecture

```
Inputs:
  /uav_manager/mode                          → Flight mode
  /multi_agent_coordinator/collision_constraint → BVC constraints
  
Actions (async trajectory requests):
  /trajectory_manager/set_trajectory         → Execute trajectory
  /trajectory_manager/go_to_position         → Simple goto
  
Outputs:
  /trajectory_manager/setpoint               → Current setpoint (to controller)
  /trajectory_manager/trajectory             → Full trajectory (for visualization)
  /trajectory_manager/status                 → Progress, state

Services:
  /trajectory_manager/abort                  → Stop current trajectory
  /trajectory_manager/pause                  → Pause execution
  /trajectory_manager/resume                 → Resume execution
  /trajectory_manager/set_generator          → Switch trajectory generator
```

---

## Plugin Interface (TrajectoryGeneratorBase)

### Required Methods

| Method | Purpose |
|--------|---------|
| `initialize(node, name)` | Setup generator |
| `generate(request)` | Generate trajectory from request → Trajectory msg |
| `evaluate(trajectory, t)` | Sample trajectory at time t → TrajectorySetpoint |
| `getName()` | Generator name |
| `getType()` | WAYPOINT, POLYNOMIAL, BSPLINE, etc. |
| `getOutputType()` | POSITION_ONLY, POSITION_YAW, FULL_STATE |

### Output Types

| Type | Fields Provided | Best For |
|------|-----------------|----------|
| POSITION_ONLY | x, y, z | Simple position controllers |
| POSITION_YAW | x, y, z, yaw | Most use cases |
| FULL_STATE | pos, vel, acc, jerk, yaw, yaw_rate | MPC, geometric controllers |

---

## Default Plugins to Implement

### 1. WaypointLinearGenerator
- **Purpose**: Linear interpolation between waypoints
- **Output**: POSITION_YAW
- **Use Case**: Simple missions, basic navigation
- **Implementation**: 
  - Compute segment distances
  - Interpolate position linearly
  - Interpolate yaw with shortest path

### 2. PolynomialTrajectoryGenerator
- **Purpose**: Minimum snap/jerk polynomial trajectories
- **Output**: FULL_STATE
- **Use Case**: Smooth aggressive flight, MPC controllers
- **Implementation**:
  - Solve QP for polynomial coefficients
  - Minimize snap (4th derivative) or jerk (3rd)
  - Satisfy waypoint constraints
- **Key Reference**: Mellinger & Kumar, "Minimum Snap Trajectory Generation"

### 3. HoldPositionGenerator
- **Purpose**: Generate constant setpoint at current position
- **Output**: POSITION_YAW
- **Use Case**: Hovering, emergency hold
- **Implementation**: Return fixed setpoint indefinitely

### 4. TakeoffLandGenerator
- **Purpose**: Vertical takeoff/landing trajectories
- **Output**: POSITION_YAW
- **Use Case**: Automated takeoff and landing sequences
- **Implementation**:
  - Smooth vertical velocity profile (trapezoidal or S-curve)
  - Fixed horizontal position

---

## Trajectory Request Structure

```
TrajectoryRequest:
  # What type of trajectory
  trajectory_type: WAYPOINT | GOTO | POLYNOMIAL | LAND | TAKEOFF
  
  # Waypoints (if applicable)
  waypoints: [PoseStamped]
  
  # Single target (for GOTO)
  target_position: Point
  target_yaw: float
  
  # Timing constraints
  duration_s: float              # Desired duration (0 = auto)
  velocity_mps: float            # Desired velocity (0 = default)
  
  # Constraints
  max_velocity: float
  max_acceleration: float
  
  # Options
  wait_at_waypoints: bool
  waypoint_wait_time_s: float
```

---

## Key Implementation Details

### Setpoint Publishing Rate
- Publish at controller rate (typically 50-250 Hz)
- Use timer, not trajectory completion-driven
- Interpolate if trajectory sample rate differs

### Trajectory Timing
```
On trajectory start:
  trajectory_start_time = now()
  
On each publish:
  elapsed = now() - trajectory_start_time
  setpoint = generator.evaluate(trajectory, elapsed)
  
  if elapsed >= trajectory.duration:
    mark_completed()
```

### Collision Constraint Integration
```
When constraint received from multi_agent_coordinator:
  1. Store constraint
  2. Before publishing setpoint, check if it violates constraint
  3. If violation:
     a. Project setpoint onto constraint boundary
     b. Or trigger replan if projection insufficient
```

### Trajectory Queueing (Optional)
- Support queuing multiple trajectories
- Smooth transitions between trajectories
- Allow preemption for emergency maneuvers

### Progress Tracking
```
progress_percent = (elapsed_time / total_duration) * 100
distance_remaining = integrate(trajectory, elapsed, duration)
eta_s = distance_remaining / average_velocity
```

---

## Configuration

```yaml
trajectory_manager:
  ros__parameters:
    publish_rate_hz: 50.0
    default_generator: "waypoint_linear"
    
    plugins:
      - "waypoint_linear"
      - "polynomial"
      - "hold_position"
      - "takeoff_land"
    
    # Default constraints
    default_velocity_mps: 2.0
    default_acceleration_mpss: 1.5
    
    # Waypoint generator
    waypoint_linear:
      arrival_tolerance_m: 0.3
      yaw_rate_radps: 0.5
    
    # Polynomial generator
    polynomial:
      order: 7                    # 7th order for min snap
      continuity_order: 4         # Continuous up to snap
```

---

## Major Cautions

| Issue | Mitigation |
|-------|------------|
| **Time synchronization** | Use ROS time consistently, handle sim time |
| **Trajectory discontinuity** | Ensure smooth transitions, check initial conditions |
| **Constraint violation** | Validate trajectory against constraints before execution |
| **Division by zero** | Handle zero-duration segments, stationary waypoints |
| **Memory growth** | Clear old trajectories, limit queue size |
| **Late setpoints** | Detect and handle if setpoint generation is too slow |

---

## File Structure

```
trajectory_manager/
├── include/trajectory_manager/
│   ├── trajectory_generator_base.hpp
│   └── trajectory_manager_node.hpp
├── src/
│   ├── trajectory_manager_node.cpp
│   └── main.cpp
├── plugins/
│   ├── waypoint_linear_generator.cpp
│   ├── polynomial_generator.cpp
│   ├── hold_position_generator.cpp
│   ├── takeoff_land_generator.cpp
│   └── plugins.xml
├── config/
│   └── trajectory_manager.yaml
└── launch/
    └── trajectory_manager.launch.py
```
