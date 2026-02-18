# uav_manager

**Package Type:** ROS2 Node Package  
**Dependencies:** peregrine_interfaces, rclcpp  

---

## Overview

`uav_manager` is the **central orchestrator** for a single UAV. It manages the high-level state machine, coordinates all other managers, and provides a unified interface for mission execution.

---

## Responsibilities

1. **State machine management** - IDLE → ARMED → TAKEOFF → FLYING → LANDING → LANDED
2. **Lifecycle coordination** - ensure proper startup/shutdown sequence
3. **Mode management** - track and switch flight modes
4. **Preflight checks** - verify system readiness before flight
5. **Takeoff/Land sequences** - coordinate with trajectory_manager
6. **Emergency handling** - respond to safety_monitor alerts
7. **Unified status** - aggregate status from all subsystems

---

## State Machine

```
┌──────────────────────────────────────────────────────────────────┐
│                        UAV STATE MACHINE                          │
│                                                                   │
│   ┌────────┐    arm()     ┌────────┐    takeoff()   ┌──────────┐│
│   │  IDLE  │─────────────▶│ ARMED  │───────────────▶│ TAKING   ││
│   │        │              │        │                │   OFF    ││
│   └────┬───┘              └────┬───┘                └─────┬────┘│
│        ▲                       │                          │     │
│        │ disarm()              │ disarm()                 │     │
│        │                       ▼                          ▼     │
│   ┌────┴───┐              ┌────────┐               ┌──────────┐ │
│   │ LANDED │◀─────────────│LANDING │◀──────────────│ HOVERING │ │
│   │        │   touchdown  │        │    land()     │          │ │
│   └────────┘              └────────┘               └─────┬────┘ │
│                                ▲                         │      │
│                                │ land()                  │      │
│                                │                         ▼      │
│                           ┌────┴───┐              ┌──────────┐  │
│                           │RETURNING│◀────────────│  FLYING  │  │
│                           │  HOME  │    rth()     │          │  │
│                           └────────┘              └──────────┘  │
│                                                                  │
│   ════════════════════════════════════════════════════════════  │
│   From ANY state:                                                │
│     emergency_stop() → EMERGENCY → (requires manual recovery)    │
└──────────────────────────────────────────────────────────────────┘
```

---

## Architecture

```
Inputs:
  /estimator_manager/state           → Current state
  /controller_manager/status         → Controller health
  /trajectory_manager/status         → Trajectory progress
  /safety_monitor/safety_status      → Safety alerts
  /hardware_abstraction/px4_status   → PX4 connection/mode

Actions:
  /uav_manager/takeoff               → Takeoff sequence
  /uav_manager/land                  → Landing sequence
  /uav_manager/go_to                 → Go to position
  /uav_manager/execute_trajectory    → Execute trajectory

Services:
  /uav_manager/arm                   → Arm/disarm
  /uav_manager/set_mode              → Change flight mode
  /uav_manager/emergency_stop        → Emergency stop
  /uav_manager/get_status            → Get full status

Outputs:
  /uav_manager/mode                  → Current mode (to all managers)
  /uav_manager/status                → Comprehensive status
```

---

## Key Implementation Details

### Preflight Check Sequence

```
Preflight checks (all must pass before arming):
  1. hardware_abstraction connected to PX4
  2. estimator_manager healthy and publishing
  3. controller_manager has active controller
  4. safety_monitor reports NOMINAL
  5. Battery level > minimum threshold
  6. GPS fix (outdoor) or MoCap tracking (indoor)
  7. Geofence loaded and valid
  8. No active warnings/errors
```

### Takeoff Sequence

```
takeoff(target_altitude):
  1. Verify state == ARMED
  2. Run preflight checks
  3. Set state = TAKING_OFF
  4. Request takeoff trajectory from trajectory_manager
  5. Command hardware_abstraction to offboard mode
  6. Monitor altitude until target reached
  7. Set state = HOVERING
  8. Return success
  
On failure at any step:
  - Attempt safe abort (land if airborne)
  - Set appropriate error state
  - Return failure with reason
```

### Landing Sequence

```
land(target_position=current):
  1. Verify state in [HOVERING, FLYING, RETURNING_HOME]
  2. Set state = LANDING
  3. Request landing trajectory
  4. Monitor descent
  5. Detect touchdown (velocity ≈ 0, altitude ≈ 0)
  6. Command disarm
  7. Set state = LANDED
```

### Mode Transition Guards

```
Transition guards (examples):
  IDLE → ARMED:      preflight_ok AND user_confirmed
  ARMED → TAKING_OFF: armed AND estimator_healthy
  HOVERING → FLYING:  trajectory_valid AND controller_healthy
  ANY → EMERGENCY:    always allowed (safety critical)
  EMERGENCY → IDLE:   manual_reset AND system_healthy
```

### Emergency Handling

```
On safety_monitor EMERGENCY alert:
  1. Immediately set state = EMERGENCY
  2. Based on alert type:
     - GEOFENCE_VIOLATION: Command RTH or land
     - ESTIMATOR_FAILURE: Command hover (if possible) or land
     - BATTERY_CRITICAL: Command immediate land
     - COLLISION_IMMINENT: Command stop
     - HEARTBEAT_LOSS: Follow configured failsafe
  3. Log all details
  4. Notify GCS
```

### Status Aggregation

```
UAVStatus contains:
  - uav_id, uav_name
  - current_state (from state machine)
  - current_mode (MANUAL, POSITION, OFFBOARD, etc.)
  - position, velocity, attitude (from estimator)
  - battery_percent, battery_voltage
  - active_estimator, active_controller
  - safety_status summary
  - px4_connection status
  - mission_progress (if executing)
  - active_warnings[], active_errors[]
```

---

## Configuration

```yaml
uav_manager:
  ros__parameters:
    uav_id: "uav1"
    uav_name: "Alpha"
    
    # Timeouts
    arm_timeout_s: 5.0
    takeoff_timeout_s: 30.0
    land_timeout_s: 60.0
    
    # Preflight thresholds
    min_battery_percent: 20.0
    min_gps_satellites: 8
    
    # Takeoff/Land defaults
    default_takeoff_altitude_m: 2.0
    default_climb_rate_mps: 1.0
    default_descent_rate_mps: 0.5
    
    # Safety
    emergency_land_on_estimator_failure: true
    emergency_land_on_controller_failure: true
    auto_disarm_on_land: true
    
    # Failsafes
    gcs_loss_action: "return_home"    # land, hover, return_home, continue
    battery_warning_percent: 30.0
    battery_critical_percent: 15.0
```

---

## Coordination with Other Managers

### Startup Sequence

```
1. hardware_abstraction starts, connects to PX4
2. estimator_manager starts, loads default estimator
3. controller_manager starts, loads default controller
4. trajectory_manager starts, loads generators
5. safety_monitor starts, begins monitoring
6. uav_manager starts, runs self-check
7. All managers report ready → uav_manager publishes IDLE state
```

### During Flight

```
uav_manager continuously:
  - Monitors all manager status topics
  - Responds to safety_monitor alerts
  - Handles action requests (takeoff, land, goto)
  - Publishes mode changes to all managers
  - Aggregates status for external consumers
```

---

## Major Cautions

| Issue | Mitigation |
|-------|------------|
| **Race conditions** | Use proper state machine, atomic transitions |
| **Stuck in state** | Implement timeouts for all transitions |
| **Missed safety alerts** | Use reliable QoS, check stale data |
| **Premature disarm** | Only disarm when confirmed on ground |
| **Incomplete sequences** | Always have abort/recovery path |
| **Status inconsistency** | Single source of truth for state |

---

## File Structure

```
uav_manager/
├── include/uav_manager/
│   ├── uav_manager_node.hpp
│   ├── state_machine.hpp
│   └── preflight_checker.hpp
├── src/
│   ├── uav_manager_node.cpp
│   ├── state_machine.cpp
│   ├── preflight_checker.cpp
│   └── main.cpp
├── config/
│   └── uav_manager.yaml
└── launch/
    └── uav_manager.launch.py
```
