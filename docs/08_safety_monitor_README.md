# safety_monitor

**Package Type:** ROS2 Node Package  
**Dependencies:** aerion_interfaces, rclcpp, Eigen3  

---

## Overview

`safety_monitor` is the **watchdog and safety enforcement layer** for AERION. It runs independently of other managers and can trigger emergency actions when safety violations occur.

---

## Responsibilities

1. **Geofencing** - software geofence enforcement (cylindrical and polygonal)
2. **Heartbeat monitoring** - detect loss of critical connections
3. **State validation** - detect estimator divergence, impossible states
4. **Resource monitoring** - battery, CPU, memory
5. **Flight envelope protection** - velocity, altitude, tilt limits
6. **Emergency triggers** - initiate emergency procedures
7. **Graceful degradation** - recommend fallback actions

---

## Safety Layers

```
┌─────────────────────────────────────────────────────────────────┐
│                    SAFETY LAYER STACK                            │
│                                                                  │
│  Layer 3: PX4 Hardware Failsafes (BACKUP - NOT CONTROLLED HERE) │
│  ───────────────────────────────────────────────────────────────│
│  • RC loss          • Battery failsafe     • Geofence (PX4)     │
│  • Data link loss   • Position loss        • Motor failure      │
│                                                                  │
│  Layer 2: AERION Safety Monitor (THIS PACKAGE)                  │
│  ───────────────────────────────────────────────────────────────│
│  • Software geofence    • Heartbeat monitor   • State validation│
│  • Flight envelope      • Battery monitor     • Collision alert │
│                                                                  │
│  Layer 1: Manager-Level Checks                                   │
│  ───────────────────────────────────────────────────────────────│
│  • Controller limits    • Estimator confidence  • Trajectory    │
│                           bounds               │
└─────────────────────────────────────────────────────────────────┘
```

---

## Architecture

```
Inputs (monitored continuously):
  /estimator_manager/state           → Position, velocity, attitude
  /estimator_manager/status          → Estimator health
  /controller_manager/status         → Controller health  
  /trajectory_manager/status         → Trajectory status
  /hardware_abstraction/px4_status   → PX4 connection
  /hardware_abstraction/battery      → Battery state
  
  # From other UAVs (for collision)
  /uavN/multi_agent/own_state        → Neighbor positions

Outputs:
  /safety_monitor/safety_status      → Overall safety state
  /safety_monitor/geofence_status    → Geofence-specific status
  /safety_monitor/alerts             → Individual alerts

Services:
  /safety_monitor/emergency_stop     → Trigger emergency
  /safety_monitor/set_geofence       → Update geofence
  /safety_monitor/reset              → Clear alerts (manual)
```

---

## Safety Checks

### 1. Geofence Enforcement

```
Geofence types:
  Cylindrical:
    - max_radius_m from home
    - max_altitude_m / min_altitude_m
    
  Polygonal:
    - 2D polygon boundary
    - min/max altitude

Check algorithm:
  distance_to_boundary = compute_distance(position, geofence)
  
  if distance < critical_buffer:
    trigger GEOFENCE_VIOLATION, action = LAND/RTH
  elif distance < warning_buffer:
    trigger GEOFENCE_WARNING, action = WARN
```

### 2. Heartbeat Monitoring

```
Heartbeat sources:
  CRITICAL (loss = emergency):
    - hardware_abstraction (PX4 link)
    - estimator_manager
    
  HIGH (loss = RTH):
    - GCS connection
    - controller_manager
    
  MEDIUM (loss = warning):
    - trajectory_manager
    - multi_agent_coordinator
    
Check:
  For each source:
    if (now - last_heartbeat) > timeout:
      trigger alert based on priority
```

### 3. State Validation

```
Validate estimated state:
  Position:
    - Within geofence (redundant check)
    - Not NaN or Inf
    - Reasonable magnitude (< 10km from home)
    
  Velocity:
    - Below max_velocity limit
    - Not NaN or Inf
    - Rate of change reasonable
    
  Attitude:
    - Tilt angle < max_tilt
    - Quaternion normalized
    - Angular rates reasonable
    
  Consistency:
    - Velocity matches position derivative (roughly)
    - Attitude matches angular velocity integration
```

### 4. Flight Envelope Protection

```
Limits:
  max_velocity_mps: 10.0
  max_altitude_m: 120.0
  min_altitude_m: 0.5      # AGL
  max_tilt_rad: 0.7        # ~40 degrees
  max_angular_rate_radps: 3.0

On violation:
  - Publish alert
  - Recommend corrective action to uav_manager
```

### 5. Battery Monitoring

```
Battery states:
  OK:       > warning_threshold
  WARNING:  > critical_threshold
  CRITICAL: > emergency_threshold
  EMERGENCY: ≤ emergency_threshold

Actions:
  WARNING:   Alert, recommend RTH
  CRITICAL:  Alert, command RTH
  EMERGENCY: Command immediate land
```

### 6. Collision Monitoring

```
For each neighbor UAV:
  distance = ||own_position - neighbor_position||
  
  if distance < collision_critical_m:
    trigger COLLISION_IMMINENT, recommend STOP
  elif distance < collision_warning_m:
    trigger COLLISION_WARNING
```

---

## Safety Status State Machine

```
┌──────────┐  violation   ┌──────────┐  persists   ┌──────────┐
│ NOMINAL  │────────────▶│ WARNING  │────────────▶│ CAUTION  │
└────┬─────┘  detected    └────┬─────┘             └────┬─────┘
     ▲                         │                        │
     │ cleared                 │ cleared               │ critical
     │                         ▼                        ▼
     │                    ┌──────────┐            ┌──────────┐
     └────────────────────│  (back)  │            │EMERGENCY │
                          └──────────┘            └────┬─────┘
                                                       │
                                                       │ manual reset
                                                       ▼
                                                  ┌──────────┐
                                                  │ NOMINAL  │
                                                  └──────────┘
```

---

## Configuration

```yaml
safety_monitor:
  ros__parameters:
    # Monitoring rates
    check_rate_hz: 100.0
    publish_rate_hz: 10.0
    
    # Geofence (cylindrical)
    geofence:
      type: "cylindrical"          # or "polygonal"
      max_radius_m: 100.0
      max_altitude_m: 50.0
      min_altitude_m: 0.5
      warning_buffer_m: 10.0
      critical_buffer_m: 5.0
      violation_action: "land"     # warn, hold, return_home, land
    
    # Heartbeat timeouts
    heartbeats:
      hardware_abstraction:
        timeout_s: 1.0
        priority: "critical"
      estimator_manager:
        timeout_s: 0.5
        priority: "critical"
      controller_manager:
        timeout_s: 0.5
        priority: "high"
      gcs:
        timeout_s: 5.0
        priority: "high"
    
    # Flight envelope
    envelope:
      max_velocity_mps: 10.0
      max_tilt_rad: 0.7
      max_angular_rate_radps: 3.0
    
    # Battery
    battery:
      warning_percent: 30.0
      critical_percent: 20.0
      emergency_percent: 10.0
    
    # Collision (inter-UAV)
    collision:
      warning_distance_m: 5.0
      critical_distance_m: 2.0
```

---

## Recommended Actions

| Alert | Severity | Recommended Action |
|-------|----------|-------------------|
| Geofence warning | WARNING | Notify, no action |
| Geofence violation | EMERGENCY | Land or RTH |
| PX4 heartbeat loss | EMERGENCY | Last-resort hover, then land |
| Estimator failure | EMERGENCY | Hover if possible, land |
| Controller failure | CAUTION | Switch to backup controller |
| Battery warning | WARNING | RTH recommended |
| Battery critical | CAUTION | RTH commanded |
| Battery emergency | EMERGENCY | Immediate land |
| Collision warning | WARNING | Notify, slow down |
| Collision imminent | EMERGENCY | Emergency stop |
| Envelope violation | CAUTION | Limit output, warn |

---

## Major Cautions

| Issue | Mitigation |
|-------|------------|
| **False positives** | Implement debouncing, don't trigger on single bad reading |
| **Missed alerts** | Run at high rate, use watchdog on self |
| **Blocking operations** | Never block in safety checks, use async |
| **Alert storms** | Rate-limit repeated alerts, aggregate |
| **Recovery deadlock** | Clear path from any state to safe state |
| **Time synchronization** | Use consistent time source for all checks |

---

## File Structure

```
safety_monitor/
├── include/safety_monitor/
│   ├── safety_monitor_node.hpp
│   ├── geofence.hpp
│   ├── heartbeat_monitor.hpp
│   ├── state_validator.hpp
│   └── envelope_checker.hpp
├── src/
│   ├── safety_monitor_node.cpp
│   ├── geofence.cpp
│   ├── heartbeat_monitor.cpp
│   ├── state_validator.cpp
│   ├── envelope_checker.cpp
│   └── main.cpp
├── config/
│   └── safety_monitor.yaml
└── launch/
    └── safety_monitor.launch.py
```
