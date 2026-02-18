# multi_agent_coordinator

**Package Type:** ROS2 Node Package  
**Dependencies:** peregrine_interfaces, rclcpp, Eigen3  

---

## Overview

`multi_agent_coordinator` handles **multi-UAV coordination** including collision avoidance, shared reference frames, and fleet-level awareness. Currently implements centralized coordination with pointers for future decentralized operation.

---

## Responsibilities

1. **State sharing** - broadcast own state, receive neighbor states
2. **Collision avoidance** - compute Buffered Voronoi Cells (BVC)
3. **Constraint publishing** - send avoidance constraints to trajectory_manager
4. **Reference frame management** - ensure shared GPS datum or MoCap origin
5. **Fleet awareness** - track all UAVs, detect failures
6. **Future: consensus** - distributed decision making (decentralized mode)

---

## Architecture

### Centralized Mode (Current)

```
                    ┌─────────────────────────────┐
                    │    Ground Control Station   │
                    │   (optional - monitoring)   │
                    └──────────────┬──────────────┘
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
        ▼                          ▼                          ▼
┌───────────────┐          ┌───────────────┐          ┌───────────────┐
│    UAV 1      │          │    UAV 2      │          │    UAV N      │
│               │          │               │          │               │
│ multi_agent_  │◀────────▶│ multi_agent_  │◀────────▶│ multi_agent_  │
│ coordinator   │  state   │ coordinator   │  state   │ coordinator   │
│               │  sharing │               │  sharing │               │
└───────────────┘          └───────────────┘          └───────────────┘
```

### Topic Structure

```
Each UAV publishes:
  /uavN/multi_agent/own_state        → Own position/velocity for others

Each UAV subscribes:
  /uav*/multi_agent/own_state        → All other UAV states (wildcard)

Each UAV publishes internally:
  /uavN/multi_agent/fleet_states     → Aggregated fleet state
  /uavN/multi_agent/collision_constraint → BVC constraints for trajectory_manager
```

---

## Buffered Voronoi Cell (BVC) Algorithm

### Concept

```
Voronoi cell: Region closer to self than any neighbor
Buffered Voronoi: Voronoi cell shrunk by safety buffer

      ┌─────────────────────────────────────┐
      │           Workspace                  │
      │                                      │
      │     UAV1 ◇─────────┼─────────◇ UAV2 │
      │           ╲   BVC1 │ BVC2   ╱       │
      │            ╲       │       ╱        │
      │             ╲      │      ╱         │
      │              ╲     │     ╱          │
      │               ╲    │    ╱           │
      │    Voronoi     ╲   │   ╱  Voronoi   │
      │    boundary     ───┼───   boundary  │
      │                    │                │
      └─────────────────────────────────────┘

If UAV1 stays in BVC1 and UAV2 stays in BVC2, 
they maintain at least 2*buffer separation.
```

### Algorithm

```
compute_bvc(own_position, neighbor_positions, safety_buffer):
  
  constraints = []
  
  for each neighbor in neighbor_positions:
    # Compute separating hyperplane
    midpoint = (own_position + neighbor) / 2
    normal = normalize(own_position - neighbor)
    
    # Buffer the constraint inward
    buffered_point = midpoint + normal * safety_buffer
    
    # Half-plane constraint: normal · (x - buffered_point) ≤ 0
    # Equivalently: normal · x ≤ normal · buffered_point
    constraint = HalfPlaneConstraint(
      normal = normal,
      offset = dot(normal, buffered_point)
    )
    constraints.append(constraint)
  
  return constraints
```

### Constraint Application

```
In trajectory_manager:
  
  For each setpoint:
    for each constraint in collision_constraints:
      if constraint.violates(setpoint.position):
        # Project setpoint onto constraint boundary
        setpoint.position = project_onto_halfplane(
          setpoint.position, constraint)
```

---

## Implementation Details

### State Broadcasting

```
Broadcast rate: 20 Hz (configurable)

Broadcast message contains:
  - UAV ID
  - Timestamp
  - Position (ENU)
  - Velocity (ENU)
  - Current mode (for behavior prediction)
  - Trajectory intent (optional, for cooperative planning)
```

### Neighbor Tracking

```
Neighbor data structure:
  - Last received state
  - Last update timestamp
  - Estimated current position (with prediction)
  - Health status (alive/stale/lost)

Neighbor timeout handling:
  if (now - last_update) > stale_threshold:
    mark as STALE, increase safety buffer
  if (now - last_update) > lost_threshold:
    mark as LOST, assume worst-case position
```

### Velocity-Based Prediction

```
For more accurate collision avoidance:

predicted_position(neighbor, dt):
  return neighbor.position + neighbor.velocity * dt

Use predicted positions when computing BVC
for the planning horizon.
```

### Reference Frame Management

```
GPS Environment:
  - All UAVs use same GPS datum (configured)
  - Datum: first UAV's home position OR pre-configured
  - Local ENU computed from shared datum

MoCap Environment:
  - All UAVs share MoCap origin
  - MoCap system defines world frame
  - No additional alignment needed
```

---

## Configuration

```yaml
multi_agent_coordinator:
  ros__parameters:
    # This UAV's ID
    uav_id: "uav1"
    
    # Fleet configuration
    fleet_size: 4
    fleet_uav_ids: ["uav1", "uav2", "uav3", "uav4"]
    
    # State sharing
    broadcast_rate_hz: 20.0
    
    # BVC parameters
    safety_buffer_m: 2.0           # Minimum separation / 2
    planning_horizon_s: 2.0        # Prediction horizon
    
    # Neighbor tracking
    neighbor_stale_timeout_s: 0.5
    neighbor_lost_timeout_s: 2.0
    stale_buffer_multiplier: 1.5   # Increase buffer for stale neighbors
    
    # Constraint publishing
    constraint_rate_hz: 20.0
    
    # Reference frame
    environment: "gps"             # or "mocap"
    gps_datum:                     # For GPS environment
      latitude: 47.397742
      longitude: 8.545594
      altitude: 488.0
    
    # QoS for inter-UAV communication
    qos_reliability: "best_effort"
    qos_history_depth: 5
```

---

## Future: Decentralized Mode Pointers

### Key Changes Needed

1. **Remove GCS dependency**
   - Each UAV makes local decisions
   - No single point of failure

2. **Peer Discovery**
   - Use ROS2 DDS discovery mechanism
   - Or implement heartbeat-based discovery

3. **Consensus Protocols**
   - Leader election (RAFT, Bully)
   - Distributed task allocation
   - Formation consensus

4. **Network Topology**
   - Define neighbor relationships (not all-to-all)
   - Handle partial connectivity
   - Gossip protocols for state dissemination

5. **Partition Tolerance**
   - Continue operation if subgroups disconnect
   - Reconvergence when connectivity restored

### Suggested Package Additions

```
multi_agent_coordinator/
├── src/
│   ├── centralized/           # Current implementation
│   │   └── centralized_coordinator.cpp
│   └── decentralized/         # Future implementation
│       ├── peer_discovery.cpp
│       ├── consensus.cpp
│       └── decentralized_coordinator.cpp
```

---

## Major Cautions

| Issue | Mitigation |
|-------|------------|
| **Network latency** | Use prediction, handle stale data gracefully |
| **Packet loss** | Design for best-effort, use redundant broadcasts |
| **ID conflicts** | Validate unique IDs at startup |
| **Frame misalignment** | Verify shared reference before flight |
| **Constraint infeasibility** | Detect when BVC is empty, emergency stop |
| **Deadlock** | Implement priority-based conflict resolution |

---

## File Structure

```
multi_agent_coordinator/
├── include/multi_agent_coordinator/
│   ├── multi_agent_coordinator_node.hpp
│   ├── bvc_calculator.hpp
│   ├── neighbor_tracker.hpp
│   └── reference_frame_manager.hpp
├── src/
│   ├── multi_agent_coordinator_node.cpp
│   ├── bvc_calculator.cpp
│   ├── neighbor_tracker.cpp
│   ├── reference_frame_manager.cpp
│   └── main.cpp
├── config/
│   └── multi_agent_coordinator.yaml
└── launch/
    └── multi_agent_coordinator.launch.py
```
