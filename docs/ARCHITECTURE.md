# PEREGRINE Flight Stack - System Architecture Document

**Version:** 1.0  
**Target Platform:** ROS2 Humble/Jazzy, PX4 v1.15+, Jetson Orin Series  
**Language:** C++17  

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Design Philosophy](#2-design-philosophy)
3. [System Overview](#3-system-overview)
4. [Package Architecture](#4-package-architecture)
5. [Data Flow Architecture](#5-data-flow-architecture)
6. [Multi-Agent Architecture](#6-multi-agent-architecture)
7. [Frame Convention Strategy](#7-frame-convention-strategy)
8. [Plugin Architecture](#8-plugin-architecture)
9. [Safety Architecture](#9-safety-architecture)
10. [Operational Scenario: 4-UAV Autonomous Mission](#10-operational-scenario-4-uav-autonomous-mission)
11. [Package Dependency Graph](#11-package-dependency-graph)
12. [Future Considerations](#12-future-considerations)

---

## 1. Executive Summary

PEREGRINE (Aerial Robotics Infrastructure for Operational Navigation) is a ROS2-based flight stack designed for multi-agent UAV operations with PX4 autopilots. The stack prioritizes:

- **Manager-based architecture**: Each functional domain has a dedicated manager node
- **Plugin flexibility**: Controllers, estimators, and trajectory generators are runtime-loadable
- **Research-friendly design**: Easy to swap algorithms without modifying core infrastructure
- **Safety-first approach**: Multiple layers of safety monitoring with graceful degradation
- **Multi-environment support**: Seamless operation in GPS (outdoor) and MoCap (indoor) environments

### Core Principles

1. **Separation of Concerns**: Each package handles one domain; inter-package communication via well-defined ROS2 interfaces
2. **Standard Robotics Frames**: All internal computation in ENU/FLU; conversion to PX4's NED/FRD only at hardware boundary
3. **Fail-Safe by Default**: System defaults to safe states; explicit action required to enter risky modes
4. **Deterministic Behavior**: Real-time capable design suitable for Jetson deployment

---

## 2. Design Philosophy

### 2.1 Manager-Based Architecture

Unlike monolithic designs, PEREGRINE uses dedicated manager nodes that orchestrate their domains:

```
┌─────────────────────────────────────────────────────────────────┐
│                      MISSION LAYER (Future)                      │
│                   (Behavior Trees / Mission Executor)            │
└─────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                         UAV MANAGER                              │
│  • Lifecycle orchestration    • Mode management                  │
│  • State machine              • Inter-manager coordination       │
└─────────────────────────────────────────────────────────────────┘
         │              │              │              │
         ▼              ▼              ▼              ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│  CONTROLLER  │ │  ESTIMATOR   │ │  TRAJECTORY  │ │   SAFETY     │
│   MANAGER    │ │   MANAGER    │ │   MANAGER    │ │   MONITOR    │
└──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘
         │              │              │              
         ▼              ▼              ▼              
┌─────────────────────────────────────────────────────────────────┐
│                    HARDWARE ABSTRACTION                          │
│  • PX4 uXRCE-DDS Interface    • Frame Transforms                │
└─────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
                         ┌──────────────┐
                         │     PX4      │
                         │  Autopilot   │
                         └──────────────┘
```

### 2.2 Why Managers?

| Aspect | Traditional | Manager-Based |
|--------|-------------|---------------|
| Adding new controller | Modify core code | Load new plugin, no core changes |
| Runtime switching | Often impossible | Service call to manager |
| Testing | Full system required | Mock manager interfaces |
| Multi-UAV | Namespace everything | Each UAV has its manager set |

### 2.3 Inspirations

- **CTU-MRS UAV System**: Manager architecture, tracker/controller separation, constraint management
- **Aerostack2**: Platform independence, behavior-based mission control, standard interfaces
- **CMU AirStack**: Modular autonomy layers, simulation integration, multi-robot focus
- **PX4-ROS2-Interface-Lib**: Clean uXRCE-DDS patterns, frame conversion utilities

---

## 3. System Overview

### 3.1 Package List

| Package | Purpose | Key Responsibility |
|---------|---------|-------------------|
| `peregrine_interfaces` | Message/Service definitions | Central interface definitions |
| `hardware_abstraction` | PX4 communication | uXRCE-DDS bridge, frame conversion at boundary |
| `frame_transforms` | Coordinate transforms | ENU↔NED, FLU↔FRD, TF2 broadcasting |
| `estimator_manager` | State estimation | Plugin loading, estimator switching, state publishing |
| `controller_manager` | Flight control | Plugin loading, controller switching, setpoint routing |
| `trajectory_manager` | Trajectory handling | Plugin loading, trajectory serving, progress tracking |
| `uav_manager` | UAV orchestration | Lifecycle, mode FSM, takeoff/land sequences |
| `safety_monitor` | Safety systems | Geofencing, heartbeat, emergency handling |
| `multi_agent_coordinator` | Fleet coordination | Collision avoidance, shared reference frames |
| `tui_status` | Terminal interface | ncurses-based monitoring display |
| `peregrine_bringup` | Launch infrastructure | Launch files, configurations, simulation setup |

### 3.2 Technology Stack

```
┌────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│  C++17 │ ROS2 Humble/Jazzy │ pluginlib │ BehaviorTree.CPP  │
├────────────────────────────────────────────────────────────┤
│                    Middleware Layer                         │
│  DDS (Cyclone/FastDDS) │ uXRCE-DDS │ TF2 │ lifecycle_msgs  │
├────────────────────────────────────────────────────────────┤
│                    Hardware Layer                           │
│  PX4 Autopilot │ Jetson Orin │ GPS/MoCap │ Sensors         │
├────────────────────────────────────────────────────────────┤
│                    Simulation Layer                         │
│  Gazebo Harmonic │ PX4 SITL │ gz-ros2-bridge               │
└────────────────────────────────────────────────────────────┘
```

---

## 4. Package Architecture

### 4.1 Layered View

```
Layer 4: Coordination    ┌─────────────────────────────────┐
                         │    multi_agent_coordinator       │
                         └─────────────────────────────────┘
                                        │
Layer 3: Orchestration   ┌─────────────────────────────────┐
                         │         uav_manager              │
                         └─────────────────────────────────┘
                                        │
                         ┌──────────┬───┴───┬──────────┐
Layer 2: Functional      │          │       │          │
                    ┌────┴────┐ ┌───┴───┐ ┌─┴────┐ ┌───┴────┐
                    │estimator│ │control│ │traj  │ │safety  │
                    │_manager │ │_manager│ │_mgr  │ │_monitor│
                    └────┬────┘ └───┬───┘ └─┬────┘ └───┬────┘
                         │          │       │          │
Layer 1: Infrastructure  └──────────┴───┬───┴──────────┘
                         ┌──────────────┴──────────────┐
                         │     hardware_abstraction     │
                         │      frame_transforms        │
                         └──────────────┬──────────────┘
                                        │
Layer 0: External        ┌──────────────┴──────────────┐
                         │     PX4 (via uXRCE-DDS)      │
                         └─────────────────────────────┘
```

### 4.2 Node Communication Matrix

| Publisher → Subscriber | Topic/Service | Message Type | Rate |
|------------------------|---------------|--------------|------|
| hardware_abstraction → estimator_manager | `/px4_state` | `VehicleOdometry` | 250Hz |
| estimator_manager → controller_manager | `/estimated_state` | `peregrine_interfaces/State` | 250Hz |
| trajectory_manager → controller_manager | `/trajectory_setpoint` | `peregrine_interfaces/TrajectorySetpoint` | 50Hz |
| controller_manager → hardware_abstraction | `/control_output` | `peregrine_interfaces/ControlOutput` | 250Hz |
| safety_monitor → uav_manager | `/safety_status` | `peregrine_interfaces/SafetyStatus` | 10Hz |
| uav_manager → all managers | `/uav_mode` | `peregrine_interfaces/UAVMode` | 10Hz |
| multi_agent_coordinator → trajectory_manager | `/collision_constraint` | `peregrine_interfaces/CollisionConstraint` | 20Hz |

---

## 5. Data Flow Architecture

### 5.1 Control Loop Data Flow

```
                                    ┌─────────────────┐
                                    │  Mission/User   │
                                    │   Commands      │
                                    └────────┬────────┘
                                             │
                                             ▼
┌──────────────────────────────────────────────────────────────────┐
│                        UAV MANAGER                                │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐          │
│  │   IDLE      │───▶│   ARMED     │───▶│  TAKEOFF    │──┐       │
│  └─────────────┘    └─────────────┘    └─────────────┘  │       │
│        ▲                                                 │       │
│        │            ┌─────────────┐    ┌─────────────┐  │       │
│        └────────────│   LANDED    │◀───│   LANDING   │◀─┤       │
│                     └─────────────┘    └─────────────┘  │       │
│                                        ┌─────────────┐  │       │
│                                        │   FLYING    │◀─┘       │
│                                        └─────────────┘          │
└──────────────────────────────────────────────────────────────────┘
                                             │
              ┌──────────────────────────────┼──────────────────────────────┐
              │                              │                              │
              ▼                              ▼                              ▼
┌─────────────────────────┐   ┌─────────────────────────┐   ┌─────────────────────────┐
│   TRAJECTORY MANAGER    │   │   CONTROLLER MANAGER    │   │   ESTIMATOR MANAGER     │
│                         │   │                         │   │                         │
│  ┌───────────────────┐  │   │  ┌───────────────────┐  │   │  ┌───────────────────┐  │
│  │ Active Trajectory │  │   │  │ Active Controller │  │   │  │ Active Estimator  │  │
│  │     Plugin        │  │   │  │     Plugin        │  │   │  │     Plugin        │  │
│  └─────────┬─────────┘  │   │  └─────────┬─────────┘  │   │  └─────────┬─────────┘  │
│            │            │   │            │            │   │            │            │
│  Reference │            │   │  Control   │            │   │   State    │            │
│  Setpoints │            │   │  Commands  │            │   │  Estimate  │            │
└────────────┼────────────┘   └────────────┼────────────┘   └────────────┼────────────┘
             │                             │                             │
             │     ┌───────────────────────┼─────────────────────────────┘
             │     │                       │
             ▼     ▼                       ▼
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                            HARDWARE ABSTRACTION                                      │
│                                                                                      │
│   ┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐            │
│   │  Frame Convert  │      │  PX4 Publisher  │      │  PX4 Subscriber │            │
│   │   ENU → NED     │─────▶│  /fmu/in/*      │      │  /fmu/out/*     │────┐       │
│   │   FLU → FRD     │      └─────────────────┘      └─────────────────┘    │       │
│   └─────────────────┘                                                       │       │
│                                                                             │       │
│   ┌─────────────────────────────────────────────────────────────────────────┘       │
│   │  Frame Convert                                                                  │
│   │   NED → ENU                                                                     │
│   │   FRD → FLU                                                                     │
│   └─────────────────                                                                │
└─────────────────────────────────────────────────────────────────────────────────────┘
                                             │
                                             ▼
                                    ┌─────────────────┐
                                    │       PX4       │
                                    │   (uXRCE-DDS)   │
                                    └─────────────────┘
```

### 5.2 Timing Requirements

| Loop | Frequency | Deadline | Priority |
|------|-----------|----------|----------|
| State Estimation | 250 Hz | 4 ms | SCHED_FIFO 90 |
| Control Output | 250 Hz | 4 ms | SCHED_FIFO 89 |
| Trajectory Tracking | 50 Hz | 20 ms | SCHED_FIFO 80 |
| Safety Monitor | 100 Hz | 10 ms | SCHED_FIFO 95 |
| UAV Manager | 50 Hz | 20 ms | SCHED_RR 70 |
| TUI Update | 10 Hz | 100 ms | SCHED_OTHER |

---

## 6. Multi-Agent Architecture

### 6.1 Centralized Coordination (Current)

```
                         ┌─────────────────────────────────┐
                         │      GROUND CONTROL STATION     │
                         │                                 │
                         │  ┌─────────────────────────┐   │
                         │  │  Mission Planner        │   │
                         │  │  Fleet Monitor          │   │
                         │  │  Emergency Override     │   │
                         │  └─────────────────────────┘   │
                         └───────────────┬─────────────────┘
                                         │
                    ┌────────────────────┼────────────────────┐
                    │                    │                    │
                    ▼                    ▼                    ▼
         ┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
         │  UAV 1 Namespace  │ │  UAV 2 Namespace  │ │  UAV N Namespace  │
         │  /uav1/...        │ │  /uav2/...        │ │  /uavN/...        │
         │                   │ │                   │ │                   │
         │ ┌──────────────┐  │ │ ┌──────────────┐  │ │ ┌──────────────┐  │
         │ │multi_agent_  │  │ │ │multi_agent_  │  │ │ │multi_agent_  │  │
         │ │coordinator   │◀─┼─┼─│coordinator   │◀─┼─┼─│coordinator   │  │
         │ └──────────────┘  │ │ └──────────────┘  │ │ └──────────────┘  │
         │        │          │ │        │          │ │        │          │
         │        ▼          │ │        ▼          │ │        ▼          │
         │ ┌──────────────┐  │ │ ┌──────────────┐  │ │ ┌──────────────┐  │
         │ │  uav_manager │  │ │ │  uav_manager │  │ │ │  uav_manager │  │
         │ └──────────────┘  │ │ └──────────────┘  │ │ └──────────────┘  │
         │        │          │ │        │          │ │        │          │
         │       ...         │ │       ...         │ │       ...         │
         └──────────────────┘ └──────────────────┘ └──────────────────┘
```

### 6.2 Namespace Convention

```
/uav{ID}/                           # UAV namespace root
├── hardware_abstraction/
│   ├── px4_state                   # Raw PX4 state (NED, for debugging)
│   └── px4_command                 # Raw PX4 commands (NED, for debugging)
├── estimator_manager/
│   ├── state                       # Estimated state (ENU/FLU)
│   ├── set_estimator              # Service to switch estimator
│   └── estimator_status           # Active estimator info
├── controller_manager/
│   ├── control_output             # Control commands (ENU/FLU)
│   ├── set_controller             # Service to switch controller
│   └── controller_status          # Active controller info
├── trajectory_manager/
│   ├── trajectory                 # Current trajectory
│   ├── trajectory_setpoint        # Current setpoint on trajectory
│   ├── set_trajectory             # Action to set new trajectory
│   └── trajectory_status          # Progress, completion status
├── uav_manager/
│   ├── mode                       # Current UAV mode
│   ├── arm                        # Service to arm
│   ├── takeoff                    # Action to takeoff
│   ├── land                       # Action to land
│   └── status                     # Comprehensive UAV status
├── safety_monitor/
│   ├── safety_status              # Current safety state
│   ├── geofence_status            # Geofence violations
│   └── emergency_stop             # Service for emergency stop
└── multi_agent/
    ├── own_state                  # This UAV's state for others
    ├── fleet_states               # All UAV states
    └── collision_constraints      # Computed avoidance constraints
```

### 6.3 Collision Avoidance: Buffered Voronoi Cells

The `multi_agent_coordinator` implements Buffered Voronoi Cell (BVC) based collision avoidance:

```
Algorithm: Buffered Voronoi Collision Avoidance
═══════════════════════════════════════════════

Input: 
  - Own position p_i
  - Neighbor positions {p_j} for j ≠ i  
  - Safety buffer d_safe
  - Planning horizon T

Output:
  - Collision-free position constraint region R_i

1. Compute Voronoi cell V_i for agent i:
   V_i = {x : ||x - p_i|| ≤ ||x - p_j|| ∀j ≠ i}

2. Buffer the cell inward by d_safe:
   BVC_i = {x ∈ V_i : dist(x, ∂V_i) ≥ d_safe}

3. Intersect with velocity constraints:
   R_i = BVC_i ∩ {x : ||x - p_i|| ≤ v_max * T}

4. Publish R_i as LinearConstraint to trajectory_manager

The trajectory_manager then ensures generated setpoints 
remain within R_i at all times.
```

### 6.4 Future: Decentralized Architecture Pointers

When transitioning to decentralized operation:

1. **Replace GCS dependency**: Each UAV runs local decision making
2. **Peer discovery**: Use ROS2 DDS discovery or implement custom heartbeat
3. **Consensus protocols**: Add package for distributed consensus (e.g., RAFT for leader election)
4. **Communication graph**: Define neighbor relationships, not all-to-all
5. **Partition tolerance**: System continues if subgroups lose connectivity

---

## 7. Frame Convention Strategy

### 7.1 The Frame Problem

```
PX4 World (NED)              ROS World (ENU)
     North (+X)                   East (+X)
        │                            │
        │                            │
        │                            │
        └──────▶ East (+Y)           └──────▶ North (+Y)
       ╱                            ╱
      ╱                            ╱
     ▼                            ▼
   Down (+Z)                    Up (+Z)


PX4 Body (FRD)               ROS Body (FLU)
   Forward (+X)                Forward (+X)
        │                            │
        │                            │
        │                            │
        └──────▶ Right (+Y)          └──────▶ Left (+Y)
       ╱                            ╱
      ╱                            ╱
     ▼                            ▼
   Down (+Z)                    Up (+Z)
```

### 7.2 Conversion Strategy

**Golden Rule**: All PEREGRINE packages work in ENU/FLU. Conversion happens ONLY in `hardware_abstraction`.

```cpp
// frame_transforms/include/frame_transforms/conversions.hpp

namespace peregrine::frame_transforms {

// Position: ENU ↔ NED
// x_ned =  y_enu
// y_ned =  x_enu  
// z_ned = -z_enu

inline Eigen::Vector3d enu_to_ned(const Eigen::Vector3d& enu) {
    return Eigen::Vector3d(enu.y(), enu.x(), -enu.z());
}

inline Eigen::Vector3d ned_to_enu(const Eigen::Vector3d& ned) {
    return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
}

// Velocity: Same transformation as position
inline Eigen::Vector3d velocity_enu_to_ned(const Eigen::Vector3d& v_enu) {
    return enu_to_ned(v_enu);
}

// Quaternion: ENU ↔ NED
// Requires rotation: 180° about X, then 90° about Z
inline Eigen::Quaterniond quaternion_enu_to_ned(const Eigen::Quaterniond& q_enu) {
    // R_ned = R_z(π/2) * R_x(π) * R_enu
    static const Eigen::Quaterniond q_transform(
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
    );
    return q_transform * q_enu;
}

// Body frame: FLU ↔ FRD  
// x_frd =  x_flu
// y_frd = -y_flu
// z_frd = -z_flu

inline Eigen::Vector3d flu_to_frd(const Eigen::Vector3d& flu) {
    return Eigen::Vector3d(flu.x(), -flu.y(), -flu.z());
}

}  // namespace peregrine::frame_transforms
```

### 7.3 TF2 Frame Tree

```
world (ENU)                      # Global reference frame
├── map (ENU)                    # Map frame (if mapping enabled)
│   └── odom (ENU)               # Odometry frame
│       └── base_link (FLU)      # UAV body frame
│           ├── imu_link         # IMU frame
│           ├── camera_link      # Camera frame
│           └── lidar_link       # LiDAR frame
│
├── uav1/base_link               # UAV 1 body in world
├── uav2/base_link               # UAV 2 body in world
└── uavN/base_link               # UAV N body in world
```

---

## 8. Plugin Architecture

### 8.1 Plugin System Overview

PEREGRINE uses ROS2 `pluginlib` for runtime-loadable algorithms:

```
┌─────────────────────────────────────────────────────────────┐
│                        MANAGER NODE                          │
│                                                              │
│   ┌─────────────────────────────────────────────────────┐   │
│   │              Plugin Loader (pluginlib)               │   │
│   └─────────────────────────────────────────────────────┘   │
│                            │                                 │
│          ┌─────────────────┼─────────────────┐              │
│          │                 │                 │              │
│          ▼                 ▼                 ▼              │
│   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐      │
│   │  Plugin A   │   │  Plugin B   │   │  Plugin C   │      │
│   │ (default)   │   │ (research)  │   │ (custom)    │      │
│   └─────────────┘   └─────────────┘   └─────────────┘      │
│          │                 │                 │              │
│          └─────────────────┼─────────────────┘              │
│                            ▼                                 │
│   ┌─────────────────────────────────────────────────────┐   │
│   │            Base Interface (Abstract Class)           │   │
│   └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 8.2 Controller Plugin Interface

```cpp
// controller_manager/include/controller_manager/controller_base.hpp

namespace peregrine::controller {

class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    
    // Lifecycle
    virtual bool initialize(
        const rclcpp::Node::SharedPtr& node,
        const std::string& name) = 0;
    virtual bool activate() = 0;
    virtual bool deactivate() = 0;
    
    // Core computation
    virtual ControlOutput compute(
        const State& current_state,
        const TrajectorySetpoint& reference,
        const double dt) = 0;
    
    // Information
    virtual std::string getName() const = 0;
    virtual ControlMode getOutputMode() const = 0;  // POSITION, VELOCITY, ATTITUDE, RATE, THRUST
    
    // Configuration
    virtual void setParameters(const ParameterMap& params) = 0;
    virtual ParameterMap getParameters() const = 0;
    
protected:
    rclcpp::Node::SharedPtr node_;
    std::string name_;
};

// Control output modes - determines what hardware_abstraction sends to PX4
enum class ControlMode {
    POSITION,           // Send position setpoint to PX4
    VELOCITY,           // Send velocity setpoint to PX4
    ATTITUDE,           // Send attitude + thrust to PX4
    BODY_RATE,          // Send body rates + thrust to PX4
    DIRECT_ALLOCATION   // Send motor commands (future)
};

}  // namespace peregrine::controller
```

### 8.3 Estimator Plugin Interface

```cpp
// estimator_manager/include/estimator_manager/estimator_base.hpp

namespace peregrine::estimator {

class EstimatorBase {
public:
    virtual ~EstimatorBase() = default;
    
    // Lifecycle
    virtual bool initialize(
        const rclcpp::Node::SharedPtr& node,
        const std::string& name) = 0;
    virtual bool activate() = 0;
    virtual bool deactivate() = 0;
    
    // Core computation
    virtual State estimate(const SensorData& sensors, const double dt) = 0;
    
    // State injection (for external estimates like MoCap)
    virtual void injectPose(const geometry_msgs::msg::PoseStamped& pose) = 0;
    
    // Information
    virtual std::string getName() const = 0;
    virtual EstimatorType getType() const = 0;  // PASSTHROUGH, EKF, UKF, etc.
    
    // Health
    virtual bool isHealthy() const = 0;
    virtual double getConfidence() const = 0;  // 0.0 - 1.0
    
protected:
    rclcpp::Node::SharedPtr node_;
    std::string name_;
};

enum class EstimatorType {
    PASSTHROUGH,    // Just forward PX4's estimate
    EKF,            // Extended Kalman Filter
    UKF,            // Unscented Kalman Filter
    COMPLEMENTARY,  // Complementary filter
    CUSTOM          // Research implementations
};

}  // namespace peregrine::estimator
```

### 8.4 Trajectory Generator Plugin Interface

```cpp
// trajectory_manager/include/trajectory_manager/trajectory_generator_base.hpp

namespace peregrine::trajectory {

class TrajectoryGeneratorBase {
public:
    virtual ~TrajectoryGeneratorBase() = default;
    
    // Lifecycle
    virtual bool initialize(
        const rclcpp::Node::SharedPtr& node,
        const std::string& name) = 0;
    
    // Generation
    virtual Trajectory generate(const TrajectoryRequest& request) = 0;
    
    // Evaluation - sample trajectory at time t
    virtual TrajectorySetpoint evaluate(
        const Trajectory& trajectory,
        const double t) const = 0;
    
    // Information
    virtual std::string getName() const = 0;
    virtual TrajectoryType getType() const = 0;
    virtual SetpointType getOutputType() const = 0;  // What the trajectory provides
    
protected:
    rclcpp::Node::SharedPtr node_;
    std::string name_;
};

enum class TrajectoryType {
    WAYPOINT_LINEAR,     // Linear interpolation between waypoints
    POLYNOMIAL,          // Minimum snap/jerk polynomial
    BSPLINE,             // B-spline trajectory
    BEZIER,              // Bezier curve
    TIME_OPTIMAL,        // Time-optimal trajectory
    CUSTOM               // Research implementations
};

enum class SetpointType {
    POSITION_ONLY,       // x, y, z
    POSITION_YAW,        // x, y, z, yaw
    FULL_STATE,          // pos, vel, acc, jerk, yaw, yaw_rate
};

}  // namespace peregrine::trajectory
```

---

## 9. Safety Architecture

### 9.1 Safety Layers

```
┌─────────────────────────────────────────────────────────────────────┐
│                    SAFETY LAYER HIERARCHY                            │
│                                                                      │
│   Layer 4: PX4 Hardware Failsafes (ULTIMATE BACKUP)                 │
│   ─────────────────────────────────────────────────                 │
│   • RC loss failsafe           • Geofence (PX4 level)               │
│   • Battery critical           • Data link loss                      │
│   • Position loss              • Motor failure detection             │
│                                                                      │
│   Layer 3: PEREGRINE Safety Monitor                                     │
│   ─────────────────────────────────                                 │
│   • Software geofencing        • Heartbeat monitoring                │
│   • Trajectory bounds check    • Inter-UAV collision                 │
│   • State estimation health    • Emergency land trigger              │
│                                                                      │
│   Layer 2: Manager-Level Safety                                      │
│   ─────────────────────────────                                     │
│   • Controller output limits   • Trajectory feasibility              │
│   • Estimator confidence       • Mode transition guards              │
│                                                                      │
│   Layer 1: Component-Level Safety                                    │
│   ─────────────────────────────                                     │
│   • Input validation           • Output saturation                   │
│   • Watchdog timers            • Sanity checks                       │
└─────────────────────────────────────────────────────────────────────┘
```

### 9.2 Safety Monitor State Machine

```
                              ┌─────────────┐
                              │             │
            ┌────────────────▶│   NOMINAL   │◀────────────────┐
            │                 │             │                 │
            │                 └──────┬──────┘                 │
            │                        │                        │
            │              violation │ detected               │
            │                        ▼                        │
            │                 ┌─────────────┐                 │
            │    cleared      │             │                 │
            └─────────────────│   WARNING   │                 │
                              │             │                 │
                              └──────┬──────┘                 │
                                     │                        │
                           violation │ persists               │ manual
                                     ▼                        │ reset
                              ┌─────────────┐                 │
                              │             │                 │
                              │   CAUTION   │─────────────────┤
                              │             │                 │
                              └──────┬──────┘                 │
                                     │                        │
                           critical  │ violation              │
                                     ▼                        │
                              ┌─────────────┐                 │
                              │             │                 │
                              │  EMERGENCY  │─────────────────┘
                              │             │
                              └──────┬──────┘
                                     │
                           auto land │ or manual recovery
                                     ▼
                              ┌─────────────┐
                              │             │
                              │   LANDED    │
                              │             │
                              └─────────────┘
```

### 9.3 Geofence Implementation

```cpp
// safety_monitor/include/safety_monitor/geofence.hpp

namespace peregrine::safety {

struct GeofenceConfig {
    // Cylindrical geofence (simple)
    double max_radius_m;           // Horizontal distance from home
    double max_altitude_m;         // Maximum altitude AGL
    double min_altitude_m;         // Minimum altitude AGL
    
    // Polygonal geofence (complex)
    std::vector<Eigen::Vector2d> boundary_polygon;  // 2D polygon vertices
    
    // Buffer zones
    double warning_buffer_m;       // Distance from boundary to warn
    double critical_buffer_m;      // Distance from boundary to act
    
    // Actions
    GeofenceAction warning_action;
    GeofenceAction violation_action;
};

enum class GeofenceAction {
    NONE,           // Log only
    WARN,           // Visual/audio warning
    HOLD,           // Stop and hover
    RETURN_HOME,    // RTL
    LAND,           // Land in place
    EMERGENCY_STOP  // Kill motors (dangerous!)
};

class Geofence {
public:
    GeofenceStatus check(const Eigen::Vector3d& position) const;
    double distanceToBoundary(const Eigen::Vector3d& position) const;
    Eigen::Vector3d nearestSafePoint(const Eigen::Vector3d& position) const;
    
private:
    GeofenceConfig config_;
    Eigen::Vector3d home_position_;
};

}  // namespace peregrine::safety
```

### 9.4 Heartbeat Monitoring

```cpp
// safety_monitor/include/safety_monitor/heartbeat_monitor.hpp

namespace peregrine::safety {

struct HeartbeatSource {
    std::string name;
    std::string topic;
    double timeout_s;
    HeartbeatPriority priority;
    bool required_for_flight;
};

enum class HeartbeatPriority {
    CRITICAL,    // Loss triggers emergency (e.g., PX4 connection)
    HIGH,        // Loss triggers RTL (e.g., GCS link)
    MEDIUM,      // Loss triggers warning (e.g., secondary sensors)
    LOW          // Loss logged only (e.g., optional components)
};

class HeartbeatMonitor {
public:
    void registerSource(const HeartbeatSource& source);
    void update();  // Called at monitor rate
    
    bool allCriticalAlive() const;
    bool allRequiredAlive() const;
    std::vector<std::string> getDeadSources() const;
    
private:
    std::map<std::string, HeartbeatSource> sources_;
    std::map<std::string, rclcpp::Time> last_heartbeat_;
};

}  // namespace peregrine::safety
```

### 9.5 Graceful Degradation

| Failure | Detection | Response | Fallback |
|---------|-----------|----------|----------|
| Trajectory tracker fails | No setpoint for 500ms | Hold position | Switch to position hold mode |
| Estimator diverges | Confidence < 0.5 | Use backup estimator | PX4 passthrough estimator |
| Controller unstable | Output saturation > 2s | Switch controller | Position controller (safe) |
| GCS link lost | No heartbeat 5s | RTL | Land after 30s RTL timeout |
| Single UAV collision risk | BVC violation | Emergency stop that UAV | Others continue |

---

## 10. Operational Scenario: 4-UAV Autonomous Mission

### 10.1 Mission Description

**Scenario**: Four large UAVs perform coordinated inspection with online replanning.

```
Mission Phases:
1. System Startup & Preflight
2. Sequential Arming
3. Sequential Takeoff
4. Formation Flight to Inspection Area  
5. Inspection Pattern Execution
6. Obstacle Detected - Online Replan
7. Return to Home
8. Sequential Landing
9. Disarm & Shutdown
```

### 10.2 Phase 1: System Startup & Preflight

```
Timeline: T-300s to T-60s
══════════════════════════════════════════════════════════════════════

GCS Terminal                           UAV Onboard Computers
─────────────────                      ────────────────────────────────

$ ros2 launch peregrine_bringup           For each UAV (1-4):
    multi_uav.launch.py                
    num_uavs:=4                        ┌─────────────────────────────┐
    environment:=outdoor               │ 1. Start uXRCE-DDS Agent    │
    config:=inspection_mission.yaml    │ 2. Launch peregrine stack      │
                                       │ 3. Wait for PX4 connection  │
                                       │ 4. Load plugins             │
                                       │ 5. Initialize safety monitor│
                                       │ 6. Publish /uavN/status     │
                                       └─────────────────────────────┘

Preflight Checks (automatic):
─────────────────────────────
□ PX4 connection established
□ GPS fix acquired (outdoor) / MoCap tracking (indoor)
□ Battery level > 80%
□ Estimator healthy
□ All heartbeats alive
□ Geofence loaded
□ Trajectories valid
□ Inter-UAV communication verified

TUI Display (each UAV):
┌──────────────────────────────────────────────────────────────┐
│ PEREGRINE Flight Stack - UAV 1                    [PREFLIGHT]   │
├──────────────────────────────────────────────────────────────┤
│ State: IDLE          Mode: MANUAL         Armed: NO          │
│ Position: 47.3978°N, 8.5456°E, 0.0m       Heading: 045°     │
│ Battery: 94% (23.8V)  GPS: 18 sats       Signal: ████████   │
├──────────────────────────────────────────────────────────────┤
│ Estimator: px4_passthrough [HEALTHY]      Conf: 0.98        │
│ Controller: position_pid [LOADED]         Status: IDLE      │
│ Trajectory: waypoint_linear [LOADED]      Status: EMPTY     │
├──────────────────────────────────────────────────────────────┤
│ Safety: NOMINAL  Geofence: OK  Heartbeats: 4/4  Neighbors: 3│
└──────────────────────────────────────────────────────────────┘
```

### 10.3 Phase 2-3: Sequential Arming and Takeoff

```
Timeline: T-60s to T+30s
══════════════════════════════════════════════════════════════════════

GCS Command:
$ ros2 service call /fleet/sequential_arm peregrine_interfaces/srv/FleetCommand

Service Implementation (pseudo-code):
─────────────────────────────────────
for uav_id in [1, 2, 3, 4]:
    # Arm UAV
    call /uav{id}/uav_manager/arm
    wait_for_response(timeout=5s)
    
    # Verify armed
    assert uav{id}.status.armed == true
    
    # Stagger for safety
    sleep(2s)

GCS Command:
$ ros2 action send_goal /fleet/sequential_takeoff \
    peregrine_interfaces/action/FleetTakeoff \
    "{target_altitude: 10.0, spacing_seconds: 5.0}"

Action Implementation:
─────────────────────
for uav_id in [1, 2, 3, 4]:
    # Send takeoff action
    goal = /uav{id}/uav_manager/takeoff {altitude: 10.0}
    send_goal_async(goal)
    
    # Wait for UAV to reach hover
    while not uav{id}.status.mode == HOVERING:
        check_safety()
        sleep(0.1s)
    
    # Stagger next takeoff
    sleep(5s)

UAV Manager State Transitions (each UAV):
─────────────────────────────────────────
IDLE ──[arm service]──▶ ARMED ──[takeoff action]──▶ TAKING_OFF
                                                         │
                                                         ▼
                                                    HOVERING
```

### 10.4 Phase 4: Formation Flight

```
Timeline: T+30s to T+120s
══════════════════════════════════════════════════════════════════════

Formation: Diamond pattern, 15m separation
─────────────────────────────────────────

        UAV 1 (Leader)
            ◇
           ╱ ╲
          ╱   ╲
     UAV 2     UAV 3
        ◇       ◇
          ╲   ╱
           ╲ ╱
            ◇
        UAV 4 (Tail)

GCS Command:
$ ros2 action send_goal /fleet/formation_goto \
    peregrine_interfaces/action/FormationGoto \
    "{target: {x: 100.0, y: 50.0, z: 15.0}, formation: diamond, separation: 15.0}"

Multi-Agent Coordinator (each UAV):
───────────────────────────────────
1. Receive fleet target position
2. Compute own formation offset
3. Subscribe to neighbor positions
4. Compute Buffered Voronoi Cell
5. Publish collision constraints to trajectory_manager
6. trajectory_manager generates safe path within BVC

Data Flow During Formation Flight:
──────────────────────────────────

┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│    UAV 1    │     │    UAV 2    │     │    UAV 3    │     │    UAV 4    │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │                   │
       │   /uavN/multi_agent/own_state (broadcast)                │
       ├───────────────────┼───────────────────┼───────────────────┤
       │                   │                   │                   │
       ▼                   ▼                   ▼                   ▼
  ┌─────────┐         ┌─────────┐         ┌─────────┐         ┌─────────┐
  │Compute  │         │Compute  │         │Compute  │         │Compute  │
  │  BVC    │         │  BVC    │         │  BVC    │         │  BVC    │
  └────┬────┘         └────┬────┘         └────┬────┘         └────┬────┘
       │                   │                   │                   │
       ▼                   ▼                   ▼                   ▼
  ┌─────────┐         ┌─────────┐         ┌─────────┐         ┌─────────┐
  │Traj Gen │         │Traj Gen │         │Traj Gen │         │Traj Gen │
  │within   │         │within   │         │within   │         │within   │
  │  BVC    │         │  BVC    │         │  BVC    │         │  BVC    │
  └─────────┘         └─────────┘         └─────────┘         └─────────┘
```

### 10.5 Phase 5-6: Inspection and Online Replan

```
Timeline: T+120s to T+300s
══════════════════════════════════════════════════════════════════════

Normal Inspection Pattern:
─────────────────────────

UAV 1: Survey north sector    ┌─────────────────────┐
UAV 2: Survey east sector     │      INSPECTION     │
UAV 3: Survey south sector    │        AREA         │
UAV 4: Survey west sector     │                     │
                              │   ◇1    ◇2          │
                              │                     │
                              │   ◇4    ◇3          │
                              └─────────────────────┘

OBSTACLE DETECTED (T+180s):
───────────────────────────
UAV 2 detects unexpected obstacle in its sector.
(Detection would come from perception layer - not part of core stack)

Replan Trigger:
$ ros2 topic pub /uav2/trajectory_manager/abort std_msgs/msg/Empty

Online Replan Sequence:
───────────────────────
1. UAV 2 trajectory_manager receives abort
2. UAV 2 immediately enters HOLD mode
3. UAV 2 publishes obstacle to /fleet/obstacles
4. GCS or onboard planner computes new trajectory avoiding obstacle
5. New trajectory sent to UAV 2

GCS/Planner Command:
$ ros2 action send_goal /uav2/trajectory_manager/set_trajectory \
    peregrine_interfaces/action/SetTrajectory \
    "{trajectory: <recomputed_trajectory>, replace: true}"

Other UAVs:
───────────
- Receive obstacle notification
- Multi-agent coordinator adds obstacle to local map
- BVC computation includes obstacle as virtual agent
- Trajectories automatically adjusted if needed
```

### 10.6 Phase 7-9: Return and Landing

```
Timeline: T+300s to T+420s
══════════════════════════════════════════════════════════════════════

GCS Command:
$ ros2 action send_goal /fleet/return_to_home \
    peregrine_interfaces/action/FleetRTH \
    "{sequential: true, spacing_seconds: 10.0}"

Return Sequence:
────────────────
1. All UAVs compute return path to home position
2. Multi-agent coordinator ensures deconflicted paths
3. UAVs return in sequence (UAV 4, 3, 2, 1 - reverse order)
4. Each UAV enters holding pattern above home

Landing Sequence:
─────────────────
for uav_id in [4, 3, 2, 1]:  # Reverse order
    # Descend to landing
    call /uav{id}/uav_manager/land
    
    # Wait for touchdown
    while not uav{id}.status.mode == LANDED:
        check_safety()
        sleep(0.1s)
    
    # Disarm
    call /uav{id}/uav_manager/disarm
    
    # Stagger next landing
    sleep(10s)

Final State:
────────────
All UAVs: IDLE, DISARMED, position verified at home

TUI Display (final):
┌──────────────────────────────────────────────────────────────┐
│ PEREGRINE Flight Stack - UAV 1                    [MISSION END] │
├──────────────────────────────────────────────────────────────┤
│ State: IDLE          Mode: MANUAL         Armed: NO          │
│ Position: 47.3978°N, 8.5456°E, 0.1m       Heading: 045°     │
│ Battery: 67% (22.1V)  GPS: 19 sats       Signal: ████████   │
├──────────────────────────────────────────────────────────────┤
│ Mission Summary:                                              │
│   Duration: 7m 12s    Distance: 847m    Max Alt: 15.2m      │
│   Replans: 1          Warnings: 0       Emergencies: 0       │
├──────────────────────────────────────────────────────────────┤
│ Safety: NOMINAL  All systems GREEN                           │
└──────────────────────────────────────────────────────────────┘
```

---

## 11. Package Dependency Graph

```
                              peregrine_interfaces
                                     │
                                     │ (all packages depend on interfaces)
                 ┌───────────────────┼───────────────────┐
                 │                   │                   │
                 ▼                   ▼                   ▼
          frame_transforms    peregrine_bringup      tui_status
                 │                   │                   │
                 │                   │                   │
                 ▼                   │                   │
        hardware_abstraction ◀──────┘                   │
                 │                                       │
                 │                                       │
    ┌────────────┼────────────┬────────────┐           │
    │            │            │            │           │
    ▼            ▼            ▼            ▼           │
estimator_   controller_  trajectory_   safety_       │
 manager       manager      manager     monitor       │
    │            │            │            │           │
    │            │            │            │           │
    └────────────┴─────┬──────┴────────────┘           │
                       │                               │
                       ▼                               │
                  uav_manager ◀────────────────────────┘
                       │
                       ▼
             multi_agent_coordinator


Build Order (colcon packages-select):
─────────────────────────────────────
1. peregrine_interfaces
2. frame_transforms
3. hardware_abstraction
4. estimator_manager, controller_manager, trajectory_manager, safety_monitor (parallel)
5. uav_manager
6. multi_agent_coordinator
7. tui_status
8. peregrine_bringup
```

---

## 12. Future Considerations

### 12.1 Decentralized Multi-Agent

- Implement peer discovery without GCS
- Add consensus protocols for coordinated decisions
- Design for network partition tolerance

### 12.2 Additional Sensor Fusion

- VIO integration (requires estimator plugin)
- LiDAR odometry (requires estimator plugin)
- Multi-sensor fusion architecture

### 12.3 Advanced Control

- MPC controller plugin
- Geometric controller plugin
- Adaptive control for changing payloads

### 12.4 Perception Integration

- Obstacle detection interface
- Dynamic obstacle tracking
- Semantic understanding

### 12.5 Mapping Integration

- Local costmap interface
- Global map interface
- Map sharing between agents

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-01-17 | System Architect | Initial architecture document |

---

*This document serves as the primary architectural reference for the PEREGRINE flight stack. Individual package READMEs provide implementation-level details.*
