# aerion_interfaces

**Package Type:** ROS2 Interface Package  
**Dependencies:** std_msgs, geometry_msgs, nav_msgs, sensor_msgs  

---

## Overview

`aerion_interfaces` is the **central message and service definition package** for the AERION flight stack. All custom messages, services, and actions are defined here to:

1. **Minimize dependency coupling** - All packages depend only on this interface package
2. **Simplify rosbag compatibility** - Only this package needs version matching for replay
3. **Enforce consistency** - Single source of truth for data structures

---

## Design Principles

### Single Responsibility
This package contains ONLY interface definitions - no code, no logic, no executables.

### Semantic Clarity
Message fields use explicit units in names (e.g., `altitude_m`, `velocity_mps`, `angle_rad`).

### Frame Explicitness
All pose/velocity messages include `frame_id` to prevent frame assumption errors.

### Timestamping
All messages include `std_msgs/Header` for proper time synchronization.

---

## Message Definitions

### Core State Messages

#### `State.msg` - Complete UAV State
```
# aerion_interfaces/msg/State.msg
# Complete UAV state in ENU/FLU frame

std_msgs/Header header
string frame_id                        # Reference frame (typically "world" or "odom")

# Position (ENU)
geometry_msgs/Point position           # [m] x=East, y=North, z=Up

# Velocity (ENU)
geometry_msgs/Vector3 velocity         # [m/s] in world frame

# Acceleration (ENU)
geometry_msgs/Vector3 acceleration     # [m/s²] in world frame

# Orientation (ENU to FLU)
geometry_msgs/Quaternion orientation   # Rotation from world to body

# Angular velocity (FLU body frame)
geometry_msgs/Vector3 angular_velocity # [rad/s] in body frame

# Euler angles (for convenience, derived from quaternion)
float64 roll_rad                       # [rad] Roll angle
float64 pitch_rad                      # [rad] Pitch angle  
float64 yaw_rad                        # [rad] Yaw angle (heading)

# Covariance (optional, for estimator output)
float64[36] pose_covariance            # 6x6 position + orientation covariance
float64[36] twist_covariance           # 6x6 velocity + angular velocity covariance

# Estimator metadata
string estimator_name                  # Which estimator produced this state
float64 confidence                     # [0.0 - 1.0] Estimator confidence
```

#### `TrajectorySetpoint.msg` - Reference Setpoint
```
# aerion_interfaces/msg/TrajectorySetpoint.msg
# Setpoint for trajectory tracking in ENU/FLU frame

std_msgs/Header header

# Setpoint type flags (which fields are valid)
bool position_valid
bool velocity_valid
bool acceleration_valid
bool jerk_valid
bool yaw_valid
bool yaw_rate_valid

# Position setpoint (ENU)
geometry_msgs/Point position           # [m]

# Velocity feedforward (ENU)
geometry_msgs/Vector3 velocity         # [m/s]

# Acceleration feedforward (ENU)
geometry_msgs/Vector3 acceleration     # [m/s²]

# Jerk feedforward (ENU)
geometry_msgs/Vector3 jerk             # [m/s³]

# Yaw setpoint
float64 yaw_rad                        # [rad] Desired heading
float64 yaw_rate_radps                 # [rad/s] Desired yaw rate
```

#### `ControlOutput.msg` - Controller Output
```
# aerion_interfaces/msg/ControlOutput.msg
# Output from controller to hardware abstraction

std_msgs/Header header

# Control mode (determines which fields are used)
uint8 control_mode
uint8 CONTROL_MODE_POSITION = 0        # Position setpoint to PX4
uint8 CONTROL_MODE_VELOCITY = 1        # Velocity setpoint to PX4
uint8 CONTROL_MODE_ATTITUDE = 2        # Attitude + thrust to PX4
uint8 CONTROL_MODE_BODY_RATE = 3       # Body rates + thrust to PX4

# Position setpoint (ENU) - used if mode = POSITION
geometry_msgs/Point position           # [m]
float64 yaw_rad                        # [rad]

# Velocity setpoint (ENU) - used if mode = VELOCITY
geometry_msgs/Vector3 velocity         # [m/s]
float64 yaw_rate_radps                 # [rad/s]

# Attitude setpoint (FLU) - used if mode = ATTITUDE
geometry_msgs/Quaternion orientation   # Desired orientation
float64 thrust_normalized              # [0.0 - 1.0] Normalized thrust

# Body rate setpoint (FLU) - used if mode = BODY_RATE
geometry_msgs/Vector3 body_rates       # [rad/s] Roll, pitch, yaw rates
float64 thrust_normalized              # [0.0 - 1.0] Normalized thrust

# Controller metadata
string controller_name                 # Which controller produced this output
```

### Trajectory Messages

#### `Trajectory.msg` - Complete Trajectory
```
# aerion_interfaces/msg/Trajectory.msg
# Complete trajectory representation

std_msgs/Header header

# Trajectory metadata
string trajectory_id                   # Unique identifier
string generator_name                  # Which generator created this

# Trajectory type
uint8 trajectory_type
uint8 TRAJECTORY_WAYPOINT = 0
uint8 TRAJECTORY_POLYNOMIAL = 1
uint8 TRAJECTORY_BSPLINE = 2
uint8 TRAJECTORY_BEZIER = 3

# Timing
builtin_interfaces/Time start_time
builtin_interfaces/Duration duration
float64 total_duration_s               # [s] Total trajectory duration

# Waypoints (for WAYPOINT type)
geometry_msgs/PoseStamped[] waypoints
float64[] waypoint_times_s             # Time to reach each waypoint

# Polynomial coefficients (for POLYNOMIAL type)
# Each array is [x_coeffs, y_coeffs, z_coeffs, yaw_coeffs]
float64[] poly_coeffs_x                # Polynomial coefficients for x
float64[] poly_coeffs_y                # Polynomial coefficients for y
float64[] poly_coeffs_z                # Polynomial coefficients for z
float64[] poly_coeffs_yaw              # Polynomial coefficients for yaw
uint32 poly_order                      # Polynomial order

# Setpoint type this trajectory provides
uint8 setpoint_type
uint8 SETPOINT_POSITION_ONLY = 0
uint8 SETPOINT_POSITION_YAW = 1
uint8 SETPOINT_FULL_STATE = 2
```

#### `TrajectoryStatus.msg` - Trajectory Progress
```
# aerion_interfaces/msg/TrajectoryStatus.msg
# Current trajectory execution status

std_msgs/Header header

# Trajectory being executed
string trajectory_id

# Progress
float64 progress_percent               # [0.0 - 100.0] Completion percentage
float64 elapsed_time_s                 # [s] Time since trajectory start
float64 remaining_time_s               # [s] Estimated time to completion

# Current state on trajectory
TrajectorySetpoint current_setpoint

# Tracking error
float64 position_error_m               # [m] Distance from reference
float64 velocity_error_mps             # [m/s] Velocity tracking error

# Status
uint8 status
uint8 STATUS_IDLE = 0
uint8 STATUS_EXECUTING = 1
uint8 STATUS_PAUSED = 2
uint8 STATUS_COMPLETED = 3
uint8 STATUS_ABORTED = 4
uint8 STATUS_ERROR = 5
```

### Safety Messages

#### `SafetyStatus.msg` - Overall Safety State
```
# aerion_interfaces/msg/SafetyStatus.msg
# Comprehensive safety status

std_msgs/Header header

# Overall safety level
uint8 safety_level
uint8 LEVEL_NOMINAL = 0
uint8 LEVEL_WARNING = 1
uint8 LEVEL_CAUTION = 2
uint8 LEVEL_EMERGENCY = 3

# Component health
bool estimator_healthy
bool controller_healthy
bool trajectory_healthy
bool px4_connected
bool gcs_connected

# Geofence status
uint8 geofence_status
uint8 GEOFENCE_OK = 0
uint8 GEOFENCE_WARNING = 1
uint8 GEOFENCE_VIOLATION = 2
float64 distance_to_geofence_m         # [m] Closest distance to boundary

# Heartbeat status
uint8 heartbeats_alive                 # Number of alive heartbeats
uint8 heartbeats_total                 # Total registered heartbeats
string[] dead_heartbeats               # Names of dead heartbeats

# Battery
float64 battery_percent                # [%] Remaining battery
float64 battery_voltage_v              # [V] Current voltage
uint8 battery_status
uint8 BATTERY_OK = 0
uint8 BATTERY_WARNING = 1
uint8 BATTERY_CRITICAL = 2

# Active warnings/errors
string[] active_warnings
string[] active_errors

# Recommended action
uint8 recommended_action
uint8 ACTION_NONE = 0
uint8 ACTION_WARN = 1
uint8 ACTION_HOLD = 2
uint8 ACTION_RTH = 3
uint8 ACTION_LAND = 4
uint8 ACTION_EMERGENCY_STOP = 5
```

#### `CollisionConstraint.msg` - Collision Avoidance Constraint
```
# aerion_interfaces/msg/CollisionConstraint.msg
# Collision avoidance constraint from multi-agent coordinator

std_msgs/Header header

# Constraint type
uint8 constraint_type
uint8 CONSTRAINT_HALFPLANE = 0         # Linear half-plane constraint
uint8 CONSTRAINT_SPHERE = 1            # Spherical keep-out zone
uint8 CONSTRAINT_POLYGON = 2           # Polygonal region

# Half-plane constraint: a'x <= b
geometry_msgs/Vector3 normal           # Normal vector (a)
float64 offset                         # Offset value (b)

# Sphere constraint
geometry_msgs/Point center             # Sphere center
float64 radius_m                       # Sphere radius

# Source of constraint
string source_uav_id                   # Which UAV caused this constraint
float64 confidence                     # [0.0 - 1.0] Constraint confidence
float64 expiry_time_s                  # [s] When constraint expires
```

### UAV Manager Messages

#### `UAVMode.msg` - Current UAV Mode
```
# aerion_interfaces/msg/UAVMode.msg
# Current operational mode

std_msgs/Header header

# High-level state
uint8 state
uint8 STATE_IDLE = 0
uint8 STATE_ARMED = 1
uint8 STATE_TAKING_OFF = 2
uint8 STATE_HOVERING = 3
uint8 STATE_FLYING = 4
uint8 STATE_LANDING = 5
uint8 STATE_LANDED = 6
uint8 STATE_EMERGENCY = 7

# Control mode
uint8 control_mode
uint8 MODE_MANUAL = 0
uint8 MODE_POSITION = 1
uint8 MODE_VELOCITY = 2
uint8 MODE_TRAJECTORY = 3
uint8 MODE_OFFBOARD = 4

# Submode
string submode                         # e.g., "formation", "inspection", "rth"

# Armed status
bool armed

# Can transition flags
bool can_arm
bool can_takeoff
bool can_land
```

#### `UAVStatus.msg` - Comprehensive UAV Status
```
# aerion_interfaces/msg/UAVStatus.msg
# Complete UAV status for monitoring

std_msgs/Header header

# Identification
string uav_id
string uav_name

# Current state
State current_state
UAVMode current_mode

# Active components
string active_estimator
string active_controller
string active_trajectory_generator

# Safety
SafetyStatus safety_status

# Resource status
float64 battery_percent
float64 cpu_percent
float64 memory_percent

# Communication
bool px4_connected
bool gcs_connected
float64 px4_latency_ms
float64 gcs_latency_ms

# Mission
string current_mission_id
float64 mission_progress_percent
```

---

## Service Definitions

### Manager Services

#### `SetController.srv`
```
# aerion_interfaces/srv/SetController.srv
# Switch active controller

string controller_name                 # Plugin name to activate
---
bool success
string message
string active_controller               # Currently active controller after switch
```

#### `SetEstimator.srv`
```
# aerion_interfaces/srv/SetEstimator.srv
# Switch active estimator

string estimator_name                  # Plugin name to activate
---
bool success
string message
string active_estimator                # Currently active estimator after switch
```

#### `SetTrajectoryGenerator.srv`
```
# aerion_interfaces/srv/SetTrajectoryGenerator.srv
# Switch active trajectory generator

string generator_name                  # Plugin name to activate
---
bool success
string message
string active_generator                # Currently active generator after switch
```

### UAV Manager Services

#### `Arm.srv`
```
# aerion_interfaces/srv/Arm.srv
# Arm or disarm the UAV

bool arm                               # true = arm, false = disarm
---
bool success
string message
bool armed                             # Current armed state
```

#### `EmergencyStop.srv`
```
# aerion_interfaces/srv/EmergencyStop.srv
# Emergency stop - use with extreme caution!

bool confirm                           # Must be true to execute
string reason                          # Reason for emergency stop
---
bool executed
string message
```

---

## Action Definitions

### `Takeoff.action`
```
# aerion_interfaces/action/Takeoff.action
# Takeoff to specified altitude

# Goal
float64 target_altitude_m              # [m] Target altitude AGL
float64 climb_rate_mps                 # [m/s] Desired climb rate (0 = default)
---
# Result
bool success
string message
float64 final_altitude_m               # [m] Actual achieved altitude
---
# Feedback
float64 current_altitude_m             # [m] Current altitude
float64 progress_percent               # [%] Completion progress
```

### `Land.action`
```
# aerion_interfaces/action/Land.action
# Land at current position or specified location

# Goal
bool land_at_current_position          # true = land here, false = use target
geometry_msgs/Point target_position    # Target landing position (if not current)
float64 descent_rate_mps               # [m/s] Desired descent rate (0 = default)
---
# Result
bool success
string message
geometry_msgs/Point final_position     # Actual landing position
---
# Feedback
float64 altitude_agl_m                 # [m] Current altitude AGL
float64 progress_percent               # [%] Completion progress
```

### `SetTrajectory.action`
```
# aerion_interfaces/action/SetTrajectory.action
# Execute a trajectory

# Goal
Trajectory trajectory                  # Trajectory to execute
bool replace_current                   # true = replace, false = append
---
# Result
bool success
string message
uint8 completion_status                # Final status (completed/aborted/error)
---
# Feedback
TrajectoryStatus status                # Current trajectory status
```

### `GoToPosition.action`
```
# aerion_interfaces/action/GoToPosition.action
# Go to a position

# Goal
geometry_msgs/Point position           # Target position (ENU)
float64 yaw_rad                        # Target heading (NaN = maintain current)
float64 velocity_mps                   # Desired velocity (0 = default)
float64 acceptance_radius_m            # [m] Position tolerance
---
# Result
bool success
string message
geometry_msgs/Point final_position     # Actual final position
---
# Feedback
float64 distance_remaining_m           # [m] Distance to target
float64 eta_s                          # [s] Estimated time of arrival
```

---

## Implementation Guidelines

### ROS2 Concepts Used

1. **Interface Packages**
   - Use `rosidl_generate_interfaces()` in CMakeLists.txt
   - Separate package with only `.msg`, `.srv`, `.action` files
   - Export dependencies properly

2. **CMakeLists.txt Structure**
   ```cmake
   cmake_minimum_required(VERSION 3.8)
   project(aerion_interfaces)
   
   find_package(ament_cmake REQUIRED)
   find_package(rosidl_default_generators REQUIRED)
   find_package(std_msgs REQUIRED)
   find_package(geometry_msgs REQUIRED)
   find_package(builtin_interfaces REQUIRED)
   
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/State.msg"
     "msg/TrajectorySetpoint.msg"
     "msg/ControlOutput.msg"
     # ... all other messages
     "srv/SetController.srv"
     "srv/Arm.srv"
     # ... all other services
     "action/Takeoff.action"
     "action/Land.action"
     # ... all other actions
     DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
   )
   
   ament_export_dependencies(rosidl_default_runtime)
   ament_package()
   ```

3. **package.xml Structure**
   ```xml
   <package format="3">
     <name>aerion_interfaces</name>
     <version>1.0.0</version>
     <description>AERION flight stack interfaces</description>
     
     <buildtool_depend>ament_cmake</buildtool_depend>
     <buildtool_depend>rosidl_default_generators</buildtool_depend>
     
     <depend>std_msgs</depend>
     <depend>geometry_msgs</depend>
     <depend>builtin_interfaces</depend>
     
     <exec_depend>rosidl_default_runtime</exec_depend>
     
     <member_of_group>rosidl_interface_packages</member_of_group>
     
     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>
   ```

---

## Major Cautions

### ⚠️ Version Compatibility
- Message changes break rosbag playback compatibility
- Use message versioning strategy for research data preservation
- Consider deprecation fields before removal

### ⚠️ Field Units
- ALWAYS include units in field names or comments
- Default to SI units (meters, radians, seconds)
- Document any non-SI usage explicitly

### ⚠️ Frame Conventions
- All spatial data in ENU/FLU unless explicitly stated
- Include `frame_id` in messages with positional data
- Conversion to PX4 frames (NED/FRD) happens ONLY in hardware_abstraction

### ⚠️ Timestamp Handling
- Always populate header.stamp
- Use ROS time, not system time
- Consider simulation time compatibility

### ⚠️ Array Sizing
- Use dynamic arrays (`float64[]`) not fixed (`float64[6]`) for flexibility
- Document expected array sizes in comments
- Validate array sizes in consuming code

---

## File Structure

```
aerion_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── State.msg
│   ├── TrajectorySetpoint.msg
│   ├── ControlOutput.msg
│   ├── Trajectory.msg
│   ├── TrajectoryStatus.msg
│   ├── SafetyStatus.msg
│   ├── CollisionConstraint.msg
│   ├── UAVMode.msg
│   └── UAVStatus.msg
├── srv/
│   ├── SetController.srv
│   ├── SetEstimator.srv
│   ├── SetTrajectoryGenerator.srv
│   ├── Arm.srv
│   └── EmergencyStop.srv
└── action/
    ├── Takeoff.action
    ├── Land.action
    ├── SetTrajectory.action
    └── GoToPosition.action
```
