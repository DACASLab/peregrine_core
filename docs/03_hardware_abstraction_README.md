# hardware_abstraction

**Package Type:** ROS2 Node Package  
**Dependencies:** aerion_interfaces, frame_transforms, px4_msgs, rclcpp  

---

## Overview

`hardware_abstraction` is the **sole interface between AERION and PX4**. This package:

1. **Bridges ROS2 and PX4** via uXRCE-DDS (no MAVLink)
2. **Handles all frame conversions** at the hardware boundary
3. **Manages PX4 connection lifecycle**
4. **Abstracts PX4 specifics** from the rest of the stack

All other AERION packages interact with PX4 ONLY through this package.

---

## Design Philosophy

### Single Gateway Principle
```
┌─────────────────────────────────────────────────────────────────┐
│                        AERION STACK                              │
│     (Everything in ENU/FLU, uses aerion_interfaces)             │
└─────────────────────────────────────┬───────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    HARDWARE ABSTRACTION                          │
│    • Frame conversion (ENU↔NED, FLU↔FRD)                        │
│    • Message translation (aerion_interfaces ↔ px4_msgs)         │
│    • Connection management                                       │
│    • Time synchronization                                        │
└─────────────────────────────────────┬───────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                         PX4 AUTOPILOT                            │
│        (Everything in NED/FRD, uses px4_msgs/uORB)              │
└─────────────────────────────────────────────────────────────────┘
```

### Why This Design?
1. **Isolation**: PX4 API changes don't propagate through the stack
2. **Testing**: Mock hardware_abstraction for simulation without PX4
3. **Clarity**: All frame conventions enforced at one point
4. **Future-proof**: Could support other autopilots by swapping this package

---

## Architecture

### Node Structure

```
hardware_abstraction_node
│
├── PX4 Subscribers (from PX4)
│   ├── /fmu/out/vehicle_odometry → processOdometry()
│   ├── /fmu/out/vehicle_status → processStatus()
│   ├── /fmu/out/vehicle_local_position → processLocalPosition()
│   ├── /fmu/out/battery_status → processBattery()
│   ├── /fmu/out/sensor_combined → processSensors()
│   └── /fmu/out/timesync_status → processTimesync()
│
├── PX4 Publishers (to PX4)
│   ├── processControlOutput() → /fmu/in/trajectory_setpoint
│   ├── processControlOutput() → /fmu/in/vehicle_attitude_setpoint
│   ├── processControlOutput() → /fmu/in/vehicle_rates_setpoint
│   ├── publishOffboardMode() → /fmu/in/offboard_control_mode
│   └── publishCommand() → /fmu/in/vehicle_command
│
├── AERION Publishers (to stack)
│   ├── /hardware_abstraction/px4_state (aerion_interfaces/State)
│   ├── /hardware_abstraction/px4_status (aerion_interfaces/PX4Status)
│   └── /hardware_abstraction/battery (sensor_msgs/BatteryState)
│
├── AERION Subscribers (from stack)
│   └── /hardware_abstraction/control_output (aerion_interfaces/ControlOutput)
│
└── AERION Services
    ├── /hardware_abstraction/arm
    ├── /hardware_abstraction/disarm
    ├── /hardware_abstraction/set_mode
    └── /hardware_abstraction/send_command
```

---

## PX4 Communication

### uXRCE-DDS Topics Used

#### Subscribed from PX4 (Reading State)

| PX4 Topic | px4_msgs Type | Purpose | Rate |
|-----------|---------------|---------|------|
| `/fmu/out/vehicle_odometry` | `VehicleOdometry` | Full state estimate | 50-250Hz |
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | Position/velocity | 50Hz |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | Orientation | 250Hz |
| `/fmu/out/vehicle_status` | `VehicleStatus` | Arming state, mode | 1Hz |
| `/fmu/out/battery_status` | `BatteryStatus` | Battery info | 1Hz |
| `/fmu/out/failsafe_flags` | `FailsafeFlags` | Failsafe status | 1Hz |
| `/fmu/out/timesync_status` | `TimesyncStatus` | Time synchronization | 10Hz |

#### Published to PX4 (Sending Commands)

| PX4 Topic | px4_msgs Type | Purpose | Rate |
|-----------|---------------|---------|------|
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | Enable offboard | ≥2Hz |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | Position/velocity cmd | 50Hz |
| `/fmu/in/vehicle_attitude_setpoint` | `VehicleAttitudeSetpoint` | Attitude cmd | 250Hz |
| `/fmu/in/vehicle_rates_setpoint` | `VehicleRatesSetpoint` | Rate cmd | 250Hz |
| `/fmu/in/vehicle_command` | `VehicleCommand` | Arm/mode/commands | On demand |

### QoS Configuration

```cpp
// PX4 requires specific QoS settings
rmw_qos_profile_t px4_qos_profile = {
    .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    .depth = 5,
    .reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    .durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    // ... other fields with defaults
};

// Usage
auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(px4_qos_profile));
```

---

## Core Implementation

### Main Node Class

```cpp
// hardware_abstraction/include/hardware_abstraction/hardware_abstraction_node.hpp

namespace aerion::hardware_abstraction {

class HardwareAbstractionNode : public rclcpp::Node {
public:
    explicit HardwareAbstractionNode(const rclcpp::NodeOptions& options);
    
private:
    // ═══════════════════════════════════════════════════════════════
    // PX4 Callbacks (NED/FRD in, convert to ENU/FLU, publish)
    // ═══════════════════════════════════════════════════════════════
    
    void onVehicleOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void onBatteryStatus(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
    void onTimesyncStatus(const px4_msgs::msg::TimesyncStatus::SharedPtr msg);
    void onFailsafeFlags(const px4_msgs::msg::FailsafeFlags::SharedPtr msg);
    
    // ═══════════════════════════════════════════════════════════════
    // Stack Callbacks (ENU/FLU in, convert to NED/FRD, send to PX4)
    // ═══════════════════════════════════════════════════════════════
    
    void onControlOutput(const aerion_interfaces::msg::ControlOutput::SharedPtr msg);
    
    // ═══════════════════════════════════════════════════════════════
    // Services
    // ═══════════════════════════════════════════════════════════════
    
    void armCallback(
        const aerion_interfaces::srv::Arm::Request::SharedPtr request,
        aerion_interfaces::srv::Arm::Response::SharedPtr response);
    
    void setModeCallback(
        const aerion_interfaces::srv::SetMode::Request::SharedPtr request,
        aerion_interfaces::srv::SetMode::Response::SharedPtr response);
    
    // ═══════════════════════════════════════════════════════════════
    // Internal Methods
    // ═══════════════════════════════════════════════════════════════
    
    void publishOffboardControlMode();
    void sendVehicleCommand(uint16_t command, float param1 = 0.0f, 
                           float param2 = 0.0f);
    uint64_t getTimestamp();  // PX4-synchronized timestamp
    
    // ═══════════════════════════════════════════════════════════════
    // Frame Conversion Helpers
    // ═══════════════════════════════════════════════════════════════
    
    aerion_interfaces::msg::State convertToAerionState(
        const px4_msgs::msg::VehicleOdometry& odom);
    
    px4_msgs::msg::TrajectorySetpoint convertToTrajectorySetpoint(
        const aerion_interfaces::msg::ControlOutput& output);
    
    px4_msgs::msg::VehicleAttitudeSetpoint convertToAttitudeSetpoint(
        const aerion_interfaces::msg::ControlOutput& output);
    
    // ═══════════════════════════════════════════════════════════════
    // Member Variables
    // ═══════════════════════════════════════════════════════════════
    
    // PX4 subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
    
    // PX4 publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr att_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rate_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
    
    // Stack publishers
    rclcpp::Publisher<aerion_interfaces::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<aerion_interfaces::msg::PX4Status>::SharedPtr px4_status_pub_;
    
    // Stack subscribers
    rclcpp::Subscription<aerion_interfaces::msg::ControlOutput>::SharedPtr control_sub_;
    
    // Services
    rclcpp::Service<aerion_interfaces::srv::Arm>::SharedPtr arm_service_;
    rclcpp::Service<aerion_interfaces::srv::SetMode>::SharedPtr mode_service_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr offboard_timer_;  // Publish offboard mode at 10Hz
    rclcpp::TimerBase::SharedPtr watchdog_timer_;  // Check PX4 connection
    
    // State
    std::atomic<uint64_t> timestamp_offset_{0};  // For time sync
    std::atomic<bool> px4_connected_{false};
    std::atomic<bool> armed_{false};
    std::atomic<uint8_t> current_mode_{0};
    
    // Current control mode (determines which PX4 setpoints to publish)
    std::atomic<uint8_t> control_mode_{0};
    
    // Configuration
    std::string namespace_;
    double offboard_rate_hz_{10.0};
    double watchdog_timeout_s_{1.0};
};

}  // namespace aerion::hardware_abstraction
```

### State Conversion Implementation

```cpp
// hardware_abstraction/src/hardware_abstraction_node.cpp

aerion_interfaces::msg::State HardwareAbstractionNode::convertToAerionState(
    const px4_msgs::msg::VehicleOdometry& odom) {
    
    using namespace aerion::frame_transforms;
    
    aerion_interfaces::msg::State state;
    state.header.stamp = this->get_clock()->now();
    state.header.frame_id = "world";  // ENU frame
    
    // Position: NED → ENU
    Eigen::Vector3d pos_ned(odom.position[0], odom.position[1], odom.position[2]);
    Eigen::Vector3d pos_enu = ned_to_enu(pos_ned);
    state.position.x = pos_enu.x();
    state.position.y = pos_enu.y();
    state.position.z = pos_enu.z();
    
    // Velocity: NED → ENU (in world frame)
    // Note: PX4 VehicleOdometry velocity is in body frame (FRD)
    // We need to rotate to world frame first, then convert
    Eigen::Quaterniond q_ned(odom.q[0], odom.q[1], odom.q[2], odom.q[3]);
    Eigen::Vector3d vel_body_frd(odom.velocity[0], odom.velocity[1], odom.velocity[2]);
    
    // Body FRD to World NED
    Eigen::Vector3d vel_world_ned = q_ned * vel_body_frd;
    
    // World NED to World ENU
    Eigen::Vector3d vel_world_enu = ned_to_enu(vel_world_ned);
    state.velocity.x = vel_world_enu.x();
    state.velocity.y = vel_world_enu.y();
    state.velocity.z = vel_world_enu.z();
    
    // Orientation: NED-FRD → ENU-FLU
    Eigen::Quaterniond q_enu_flu = quaternion_ned_to_enu_simple(q_ned);
    state.orientation.w = q_enu_flu.w();
    state.orientation.x = q_enu_flu.x();
    state.orientation.y = q_enu_flu.y();
    state.orientation.z = q_enu_flu.z();
    
    // Angular velocity: FRD → FLU
    Eigen::Vector3d omega_frd(odom.angular_velocity[0], 
                               odom.angular_velocity[1],
                               odom.angular_velocity[2]);
    Eigen::Vector3d omega_flu = frd_to_flu(omega_frd);
    state.angular_velocity.x = omega_flu.x();
    state.angular_velocity.y = omega_flu.y();
    state.angular_velocity.z = omega_flu.z();
    
    // Compute Euler angles from quaternion
    auto euler = q_enu_flu.toRotationMatrix().eulerAngles(0, 1, 2);
    state.roll_rad = euler[0];
    state.pitch_rad = euler[1];
    state.yaw_rad = euler[2];
    
    // Metadata
    state.estimator_name = "px4_ekf2";
    state.confidence = 1.0;  // PX4 doesn't expose this directly
    
    return state;
}
```

### Control Output Processing

```cpp
void HardwareAbstractionNode::onControlOutput(
    const aerion_interfaces::msg::ControlOutput::SharedPtr msg) {
    
    using namespace aerion::frame_transforms;
    
    // Update control mode
    control_mode_.store(msg->control_mode);
    
    switch (msg->control_mode) {
        case aerion_interfaces::msg::ControlOutput::CONTROL_MODE_POSITION: {
            // Convert position setpoint ENU → NED
            px4_msgs::msg::TrajectorySetpoint setpoint;
            setpoint.timestamp = getTimestamp();
            
            Eigen::Vector3d pos_enu(msg->position.x, msg->position.y, msg->position.z);
            Eigen::Vector3d pos_ned = enu_to_ned(pos_enu);
            setpoint.position[0] = pos_ned.x();
            setpoint.position[1] = pos_ned.y();
            setpoint.position[2] = pos_ned.z();
            
            // Yaw: ENU → NED convention
            setpoint.yaw = yaw_enu_to_ned(msg->yaw_rad);
            
            // Mark velocities as NaN (not used)
            setpoint.velocity[0] = NAN;
            setpoint.velocity[1] = NAN;
            setpoint.velocity[2] = NAN;
            
            traj_setpoint_pub_->publish(setpoint);
            break;
        }
        
        case aerion_interfaces::msg::ControlOutput::CONTROL_MODE_VELOCITY: {
            px4_msgs::msg::TrajectorySetpoint setpoint;
            setpoint.timestamp = getTimestamp();
            
            // Convert velocity setpoint ENU → NED
            Eigen::Vector3d vel_enu(msg->velocity.x, msg->velocity.y, msg->velocity.z);
            Eigen::Vector3d vel_ned = velocity_enu_to_ned(vel_enu);
            setpoint.velocity[0] = vel_ned.x();
            setpoint.velocity[1] = vel_ned.y();
            setpoint.velocity[2] = vel_ned.z();
            
            // Yaw rate
            setpoint.yawspeed = msg->yaw_rate_radps;  // Same sign convention
            
            // Mark positions as NaN
            setpoint.position[0] = NAN;
            setpoint.position[1] = NAN;
            setpoint.position[2] = NAN;
            
            traj_setpoint_pub_->publish(setpoint);
            break;
        }
        
        case aerion_interfaces::msg::ControlOutput::CONTROL_MODE_ATTITUDE: {
            px4_msgs::msg::VehicleAttitudeSetpoint setpoint;
            setpoint.timestamp = getTimestamp();
            
            // Convert quaternion ENU-FLU → NED-FRD
            Eigen::Quaterniond q_enu_flu(
                msg->orientation.w, msg->orientation.x,
                msg->orientation.y, msg->orientation.z);
            Eigen::Quaterniond q_ned_frd = quaternion_enu_to_ned_simple(q_enu_flu);
            
            setpoint.q_d[0] = q_ned_frd.w();
            setpoint.q_d[1] = q_ned_frd.x();
            setpoint.q_d[2] = q_ned_frd.y();
            setpoint.q_d[3] = q_ned_frd.z();
            
            // Thrust (normalized, same in both frames)
            setpoint.thrust_body[2] = -msg->thrust_normalized;  // PX4 uses -Z for thrust
            
            att_setpoint_pub_->publish(setpoint);
            break;
        }
        
        case aerion_interfaces::msg::ControlOutput::CONTROL_MODE_BODY_RATE: {
            px4_msgs::msg::VehicleRatesSetpoint setpoint;
            setpoint.timestamp = getTimestamp();
            
            // Convert body rates FLU → FRD
            Eigen::Vector3d rates_flu(
                msg->body_rates.x, msg->body_rates.y, msg->body_rates.z);
            Eigen::Vector3d rates_frd = flu_to_frd(rates_flu);
            
            setpoint.roll = rates_frd.x();
            setpoint.pitch = rates_frd.y();
            setpoint.yaw = rates_frd.z();
            
            // Thrust (normalized)
            setpoint.thrust_body[2] = -msg->thrust_normalized;
            
            rate_setpoint_pub_->publish(setpoint);
            break;
        }
    }
}
```

### Offboard Mode Management

```cpp
void HardwareAbstractionNode::publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = getTimestamp();
    
    // Set flags based on current control mode
    switch (control_mode_.load()) {
        case aerion_interfaces::msg::ControlOutput::CONTROL_MODE_POSITION:
            msg.position = true;
            msg.velocity = false;
            msg.acceleration = false;
            msg.attitude = false;
            msg.body_rate = false;
            break;
            
        case aerion_interfaces::msg::ControlOutput::CONTROL_MODE_VELOCITY:
            msg.position = false;
            msg.velocity = true;
            msg.acceleration = false;
            msg.attitude = false;
            msg.body_rate = false;
            break;
            
        case aerion_interfaces::msg::ControlOutput::CONTROL_MODE_ATTITUDE:
            msg.position = false;
            msg.velocity = false;
            msg.acceleration = false;
            msg.attitude = true;
            msg.body_rate = false;
            break;
            
        case aerion_interfaces::msg::ControlOutput::CONTROL_MODE_BODY_RATE:
            msg.position = false;
            msg.velocity = false;
            msg.acceleration = false;
            msg.attitude = false;
            msg.body_rate = true;
            break;
    }
    
    offboard_pub_->publish(msg);
}
```

### Time Synchronization

```cpp
void HardwareAbstractionNode::onTimesyncStatus(
    const px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
    
    // Update timestamp offset for synchronization
    // PX4 timestamp is in microseconds since boot
    // ROS timestamp is in nanoseconds since epoch
    
    auto ros_now = this->get_clock()->now().nanoseconds();
    auto px4_now = msg->timestamp;  // microseconds
    
    // Compute offset (this is simplified; production should use filtering)
    timestamp_offset_.store(ros_now / 1000 - px4_now);
    
    // Update connection status
    px4_connected_.store(true);
}

uint64_t HardwareAbstractionNode::getTimestamp() {
    // Convert ROS time to PX4 timestamp
    auto ros_now = this->get_clock()->now().nanoseconds();
    return ros_now / 1000 - timestamp_offset_.load();
}
```

### Vehicle Commands

```cpp
void HardwareAbstractionNode::sendVehicleCommand(
    uint16_t command, float param1, float param2) {
    
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = getTimestamp();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;      // Default system ID
    msg.target_component = 1;   // Default component ID
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    
    command_pub_->publish(msg);
}

// Arming service
void HardwareAbstractionNode::armCallback(
    const aerion_interfaces::srv::Arm::Request::SharedPtr request,
    aerion_interfaces::srv::Arm::Response::SharedPtr response) {
    
    if (request->arm) {
        // Arm command
        sendVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0f);  // 1 = arm
        RCLCPP_INFO(get_logger(), "Sent ARM command to PX4");
    } else {
        // Disarm command
        sendVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0f);  // 0 = disarm
        RCLCPP_INFO(get_logger(), "Sent DISARM command to PX4");
    }
    
    // Wait briefly for status update
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    response->success = true;  // Actual confirmation comes from status callback
    response->armed = armed_.load();
}
```

---

## Configuration

### Parameters

```yaml
# config/hardware_abstraction.yaml

hardware_abstraction:
  ros__parameters:
    # PX4 namespace (for multi-vehicle)
    px4_namespace: ""  # Empty for single vehicle, "px4_1" for multi
    
    # Publishing rates
    offboard_rate_hz: 10.0  # Must be > 2Hz for PX4
    state_rate_hz: 250.0    # Rate to publish state to stack
    
    # Connection monitoring
    watchdog_timeout_s: 1.0  # Time before declaring PX4 disconnected
    
    # QoS settings
    qos_history_depth: 5
    qos_reliability: "best_effort"
    
    # Frame IDs
    world_frame: "world"
    body_frame: "base_link"
```

### Multi-Vehicle Configuration

For multiple UAVs, each vehicle has its own namespace:

```yaml
# UAV 1
px4_namespace: "px4_1"  # Topics: /px4_1/fmu/out/*, /px4_1/fmu/in/*

# UAV 2  
px4_namespace: "px4_2"  # Topics: /px4_2/fmu/out/*, /px4_2/fmu/in/*
```

---

## Launch File

```python
# launch/hardware_abstraction.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('px4_namespace', default_value=''),
        
        Node(
            package='hardware_abstraction',
            executable='hardware_abstraction_node',
            name='hardware_abstraction',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'px4_namespace': LaunchConfiguration('px4_namespace'),
                'offboard_rate_hz': 10.0,
                'state_rate_hz': 250.0,
            }],
            output='screen',
            remappings=[
                # Remap PX4 topics if needed
            ],
        ),
    ])
```

---

## Implementation Guidelines

### ROS2 Concepts Used

1. **Multi-Threaded Executor**: For handling high-rate callbacks
   ```cpp
   rclcpp::executors::MultiThreadedExecutor executor;
   executor.add_node(node);
   executor.spin();
   ```

2. **Callback Groups**: Separate groups for PX4 and stack callbacks
   ```cpp
   px4_callback_group_ = create_callback_group(
       rclcpp::CallbackGroupType::MutuallyExclusive);
   stack_callback_group_ = create_callback_group(
       rclcpp::CallbackGroupType::MutuallyExclusive);
   ```

3. **Lifecycle Nodes** (Optional): For proper startup/shutdown
   ```cpp
   class HardwareAbstractionNode : public rclcpp_lifecycle::LifecycleNode
   ```

4. **QoS Profiles**: Match PX4's expected QoS
   ```cpp
   auto qos = rclcpp::QoS(5)
       .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
       .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
   ```

### Thread Safety

- Use `std::atomic` for shared state
- Minimize lock contention in high-rate paths
- Use lock-free queues if needed

---

## Major Cautions

### ⚠️ Offboard Mode Requirements
- **MUST publish OffboardControlMode at ≥2Hz** or PX4 exits offboard
- Start publishing BEFORE requesting offboard mode
- Continue publishing even when not sending setpoints

### ⚠️ Timestamp Handling
- PX4 timestamps are microseconds since boot
- ROS timestamps are nanoseconds since epoch
- Incorrect timestamps cause EKF issues with external state

### ⚠️ NaN Handling
- PX4 TrajectorySetpoint uses NaN to indicate "not used"
- ALWAYS set unused fields to NaN, not zero
- Zero is a valid setpoint!

### ⚠️ QoS Mismatch
- Wrong QoS = no communication
- PX4 uses BEST_EFFORT reliability
- Use TRANSIENT_LOCAL durability for late-joining subscribers

### ⚠️ Frame Conversion Bugs
- Most common source of "drone flew in wrong direction"
- Test with known setpoints before flight
- Log both ENU and NED values for debugging

### ⚠️ Arming Safety
- Never auto-arm without operator confirmation
- Implement arming checks in uav_manager, not here
- This package just forwards commands

### ⚠️ Simulation vs Hardware
- SITL uses different default ports
- Hardware may need serial configuration
- Test the connection before flight

---

## File Structure

```
hardware_abstraction/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── hardware_abstraction/
│       └── hardware_abstraction_node.hpp
├── src/
│   ├── hardware_abstraction_node.cpp
│   └── main.cpp
├── config/
│   └── hardware_abstraction.yaml
├── launch/
│   └── hardware_abstraction.launch.py
└── test/
    ├── test_conversions.cpp      # Test frame conversions
    └── test_integration.cpp      # SITL integration test
```

---

## Testing

### SITL Integration Test

```cpp
// test/test_integration.cpp

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "hardware_abstraction/hardware_abstraction_node.hpp"

class HardwareAbstractionTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<aerion::hardware_abstraction::HardwareAbstractionNode>(
            rclcpp::NodeOptions());
    }
    
    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }
    
    std::shared_ptr<aerion::hardware_abstraction::HardwareAbstractionNode> node_;
};

TEST_F(HardwareAbstractionTest, PublishesState) {
    // Verify state is published when PX4 data arrives
    bool received = false;
    auto sub = node_->create_subscription<aerion_interfaces::msg::State>(
        "hardware_abstraction/px4_state", 10,
        [&received](aerion_interfaces::msg::State::SharedPtr) {
            received = true;
        });
    
    // Wait for message (requires SITL running)
    auto start = std::chrono::steady_clock::now();
    while (!received && 
           std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
        rclcpp::spin_some(node_);
    }
    
    EXPECT_TRUE(received);
}
```
