# controller_manager

**Package Type:** ROS2 Node Package with Plugin Architecture  
**Dependencies:** aerion_interfaces, frame_transforms, pluginlib, rclcpp, Eigen3  

---

## Overview

`controller_manager` provides **plugin-based flight control** for the AERION flight stack. This package:

1. **Loads controller plugins** at runtime using pluginlib
2. **Manages controller lifecycle** (activate/deactivate/switch mid-flight)
3. **Routes control outputs** to hardware_abstraction
4. **Supports multiple control modes** (position, velocity, attitude, body rate)

---

## Architecture

### Control Hierarchy

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           CONTROL ARCHITECTURE                                   │
│                                                                                  │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │                      TRAJECTORY MANAGER                                    │  │
│  │   Provides: TrajectorySetpoint (position, velocity, acceleration, yaw)    │  │
│  └───────────────────────────────────────┬───────────────────────────────────┘  │
│                                          │                                       │
│                                          ▼                                       │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │                      CONTROLLER MANAGER                                    │  │
│  │                                                                            │  │
│  │   ┌──────────────────────────────────────────────────────────────────┐   │  │
│  │   │                    ACTIVE CONTROLLER PLUGIN                       │   │  │
│  │   │                                                                   │   │  │
│  │   │  Options:                                                         │   │  │
│  │   │    • PX4PassthroughController (sends position/velocity to PX4)   │   │  │
│  │   │    • GeometricController (computes attitude + thrust)            │   │  │
│  │   │    • MpcController (model predictive control)                    │   │  │
│  │   │    • CustomResearchController (your research!)                   │   │  │
│  │   │                                                                   │   │  │
│  │   └──────────────────────────────────────┬────────────────────────────┘   │  │
│  │                                          │                                │  │
│  │   Output: ControlOutput (mode-dependent)                                  │  │
│  │     • POSITION mode  → position + yaw                                    │  │
│  │     • VELOCITY mode  → velocity + yaw_rate                               │  │
│  │     • ATTITUDE mode  → quaternion + thrust                               │  │
│  │     • BODY_RATE mode → angular rates + thrust                            │  │
│  └───────────────────────────────────────┬───────────────────────────────────┘  │
│                                          │                                       │
│                                          ▼                                       │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │                    HARDWARE ABSTRACTION                                    │  │
│  │   Converts ENU/FLU → NED/FRD and publishes to PX4                        │  │
│  └───────────────────────────────────────────────────────────────────────────┘  │
│                                          │                                       │
│                                          ▼                                       │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │                           PX4 AUTOPILOT                                    │  │
│  │                                                                            │  │
│  │   If POSITION/VELOCITY mode:                                              │  │
│  │     PX4 Position Controller → Attitude Controller → Rate Controller       │  │
│  │                                                                            │  │
│  │   If ATTITUDE mode:                                                        │  │
│  │     PX4 Attitude Controller → Rate Controller                             │  │
│  │                                                                            │  │
│  │   If BODY_RATE mode:                                                       │  │
│  │     PX4 Rate Controller only                                              │  │
│  │                                                                            │  │
│  └───────────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Plugin System

```
┌─────────────────────────────────────────────────────────────────┐
│                    CONTROLLER MANAGER                            │
│                                                                  │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │                   Plugin Loader                          │   │
│   │              (pluginlib::ClassLoader)                    │   │
│   └─────────────────────────────────────────────────────────┘   │
│                             │                                    │
│         ┌───────────────────┼───────────────────┐               │
│         ▼                   ▼                   ▼               │
│   ┌───────────┐       ┌───────────┐       ┌───────────┐        │
│   │    PX4    │       │ Geometric │       │    MPC    │        │
│   │Passthrough│       │Controller │       │Controller │        │
│   └─────┬─────┘       └─────┬─────┘       └─────┬─────┘        │
│         │                   │                   │               │
│         └───────────────────┼───────────────────┘               │
│                             ▼                                    │
│              ┌───────────────────────────┐                      │
│              │   ControllerBase Interface │                      │
│              └───────────────────────────┘                      │
└─────────────────────────────────────────────────────────────────┘
```

---

## Plugin Interface

### Base Class

```cpp
// controller_manager/include/controller_manager/controller_base.hpp

#ifndef AERION_CONTROLLER_MANAGER_CONTROLLER_BASE_HPP
#define AERION_CONTROLLER_MANAGER_CONTROLLER_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <aerion_interfaces/msg/state.hpp>
#include <aerion_interfaces/msg/trajectory_setpoint.hpp>
#include <aerion_interfaces/msg/control_output.hpp>

namespace aerion::control {

/**
 * @brief Control output mode - determines what hardware_abstraction sends to PX4
 */
enum class ControlMode {
    POSITION,    // Position + yaw setpoint → PX4 position controller
    VELOCITY,    // Velocity + yaw_rate → PX4 velocity controller
    ATTITUDE,    // Quaternion + thrust → PX4 attitude controller
    BODY_RATE    // Angular rates + thrust → PX4 rate controller
};

/**
 * @brief Base class for all controller plugins
 * 
 * Controllers receive state estimates and trajectory setpoints,
 * and produce control outputs. The output mode determines which
 * PX4 controller layer processes the commands.
 */
class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    
    /**
     * @brief Initialize the controller
     * @param node Shared pointer to parent node (for params, logging)
     * @param name Unique name for this controller instance
     * @return true if initialization succeeded
     */
    virtual bool initialize(
        const rclcpp::Node::SharedPtr& node,
        const std::string& name) = 0;
    
    /**
     * @brief Activate the controller (start computing outputs)
     * @return true if activation succeeded
     */
    virtual bool activate() = 0;
    
    /**
     * @brief Deactivate the controller (stop computing outputs)
     * @return true if deactivation succeeded
     */
    virtual bool deactivate() = 0;
    
    /**
     * @brief Compute control output
     * @param current_state Current UAV state (ENU/FLU)
     * @param reference Trajectory setpoint to track
     * @param dt Time since last update [s]
     * @return Control output (in mode specified by getControlMode())
     */
    virtual aerion_interfaces::msg::ControlOutput compute(
        const aerion_interfaces::msg::State& current_state,
        const aerion_interfaces::msg::TrajectorySetpoint& reference,
        double dt) = 0;
    
    /**
     * @brief Get controller name
     */
    virtual std::string getName() const = 0;
    
    /**
     * @brief Get the control mode this controller outputs
     */
    virtual ControlMode getControlMode() const = 0;
    
    /**
     * @brief Check if controller is healthy
     */
    virtual bool isHealthy() const = 0;
    
    /**
     * @brief Reset controller internal state
     */
    virtual void reset() = 0;
    
    /**
     * @brief Get controller parameters (for introspection)
     */
    virtual std::map<std::string, double> getParameters() const = 0;
    
    /**
     * @brief Set controller parameters dynamically
     * @param params Map of parameter names to values
     * @return true if all parameters were set successfully
     */
    virtual bool setParameters(const std::map<std::string, double>& params) = 0;
    
protected:
    rclcpp::Node::SharedPtr node_;
    std::string name_;
    bool active_{false};
};

}  // namespace aerion::control

#endif  // AERION_CONTROLLER_MANAGER_CONTROLLER_BASE_HPP
```

### PX4 Passthrough Controller (Default)

```cpp
// controller_plugins/src/px4_passthrough_controller.cpp

namespace aerion::control {

/**
 * @brief Passthrough controller - sends position/velocity directly to PX4
 * 
 * This is the simplest controller. It takes trajectory setpoints and
 * passes them to PX4's internal position/velocity controller. Use this
 * when you want PX4 to handle all the control.
 * 
 * Output Mode: POSITION or VELOCITY (configurable)
 */
class Px4PassthroughController : public ControllerBase {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node,
                   const std::string& name) override {
        node_ = node;
        name_ = name;
        
        // Load parameters
        use_velocity_mode_ = node_->declare_parameter(
            name_ + ".use_velocity_mode", false);
            
        RCLCPP_INFO(node_->get_logger(),
            "PX4 Passthrough controller initialized: %s (mode: %s)",
            name_.c_str(), use_velocity_mode_ ? "VELOCITY" : "POSITION");
        return true;
    }
    
    bool activate() override {
        active_ = true;
        return true;
    }
    
    bool deactivate() override {
        active_ = false;
        return true;
    }
    
    aerion_interfaces::msg::ControlOutput compute(
        const aerion_interfaces::msg::State& current_state,
        const aerion_interfaces::msg::TrajectorySetpoint& reference,
        double dt) override {
        
        aerion_interfaces::msg::ControlOutput output;
        output.header.stamp = node_->get_clock()->now();
        output.controller_name = name_;
        
        if (use_velocity_mode_ && reference.velocity_valid) {
            output.control_mode = 
                aerion_interfaces::msg::ControlOutput::CONTROL_MODE_VELOCITY;
            output.velocity = reference.velocity;
            output.yaw_rate_radps = reference.yaw_rate_radps;
        } else {
            output.control_mode = 
                aerion_interfaces::msg::ControlOutput::CONTROL_MODE_POSITION;
            output.position = reference.position;
            output.yaw_rad = reference.yaw_rad;
        }
        
        return output;
    }
    
    std::string getName() const override { return name_; }
    
    ControlMode getControlMode() const override {
        return use_velocity_mode_ ? ControlMode::VELOCITY : ControlMode::POSITION;
    }
    
    bool isHealthy() const override { return active_; }
    void reset() override { /* No internal state */ }
    
    std::map<std::string, double> getParameters() const override {
        return {{"use_velocity_mode", use_velocity_mode_ ? 1.0 : 0.0}};
    }
    
    bool setParameters(const std::map<std::string, double>& params) override {
        auto it = params.find("use_velocity_mode");
        if (it != params.end()) {
            use_velocity_mode_ = it->second > 0.5;
        }
        return true;
    }
    
private:
    bool use_velocity_mode_{false};
};

}  // namespace aerion::control

PLUGINLIB_EXPORT_CLASS(
    aerion::control::Px4PassthroughController,
    aerion::control::ControllerBase)
```

### Geometric Controller Example

```cpp
// controller_plugins/src/geometric_controller.cpp

namespace aerion::control {

/**
 * @brief Geometric tracking controller for quadrotors
 * 
 * Implements the geometric controller from:
 * "Geometric Tracking Control of a Quadrotor UAV on SE(3)"
 * by T. Lee, M. Leok, and N. H. McClamroch
 * 
 * Output Mode: ATTITUDE (quaternion + thrust)
 * 
 * This controller computes desired attitude and thrust based on
 * position/velocity errors, then sends to PX4's attitude controller.
 */
class GeometricController : public ControllerBase {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node,
                   const std::string& name) override {
        node_ = node;
        name_ = name;
        
        // Position gains
        kp_pos_ = node_->declare_parameter(name_ + ".kp_pos", 
            std::vector<double>{6.0, 6.0, 8.0});
        kd_pos_ = node_->declare_parameter(name_ + ".kd_pos", 
            std::vector<double>{4.5, 4.5, 5.0});
        
        // Attitude gains
        kp_att_ = node_->declare_parameter(name_ + ".kp_att", 
            std::vector<double>{8.0, 8.0, 2.0});
        kd_att_ = node_->declare_parameter(name_ + ".kd_att", 
            std::vector<double>{1.5, 1.5, 0.4});
        
        // Vehicle parameters
        mass_ = node_->declare_parameter(name_ + ".mass", 1.5);  // kg
        gravity_ = node_->declare_parameter(name_ + ".gravity", 9.81);
        max_thrust_ = node_->declare_parameter(name_ + ".max_thrust", 30.0);  // N
        
        RCLCPP_INFO(node_->get_logger(),
            "Geometric controller initialized: %s (mass: %.2f kg)",
            name_.c_str(), mass_);
        return true;
    }
    
    aerion_interfaces::msg::ControlOutput compute(
        const aerion_interfaces::msg::State& state,
        const aerion_interfaces::msg::TrajectorySetpoint& ref,
        double dt) override {
        
        using namespace Eigen;
        
        // Current state (ENU/FLU)
        Vector3d pos(state.position.x, state.position.y, state.position.z);
        Vector3d vel(state.velocity.x, state.velocity.y, state.velocity.z);
        Quaterniond q(state.orientation.w, state.orientation.x,
                      state.orientation.y, state.orientation.z);
        Matrix3d R = q.toRotationMatrix();
        
        // Reference
        Vector3d pos_des(ref.position.x, ref.position.y, ref.position.z);
        Vector3d vel_des = ref.velocity_valid ? 
            Vector3d(ref.velocity.x, ref.velocity.y, ref.velocity.z) :
            Vector3d::Zero();
        Vector3d acc_des = ref.acceleration_valid ?
            Vector3d(ref.acceleration.x, ref.acceleration.y, ref.acceleration.z) :
            Vector3d::Zero();
        double yaw_des = ref.yaw_valid ? ref.yaw_rad : state.yaw_rad;
        
        // Position error
        Vector3d e_pos = pos - pos_des;
        Vector3d e_vel = vel - vel_des;
        
        // Gain matrices
        Matrix3d Kp = Vector3d(kp_pos_[0], kp_pos_[1], kp_pos_[2]).asDiagonal();
        Matrix3d Kd = Vector3d(kd_pos_[0], kd_pos_[1], kd_pos_[2]).asDiagonal();
        
        // Desired acceleration (with gravity compensation)
        Vector3d gravity_vec(0, 0, -gravity_);
        Vector3d a_des = -Kp * e_pos - Kd * e_vel + acc_des - gravity_vec;
        
        // Desired thrust (project onto body z-axis)
        Vector3d z_body = R.col(2);  // Body z-axis in world frame
        double thrust = mass_ * a_des.dot(z_body);
        thrust = std::clamp(thrust, 0.0, max_thrust_);
        
        // Desired body z-axis (thrust direction)
        Vector3d z_des = a_des.normalized();
        if (a_des.norm() < 1e-6) {
            z_des = Vector3d(0, 0, 1);
        }
        
        // Desired yaw gives us x-axis constraint
        Vector3d x_c(cos(yaw_des), sin(yaw_des), 0);
        
        // Construct desired rotation matrix
        Vector3d y_des = z_des.cross(x_c).normalized();
        Vector3d x_des = y_des.cross(z_des);
        Matrix3d R_des;
        R_des.col(0) = x_des;
        R_des.col(1) = y_des;
        R_des.col(2) = z_des;
        
        // Convert to quaternion
        Quaterniond q_des(R_des);
        q_des.normalize();
        
        // Build output
        aerion_interfaces::msg::ControlOutput output;
        output.header.stamp = node_->get_clock()->now();
        output.controller_name = name_;
        output.control_mode = 
            aerion_interfaces::msg::ControlOutput::CONTROL_MODE_ATTITUDE;
        
        output.orientation.w = q_des.w();
        output.orientation.x = q_des.x();
        output.orientation.y = q_des.y();
        output.orientation.z = q_des.z();
        output.thrust_normalized = thrust / max_thrust_;
        
        return output;
    }
    
    // ... other methods ...
    
    ControlMode getControlMode() const override { return ControlMode::ATTITUDE; }
    
private:
    std::vector<double> kp_pos_, kd_pos_;
    std::vector<double> kp_att_, kd_att_;
    double mass_, gravity_, max_thrust_;
};

}  // namespace aerion::control

PLUGINLIB_EXPORT_CLASS(
    aerion::control::GeometricController,
    aerion::control::ControllerBase)
```

---

## Manager Node Implementation

### Core Structure

```cpp
// controller_manager/include/controller_manager/controller_manager_node.hpp

namespace aerion::control {

class ControllerManagerNode : public rclcpp::Node {
public:
    explicit ControllerManagerNode(const rclcpp::NodeOptions& options);
    
private:
    // Plugin management
    void loadPlugins();
    bool switchController(const std::string& name);
    
    // Control loop
    void controlLoop();
    
    // Callbacks
    void onState(const aerion_interfaces::msg::State::SharedPtr msg);
    void onTrajectorySetpoint(
        const aerion_interfaces::msg::TrajectorySetpoint::SharedPtr msg);
    
    // Services
    void setControllerCallback(
        const aerion_interfaces::srv::SetController::Request::SharedPtr req,
        aerion_interfaces::srv::SetController::Response::SharedPtr res);
    
    // Plugin loader
    std::unique_ptr<pluginlib::ClassLoader<ControllerBase>> plugin_loader_;
    
    // Loaded plugins
    std::map<std::string, std::shared_ptr<ControllerBase>> controllers_;
    
    // Active controller
    std::shared_ptr<ControllerBase> active_controller_;
    std::string active_controller_name_;
    std::mutex controller_mutex_;
    
    // State and reference storage
    aerion_interfaces::msg::State current_state_;
    aerion_interfaces::msg::TrajectorySetpoint current_reference_;
    std::mutex data_mutex_;
    bool state_received_{false};
    bool reference_received_{false};
    
    // Timing
    rclcpp::Time last_control_time_;
    
    // Subscribers
    rclcpp::Subscription<aerion_interfaces::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<aerion_interfaces::msg::TrajectorySetpoint>::SharedPtr 
        reference_sub_;
    
    // Publishers
    rclcpp::Publisher<aerion_interfaces::msg::ControlOutput>::SharedPtr 
        control_output_pub_;
    rclcpp::Publisher<aerion_interfaces::msg::ControllerStatus>::SharedPtr 
        status_pub_;
    
    // Services
    rclcpp::Service<aerion_interfaces::srv::SetController>::SharedPtr 
        set_controller_srv_;
    
    // Control timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Parameters
    double control_rate_hz_;
    std::string default_controller_;
    std::vector<std::string> plugin_names_;
};

}  // namespace aerion::control
```

### Control Loop

```cpp
void ControllerManagerNode::controlLoop() {
    std::lock_guard<std::mutex> ctrl_lock(controller_mutex_);
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    
    if (!active_controller_ || !state_received_) {
        return;  // Wait for controller and state
    }
    
    // Compute dt
    auto now = this->get_clock()->now();
    double dt = (now - last_control_time_).seconds();
    last_control_time_ = now;
    
    // Sanity check dt
    if (dt <= 0.0 || dt > 0.5) {
        dt = 1.0 / control_rate_hz_;  // Use nominal dt
    }
    
    // Default reference if none received
    aerion_interfaces::msg::TrajectorySetpoint reference;
    if (reference_received_) {
        reference = current_reference_;
    } else {
        // Hold current position if no reference
        reference.position = current_state_.position;
        reference.position_valid = true;
        reference.yaw_rad = current_state_.yaw_rad;
        reference.yaw_valid = true;
    }
    
    // Compute control output
    auto output = active_controller_->compute(current_state_, reference, dt);
    
    // Publish
    control_output_pub_->publish(output);
}
```

---

## Configuration

### Parameters

```yaml
# config/controller_manager.yaml

controller_manager:
  ros__parameters:
    # Control loop rate
    control_rate_hz: 250.0
    
    # Default controller to activate on startup
    default_controller: "px4_passthrough"
    
    # List of plugins to load
    plugins:
      - "px4_passthrough"
      - "geometric_controller"
      # - "mpc_controller"  # Add when implemented
    
    # PX4 Passthrough controller config
    px4_passthrough:
      use_velocity_mode: false
    
    # Geometric controller config
    geometric_controller:
      kp_pos: [6.0, 6.0, 8.0]
      kd_pos: [4.5, 4.5, 5.0]
      kp_att: [8.0, 8.0, 2.0]
      kd_att: [1.5, 1.5, 0.4]
      mass: 1.5
      gravity: 9.81
      max_thrust: 30.0
```

---

## Usage

### Runtime Controller Switching

```bash
# Switch to geometric controller
ros2 service call /controller_manager/set_controller \
    aerion_interfaces/srv/SetController \
    "{controller_name: 'geometric_controller'}"

# Check status
ros2 topic echo /controller_manager/status
```

### Adding Custom Controllers

1. **Create Plugin Class** (implement ControllerBase)
2. **Register Plugin** (PLUGINLIB_EXPORT_CLASS macro)
3. **Add Plugin XML** (plugins.xml)
4. **Configure Parameters** (config yaml)

---

## Major Cautions

### ⚠️ Controller Switching Mid-Flight
- Switching from POSITION to ATTITUDE mode causes control discontinuity
- Ensure new controller's initial output is close to current state
- Consider "bumpless transfer" implementation

### ⚠️ Gain Tuning
- Gains from simulation often don't transfer to hardware
- Start with conservative gains, tune on hardware
- Different controllers may need different gains for same vehicle

### ⚠️ Thrust Mapping
- `thrust_normalized` [0,1] maps differently on each vehicle
- PX4 hover thrust parameter affects mapping
- Test hover first with passthrough controller

### ⚠️ Control Loop Timing
- Control loop must be deterministic
- Don't do blocking operations in compute()
- Log timing violations

### ⚠️ State Validity
- Check state timestamp freshness
- Handle estimator failures gracefully
- Consider switching to safe controller if state is stale

---

## File Structure

```
controller_manager/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── controller_manager/
│       ├── controller_base.hpp
│       └── controller_manager_node.hpp
├── src/
│   ├── controller_manager_node.cpp
│   └── main.cpp
├── plugins/
│   ├── px4_passthrough_controller.cpp
│   ├── geometric_controller.cpp
│   └── plugins.xml
├── config/
│   └── controller_manager.yaml
├── launch/
│   └── controller_manager.launch.py
└── test/
    ├── test_passthrough.cpp
    └── test_geometric.cpp
```
