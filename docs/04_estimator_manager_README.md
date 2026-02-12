# estimator_manager

**Package Type:** ROS2 Node Package with Plugin Architecture  
**Dependencies:** aerion_interfaces, frame_transforms, pluginlib, rclcpp  

---

## Overview

`estimator_manager` provides **plugin-based state estimation** for the AERION flight stack. This package:

1. **Loads estimator plugins** at runtime using pluginlib
2. **Manages estimator lifecycle** (activate/deactivate/switch)
3. **Publishes unified state** to the rest of the stack
4. **Monitors estimator health** and confidence

---

## Architecture

### Plugin System

```
┌─────────────────────────────────────────────────────────────────┐
│                      ESTIMATOR MANAGER                           │
│                                                                  │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │                   Plugin Loader                          │   │
│   │              (pluginlib::ClassLoader)                    │   │
│   └─────────────────────────────────────────────────────────┘   │
│                             │                                    │
│         ┌───────────────────┼───────────────────┐               │
│         ▼                   ▼                   ▼               │
│   ┌───────────┐       ┌───────────┐       ┌───────────┐        │
│   │  PX4      │       │  Custom   │       │  Research │        │
│   │Passthrough│       │   EKF     │       │ Estimator │        │
│   │  Plugin   │       │  Plugin   │       │  Plugin   │        │
│   └─────┬─────┘       └─────┬─────┘       └─────┬─────┘        │
│         │                   │                   │               │
│         └───────────────────┼───────────────────┘               │
│                             ▼                                    │
│              ┌───────────────────────────┐                      │
│              │  EstimatorBase Interface  │                      │
│              └───────────────────────────┘                      │
│                             │                                    │
│                             ▼                                    │
│                    Unified State Output                          │
│               (aerion_interfaces/State)                         │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow

```
                     ┌─────────────────┐
                     │    Sensors      │
                     │  (IMU, GPS,     │
                     │   MoCap, etc.)  │
                     └────────┬────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                   hardware_abstraction                           │
│             (publishes /px4_state in ENU/FLU)                   │
└────────────────────────────┬────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     estimator_manager                            │
│                                                                  │
│   Inputs:                                                        │
│     /hardware_abstraction/px4_state   → Raw PX4 state           │
│     /mocap/pose                       → External MoCap          │
│     /vio/odometry                     → Visual-Inertial (future)│
│                                                                  │
│   Processing:                                                    │
│     Active Estimator Plugin processes inputs                     │
│     Health/confidence monitoring                                 │
│                                                                  │
│   Outputs:                                                       │
│     /estimator_manager/state          → Estimated state         │
│     /estimator_manager/status         → Estimator health        │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    controller_manager
                    trajectory_manager
                    safety_monitor
```

---

## Plugin Interface

### Base Class

```cpp
// estimator_manager/include/estimator_manager/estimator_base.hpp

#ifndef AERION_ESTIMATOR_MANAGER_ESTIMATOR_BASE_HPP
#define AERION_ESTIMATOR_MANAGER_ESTIMATOR_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <aerion_interfaces/msg/state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace aerion::estimator {

/**
 * @brief Base class for all estimator plugins
 * 
 * Estimator plugins receive sensor data and produce state estimates.
 * The simplest plugin (passthrough) just forwards PX4's estimate.
 * Complex plugins can fuse multiple sensors with custom filters.
 */
class EstimatorBase {
public:
    virtual ~EstimatorBase() = default;
    
    /**
     * @brief Initialize the estimator
     * @param node Shared pointer to parent node (for params, logging)
     * @param name Unique name for this estimator instance
     * @return true if initialization succeeded
     */
    virtual bool initialize(
        const rclcpp::Node::SharedPtr& node,
        const std::string& name) = 0;
    
    /**
     * @brief Activate the estimator (start processing)
     */
    virtual bool activate() = 0;
    
    /**
     * @brief Deactivate the estimator (stop processing)
     */
    virtual bool deactivate() = 0;
    
    /**
     * @brief Process new PX4 state (called at PX4 rate)
     * @param px4_state State from hardware_abstraction (ENU/FLU)
     */
    virtual void processPX4State(
        const aerion_interfaces::msg::State& px4_state) = 0;
    
    /**
     * @brief Inject external pose (e.g., from MoCap)
     * @param pose External pose measurement (ENU/FLU)
     */
    virtual void injectExternalPose(
        const geometry_msgs::msg::PoseStamped& pose) = 0;
    
    /**
     * @brief Inject IMU data (if plugin does own integration)
     * @param imu IMU measurement
     */
    virtual void injectIMU(
        const sensor_msgs::msg::Imu& imu) {}
    
    /**
     * @brief Get current state estimate
     * @return Current estimated state (ENU/FLU)
     */
    virtual aerion_interfaces::msg::State getState() const = 0;
    
    /**
     * @brief Get estimator name
     */
    virtual std::string getName() const = 0;
    
    /**
     * @brief Get estimator type
     */
    virtual EstimatorType getType() const = 0;
    
    /**
     * @brief Check if estimator is healthy
     */
    virtual bool isHealthy() const = 0;
    
    /**
     * @brief Get confidence in current estimate [0.0 - 1.0]
     */
    virtual double getConfidence() const = 0;
    
    /**
     * @brief Reset estimator state
     * @param initial_state Optional initial state
     */
    virtual void reset(
        const aerion_interfaces::msg::State* initial_state = nullptr) = 0;
    
protected:
    rclcpp::Node::SharedPtr node_;
    std::string name_;
    bool active_{false};
};

enum class EstimatorType {
    PASSTHROUGH,    // Forward PX4 estimate directly
    EKF,            // Extended Kalman Filter
    UKF,            // Unscented Kalman Filter
    COMPLEMENTARY,  // Complementary filter
    CUSTOM          // Research/custom implementation
};

}  // namespace aerion::estimator

#endif  // AERION_ESTIMATOR_MANAGER_ESTIMATOR_BASE_HPP
```

### Plugin Registration

```cpp
// estimator_plugins/src/px4_passthrough.cpp

#include <pluginlib/class_list_macros.hpp>
#include "estimator_manager/estimator_base.hpp"

namespace aerion::estimator {

/**
 * @brief Passthrough estimator - forwards PX4 state directly
 * 
 * This is the default estimator. It simply passes through the
 * state estimate from PX4's EKF2, with optional external pose
 * injection support for MoCap environments.
 */
class PX4PassthroughEstimator : public EstimatorBase {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node,
                   const std::string& name) override {
        node_ = node;
        name_ = name;
        
        // Load parameters
        use_external_pose_ = node_->declare_parameter(
            name_ + ".use_external_pose", false);
        external_pose_timeout_s_ = node_->declare_parameter(
            name_ + ".external_pose_timeout_s", 0.1);
            
        RCLCPP_INFO(node_->get_logger(), 
            "PX4 Passthrough estimator initialized: %s", name_.c_str());
        return true;
    }
    
    bool activate() override {
        active_ = true;
        healthy_ = true;
        confidence_ = 1.0;
        return true;
    }
    
    bool deactivate() override {
        active_ = false;
        return true;
    }
    
    void processPX4State(const aerion_interfaces::msg::State& px4_state) override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        if (use_external_pose_ && hasRecentExternalPose()) {
            // Fuse external pose with PX4 state
            current_state_ = px4_state;
            current_state_.position = external_pose_.pose.position;
            current_state_.orientation = external_pose_.pose.orientation;
            // Keep velocity/acceleration from PX4
        } else {
            // Pure passthrough
            current_state_ = px4_state;
        }
        
        current_state_.estimator_name = name_;
        current_state_.confidence = confidence_;
    }
    
    void injectExternalPose(
        const geometry_msgs::msg::PoseStamped& pose) override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        external_pose_ = pose;
        last_external_pose_time_ = node_->get_clock()->now();
    }
    
    aerion_interfaces::msg::State getState() const override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_state_;
    }
    
    std::string getName() const override { return name_; }
    EstimatorType getType() const override { return EstimatorType::PASSTHROUGH; }
    bool isHealthy() const override { return healthy_; }
    double getConfidence() const override { return confidence_; }
    
    void reset(const aerion_interfaces::msg::State* initial_state) override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (initial_state) {
            current_state_ = *initial_state;
        }
    }
    
private:
    bool hasRecentExternalPose() const {
        auto now = node_->get_clock()->now();
        auto age = (now - last_external_pose_time_).seconds();
        return age < external_pose_timeout_s_;
    }
    
    mutable std::mutex state_mutex_;
    aerion_interfaces::msg::State current_state_;
    geometry_msgs::msg::PoseStamped external_pose_;
    rclcpp::Time last_external_pose_time_;
    
    bool use_external_pose_{false};
    double external_pose_timeout_s_{0.1};
    bool healthy_{false};
    double confidence_{0.0};
};

}  // namespace aerion::estimator

// Register plugin
PLUGINLIB_EXPORT_CLASS(
    aerion::estimator::PX4PassthroughEstimator,
    aerion::estimator::EstimatorBase)
```

### Plugin Registration XML

```xml
<!-- estimator_plugins/plugins.xml -->
<library path="estimator_plugins">
  <class name="aerion::estimator::PX4PassthroughEstimator"
         type="aerion::estimator::PX4PassthroughEstimator"
         base_class_type="aerion::estimator::EstimatorBase">
    <description>
      Passthrough estimator that forwards PX4 state directly.
      Optionally fuses external pose (MoCap) for position.
    </description>
  </class>
  
  <!-- Add more estimator plugins here -->
</library>
```

---

## Manager Node Implementation

```cpp
// estimator_manager/include/estimator_manager/estimator_manager_node.hpp

namespace aerion::estimator {

class EstimatorManagerNode : public rclcpp::Node {
public:
    explicit EstimatorManagerNode(const rclcpp::NodeOptions& options);
    
private:
    // Plugin management
    void loadPlugins();
    bool switchEstimator(const std::string& name);
    
    // Callbacks
    void onPX4State(const aerion_interfaces::msg::State::SharedPtr msg);
    void onExternalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishLoop();
    void healthCheckLoop();
    
    // Services
    void setEstimatorCallback(
        const aerion_interfaces::srv::SetEstimator::Request::SharedPtr request,
        aerion_interfaces::srv::SetEstimator::Response::SharedPtr response);
    
    // Plugin loader
    std::unique_ptr<pluginlib::ClassLoader<EstimatorBase>> plugin_loader_;
    
    // Loaded plugins
    std::map<std::string, std::shared_ptr<EstimatorBase>> estimators_;
    
    // Active estimator
    std::shared_ptr<EstimatorBase> active_estimator_;
    std::string active_estimator_name_;
    std::mutex estimator_mutex_;
    
    // Subscribers
    rclcpp::Subscription<aerion_interfaces::msg::State>::SharedPtr px4_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr external_pose_sub_;
    
    // Publishers
    rclcpp::Publisher<aerion_interfaces::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<aerion_interfaces::msg::EstimatorStatus>::SharedPtr status_pub_;
    
    // Services
    rclcpp::Service<aerion_interfaces::srv::SetEstimator>::SharedPtr set_estimator_srv_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    
    // Parameters
    double publish_rate_hz_;
    std::string default_estimator_;
    std::vector<std::string> plugin_names_;
};

}  // namespace aerion::estimator
```

---

## Configuration

### Parameters

```yaml
# config/estimator_manager.yaml

estimator_manager:
  ros__parameters:
    # Publishing rate
    publish_rate_hz: 250.0
    
    # Default estimator to activate on startup
    default_estimator: "px4_passthrough"
    
    # List of plugins to load
    plugins:
      - "px4_passthrough"
      # - "custom_ekf"  # Add custom plugins here
    
    # Passthrough estimator config
    px4_passthrough:
      use_external_pose: false
      external_pose_timeout_s: 0.1
    
    # Health monitoring
    health_check_rate_hz: 10.0
    confidence_threshold: 0.5  # Switch to backup if below this
    
    # Backup estimator (used if primary fails)
    backup_estimator: "px4_passthrough"
```

### MoCap Environment Configuration

```yaml
# config/estimator_manager_mocap.yaml

estimator_manager:
  ros__parameters:
    publish_rate_hz: 250.0
    default_estimator: "px4_passthrough"
    
    plugins:
      - "px4_passthrough"
    
    px4_passthrough:
      use_external_pose: true           # Enable MoCap fusion
      external_pose_timeout_s: 0.05     # 50ms timeout (20Hz MoCap)
```

---

## Usage

### Launch File

```python
# launch/estimator_manager.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('estimator_manager'),
        'config',
        'estimator_manager.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='estimator_manager',
            executable='estimator_manager_node',
            name='estimator_manager',
            parameters=[config],
            output='screen',
        ),
    ])
```

### Runtime Estimator Switching

```bash
# Switch to a different estimator at runtime
ros2 service call /estimator_manager/set_estimator \
    aerion_interfaces/srv/SetEstimator \
    "{estimator_name: 'custom_ekf'}"

# Check current estimator status
ros2 topic echo /estimator_manager/status
```

---

## Adding Custom Estimators

### Step 1: Create Plugin Class

```cpp
// my_estimator_plugins/src/my_custom_ekf.cpp

#include "estimator_manager/estimator_base.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace my_plugins {

class MyCustomEKF : public aerion::estimator::EstimatorBase {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node,
                   const std::string& name) override {
        node_ = node;
        name_ = name;
        
        // Initialize your EKF here
        // Load parameters, allocate matrices, etc.
        
        return true;
    }
    
    // Implement all virtual methods...
    
private:
    // Your EKF state, matrices, etc.
    Eigen::VectorXd state_;
    Eigen::MatrixXd P_;  // Covariance
    // ...
};

}  // namespace my_plugins

PLUGINLIB_EXPORT_CLASS(
    my_plugins::MyCustomEKF,
    aerion::estimator::EstimatorBase)
```

### Step 2: Register Plugin

```xml
<!-- my_estimator_plugins/plugins.xml -->
<library path="my_estimator_plugins">
  <class name="my_plugins::MyCustomEKF"
         type="my_plugins::MyCustomEKF"
         base_class_type="aerion::estimator::EstimatorBase">
    <description>Custom EKF implementation for research</description>
  </class>
</library>
```

### Step 3: CMakeLists.txt

```cmake
# Export plugin
pluginlib_export_plugin_description_file(
    estimator_manager plugins.xml)
```

### Step 4: Add to Config

```yaml
estimator_manager:
  ros__parameters:
    plugins:
      - "px4_passthrough"
      - "my_custom_ekf"     # Your new plugin
    
    my_custom_ekf:
      process_noise: 0.01
      measurement_noise: 0.1
      # ... your plugin's parameters
```

---

## Future: Multi-Sensor Fusion Pointers

When adding VIO, LiDAR odometry, or other sensors:

### 1. Define Sensor Interfaces

```cpp
// Additional input methods for EstimatorBase
virtual void injectVIO(const nav_msgs::msg::Odometry& vio) {}
virtual void injectLidarOdometry(const nav_msgs::msg::Odometry& odom) {}
virtual void injectGPS(const sensor_msgs::msg::NavSatFix& gps) {}
```

### 2. Create Fusion Estimator Plugin

```cpp
class MultiSensorFusionEKF : public EstimatorBase {
    // Implements proper sensor fusion with:
    // - IMU prediction (high rate)
    // - GPS/VIO/LiDAR updates (lower rates)
    // - Proper covariance handling
    // - Outlier rejection
};
```

### 3. Sensor Manager (Separate Package)

Consider a `sensor_manager` package that:
- Handles sensor preprocessing
- Detects sensor failures
- Routes data to estimator
- Provides unified sensor status

---

## Major Cautions

### ⚠️ Plugin Lifecycle
- Always deactivate old estimator before activating new one
- Handle the transition period gracefully
- Don't switch estimators during critical maneuvers

### ⚠️ State Discontinuities
- Switching estimators can cause state jumps
- Implement state transfer between estimators if possible
- Warn the controller manager before switching

### ⚠️ External Pose Timing
- MoCap data may have different timestamps
- Handle clock synchronization properly
- Use appropriate timeout for pose validity

### ⚠️ Confidence Degradation
- Monitor estimator confidence continuously
- Have automatic fallback to backup estimator
- Log all confidence drops for debugging

### ⚠️ Thread Safety
- State access must be thread-safe
- Use proper locking or lock-free structures
- Don't block the main processing loop

### ⚠️ Covariance Validity
- Ensure covariance matrices remain positive definite
- Implement covariance bounds checking
- Log when covariance grows unexpectedly

---

## File Structure

```
estimator_manager/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── estimator_manager/
│       ├── estimator_base.hpp
│       └── estimator_manager_node.hpp
├── src/
│   ├── estimator_manager_node.cpp
│   └── main.cpp
├── plugins/
│   ├── px4_passthrough.cpp
│   └── plugins.xml
├── config/
│   ├── estimator_manager.yaml
│   └── estimator_manager_mocap.yaml
├── launch/
│   └── estimator_manager.launch.py
└── test/
    ├── test_passthrough.cpp
    └── test_switching.cpp
```
