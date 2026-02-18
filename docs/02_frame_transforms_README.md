# frame_transforms

**Package Type:** ROS2 Library Package  
**Dependencies:** peregrine_interfaces, Eigen3, tf2, tf2_ros, tf2_eigen, geometry_msgs  

---

## Overview

`frame_transforms` provides **coordinate frame transformation utilities** for the PEREGRINE flight stack. This package serves two critical functions:

1. **Static Transformations**: Convert between ROS conventions (ENU/FLU) and PX4 conventions (NED/FRD)
2. **Dynamic TF Broadcasting**: Publish TF2 transforms for the UAV and sensor frames

This package is the **single source of truth** for frame conventions in PEREGRINE.

---

## Design Philosophy

### Single Conversion Point
Frame conversions between ROS and PX4 conventions happen **only** at the hardware abstraction boundary. All other packages work in ENU/FLU.

### Explicit Over Implicit
Rather than auto-detecting frames, functions explicitly state their input and output conventions.

### Performance Critical
These functions are called at high rates (250Hz+), so they must be efficient with zero dynamic allocation.

---

## Frame Conventions Reference

### World Frames

| Frame | Convention | X-Axis | Y-Axis | Z-Axis | Used By |
|-------|------------|--------|--------|--------|---------|
| ENU | ROS Standard | East | North | Up | PEREGRINE |
| NED | Aviation/PX4 | North | East | Down | PX4 |

### Body Frames

| Frame | Convention | X-Axis | Y-Axis | Z-Axis | Used By |
|-------|------------|--------|--------|--------|---------|
| FLU | ROS Standard | Forward | Left | Up | PEREGRINE |
| FRD | Aviation/PX4 | Forward | Right | Down | PX4 |

### Visual Reference

```
         ENU (ROS World)                    NED (PX4 World)
              North                              North
                │                                  │
                │ Y+                               │ X+
                │                                  │
    West ───────┼───────▶ East         West ◀─────┼───────── East
                │         X+                       │           Y+
                │                                  │
              South                              South
                
               Z+ = Up                          Z+ = Down


          FLU (ROS Body)                    FRD (PX4 Body)
             Forward                            Forward
                │                                  │
                │ X+                               │ X+
                │                                  │
     Left ◀─────┼───────── Right        Left ─────┼─────────▶ Right
         Y+     │                                  │           Y+
                │                                  │
            Backward                           Backward
                
               Z+ = Up                          Z+ = Down
```

---

## API Reference

### Position Conversions

```cpp
// frame_transforms/include/frame_transforms/conversions.hpp

namespace peregrine::frame_transforms {

// ═══════════════════════════════════════════════════════════════════
// Position Transformations
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Convert position from ENU to NED
 * 
 * Transformation:
 *   x_ned =  y_enu (North = North)
 *   y_ned =  x_enu (East = East)
 *   z_ned = -z_enu (Down = -Up)
 * 
 * @param enu Position in ENU frame [East, North, Up]
 * @return Position in NED frame [North, East, Down]
 */
inline Eigen::Vector3d enu_to_ned(const Eigen::Vector3d& enu) {
    return Eigen::Vector3d(enu.y(), enu.x(), -enu.z());
}

/**
 * @brief Convert position from NED to ENU
 * 
 * @param ned Position in NED frame [North, East, Down]
 * @return Position in ENU frame [East, North, Up]
 */
inline Eigen::Vector3d ned_to_enu(const Eigen::Vector3d& ned) {
    return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
}

// ROS message overloads
inline geometry_msgs::msg::Point enu_to_ned(
    const geometry_msgs::msg::Point& enu) {
    geometry_msgs::msg::Point ned;
    ned.x = enu.y;
    ned.y = enu.x;
    ned.z = -enu.z;
    return ned;
}

inline geometry_msgs::msg::Point ned_to_enu(
    const geometry_msgs::msg::Point& ned) {
    geometry_msgs::msg::Point enu;
    enu.x = ned.y;
    enu.y = ned.x;
    enu.z = -ned.z;
    return enu;
}

}  // namespace peregrine::frame_transforms
```

### Velocity Conversions

```cpp
namespace peregrine::frame_transforms {

// ═══════════════════════════════════════════════════════════════════
// Velocity Transformations (same as position in linear case)
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Convert linear velocity from ENU to NED
 * @note Same transformation as position
 */
inline Eigen::Vector3d velocity_enu_to_ned(const Eigen::Vector3d& v_enu) {
    return enu_to_ned(v_enu);
}

inline Eigen::Vector3d velocity_ned_to_enu(const Eigen::Vector3d& v_ned) {
    return ned_to_enu(v_ned);
}

// ROS message overloads
inline geometry_msgs::msg::Vector3 velocity_enu_to_ned(
    const geometry_msgs::msg::Vector3& v_enu) {
    geometry_msgs::msg::Vector3 v_ned;
    v_ned.x = v_enu.y;
    v_ned.y = v_enu.x;
    v_ned.z = -v_enu.z;
    return v_ned;
}

inline geometry_msgs::msg::Vector3 velocity_ned_to_enu(
    const geometry_msgs::msg::Vector3& v_ned) {
    geometry_msgs::msg::Vector3 v_enu;
    v_enu.x = v_ned.y;
    v_enu.y = v_ned.x;
    v_enu.z = -v_ned.z;
    return v_enu;
}

}  // namespace peregrine::frame_transforms
```

### Body Frame Conversions

```cpp
namespace peregrine::frame_transforms {

// ═══════════════════════════════════════════════════════════════════
// Body Frame Transformations (FLU ↔ FRD)
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Convert vector from FLU to FRD body frame
 * 
 * Transformation:
 *   x_frd =  x_flu (Forward = Forward)
 *   y_frd = -y_flu (Right = -Left)
 *   z_frd = -z_flu (Down = -Up)
 * 
 * @param flu Vector in FLU body frame
 * @return Vector in FRD body frame
 */
inline Eigen::Vector3d flu_to_frd(const Eigen::Vector3d& flu) {
    return Eigen::Vector3d(flu.x(), -flu.y(), -flu.z());
}

inline Eigen::Vector3d frd_to_flu(const Eigen::Vector3d& frd) {
    return Eigen::Vector3d(frd.x(), -frd.y(), -frd.z());
}

/**
 * @brief Convert angular velocity from FLU to FRD
 * 
 * For angular velocity:
 *   roll_frd  =  roll_flu  (rotation about forward axis)
 *   pitch_frd = -pitch_flu (sign flip due to Y-axis flip)
 *   yaw_frd   = -yaw_flu   (sign flip due to Z-axis flip)
 */
inline Eigen::Vector3d angular_velocity_flu_to_frd(
    const Eigen::Vector3d& omega_flu) {
    return Eigen::Vector3d(omega_flu.x(), -omega_flu.y(), -omega_flu.z());
}

inline Eigen::Vector3d angular_velocity_frd_to_flu(
    const Eigen::Vector3d& omega_frd) {
    return Eigen::Vector3d(omega_frd.x(), -omega_frd.y(), -omega_frd.z());
}

}  // namespace peregrine::frame_transforms
```

### Quaternion Conversions

```cpp
namespace peregrine::frame_transforms {

// ═══════════════════════════════════════════════════════════════════
// Quaternion Transformations
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Convert orientation quaternion from ENU-FLU to NED-FRD
 * 
 * The transformation requires accounting for:
 * 1. World frame rotation (ENU to NED)
 * 2. Body frame rotation (FLU to FRD)
 * 
 * q_ned_frd = R_enu_to_ned * q_enu_flu * R_frd_to_flu^T
 * 
 * @param q_enu_flu Quaternion representing rotation from ENU to FLU (ROS)
 * @return Quaternion representing rotation from NED to FRD (PX4)
 */
inline Eigen::Quaterniond quaternion_enu_flu_to_ned_frd(
    const Eigen::Quaterniond& q_enu_flu) {
    
    // Rotation matrix for ENU to NED (also works for FLU to FRD)
    // This is a 180° rotation about X followed by 90° about Z
    // Simplified: swap X-Y and negate Z
    static const Eigen::Quaterniond q_frame_transform(
        0.70710678118, 0.70710678118, 0.0, 0.0);  // 90° about Z
    static const Eigen::Quaterniond q_flip(
        0.0, 1.0, 0.0, 0.0);  // 180° about X
    
    // Combined transformation
    return q_flip * q_frame_transform * q_enu_flu * 
           q_frame_transform.inverse() * q_flip.inverse();
}

/**
 * @brief Simplified quaternion conversion using axis remapping
 * 
 * For many applications, a simpler approach works:
 * q_ned_frd = [q.w, q.y, q.x, -q.z]
 */
inline Eigen::Quaterniond quaternion_enu_to_ned_simple(
    const Eigen::Quaterniond& q_enu) {
    return Eigen::Quaterniond(q_enu.w(), q_enu.y(), q_enu.x(), -q_enu.z());
}

inline Eigen::Quaterniond quaternion_ned_to_enu_simple(
    const Eigen::Quaterniond& q_ned) {
    return Eigen::Quaterniond(q_ned.w(), q_ned.y(), q_ned.x(), -q_ned.z());
}

// ROS message overloads
inline geometry_msgs::msg::Quaternion quaternion_enu_to_ned(
    const geometry_msgs::msg::Quaternion& q_enu) {
    geometry_msgs::msg::Quaternion q_ned;
    q_ned.w = q_enu.w;
    q_ned.x = q_enu.y;
    q_ned.y = q_enu.x;
    q_ned.z = -q_enu.z;
    return q_ned;
}

}  // namespace peregrine::frame_transforms
```

### Yaw Angle Conversions

```cpp
namespace peregrine::frame_transforms {

// ═══════════════════════════════════════════════════════════════════
// Yaw/Heading Transformations
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Convert yaw from ENU to NED convention
 * 
 * ENU: Yaw measured from East, positive toward North (CCW from above)
 * NED: Yaw measured from North, positive toward East (CW from above)
 * 
 * yaw_ned = π/2 - yaw_enu
 * 
 * @param yaw_enu Yaw in ENU convention [rad]
 * @return Yaw in NED convention [rad]
 */
inline double yaw_enu_to_ned(double yaw_enu) {
    double yaw_ned = M_PI_2 - yaw_enu;
    // Normalize to [-π, π]
    while (yaw_ned > M_PI) yaw_ned -= 2.0 * M_PI;
    while (yaw_ned < -M_PI) yaw_ned += 2.0 * M_PI;
    return yaw_ned;
}

inline double yaw_ned_to_enu(double yaw_ned) {
    double yaw_enu = M_PI_2 - yaw_ned;
    while (yaw_enu > M_PI) yaw_enu -= 2.0 * M_PI;
    while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;
    return yaw_enu;
}

/**
 * @brief Normalize angle to [-π, π]
 */
inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace peregrine::frame_transforms
```

### Covariance Transformations

```cpp
namespace peregrine::frame_transforms {

// ═══════════════════════════════════════════════════════════════════
// Covariance Matrix Transformations
// ═══════════════════════════════════════════════════════════════════

/**
 * @brief Transform 3x3 position covariance from ENU to NED
 * 
 * C_ned = R * C_enu * R^T
 * 
 * where R is the rotation matrix from ENU to NED:
 * R = [0  1  0]
 *     [1  0  0]
 *     [0  0 -1]
 */
inline Eigen::Matrix3d covariance_enu_to_ned(const Eigen::Matrix3d& cov_enu) {
    Eigen::Matrix3d R;
    R << 0, 1, 0,
         1, 0, 0,
         0, 0, -1;
    return R * cov_enu * R.transpose();
}

/**
 * @brief Transform 6x6 pose covariance from ENU to NED
 * 
 * For 6x6 covariance [position; orientation]:
 * Uses block diagonal transformation
 */
inline Eigen::Matrix<double, 6, 6> pose_covariance_enu_to_ned(
    const Eigen::Matrix<double, 6, 6>& cov_enu) {
    
    Eigen::Matrix3d R;
    R << 0, 1, 0,
         1, 0, 0,
         0, 0, -1;
    
    Eigen::Matrix<double, 6, 6> R_full = Eigen::Matrix<double, 6, 6>::Zero();
    R_full.block<3, 3>(0, 0) = R;
    R_full.block<3, 3>(3, 3) = R;
    
    return R_full * cov_enu * R_full.transpose();
}

}  // namespace peregrine::frame_transforms
```

---

## TF2 Broadcasting

### TF Broadcaster Class

```cpp
// frame_transforms/include/frame_transforms/tf_broadcaster.hpp

namespace peregrine::frame_transforms {

/**
 * @brief TF2 transform broadcaster for UAV frames
 * 
 * Publishes the transform tree:
 *   world -> odom -> base_link -> sensor_frames
 */
class UAVTFBroadcaster {
public:
    explicit UAVTFBroadcaster(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Broadcast world -> base_link transform
     * @param state Current UAV state in ENU/FLU
     */
    void broadcastState(const peregrine_interfaces::msg::State& state);
    
    /**
     * @brief Broadcast static sensor transforms
     * @param sensor_frame Sensor frame name
     * @param transform Transform from base_link to sensor
     */
    void broadcastStaticSensorTransform(
        const std::string& sensor_frame,
        const geometry_msgs::msg::TransformStamped& transform);
    
    /**
     * @brief Set world frame name
     */
    void setWorldFrame(const std::string& frame) { world_frame_ = frame; }
    
    /**
     * @brief Set odom frame name
     */
    void setOdomFrame(const std::string& frame) { odom_frame_ = frame; }
    
private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    
    std::string world_frame_{"world"};
    std::string odom_frame_{"odom"};
    std::string base_link_frame_{"base_link"};
};

}  // namespace peregrine::frame_transforms
```

### Frame Tree Configuration

```yaml
# config/frames.yaml

# Frame names
frames:
  world: "world"           # Global ENU frame
  odom: "odom"             # Odometry frame (may drift from world)
  base_link: "base_link"   # UAV body frame (FLU)
  
# Static sensor transforms (base_link -> sensor)
# Format: [x, y, z, roll, pitch, yaw] in meters and radians
sensor_transforms:
  imu_link:
    translation: [0.0, 0.0, 0.0]
    rotation: [0.0, 0.0, 0.0]
    
  camera_front:
    translation: [0.10, 0.0, -0.05]
    rotation: [0.0, 0.261799, 0.0]  # 15° pitch down
    
  gps_link:
    translation: [0.0, 0.0, 0.05]
    rotation: [0.0, 0.0, 0.0]
    
  lidar_link:
    translation: [0.0, 0.0, -0.10]
    rotation: [0.0, 0.0, 0.0]
```

---

## Implementation Guidelines

### ROS2 Concepts Used

1. **tf2_ros for Transform Management**
   ```cpp
   #include <tf2_ros/transform_broadcaster.h>
   #include <tf2_ros/static_transform_broadcaster.h>
   #include <tf2_ros/buffer.h>
   #include <tf2_ros/transform_listener.h>
   #include <tf2_eigen/tf2_eigen.hpp>
   ```

2. **Header-Only Library Option**
   - Conversion functions are `inline` for performance
   - No linking required for basic conversions
   - TF broadcaster requires compiled library

3. **CMakeLists.txt Structure**
   ```cmake
   cmake_minimum_required(VERSION 3.8)
   project(frame_transforms)
   
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(tf2 REQUIRED)
   find_package(tf2_ros REQUIRED)
   find_package(tf2_eigen REQUIRED)
   find_package(geometry_msgs REQUIRED)
   find_package(Eigen3 REQUIRED)
   find_package(peregrine_interfaces REQUIRED)
   
   # Header-only conversions library
   add_library(frame_conversions INTERFACE)
   target_include_directories(frame_conversions INTERFACE
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
   )
   target_link_libraries(frame_conversions INTERFACE
     Eigen3::Eigen
   )
   
   # TF broadcaster library
   add_library(tf_broadcaster src/tf_broadcaster.cpp)
   target_include_directories(tf_broadcaster PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
   )
   ament_target_dependencies(tf_broadcaster
     rclcpp tf2 tf2_ros tf2_eigen geometry_msgs peregrine_interfaces
   )
   
   # Install
   install(DIRECTORY include/
     DESTINATION include
   )
   install(TARGETS frame_conversions tf_broadcaster
     EXPORT export_${PROJECT_NAME}
     LIBRARY DESTINATION lib
     ARCHIVE DESTINATION lib
     RUNTIME DESTINATION bin
     INCLUDES DESTINATION include
   )
   
   ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
   ament_export_dependencies(Eigen3 tf2 tf2_ros tf2_eigen)
   ament_package()
   ```

### Performance Considerations

1. **Inline Functions**: All basic conversions are inlined
2. **No Dynamic Allocation**: Eigen fixed-size types used
3. **Precomputed Constants**: Static rotation matrices/quaternions
4. **Branch-Free**: Avoid conditionals in hot paths

```cpp
// Example: Optimized quaternion conversion
// Precompute the transformation quaternion at compile time
constexpr double SQRT2_INV = 0.70710678118654752440;

inline Eigen::Quaterniond quaternion_enu_to_ned_optimized(
    const Eigen::Quaterniond& q) {
    // Direct component manipulation - no function calls
    return Eigen::Quaterniond(q.w(), q.y(), q.x(), -q.z());
}
```

---

## Major Cautions

### ⚠️ Quaternion Convention
- Eigen uses (w, x, y, z) order
- PX4 messages may use (x, y, z, w) order
- ROS geometry_msgs uses (x, y, z, w) order
- **Always verify quaternion component order when converting**

### ⚠️ Yaw Wraparound
- Always normalize angles after conversion
- Handle the ±π discontinuity in controllers
- Use `atan2` for robust angle computation

### ⚠️ Covariance Transformation
- Covariance requires R * C * R^T, not just R * C
- 6x6 pose covariance has coupled position-orientation terms
- Incorrect transformation leads to filter divergence

### ⚠️ TF2 Timing
- TF2 lookups can fail if transforms are stale
- Set appropriate `timeout` in transform lookups
- Handle `tf2::ExtrapolationException` gracefully

### ⚠️ Reference Frame Tracking
- Every transform must know source and target frames
- Log frame_id mismatches as errors, not just warnings
- Use consistent naming across the system

### ⚠️ Testing Conversions
- Round-trip test: `ned_to_enu(enu_to_ned(x)) == x`
- Test at singularities: poles, gimbal lock angles
- Verify with known values from PX4 documentation

---

## File Structure

```
frame_transforms/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── frame_transforms/
│       ├── conversions.hpp       # All conversion functions
│       ├── tf_broadcaster.hpp    # TF2 broadcaster class
│       └── constants.hpp         # Frame name constants
├── src/
│   └── tf_broadcaster.cpp        # TF broadcaster implementation
├── config/
│   └── frames.yaml               # Frame configuration
└── test/
    ├── test_conversions.cpp      # Unit tests for conversions
    └── test_tf_broadcaster.cpp   # TF broadcaster tests
```

---

## Testing

### Unit Test Example

```cpp
// test/test_conversions.cpp

#include <gtest/gtest.h>
#include "frame_transforms/conversions.hpp"

using namespace peregrine::frame_transforms;

TEST(FrameConversions, PositionRoundTrip) {
    Eigen::Vector3d original(1.0, 2.0, 3.0);
    auto converted = enu_to_ned(original);
    auto back = ned_to_enu(converted);
    
    EXPECT_NEAR(original.x(), back.x(), 1e-10);
    EXPECT_NEAR(original.y(), back.y(), 1e-10);
    EXPECT_NEAR(original.z(), back.z(), 1e-10);
}

TEST(FrameConversions, PositionValues) {
    // ENU: East=1, North=2, Up=3
    Eigen::Vector3d enu(1.0, 2.0, 3.0);
    auto ned = enu_to_ned(enu);
    
    // NED: North=2, East=1, Down=-3
    EXPECT_DOUBLE_EQ(ned.x(), 2.0);  // North = ENU.y
    EXPECT_DOUBLE_EQ(ned.y(), 1.0);  // East = ENU.x
    EXPECT_DOUBLE_EQ(ned.z(), -3.0); // Down = -ENU.z
}

TEST(FrameConversions, YawConversion) {
    // ENU: 0 = East, π/2 = North
    // NED: 0 = North, π/2 = East
    
    // Facing East in ENU (yaw=0) is facing East in NED (yaw=π/2)
    EXPECT_NEAR(yaw_enu_to_ned(0.0), M_PI_2, 1e-10);
    
    // Facing North in ENU (yaw=π/2) is facing North in NED (yaw=0)
    EXPECT_NEAR(yaw_enu_to_ned(M_PI_2), 0.0, 1e-10);
}
```
