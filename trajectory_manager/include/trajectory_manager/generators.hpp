/**
 * @note C++ Primer for Python ROS2 readers
 *
 * This file follows a few recurring C++ patterns:
 * - Ownership is explicit: `std::unique_ptr` means single owner, `std::shared_ptr` means shared ownership.
 * - References (`T&`) and `const` are used to avoid unnecessary copies and make mutation intent explicit.
 * - RAII is used for resource safety: objects such as locks clean themselves up automatically at scope exit.
 * - ROS2 callbacks may run concurrently depending on executor/callback-group setup, so shared state is guarded.
 * - Templates (for example `create_subscription<MsgT>`) are compile-time type binding, not runtime reflection.
 */
/**
 * @file generators.hpp
 * @brief Built-in trajectory generators for trajectory_manager.
 *
 * All generators follow a common pattern:
 *  1. Constructor captures the start state (position, yaw) and trajectory parameters
 *  2. sample() is called at the publication rate and returns the reference setpoint
 *     for the current time, plus progress/completion metadata
 *  3. Completion is based on MEASURED state (from currentState), not just planned
 *     progress, to account for tracking error and disturbances
 *
 * Generators produce setpoints in ENU/FLU coordinates. The conversion to PX4's NED/FRD
 * happens downstream in hardware_abstraction.
 *
 * Velocity feedforward: CircleGenerator and FigureEightGenerator enable use_velocity
 * in their setpoints and provide analytical velocity derivatives. This helps PX4's
 * position controller track the curved path without lag (pure position control would
 * always lag behind a moving reference). For straight-line or vertical trajectories,
 * position-only control is sufficient.
 */

#pragma once

#include <trajectory_manager/trajectory_generator_base.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <cstdint>

namespace trajectory_manager
{

/**
 * @brief Extracts ENU yaw from a quaternion.
 */
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

/**
 * @class HoldPositionGenerator
 * @brief Keeps the vehicle at a fixed position and yaw.
 */
class HoldPositionGenerator : public TrajectoryGeneratorBase
{
public:
  /**
   * @brief Constructs hold setpoint from a reference state.
   */
  explicit HoldPositionGenerator(const peregrine_interfaces::msg::State & referenceState);

  /**
   * @brief Constructs hold setpoint from explicit position and yaw.
   */
  HoldPositionGenerator(const geometry_msgs::msg::Point & position, double yaw);

  std::string name() const override;
  TrajectorySample sample(
    const peregrine_interfaces::msg::State & currentState,
    const rclcpp::Time & now) override;

private:
  geometry_msgs::msg::Point holdPosition_;
  double holdYaw_{0.0};
};

/**
 * @class TakeoffGenerator
 * @brief Generates a vertical climb trajectory to target altitude.
 */
class TakeoffGenerator : public TrajectoryGeneratorBase
{
public:
  /**
   * @brief Constructs a takeoff generator.
   */
  TakeoffGenerator(
    const peregrine_interfaces::msg::State & startState, double targetAltitudeM,
    double climbVelocityMps,
    const rclcpp::Time & startTime);

  std::string name() const override;
  TrajectorySample sample(
    const peregrine_interfaces::msg::State & currentState,
    const rclcpp::Time & now) override;

private:
  geometry_msgs::msg::Point startPosition_;
  double startYaw_{0.0};
  double startAltitude_{0.0};
  double targetAltitude_{0.0};
  double climbVelocity_{1.0};
  rclcpp::Time startTime_{0, 0, RCL_ROS_TIME};
};

/**
 * @class LinearGoToGenerator
 * @brief Generates linear interpolation from start to goal position.
 */
class LinearGoToGenerator : public TrajectoryGeneratorBase
{
public:
  /**
   * @brief Constructs a go-to generator.
   */
  LinearGoToGenerator(
    const peregrine_interfaces::msg::State & startState,
    const geometry_msgs::msg::Point & targetPosition,
    double targetYaw, double velocityMps, double acceptanceRadiusM, const rclcpp::Time & startTime);

  std::string name() const override;
  TrajectorySample sample(
    const peregrine_interfaces::msg::State & currentState,
    const rclcpp::Time & now) override;

private:
  geometry_msgs::msg::Point startPosition_;
  geometry_msgs::msg::Point targetPosition_;
  double targetYaw_{0.0};
  double velocity_{1.0};
  double acceptanceRadius_{0.2};
  double totalDistance_{0.0};
  double totalDurationS_{0.0};
  rclcpp::Time startTime_{0, 0, RCL_ROS_TIME};
};

/**
 * @class CircleGenerator
 * @brief Generates a fixed-altitude circular trajectory.
 */
class CircleGenerator : public TrajectoryGeneratorBase
{
public:
  /**
   * @brief Constructs a circle generator.
   */
  CircleGenerator(
    const peregrine_interfaces::msg::State & startState, double radiusM,
    double angularVelocityRadps,
    double numLoops, const rclcpp::Time & startTime);

  std::string name() const override;
  TrajectorySample sample(
    const peregrine_interfaces::msg::State & currentState,
    const rclcpp::Time & now) override;

private:
  geometry_msgs::msg::Point center_;
  double radius_{1.0};
  double angularVelocity_{0.5};
  double loops_{1.0};
  double altitude_{0.0};
  rclcpp::Time startTime_{0, 0, RCL_ROS_TIME};
};

/**
 * @class FigureEightGenerator
 * @brief Generates a fixed-altitude figure-eight trajectory.
 */
class FigureEightGenerator : public TrajectoryGeneratorBase
{
public:
  /**
   * @brief Constructs a figure-eight generator.
   */
  FigureEightGenerator(
    const peregrine_interfaces::msg::State & startState, double radiusM,
    double angularVelocityRadps,
    double numLoops, const rclcpp::Time & startTime);

  std::string name() const override;
  TrajectorySample sample(
    const peregrine_interfaces::msg::State & currentState,
    const rclcpp::Time & now) override;

private:
  geometry_msgs::msg::Point center_;
  double radius_{1.0};
  double angularVelocity_{0.5};
  double loops_{1.0};
  double altitude_{0.0};
  rclcpp::Time startTime_{0, 0, RCL_ROS_TIME};
};

/**
 * @class LandGenerator
 * @brief Generates a vertical descent trajectory to z=0.
 */
class LandGenerator : public TrajectoryGeneratorBase
{
public:
  /**
   * @brief Constructs a landing generator.
   */
  LandGenerator(
    const peregrine_interfaces::msg::State & startState, double descentVelocityMps,
    const rclcpp::Time & startTime);

  std::string name() const override;
  TrajectorySample sample(
    const peregrine_interfaces::msg::State & currentState,
    const rclcpp::Time & now) override;

private:
  geometry_msgs::msg::Point startPosition_;
  double startYaw_{0.0};
  double startAltitude_{0.0};
  double targetAltitude_{0.0};
  double descentVelocity_{0.5};
  rclcpp::Time startTime_{0, 0, RCL_ROS_TIME};
};

}  // namespace trajectory_manager
