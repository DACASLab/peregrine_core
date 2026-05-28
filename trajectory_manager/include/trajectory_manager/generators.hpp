/**
 * @file generators.hpp
 * @brief Built-in trajectory generators for trajectory_manager.
 *
 * Generators are pure reference sources.  Each follows the same pattern:
 *  1. Constructor captures start state and trajectory parameters.
 *  2. sample() returns the reference setpoint for the current time and a progress
 *     scalar [0, 1].  After progress reaches 1.0 the setpoint clamps at its final
 *     value.  Generators never inspect measured state for completion — that decision
 *     belongs to the orchestrator (trajectory_manager action server or BT).
 *
 * Setpoints are in ENU/FLU.  Conversion to PX4's NED/FRD happens in hardware_abstraction.
 *
 * Velocity feedforward: CircleGenerator and FigureEightGenerator provide analytical
 * velocity/acceleration derivatives so PX4's controller can track curves without lag.
 */

#pragma once

#include <trajectory_manager/trajectory_generator_base.hpp>

#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <peregrine_interfaces/msg/state.hpp>

#include <vector>

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

  TrajectorySample sample(const rclcpp::Time & now) override;

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

  TrajectorySample sample(const rclcpp::Time & now) override;

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
    double targetYaw, double velocityMps, const rclcpp::Time & startTime);

  TrajectorySample sample(const rclcpp::Time & now) override;

private:
  geometry_msgs::msg::Point startPosition_;
  geometry_msgs::msg::Point targetPosition_;
  double targetYaw_{0.0};
  double velocity_{1.0};
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

  TrajectorySample sample(const rclcpp::Time & now) override;

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

  TrajectorySample sample(const rclcpp::Time & now) override;

private:
  geometry_msgs::msg::Point center_;
  double radius_{1.0};
  double angularVelocity_{0.5};
  double loops_{1.0};
  double altitude_{0.0};
  rclcpp::Time startTime_{0, 0, RCL_ROS_TIME};
};

/**
 * @class StepResponseGenerator
 * @brief Generates a piecewise-constant position/yaw step for response characterization.
 *
 * The generator holds the start reference for `preStepHoldS`, then applies a fixed
 * position/yaw offset and holds it for `postStepHoldS`. Completion is time-based rather
 * than state-based so poorly tuned responses still yield a finite observation window.
 */
class StepResponseGenerator : public TrajectoryGeneratorBase
{
public:
  /**
   * @brief Constructs a step-response generator.
   *
   * Parameter semantics:
   *   - stepOffset: ENU position offset applied after the pre-step hold
   *   - yawStepRad: ENU yaw offset applied after the pre-step hold
   *   - preStepHoldS: duration before the step is applied
   *   - postStepHoldS: duration to hold the stepped reference after the transition
   */
  StepResponseGenerator(
    const peregrine_interfaces::msg::State & startState,
    const geometry_msgs::msg::Point & stepOffset,
    double yawStepRad,
    double preStepHoldS,
    double postStepHoldS,
    const rclcpp::Time & startTime);

  TrajectorySample sample(const rclcpp::Time & now) override;

private:
  geometry_msgs::msg::Point startPosition_;
  geometry_msgs::msg::Point targetPosition_;
  double startYaw_{0.0};
  double targetYaw_{0.0};
  double preStepHoldS_{2.0};
  double postStepHoldS_{8.0};
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

  TrajectorySample sample(const rclcpp::Time & now) override;

private:
  geometry_msgs::msg::Point startPosition_;
  double startYaw_{0.0};
  double startAltitude_{0.0};
  double targetAltitude_{0.0};
  double descentVelocity_{0.5};
  rclcpp::Time startTime_{0, 0, RCL_ROS_TIME};
};

/**
 * @class WaypointTrajectoryGenerator
 * @brief Follows a pre-computed smooth 2D waypoint path at constant altitude.
 *
 * Time-parameterizes the path by arc-length with slower curved sections and
 * acceleration-limited ramps between corner and straight speeds.
 * Provides position + velocity feedforward + forward yaw at each sample.
 */
class WaypointTrajectoryGenerator : public TrajectoryGeneratorBase
{
public:
  WaypointTrajectoryGenerator(
    const std::vector<Eigen::Vector2d> & waypoints_enu,
    const std::vector<double> & yaw,
    double altitude,
    double velocity_mps,
    const rclcpp::Time & startTime);

  TrajectorySample sample(const rclcpp::Time & now) override;

private:
  std::vector<Eigen::Vector2d> waypoints_;
  std::vector<double> yaw_;
  std::vector<double> cumArcLength_;
  std::vector<double> cumTime_;
  std::vector<double> speedProfile_;
  std::vector<double> speedSlope_;
  double altitude_{0.0};
  double cruiseVelocity_{1.0};
  double totalDuration_{0.0};
  rclcpp::Time startTime_{0, 0, RCL_ROS_TIME};
};

}  // namespace trajectory_manager
