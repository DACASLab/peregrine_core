/**
 * @file trajectory_generator_base.hpp
 * @brief Base interface for trajectory generators.
 *
 * Generators are pure reference sources: given (state, time), they return a setpoint and
 * a progress scalar [0, 1].  They do NOT decide when a goal is "complete" — that is the
 * trajectory_manager action server's responsibility (progress >= 1.0 → succeed the goal).
 *
 * Built-in generators (see generators.hpp):
 *  - HoldPositionGenerator: stationary position hold
 *  - TakeoffGenerator: vertical climb to target altitude
 *  - LinearGoToGenerator: constant-velocity point-to-point
 *  - CircleGenerator: constant-altitude circular orbit
 *  - FigureEightGenerator: constant-altitude Lissajous curve
 *  - LandGenerator: constant-velocity vertical descent
 */

#pragma once

#include <peregrine_interfaces/msg/trajectory_setpoint.hpp>
#include <rclcpp/time.hpp>

#include <string>
#include <utility>

namespace trajectory_manager
{

/**
 * @struct TrajectorySample
 * @brief Output of one generator sample step.
 *
 * Progress [0, 1] tracks how far through the planned reference the generator has
 * advanced.  Once progress reaches 1.0, the generator clamps at its final setpoint.
 * The trajectory_manager action server uses progress >= 1.0 to succeed the goal.
 */
struct TrajectorySample
{
  peregrine_interfaces::msg::TrajectorySetpoint setpoint;
  float progress{0.0F};
};

/**
 * @class TrajectoryGeneratorBase
 * @brief Interface implemented by all trajectory generators.
 */
class TrajectoryGeneratorBase
{
public:
  explicit TrajectoryGeneratorBase(std::string name) : name_(std::move(name)) {}
  virtual ~TrajectoryGeneratorBase() = default;

  const std::string & name() const { return name_; }

  virtual TrajectorySample sample(const rclcpp::Time & now) = 0;

private:
  std::string name_;
};

}  // namespace trajectory_manager
