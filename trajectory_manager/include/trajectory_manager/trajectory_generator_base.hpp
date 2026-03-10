/**
 * @file trajectory_generator_base.hpp
 * @brief Base interface for trajectory generators.
 *
 * Trajectory generators are owned by TrajectoryManagerNode and produce setpoints at the
 * configured publication rate (default 50 Hz). The generator pattern separates trajectory
 * math from ROS plumbing: generators are pure computational objects that take (state, time)
 * and produce (setpoint, progress, completion status). The manager handles all ROS concerns
 * (action servers, lifecycle, publishing).
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

#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/trajectory_setpoint.hpp>
#include <rclcpp/time.hpp>

#include <string>

namespace trajectory_manager
{

/**
 * @struct TrajectorySample
 * @brief Output of one generator update step.
 *
 * The `completed` flag drives goal lifecycle: when the timer callback detects
 * completed=true, it finalizes the action goal and switches back to hold mode.
 * Progress (0.0 to 1.0) and distanceRemaining are forwarded as action feedback.
 */
struct TrajectorySample
{
  peregrine_interfaces::msg::TrajectorySetpoint setpoint;
  float progress{0.0F};
  double distanceRemaining{0.0};
  bool completed{false};
};

/**
 * @class TrajectoryGeneratorBase
 * @brief Interface implemented by all trajectory generators.
 */
class TrajectoryGeneratorBase
{
public:
  virtual ~TrajectoryGeneratorBase() = default;

  /**
   * @brief Returns a short identifier for the active generator.
   */
  virtual std::string name() const = 0;

  /**
   * @brief Samples the trajectory at the given time.
   */
  virtual TrajectorySample sample(
    const peregrine_interfaces::msg::State & currentState,
    const rclcpp::Time & now) = 0;
};

}  // namespace trajectory_manager
