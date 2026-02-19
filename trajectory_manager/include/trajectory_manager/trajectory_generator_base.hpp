/**
 * @file trajectory_generator_base.hpp
 * @brief Base interface for trajectory generators.
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
  virtual TrajectorySample sample(const peregrine_interfaces::msg::State& currentState,
                                  const rclcpp::Time& now) = 0;
};

}  // namespace trajectory_manager
