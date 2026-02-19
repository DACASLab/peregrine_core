/**
 * @file controller_base.hpp
 * @brief Abstract controller interface for control_manager.
 */

#pragma once

#include <peregrine_interfaces/msg/control_output.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/trajectory_setpoint.hpp>

namespace control_manager
{

/**
 * @class ControllerBase
 * @brief Interface implemented by control backends in control_manager.
 */
class ControllerBase
{
public:
  virtual ~ControllerBase() = default;

  /**
   * @brief Computes control output from the current state and trajectory setpoint.
   */
  virtual peregrine_interfaces::msg::ControlOutput compute(
      const peregrine_interfaces::msg::State& state,
      const peregrine_interfaces::msg::TrajectorySetpoint& setpoint) const = 0;
};

}  // namespace control_manager
