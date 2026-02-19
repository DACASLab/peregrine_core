/**
 * @file px4_passthrough_controller.hpp
 * @brief Passthrough controller implementation for control_manager.
 */

#pragma once

#include <control_manager/controller_base.hpp>

namespace control_manager
{

/**
 * @class Px4PassthroughController
 * @brief Converts trajectory setpoints directly into ControlOutput trajectory mode.
 */
class Px4PassthroughController : public ControllerBase
{
public:
  /**
   * @brief Produces a trajectory-mode control command from the input setpoint.
   */
  peregrine_interfaces::msg::ControlOutput compute(
      const peregrine_interfaces::msg::State& state,
      const peregrine_interfaces::msg::TrajectorySetpoint& setpoint) const override;
};

}  // namespace control_manager
