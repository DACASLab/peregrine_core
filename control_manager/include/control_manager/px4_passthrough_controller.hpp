/**
 * @file px4_passthrough_controller.hpp
 * @brief Passthrough controller implementation for control_manager.
 *
 * This is the simplest possible ControllerBase implementation: it copies trajectory
 * setpoint fields directly into ControlOutput without modification. This is appropriate
 * when PX4's internal position/velocity controller handles the low-level tracking loop,
 * which is the normal case for trajectory mode (offboard position/velocity setpoints).
 *
 * For use cases requiring custom feedback control (e.g., external PID, MPC, or adaptive
 * controllers), a new ControllerBase subclass would replace this passthrough and use the
 * State input for feedback computation.
 */

#pragma once

#include <control_manager/controller_base.hpp>

namespace control_manager
{

/**
 * @class Px4PassthroughController
 * @brief Converts trajectory setpoints directly into ControlOutput trajectory mode.
 *
 * The `state` parameter is intentionally unused in this implementation because PX4's
 * internal controller closes the feedback loop. A future controller that implements its
 * own PID/MPC would use the state for error computation.
 */
class Px4PassthroughController : public ControllerBase
{
public:
  /**
   * @brief Produces a trajectory-mode control command from the input setpoint.
   */
  peregrine_interfaces::msg::ControlOutput compute(
    const peregrine_interfaces::msg::State & state,
    const peregrine_interfaces::msg::TrajectorySetpoint & setpoint) const override;
};

}  // namespace control_manager
