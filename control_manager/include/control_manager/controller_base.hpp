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
 * @file controller_base.hpp
 * @brief Abstract controller interface for control_manager.
 *
 * Control backends implement this interface to translate (state, setpoint) pairs into
 * ControlOutput messages. The current implementation (Px4PassthroughController) performs
 * a simple message-type remap, but this abstraction allows future backends to implement
 * PID controllers, model-predictive controllers, or other feedback strategies without
 * changing the manager's subscription/publication logic.
 *
 * Backend lifecycle:
 *   1. Constructed in ControlManagerNode::on_configure()
 *   2. compute() called once per control tick (250 Hz by default)
 *   3. Destroyed in ControlManagerNode::on_cleanup()
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
 *
 * Implementations must be stateless or thread-safe: compute() is called from the
 * control publication timer while state/setpoint data is written from subscription
 * callbacks. The current architecture copies inputs under lock before calling compute()
 * lock-free (see ControlManagerNode::publishControlOutput).
 */
class ControllerBase
{
public:
  virtual ~ControllerBase() = default;

  /**
   * @brief Computes control output from the current state and trajectory setpoint.
   */
  virtual peregrine_interfaces::msg::ControlOutput compute(
    const peregrine_interfaces::msg::State & state,
    const peregrine_interfaces::msg::TrajectorySetpoint & setpoint) const = 0;
};

}  // namespace control_manager
