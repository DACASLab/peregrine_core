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
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <string>

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

  /// Returns the controller's string identifier for status reporting.
  virtual std::string name() const = 0;

  /// Declares ROS2 parameters and initializes internal state.
  /// @param node  Lifecycle node to declare parameters on.
  /// @param dt    Control loop timestep (1.0 / publish_rate_hz).
  virtual void configure(rclcpp_lifecycle::LifecycleNode & node, double dt) = 0;

  /**
   * @brief Computes control output from the current state and trajectory setpoint.
   */
  virtual peregrine_interfaces::msg::ControlOutput compute(
    const peregrine_interfaces::msg::State & state,
    const peregrine_interfaces::msg::TrajectorySetpoint & setpoint) const = 0;

  /**
   * @brief Resets internal controller state (e.g., integral accumulators).
   *
   * Called on lifecycle deactivation to prevent integral windup carryover.
   * Default implementation is a no-op for stateless controllers.
   */
  virtual void reset() {}
};

}  // namespace control_manager
