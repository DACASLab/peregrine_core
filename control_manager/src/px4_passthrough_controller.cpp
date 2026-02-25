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
#include <control_manager/px4_passthrough_controller.hpp>

namespace control_manager
{

// `/*state*/` is a commented-out parameter name. The parameter type is still declared
// (const State &) so the function signature matches the base class, but the name is
// commented out to suppress "unused parameter" compiler warnings. In Python, you'd
// use `_` or `*args` for unused parameters; C++ uses this commenting convention or
// the `[[maybe_unused]]` attribute.
peregrine_interfaces::msg::ControlOutput Px4PassthroughController::compute(
  const peregrine_interfaces::msg::State & /*state*/,
  const peregrine_interfaces::msg::TrajectorySetpoint & setpoint) const
{
  // Passthrough policy: keep trajectory semantics unchanged and only remap message type.
  // The use_* flags are forwarded as-is so that hardware_abstraction knows which PX4
  // setpoint channels to populate and which to leave as NaN (PX4 ignores NaN channels).
  peregrine_interfaces::msg::ControlOutput output;
  output.control_mode = peregrine_interfaces::msg::ControlOutput::MODE_TRAJECTORY;
  output.use_position = setpoint.use_position;
  output.use_velocity = setpoint.use_velocity;
  output.use_acceleration = setpoint.use_acceleration;
  output.use_yaw = setpoint.use_yaw;
  output.use_yaw_rate = setpoint.use_yaw_rate;
  output.position = setpoint.position;
  output.velocity = setpoint.velocity;
  output.acceleration = setpoint.acceleration;
  // TrajectorySetpoint uses double for yaw/yaw_rate; ControlOutput uses float because
  // PX4 TrajectorySetpoint.msg uses float32 fields. Precision loss is negligible for angles.
  output.yaw = static_cast<float>(setpoint.yaw);
  output.yaw_rate = static_cast<float>(setpoint.yaw_rate);
  return output;
}

}  // namespace control_manager
