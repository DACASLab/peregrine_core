#include <control_manager/px4_passthrough_controller.hpp>

namespace control_manager
{

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

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(control_manager::Px4PassthroughController, control_manager::ControllerBase)
