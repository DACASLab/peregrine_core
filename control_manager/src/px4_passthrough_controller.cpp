#include <control_manager/px4_passthrough_controller.hpp>

namespace control_manager
{

peregrine_interfaces::msg::ControlOutput Px4PassthroughController::compute(
    const peregrine_interfaces::msg::State& /*state*/,
    const peregrine_interfaces::msg::TrajectorySetpoint& setpoint) const
{
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
  output.yaw = static_cast<float>(setpoint.yaw);
  output.yaw_rate = static_cast<float>(setpoint.yaw_rate);
  return output;
}

}  // namespace control_manager
