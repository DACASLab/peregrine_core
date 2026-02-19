#include <control_manager/control_manager_node.hpp>

#include <control_manager/px4_passthrough_controller.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <stdexcept>

namespace control_manager
{
namespace
{

constexpr char kManagerName[] = "control_manager";
constexpr char kModuleName[] = "px4_passthrough";

}  // namespace

ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions& options)
: Node(kManagerName, options), controller_(std::make_unique<Px4PassthroughController>())
{
  publishRateHz_ = this->declare_parameter<double>("publish_rate_hz", 250.0);
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 10.0);
  stateTimeoutS_ = this->declare_parameter<double>("state_timeout_s", 0.5);

  if (publishRateHz_ <= 0.0 || statusRateHz_ <= 0.0 || stateTimeoutS_ <= 0.0)
  {
    throw std::runtime_error("publish_rate_hz, status_rate_hz, and state_timeout_s must be > 0");
  }

  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
      "estimated_state", qos, std::bind(&ControlManagerNode::onEstimatedState, this, std::placeholders::_1));
  trajectorySetpointSub_ = this->create_subscription<peregrine_interfaces::msg::TrajectorySetpoint>(
      "trajectory_setpoint", qos, std::bind(&ControlManagerNode::onTrajectorySetpoint, this, std::placeholders::_1));

  controlOutputPub_ = this->create_publisher<peregrine_interfaces::msg::ControlOutput>("control_output", qos);
  statusPub_ = this->create_publisher<peregrine_interfaces::msg::ManagerStatus>("control_status", statusQos);

  publishTimer_ =
      this->create_wall_timer(periodFromHz(publishRateHz_), std::bind(&ControlManagerNode::publishControlOutput, this));
  statusTimer_ = this->create_wall_timer(periodFromHz(statusRateHz_), std::bind(&ControlManagerNode::publishStatus, this));

  RCLCPP_INFO(this->get_logger(), "control_manager started: publish_rate_hz=%.1f status_rate_hz=%.1f", publishRateHz_,
              statusRateHz_);
}

void ControlManagerNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  std::scoped_lock lock(dataMutex_);
  latestState_ = *msg;
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
  {
    lastStateTime_ = this->now();
  }
  else
  {
    lastStateTime_ = rclcpp::Time(msg->header.stamp);
  }
}

void ControlManagerNode::onTrajectorySetpoint(const peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg)
{
  std::scoped_lock lock(dataMutex_);
  latestSetpoint_ = *msg;
}

void ControlManagerNode::publishControlOutput()
{
  std::optional<peregrine_interfaces::msg::State> state;
  std::optional<peregrine_interfaces::msg::TrajectorySetpoint> setpoint;
  {
    std::scoped_lock lock(dataMutex_);
    state = latestState_;
    setpoint = latestSetpoint_;
  }

  if (!state.has_value())
  {
    return;
  }

  // MVP semantics: when no external setpoint is available, hold tracks the latest state sample.
  // For future non-passthrough controllers, this should become a latched hold reference.
  const auto activeSetpoint = setpoint.has_value() ? *setpoint : makeHoldSetpoint(*state);
  auto output = controller_->compute(*state, activeSetpoint);
  output.header.stamp = this->now();
  controlOutputPub_->publish(output);
}

void ControlManagerNode::publishStatus()
{
  peregrine_interfaces::msg::ManagerStatus status;
  status.header.stamp = this->now();
  status.manager_name = kManagerName;
  status.active_module = kModuleName;
  status.output_rate_hz = static_cast<float>(publishRateHz_);

  {
    std::scoped_lock lock(dataMutex_);
    status.active = latestState_.has_value();
    if (!status.active)
    {
      status.healthy = false;
      status.message = "waiting_for_estimated_state";
    }
    else
    {
      const double ageS = (this->now() - lastStateTime_).seconds();
      status.healthy = ageS <= stateTimeoutS_;
      status.message = status.healthy ? "ok" : "estimated_state_timeout";
    }
  }

  statusPub_->publish(status);
}

std::chrono::nanoseconds ControlManagerNode::periodFromHz(const double hz)
{
  const auto period = std::chrono::duration<double>(1.0 / hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(period);
}

peregrine_interfaces::msg::TrajectorySetpoint ControlManagerNode::makeHoldSetpoint(
    const peregrine_interfaces::msg::State& state)
{
  peregrine_interfaces::msg::TrajectorySetpoint setpoint;
  setpoint.header = state.header;
  setpoint.use_position = true;
  setpoint.use_velocity = false;
  setpoint.use_acceleration = false;
  setpoint.use_yaw = true;
  setpoint.use_yaw_rate = false;
  setpoint.position = state.pose.pose.position;
  setpoint.velocity.x = 0.0;
  setpoint.velocity.y = 0.0;
  setpoint.velocity.z = 0.0;
  setpoint.acceleration.x = 0.0;
  setpoint.acceleration.y = 0.0;
  setpoint.acceleration.z = 0.0;
  setpoint.yaw = yawFromQuaternion(state.pose.pose.orientation);
  setpoint.yaw_rate = 0.0;
  return setpoint;
}

double ControlManagerNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  const double sinyCosp = 2.0 * ((q.w * q.z) + (q.x * q.y));
  const double cosyCosp = 1.0 - 2.0 * ((q.y * q.y) + (q.z * q.z));
  return std::atan2(sinyCosp, cosyCosp);
}

}  // namespace control_manager

RCLCPP_COMPONENTS_REGISTER_NODE(control_manager::ControlManagerNode)
