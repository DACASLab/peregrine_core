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
#include <control_manager/control_manager_node.hpp>

#include <control_manager/px4_passthrough_controller.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <thread>

namespace control_manager
{
namespace
{

// Internal-linkage constants:
// Using an unnamed namespace makes these symbols file-local without using macros.
// This is the C++ equivalent of private module-level constants in Python.
constexpr char kManagerName[] = "control_manager";
constexpr char kModuleName[] = "px4_passthrough";

}  // namespace

ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kManagerName, options)
{
  publishRateHz_ = this->declare_parameter<double>("publish_rate_hz", 250.0);
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 10.0);
  stateTimeoutS_ = this->declare_parameter<double>("state_timeout_s", 0.5);
  dependencyStartupTimeoutS_ = this->declare_parameter<double>("dependency_startup_timeout_s", 2.0);
  autoStart_ = this->declare_parameter<bool>("auto_start", true);

  if (autoStart_) {
    startupTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() {
        startupTimer_->cancel();  // one-shot

        RCLCPP_INFO(get_logger(), "Auto-start: triggering configure");
        auto configResult = this->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        if (configResult.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          RCLCPP_ERROR(get_logger(), "Auto-configure failed (state=%s)", configResult.label().c_str());
          return;
        }

        RCLCPP_INFO(get_logger(), "Auto-start: triggering activate");
        auto activateResult = this->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        if (activateResult.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          RCLCPP_ERROR(get_logger(), "Auto-activate failed (state=%s)", activateResult.label().c_str());
          return;
        }

        RCLCPP_INFO(get_logger(), "Auto-start complete: ACTIVE");
      });
  }
}

ControlManagerNode::CallbackReturn ControlManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  if (publishRateHz_ <= 0.0 || statusRateHz_ <= 0.0 || stateTimeoutS_ <= 0.0 ||
    dependencyStartupTimeoutS_ <= 0.0)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "publish_rate_hz, status_rate_hz, state_timeout_s, and dependency_startup_timeout_s must be > 0");
    return CallbackReturn::FAILURE;
  }

  const auto startupDeadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(
    dependencyStartupTimeoutS_);
  // Deterministic startup gate: control_manager depends on estimated_state availability.
  while (std::chrono::steady_clock::now() < startupDeadline) {
    if (!this->get_publishers_info_by_topic("estimated_state").empty()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (this->get_publishers_info_by_topic("estimated_state").empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Cannot configure control_manager: upstream topic 'estimated_state' not available");
    return CallbackReturn::FAILURE;
  }

  controller_ = std::make_unique<Px4PassthroughController>();

  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

  // Data inputs: estimator state and trajectory manager intent.
  // `std::bind(&Class::method, this, _1)` adapts a member function into the callback
  // signature expected by create_subscription. In Python, this is analogous to passing
  // a bound method like `self.on_estimated_state`.
  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    "estimated_state", qos,
    std::bind(&ControlManagerNode::onEstimatedState, this, std::placeholders::_1));
  trajectorySetpointSub_ = this->create_subscription<peregrine_interfaces::msg::TrajectorySetpoint>(
    "trajectory_setpoint", qos,
    std::bind(&ControlManagerNode::onTrajectorySetpoint, this, std::placeholders::_1));

  // Outputs: control envelope for hardware bridge + manager health.
  controlOutputPub_ = this->create_publisher<peregrine_interfaces::msg::ControlOutput>(
    "control_output", qos);
  statusPub_ = this->create_publisher<peregrine_interfaces::msg::ManagerStatus>(
    "control_status",
    statusQos);

  publishTimer_ =
    this->create_wall_timer(
    periodFromHz(publishRateHz_),
    std::bind(&ControlManagerNode::publishControlOutput, this));
  statusTimer_ =
    this->create_wall_timer(
    periodFromHz(statusRateHz_),
    std::bind(&ControlManagerNode::publishStatus, this));

  // Timers are armed only in lifecycle ACTIVE state.
  publishTimer_->cancel();
  statusTimer_->cancel();
  configured_ = true;
  active_ = false;

  RCLCPP_INFO(
    this->get_logger(), "Configured control_manager: publish_rate_hz=%.1f status_rate_hz=%.1f", publishRateHz_,
    statusRateHz_);
  return CallbackReturn::SUCCESS;
}

ControlManagerNode::CallbackReturn ControlManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  if (!configured_) {
    RCLCPP_ERROR(this->get_logger(), "Cannot activate before configure");
    return CallbackReturn::FAILURE;
  }

  // Publisher activation first, then timer reset, avoids publishing before activation.
  controlOutputPub_->on_activate();
  statusPub_->on_activate();
  publishTimer_->reset();
  statusTimer_->reset();
  active_ = true;

  RCLCPP_INFO(this->get_logger(), "Activated control_manager");
  return CallbackReturn::SUCCESS;
}

ControlManagerNode::CallbackReturn ControlManagerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (publishTimer_) {
    publishTimer_->cancel();
  }
  if (statusTimer_) {
    statusTimer_->cancel();
  }
  if (controlOutputPub_) {
    controlOutputPub_->on_deactivate();
  }
  if (statusPub_) {
    statusPub_->on_deactivate();
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated control_manager");
  return CallbackReturn::SUCCESS;
}

ControlManagerNode::CallbackReturn ControlManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_ = false;
  configured_ = false;

  publishTimer_.reset();
  statusTimer_.reset();

  estimatedStateSub_.reset();
  trajectorySetpointSub_.reset();
  controlOutputPub_.reset();
  statusPub_.reset();

  {
    std::scoped_lock lock(dataMutex_);
    latestState_.reset();
    latestSetpoint_.reset();
    lastStateTime_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  controller_.reset();
  RCLCPP_INFO(this->get_logger(), "Cleaned up control_manager");
  return CallbackReturn::SUCCESS;
}

ControlManagerNode::CallbackReturn ControlManagerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  (void)on_cleanup(this->get_current_state());
  RCLCPP_INFO(this->get_logger(), "Shut down control_manager");
  return CallbackReturn::SUCCESS;
}

ControlManagerNode::CallbackReturn ControlManagerNode::on_error(const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (publishTimer_) {
    publishTimer_->cancel();
  }
  if (statusTimer_) {
    statusTimer_->cancel();
  }

  RCLCPP_ERROR(this->get_logger(), "Error in control_manager lifecycle; timers canceled");
  return CallbackReturn::SUCCESS;
}

void ControlManagerNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  if (!configured_) {
    return;
  }

  std::scoped_lock lock(dataMutex_);
  latestState_ = *msg;
  // Normalize freshness time even when source timestamp is unset.
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    lastStateTime_ = this->now();
  } else {
    lastStateTime_ = rclcpp::Time(msg->header.stamp);
  }
}

void ControlManagerNode::onTrajectorySetpoint(
  const peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg)
{
  if (!configured_) {
    return;
  }

  std::scoped_lock lock(dataMutex_);
  // Last-writer-wins policy: newest setpoint drives the next control tick.
  latestSetpoint_ = *msg;
}

// Copy-then-compute pattern: shared state written by subscription callbacks is copied
// under the mutex, then the controller runs entirely lock-free. This minimizes lock
// hold time and prevents the controller's compute() from blocking the executor thread
// that delivers subscription callbacks (onEstimatedState / onTrajectorySetpoint).
void ControlManagerNode::publishControlOutput()
{
  if (!active_ || !controller_ || !controlOutputPub_ || !controlOutputPub_->is_activated()) {
    return;
  }

  // Copy shared data under lock, then run controller logic lock-free.
  std::optional<peregrine_interfaces::msg::State> state;
  std::optional<peregrine_interfaces::msg::TrajectorySetpoint> setpoint;
  {
    std::scoped_lock lock(dataMutex_);
    state = latestState_;
    setpoint = latestSetpoint_;
  }

  if (!state.has_value()) {
    // Controller is strictly state-driven; never emit open-loop output without state.
    return;
  }

  // When trajectory_manager hasn't sent any setpoint yet (e.g. during initial hover),
  // we synthesize a hold-at-current-position setpoint. This ensures the PX4 offboard
  // setpoint stream never goes stale -- PX4 will exit offboard mode if setpoints stop
  // arriving for more than ~500ms.
  const auto activeSetpoint = setpoint.has_value() ? *setpoint : makeHoldSetpoint(*state);
  // Controller backend is pure; all ROS side-effects happen only around this call.
  auto output = controller_->compute(*state, activeSetpoint);
  output.header.stamp = this->now();
  // One output per timer tick keeps command cadence deterministic.
  controlOutputPub_->publish(output);
}

void ControlManagerNode::publishStatus()
{
  if (!configured_ || !statusPub_ || !statusPub_->is_activated()) {
    return;
  }

  peregrine_interfaces::msg::ManagerStatus status;
  status.header.stamp = this->now();
  status.manager_name = kManagerName;
  status.active_module = kModuleName;
  // Explicit cast avoids implicit double->float narrowing warnings and documents the
  // precision boundary at the message interface.
  status.output_rate_hz = static_cast<float>(publishRateHz_);
  status.active = active_;

  if (!active_) {
    status.healthy = false;
    status.message = "LIFECYCLE_INACTIVE";
  } else {
    std::scoped_lock lock(dataMutex_);
    if (!latestState_.has_value()) {
      // Active but waiting for first estimator sample.
      status.healthy = false;
      status.message = "WAITING_FOR_ESTIMATED_STATE";
    } else {
      // Freshness check guards against stale estimator output.
      const double ageS = (this->now() - lastStateTime_).seconds();
      status.healthy = ageS <= stateTimeoutS_;
      status.message = status.healthy ? "OK" : "ESTIMATED_STATE_STALE";
    }
  }

  statusPub_->publish(status);
}

std::chrono::nanoseconds ControlManagerNode::periodFromHz(const double hz)
{
  const auto period = std::chrono::duration<double>(1.0 / hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(period);
}

// Creates a position+yaw hold setpoint with zero velocity/acceleration, which tells
// PX4 to hold the current position. All dynamic channels (velocity, acceleration,
// yaw_rate) are explicitly zeroed rather than left as default-initialized values to
// avoid ambiguity in PX4's setpoint interpretation -- PX4 treats NaN and zero
// differently, and zero unambiguously means "no motion requested".
peregrine_interfaces::msg::TrajectorySetpoint ControlManagerNode::makeHoldSetpoint(
  const peregrine_interfaces::msg::State & state)
{
  // Hold setpoint mirrors current pose and commands zero dynamics.
  peregrine_interfaces::msg::TrajectorySetpoint setpoint;
  setpoint.header = state.header;
  setpoint.use_position = true;
  setpoint.use_velocity = false;
  setpoint.use_acceleration = false;
  setpoint.use_yaw = true;
  setpoint.use_yaw_rate = false;
  setpoint.position = state.pose.pose.position;
  // Explicitly zero dynamic channels so downstream bridge can map intent unambiguously.
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

// Standard ZYX Euler yaw extraction from an ENU orientation quaternion, commonly used
// in aerial robotics. Only the yaw (heading) component is needed; roll and pitch are
// discarded. This avoids pulling in a full Eigen dependency for a single trig operation.
double ControlManagerNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double sinyCosp = 2.0 * ((q.w * q.z) + (q.x * q.y));
  const double cosyCosp = 1.0 - 2.0 * ((q.y * q.y) + (q.z * q.z));
  return std::atan2(sinyCosp, cosyCosp);
}

}  // namespace control_manager

// Register the class as a composable node component. When loaded into a
// component_container process, all composable nodes share one process and can leverage
// intra-process zero-copy communication, avoiding serialization overhead between managers.
RCLCPP_COMPONENTS_REGISTER_NODE(control_manager::ControlManagerNode)
