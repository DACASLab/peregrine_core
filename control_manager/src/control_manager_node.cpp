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
#include <control_manager/se3_controller.hpp>

#include <Eigen/Geometry>
#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <thread>

namespace control_manager
{
using namespace std::chrono_literals;

namespace
{

// Internal-linkage constants:
// Using an unnamed namespace makes these symbols file-local without using macros.
// This is the C++ equivalent of private module-level constants in Python.
constexpr char kManagerName[] = "control_manager";

/// Reads a 3-element double vector parameter into an Eigen::Vector3d.
Eigen::Vector3d getVector3Param(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  const std::vector<double> & default_val)
{
  const auto v = node.declare_parameter<std::vector<double>>(name, default_val);
  return Eigen::Vector3d(v.at(0), v.at(1), v.at(2));
}

}  // namespace

ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kManagerName, options)
{
  publishRateHz_ = this->declare_parameter<double>("publish_rate_hz", 250.0);
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 5.0);
  stateTimeoutS_ = this->declare_parameter<double>("state_timeout_s", 0.5);
  dependencyStartupTimeoutS_ = this->declare_parameter<double>("dependency_startup_timeout_s", 2.0);
  autoStart_ = this->declare_parameter<bool>("auto_start", true);
  controllerType_ = this->declare_parameter<std::string>("controller_type", "passthrough");

  if (autoStart_) {
    startupTimer_ = this->create_wall_timer(
      200ms,
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

  const auto startupDeadline = this->now() +
    rclcpp::Duration::from_seconds(dependencyStartupTimeoutS_);
  // Deterministic startup gate: control_manager depends on estimated_state availability.
  while (this->now() < startupDeadline) {
    if (!this->get_publishers_info_by_topic("estimated_state").empty()) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }

  if (this->get_publishers_info_by_topic("estimated_state").empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Cannot configure control_manager: upstream topic 'estimated_state' not available");
    return CallbackReturn::FAILURE;
  }

  if (controllerType_ == "se3") {
    Se3ControllerConfig config;
    config.mass = this->declare_parameter<double>("se3.mass", 1.5);
    config.gravity = this->declare_parameter<double>("se3.gravity", 9.81);
    config.dt = 1.0 / publishRateHz_;
    config.k_p = getVector3Param(*this, "se3.k_p", {6.0, 6.0, 8.0});
    config.k_v = getVector3Param(*this, "se3.k_v", {4.0, 4.0, 5.0});
    config.k_i = getVector3Param(*this, "se3.k_i", {0.5, 0.5, 0.8});
    config.k_R = getVector3Param(*this, "se3.k_R", {3.0, 3.0, 1.5});
    config.k_omega = getVector3Param(*this, "se3.k_omega", {0.5, 0.5, 0.3});
    config.k_Ri = getVector3Param(*this, "se3.k_Ri", {0.1, 0.1, 0.05});
    config.pos_int_limit = getVector3Param(*this, "se3.pos_int_limit", {2.0, 2.0, 3.0});
    config.att_int_limit = getVector3Param(*this, "se3.att_int_limit", {0.5, 0.5, 0.3});
    config.max_thrust_N = this->declare_parameter<double>("se3.max_thrust_N", 29.43);
    controller_ = std::make_unique<Se3Controller>(config);
  } else {
    controller_ = std::make_unique<Px4PassthroughController>();
  }

  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

  // Data inputs: estimator state and trajectory manager intent.
  // The typed lambda `[this](peregrine_interfaces::msg::State::SharedPtr msg) { ... }` captures `this` and forwards the
  // message to the member function. In Python, this is analogous to passing
  // a bound method like `self.on_estimated_state`.
  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    "estimated_state", qos,
    [this](peregrine_interfaces::msg::State::SharedPtr msg) { onEstimatedState(msg); });
  trajectorySetpointSub_ = this->create_subscription<peregrine_interfaces::msg::TrajectorySetpoint>(
    "trajectory_setpoint", qos,
    [this](peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg) { onTrajectorySetpoint(msg); });

  // Outputs: control envelope for hardware bridge + manager health.
  controlOutputPub_ = this->create_publisher<peregrine_interfaces::msg::ControlOutput>(
    "control_output", qos);
  statusPub_ = this->create_publisher<peregrine_interfaces::msg::ManagerStatus>(
    "control_status",
    statusQos);

  publishTimer_ =
    this->create_wall_timer(
    periodFromHz(publishRateHz_),
    [this]() { publishControlOutput(); });
  statusTimer_ =
    this->create_wall_timer(
    periodFromHz(statusRateHz_),
    [this]() { publishStatus(); });

  // Timers are armed only in lifecycle ACTIVE state.
  publishTimer_->cancel();
  statusTimer_->cancel();
  configured_ = true;
  active_ = false;

  RCLCPP_INFO(
    this->get_logger(),
    "Configured control_manager: controller=%s publish_rate_hz=%.1f status_rate_hz=%.1f",
    controllerType_.c_str(), publishRateHz_, statusRateHz_);
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
  if (controller_) {
    controller_->reset();
  }
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
  if (controlOutputPub_) {
    controlOutputPub_->on_deactivate();
  }
  if (statusPub_) {
    statusPub_->on_deactivate();
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

  // Capture frame names from upstream estimated_state.
  if (!msg->header.frame_id.empty()) {
    if (odomFrame_ != "odom" && msg->header.frame_id != odomFrame_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "estimated_state frame_id changed: '%s' -> '%s'",
        odomFrame_.c_str(), msg->header.frame_id.c_str());
    }
    odomFrame_ = msg->header.frame_id;
    // Derive baseLinkFrame_ from odomFrame_: "uav1/odom" -> "uav1/base_link".
    const auto pos = odomFrame_.rfind("odom");
    if (pos != std::string::npos) {
      baseLinkFrame_ = odomFrame_.substr(0, pos) + "base_link";
    }
  }
}

void ControlManagerNode::onTrajectorySetpoint(
  const peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg)
{
  if (!configured_) {
    return;
  }

  std::scoped_lock lock(dataMutex_);

  // Validate setpoint frame_id: must be odom (world-frame) or base_link (body-frame).
  if (!msg->header.frame_id.empty() &&
    msg->header.frame_id != odomFrame_ &&
    msg->header.frame_id != baseLinkFrame_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "trajectory_setpoint has unexpected frame_id '%s' (expected '%s' or '%s')",
      msg->header.frame_id.c_str(), odomFrame_.c_str(), baseLinkFrame_.c_str());
  }

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
  auto activeSetpoint = setpoint.has_value() ? *setpoint : makeHoldSetpoint(*state);

  // If setpoint is in body frame, rotate velocity/acceleration to odom frame.
  if (activeSetpoint.header.frame_id == baseLinkFrame_) {
    const auto & q = state->pose.pose.orientation;
    const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);

    if (activeSetpoint.use_velocity) {
      const Eigen::Vector3d bodyVel(
        activeSetpoint.velocity.x, activeSetpoint.velocity.y, activeSetpoint.velocity.z);
      const Eigen::Vector3d odomVel = orientation * bodyVel;
      activeSetpoint.velocity.x = odomVel.x();
      activeSetpoint.velocity.y = odomVel.y();
      activeSetpoint.velocity.z = odomVel.z();
    }

    if (activeSetpoint.use_acceleration) {
      const Eigen::Vector3d bodyAcc(
        activeSetpoint.acceleration.x, activeSetpoint.acceleration.y,
        activeSetpoint.acceleration.z);
      const Eigen::Vector3d odomAcc = orientation * bodyAcc;
      activeSetpoint.acceleration.x = odomAcc.x();
      activeSetpoint.acceleration.y = odomAcc.y();
      activeSetpoint.acceleration.z = odomAcc.z();
    }

    if (activeSetpoint.use_position) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Position setpoint in body frame is not meaningful; ignoring position fields");
      activeSetpoint.use_position = false;
    }

    // After rotation, setpoint is now in odom frame.
    activeSetpoint.header.frame_id = odomFrame_;
  }

  // Controller backend is pure; all ROS side-effects happen only around this call.
  auto output = controller_->compute(*state, activeSetpoint);
  output.header.stamp = this->now();

  // Set frame_id based on control mode.
  switch (output.control_mode) {
    case peregrine_interfaces::msg::ControlOutput::MODE_TRAJECTORY:
    case peregrine_interfaces::msg::ControlOutput::MODE_ATTITUDE:
      output.header.frame_id = odomFrame_;
      break;
    case peregrine_interfaces::msg::ControlOutput::MODE_BODY_RATE:
      output.header.frame_id = baseLinkFrame_;
      break;
    case peregrine_interfaces::msg::ControlOutput::MODE_DIRECT_ACTUATOR:
    default:
      output.header.frame_id = "";
      break;
  }

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
  status.active_module = controllerType_;
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
