#include <control_manager/control_manager_node.hpp>

#include <Eigen/Geometry>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>

namespace control_manager
{
using namespace std::chrono_literals;

namespace
{

constexpr char kManagerName[] = "control_manager";

}  // namespace

ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kManagerName, options)
{
  pluginLoader_ = std::make_unique<pluginlib::ClassLoader<ControllerBase>>(
    "control_manager", "control_manager::ControllerBase");

  paramListener_ = std::make_shared<control_manager::ParamListener>(
      get_node_parameters_interface());
  params_ = paramListener_->get_params();

  if (params_.auto_start) {
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
  try {
    controller_ = pluginLoader_->createUniqueInstance(params_.controller_type);
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load controller '%s': %s",
                 params_.controller_type.c_str(), e.what());
    return CallbackReturn::FAILURE;
  }
  controller_->configure(*this, 1.0 / params_.publish_rate_hz);

  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

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
    periodFromHz(params_.publish_rate_hz),
    [this]() { publishControlOutput(); });
  statusTimer_ =
    this->create_wall_timer(
    periodFromHz(params_.status_rate_hz),
    [this]() { publishStatus(); });

  // Timers are armed only in lifecycle ACTIVE state.
  publishTimer_->cancel();
  statusTimer_->cancel();
  configured_ = true;
  active_ = false;

  RCLCPP_INFO(
    this->get_logger(),
    "Configured control_manager: controller=%s publish_rate_hz=%.1f status_rate_hz=%.1f",
    params_.controller_type.c_str(), params_.publish_rate_hz, params_.status_rate_hz);
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
  stopPublishing();
  if (controller_) {
    controller_->reset();
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

  std::atomic_store(&cachedState_, std::shared_ptr<const CachedState>{});
  std::atomic_store(&cachedSetpoint_,
    std::shared_ptr<const peregrine_interfaces::msg::TrajectorySetpoint>{});

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
  stopPublishing();
  RCLCPP_ERROR(this->get_logger(), "Error in control_manager lifecycle; timers canceled");
  return CallbackReturn::SUCCESS;
}

void ControlManagerNode::stopPublishing()
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
}

void ControlManagerNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  if (!configured_) {
    return;
  }

  auto snap = std::make_shared<CachedState>();
  snap->state = *msg;

  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    snap->receiveTime = this->now();
  } else {
    snap->receiveTime = rclcpp::Time(msg->header.stamp);
  }

  auto prev = std::atomic_load(&cachedState_);
  std::string prevOdom = prev ? prev->odomFrame : "odom";
  if (!msg->header.frame_id.empty()) {
    if (prevOdom != "odom" && msg->header.frame_id != prevOdom) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "estimated_state frame_id changed: '%s' -> '%s'",
        prevOdom.c_str(), msg->header.frame_id.c_str());
    }
    snap->odomFrame = msg->header.frame_id;
    const auto pos = snap->odomFrame.rfind("odom");
    if (pos != std::string::npos) {
      snap->baseLinkFrame = snap->odomFrame.substr(0, pos) + "base_link";
    }
  } else if (prev) {
    snap->odomFrame = prev->odomFrame;
    snap->baseLinkFrame = prev->baseLinkFrame;
  }

  std::atomic_store(&cachedState_, std::shared_ptr<const CachedState>(std::move(snap)));
}

void ControlManagerNode::onTrajectorySetpoint(
  const peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg)
{
  if (!configured_) {
    return;
  }

  auto stateSnap = std::atomic_load(&cachedState_);
  const std::string & odom = stateSnap ? stateSnap->odomFrame : "odom";
  const std::string & baseLink = stateSnap ? stateSnap->baseLinkFrame : "base_link";

  if (!msg->header.frame_id.empty() &&
    msg->header.frame_id != odom &&
    msg->header.frame_id != baseLink)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "trajectory_setpoint has unexpected frame_id '%s' (expected '%s' or '%s')",
      msg->header.frame_id.c_str(), odom.c_str(), baseLink.c_str());
  }

  std::atomic_store(&cachedSetpoint_,
    std::make_shared<const peregrine_interfaces::msg::TrajectorySetpoint>(*msg));
}

void ControlManagerNode::publishControlOutput()
{
  if (!active_ || !controller_ || !controlOutputPub_ || !controlOutputPub_->is_activated()) {
    return;
  }

  auto stateSnap = std::atomic_load(&cachedState_);
  if (!stateSnap) {
    return;
  }

  auto setpointSnap = std::atomic_load(&cachedSetpoint_);
  auto activeSetpoint = setpointSnap ? *setpointSnap : makeHoldSetpoint(stateSnap->state);

  if (activeSetpoint.header.frame_id == stateSnap->baseLinkFrame) {
    const auto & q = stateSnap->state.pose.pose.orientation;
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

    activeSetpoint.header.frame_id = stateSnap->odomFrame;
  }

  auto output = controller_->compute(stateSnap->state, activeSetpoint);
  output.header.stamp = this->now();

  switch (output.control_mode) {
    case peregrine_interfaces::msg::ControlOutput::MODE_TRAJECTORY:
    case peregrine_interfaces::msg::ControlOutput::MODE_ATTITUDE:
      output.header.frame_id = stateSnap->odomFrame;
      break;
    case peregrine_interfaces::msg::ControlOutput::MODE_BODY_RATE:
      output.header.frame_id = stateSnap->baseLinkFrame;
      break;
    case peregrine_interfaces::msg::ControlOutput::MODE_DIRECT_ACTUATOR:
    default:
      output.header.frame_id = "";
      break;
  }

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
  status.active_module = controller_ ? controller_->name() : params_.controller_type;
  // Explicit cast avoids implicit double->float narrowing warnings and documents the
  // precision boundary at the message interface.
  status.output_rate_hz = static_cast<float>(params_.publish_rate_hz);
  status.active = active_;

  if (!active_) {
    status.healthy = false;
    status.message = "LIFECYCLE_INACTIVE";
  } else {
    auto stateSnap = std::atomic_load(&cachedState_);
    if (!stateSnap) {
      status.healthy = false;
      status.message = "WAITING_FOR_ESTIMATED_STATE";
    } else {
      const double ageS = (this->now() - stateSnap->receiveTime).seconds();
      status.healthy = ageS <= params_.state_timeout_s;
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

// Registers this node as a composable component so it can be loaded into a
// component_container alongside other managers for intra-process communication.
RCLCPP_COMPONENTS_REGISTER_NODE(control_manager::ControlManagerNode)
