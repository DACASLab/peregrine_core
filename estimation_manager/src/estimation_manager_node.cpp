#include <estimation_manager/estimation_manager_node.hpp>

#include <estimation_manager/px4_passthrough_estimator.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <stdexcept>

namespace estimation_manager
{
using namespace std::chrono_literals;

namespace
{

constexpr char kManagerName[] = "estimation_manager";
constexpr char kModuleName[] = "px4_passthrough";

}  // namespace

// Parameters are declared in the constructor because ROS2 lifecycle nodes require
// all parameters to be declared before the configure transition is triggered.
EstimationManagerNode::EstimationManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kManagerName, options)
{
  publishRateHz_ = this->declare_parameter<double>("publish_rate_hz", 250.0);
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 5.0);
  stateTimeoutS_ = this->declare_parameter<double>("state_timeout_s", 0.5);
  dependencyStartupTimeoutS_ = this->declare_parameter<double>("dependency_startup_timeout_s", 2.0);
  autoStart_ = this->declare_parameter<bool>("auto_start", true);

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

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (publishRateHz_ <= 0.0 || statusRateHz_ <= 0.0 || stateTimeoutS_ <= 0.0)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "publish_rate_hz, status_rate_hz, state_timeout_s, and dependency_startup_timeout_s must be > 0");
    return CallbackReturn::FAILURE;
  }

  estimator_ = std::make_unique<Px4PassthroughEstimator>();

  // Reliable QoS with generous depth absorbs message bursts during startup when
  // multiple nodes begin publishing simultaneously.
  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

  stateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    "state", qos, [this](peregrine_interfaces::msg::State::SharedPtr msg) { onState(msg); });
  estimatedStatePub_ = this->create_publisher<peregrine_interfaces::msg::State>(
    "estimated_state",
    qos);
  statusPub_ = this->create_publisher<peregrine_interfaces::msg::ManagerStatus>(
    "estimation_status",
    statusQos);

  // Timers are created here but immediately canceled. They are re-enabled in
  // on_activate, avoiding the overhead of destroying and recreating them on each
  // activate/deactivate cycle.
  publishTimer_ =
    this->create_wall_timer(
    periodFromHz(publishRateHz_),
    [this]() { publishEstimatedState(); });
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
    this->get_logger(), "Configured estimation_manager: publish_rate_hz=%.1f status_rate_hz=%.1f",
    publishRateHz_, statusRateHz_);
  return CallbackReturn::SUCCESS;
}

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!configured_) {
    RCLCPP_ERROR(this->get_logger(), "Cannot activate before configure");
    return CallbackReturn::FAILURE;
  }

  // Activate publishers before resetting timers to ensure the first timer tick
  // does not fire before the publisher is ready to send messages.
  estimatedStatePub_->on_activate();
  statusPub_->on_activate();
  publishTimer_->reset();
  statusTimer_->reset();
  active_ = true;
  RCLCPP_INFO(this->get_logger(), "Activated estimation_manager");
  return CallbackReturn::SUCCESS;
}

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (publishTimer_) {
    publishTimer_->cancel();
  }
  if (statusTimer_) {
    statusTimer_->cancel();
  }
  if (estimatedStatePub_) {
    estimatedStatePub_->on_deactivate();
  }
  if (statusPub_) {
    statusPub_->on_deactivate();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated estimation_manager");
  return CallbackReturn::SUCCESS;
}

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  configured_ = false;
  publishTimer_.reset();
  statusTimer_.reset();
  stateSub_.reset();
  estimatedStatePub_.reset();
  statusPub_.reset();
  estimator_.reset();
  RCLCPP_INFO(this->get_logger(), "Cleaned up estimation_manager");
  return CallbackReturn::SUCCESS;
}

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  (void)on_cleanup(this->get_current_state());
  RCLCPP_INFO(this->get_logger(), "Shut down estimation_manager");
  return CallbackReturn::SUCCESS;
}

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_error(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (publishTimer_) {
    publishTimer_->cancel();
  }
  if (statusTimer_) {
    statusTimer_->cancel();
  }
  if (estimatedStatePub_) {
    estimatedStatePub_->on_deactivate();
  }
  if (statusPub_) {
    statusPub_->on_deactivate();
  }
  RCLCPP_ERROR(this->get_logger(), "Error in estimation_manager lifecycle; timers canceled");
  return CallbackReturn::SUCCESS;
}

void EstimationManagerNode::onState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  if (!configured_ || !estimator_) {
    return;
  }
  estimator_->processState(*msg);
}

/// Preserves the upstream source timestamp for end-to-end latency tracking.
/// Falls back to this->now() (which respects use_sim_time) if the stamp is unset.
void EstimationManagerNode::publishEstimatedState()
{
  if (!active_ || !estimator_ || !estimatedStatePub_ || !estimatedStatePub_->is_activated()) {
    return;
  }
  if (!estimator_->hasEstimate()) {
    return;
  }

  auto estimate = estimator_->getEstimate();
  if (estimate.header.stamp.sec == 0 && estimate.header.stamp.nanosec == 0) {
    estimate.header.stamp = this->now();
  }
  estimatedStatePub_->publish(estimate);
}

/// Health evaluation follows a priority ladder: INACTIVE > no estimate > stale > OK.
void EstimationManagerNode::publishStatus()
{
  if (!configured_ || !statusPub_ || !statusPub_->is_activated()) {
    return;
  }

  peregrine_interfaces::msg::ManagerStatus status;
  status.header.stamp = this->now();
  status.manager_name = kManagerName;
  status.active_module = kModuleName;
  status.output_rate_hz = static_cast<float>(publishRateHz_);
  status.active = active_;

  if (!active_) {
    status.healthy = false;
    status.message = "LIFECYCLE_INACTIVE";
  } else if (!estimator_ || !estimator_->hasEstimate()) {
    status.healthy = false;
    status.message = "WAITING_FOR_STATE";
  } else {
    const double ageS = (this->now() - estimator_->lastUpdateTime()).seconds();
    status.healthy = ageS <= stateTimeoutS_;
    status.message = status.healthy ? "OK" : "STATE_STALE";
  }

  statusPub_->publish(status);
}

std::chrono::nanoseconds EstimationManagerNode::periodFromHz(const double hz)
{
  const auto period = std::chrono::duration<double>(1.0 / hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(period);
}

}  // namespace estimation_manager

// Registers this node as a composable component so it can be loaded into a
// component_container alongside other managers for intra-process communication.
RCLCPP_COMPONENTS_REGISTER_NODE(estimation_manager::EstimationManagerNode)
