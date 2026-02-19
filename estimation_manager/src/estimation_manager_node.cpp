#include <estimation_manager/estimation_manager_node.hpp>

#include <estimation_manager/px4_passthrough_estimator.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <stdexcept>

namespace estimation_manager
{
namespace
{

constexpr char kManagerName[] = "estimation_manager";
constexpr char kModuleName[] = "px4_passthrough";

}  // namespace

EstimationManagerNode::EstimationManagerNode(const rclcpp::NodeOptions& options)
: Node(kManagerName, options), estimator_(std::make_unique<Px4PassthroughEstimator>())
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

  stateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
      "state", qos, std::bind(&EstimationManagerNode::onState, this, std::placeholders::_1));
  estimatedStatePub_ = this->create_publisher<peregrine_interfaces::msg::State>("estimated_state", qos);
  statusPub_ = this->create_publisher<peregrine_interfaces::msg::ManagerStatus>("estimation_status", statusQos);

  publishTimer_ =
      this->create_wall_timer(periodFromHz(publishRateHz_), std::bind(&EstimationManagerNode::publishEstimatedState, this));
  statusTimer_ = this->create_wall_timer(periodFromHz(statusRateHz_), std::bind(&EstimationManagerNode::publishStatus, this));

  RCLCPP_INFO(this->get_logger(), "estimation_manager started: publish_rate_hz=%.1f status_rate_hz=%.1f",
              publishRateHz_, statusRateHz_);
}

void EstimationManagerNode::onState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  estimator_->processState(*msg);
}

void EstimationManagerNode::publishEstimatedState()
{
  if (!estimator_->hasEstimate())
  {
    return;
  }

  auto estimate = estimator_->getEstimate();
  if (estimate.header.stamp.sec == 0 && estimate.header.stamp.nanosec == 0)
  {
    estimate.header.stamp = this->now();
  }
  estimatedStatePub_->publish(estimate);
}

void EstimationManagerNode::publishStatus()
{
  peregrine_interfaces::msg::ManagerStatus status;
  status.header.stamp = this->now();
  status.manager_name = kManagerName;
  status.active_module = kModuleName;
  status.output_rate_hz = static_cast<float>(publishRateHz_);
  status.active = estimator_->hasEstimate();

  if (!status.active)
  {
    status.healthy = false;
    status.message = "waiting_for_state";
  }
  else
  {
    const double ageS = (this->now() - estimator_->lastUpdateTime()).seconds();
    status.healthy = ageS <= stateTimeoutS_;
    status.message = status.healthy ? "ok" : "state_timeout";
  }

  statusPub_->publish(status);
}

std::chrono::nanoseconds EstimationManagerNode::periodFromHz(const double hz)
{
  const auto period = std::chrono::duration<double>(1.0 / hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(period);
}

}  // namespace estimation_manager

RCLCPP_COMPONENTS_REGISTER_NODE(estimation_manager::EstimationManagerNode)
