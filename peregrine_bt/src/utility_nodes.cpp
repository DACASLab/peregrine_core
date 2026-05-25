#include "peregrine_bt/utility_nodes.hpp"

namespace peregrine_bt
{

BT::NodeStatus WaitAction::onStart()
{
  double duration_s = 0.0;
  getInput("duration_s", duration_s);
  deadline_ = node_->now() + rclcpp::Duration::from_seconds(duration_s);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitAction::onRunning()
{
  return (node_->now() >= deadline_)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitUntilReady::onStart()
{
  double timeout_s = 300.0;
  getInput("timeout_s", timeout_s);
  deadline_ = node_->now() + rclcpp::Duration::from_seconds(timeout_s);
  ready_seen_ = false;
  deps_ready_ = false;

  sub_ = node_->create_subscription<peregrine_interfaces::msg::UAVState>(
      "uav_state", 10,
      [this](const peregrine_interfaces::msg::UAVState::SharedPtr msg) {
        deps_ready_ = msg->dependencies_ready;
      });

  RCLCPP_INFO(node_->get_logger(), "WaitUntilReady: waiting for dependencies (timeout %.0fs)", timeout_s);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitUntilReady::onRunning()
{
  if (node_->now() >= deadline_)
  {
    RCLCPP_ERROR(node_->get_logger(), "WaitUntilReady: timed out");
    sub_.reset();
    return BT::NodeStatus::FAILURE;
  }

  if (deps_ready_)
  {
    if (!ready_seen_)
    {
      ready_seen_ = true;
      double settle_s = 3.0;
      getInput("settle_s", settle_s);
      settle_deadline_ = node_->now() + rclcpp::Duration::from_seconds(settle_s);
      RCLCPP_INFO(node_->get_logger(), "WaitUntilReady: dependencies ready, settling %.1fs", settle_s);
    }
    if (node_->now() >= settle_deadline_)
    {
      RCLCPP_INFO(node_->get_logger(), "WaitUntilReady: ready and settled");
      sub_.reset();
      return BT::NodeStatus::SUCCESS;
    }
  }
  else
  {
    ready_seen_ = false;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitUntilReady::onHalted()
{
  sub_.reset();
}

}  // namespace peregrine_bt
