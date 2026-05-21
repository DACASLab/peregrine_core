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

}  // namespace peregrine_bt
