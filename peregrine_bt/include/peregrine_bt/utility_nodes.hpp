#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace peregrine_bt
{

class WaitAction : public BT::StatefulActionNode
{
public:
  WaitAction(const std::string& name, const BT::NodeConfig& config,
             rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration_s", "Wait duration in seconds"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time deadline_;
};

}  // namespace peregrine_bt
