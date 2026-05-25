#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>
#include <peregrine_interfaces/msg/safety_status.hpp>

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

class WaitUntilReady : public BT::StatefulActionNode
{
public:
  WaitUntilReady(const std::string& name, const BT::NodeConfig& config,
                 rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout_s", 300.0, "Maximum wait time in seconds"),
      BT::InputPort<double>("settle_s", 3.0, "Seconds to wait after ready before succeeding"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time deadline_;
  rclcpp::Time settle_deadline_;
  bool ready_seen_{false};
  rclcpp::Subscription<peregrine_interfaces::msg::UAVState>::SharedPtr sub_;
  std::atomic<bool> deps_ready_{false};
};

}  // namespace peregrine_bt
