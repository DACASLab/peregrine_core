#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace peregrine_bt
{

void registerAllNodes(BT::BehaviorTreeFactory& factory,
                      rclcpp::Node::SharedPtr node);

}  // namespace peregrine_bt
