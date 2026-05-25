#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <peregrine_interfaces/srv/arm.hpp>
#include <peregrine_interfaces/srv/clear_emergency.hpp>

namespace peregrine_bt
{

// ---------------------------------------------------------------------------
// ArmService — calls hardware_abstraction/arm
// ---------------------------------------------------------------------------

class ArmService : public BT::RosServiceNode<peregrine_interfaces::srv::Arm>
{
public:
  using BT::RosServiceNode<peregrine_interfaces::srv::Arm>::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<bool>("arm", true, "true to arm, false to disarm"),
    });
  }

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

// ---------------------------------------------------------------------------
// ClearEmergencyService — calls uav_manager/~/clear_emergency
// ---------------------------------------------------------------------------

class ClearEmergencyService
  : public BT::RosServiceNode<peregrine_interfaces::srv::ClearEmergency>
{
public:
  using BT::RosServiceNode<peregrine_interfaces::srv::ClearEmergency>::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

}  // namespace peregrine_bt
