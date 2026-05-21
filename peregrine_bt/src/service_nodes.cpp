#include "peregrine_bt/service_nodes.hpp"

namespace peregrine_bt
{

// ---------------------------------------------------------------------------
// ArmService
// ---------------------------------------------------------------------------

bool ArmService::setRequest(Request::SharedPtr& request)
{
  bool arm = true;
  getInput("arm", arm);
  request->arm = arm;
  RCLCPP_INFO(logger(), "ArmService: %s", arm ? "ARM" : "DISARM");
  return true;
}

BT::NodeStatus ArmService::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(logger(), "ArmService failed: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ArmService::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "ArmService error: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// SetModeService
// ---------------------------------------------------------------------------

bool SetModeService::setRequest(Request::SharedPtr& request)
{
  getInput("mode", request->mode);
  RCLCPP_INFO(logger(), "SetModeService: %s", request->mode.c_str());
  return true;
}

BT::NodeStatus SetModeService::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(logger(), "SetModeService failed: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SetModeService::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "SetModeService error: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// ClearEmergencyService
// ---------------------------------------------------------------------------

bool ClearEmergencyService::setRequest(Request::SharedPtr& /*request*/)
{
  RCLCPP_INFO(logger(), "ClearEmergencyService: requesting emergency clear");
  return true;
}

BT::NodeStatus ClearEmergencyService::onResponseReceived(
  const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Emergency cleared");
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(logger(), "ClearEmergency failed: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ClearEmergencyService::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "ClearEmergencyService error: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace peregrine_bt
