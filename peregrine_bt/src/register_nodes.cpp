#include "peregrine_bt/register_nodes.hpp"
#include "peregrine_bt/condition_nodes.hpp"
#include "peregrine_bt/action_nodes.hpp"
#include "peregrine_bt/service_nodes.hpp"
#include "peregrine_bt/utility_nodes.hpp"

#include <behaviortree_ros2/ros_node_params.hpp>

namespace peregrine_bt
{

void registerAllNodes(BT::BehaviorTreeFactory& factory,
                      rclcpp::Node::SharedPtr node)
{
  BT::RosNodeParams params;
  params.nh = node;
  // Goal-acceptance timeout at the ROS action client level. Must exceed the
  // uav_manager's own service_timeout_s (3s for arm/offboard PX4 calls) to
  // avoid the BT client timing out while the server is still processing.
  params.server_timeout = std::chrono::milliseconds(30000);
  // Don't block during tree construction — preflight retry nodes in the tree
  // handle the wait for DDS discovery.
  params.wait_for_server_timeout = std::chrono::milliseconds(0);

  // -- UAV state conditions (topic: uav_state) --
  params.default_port_value = "uav_state";
  factory.registerNodeType<IsArmed>("IsArmed", params);
  factory.registerNodeType<IsFlying>("IsFlying", params);
  factory.registerNodeType<IsLanded>("IsLanded", params);
  factory.registerNodeType<IsDependenciesReady>("IsDependenciesReady", params);
  factory.registerNodeType<IsEmergency>("IsEmergency", params);
  factory.registerNodeType<IsConnected>("IsConnected", params);
  factory.registerNodeType<IsOffboard>("IsOffboard", params);

  // -- Safety conditions (topic: safety_status) --
  params.default_port_value = "safety_status";
  factory.registerNodeType<IsSafetyNominal>("IsSafetyNominal", params);
  factory.registerNodeType<IsSafetyAtLeast>("IsSafetyAtLeast", params);

  // -- Sensor conditions --
  params.default_port_value = "status";
  factory.registerNodeType<IsBatteryAbove>("IsBatteryAbove", params);

  params.default_port_value = "gps_status";
  factory.registerNodeType<IsGpsHealthy>("IsGpsHealthy", params);

  // -- Position conditions (topic: state — estimated state) --
  params.default_port_value = "state";
  factory.registerNodeType<HasValidState>("HasValidState", params);
  factory.registerNodeType<IsAboveAltitude>("IsAboveAltitude", params);
  factory.registerNodeType<IsBelowAltitude>("IsBelowAltitude", params);
  factory.registerNodeType<IsAtPosition>("IsAtPosition", params);

  // -- Action nodes --
  params.default_port_value = "uav_manager/takeoff";
  factory.registerNodeType<TakeoffAction>("TakeoffAction", params);

  params.default_port_value = "uav_manager/land";
  factory.registerNodeType<LandAction>("LandAction", params);

  params.default_port_value = "trajectory_manager/execute_trajectory";
  factory.registerNodeType<ExecuteTrajectoryAction>("ExecuteTrajectoryAction", params);

  params.default_port_value = "trajectory_manager/go_to";
  factory.registerNodeType<GoToAction>("GoToAction", params);

  // -- Service nodes --
  params.default_port_value = "arm";
  factory.registerNodeType<ArmService>("ArmService", params);

  params.default_port_value = "uav_manager/clear_emergency";
  factory.registerNodeType<ClearEmergencyService>("ClearEmergencyService", params);

  // -- Utility nodes --
  factory.registerNodeType<WaitAction>("WaitAction", node);
  factory.registerNodeType<WaitUntilReady>("WaitUntilReady", node);
}

}  // namespace peregrine_bt
