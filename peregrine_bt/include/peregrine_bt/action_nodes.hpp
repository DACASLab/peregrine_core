#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <peregrine_interfaces/action/takeoff.hpp>
#include <peregrine_interfaces/action/land.hpp>
#include <peregrine_interfaces/action/execute_trajectory.hpp>
#include <peregrine_interfaces/action/go_to.hpp>

namespace peregrine_bt
{

// ---------------------------------------------------------------------------
// TakeoffAction — calls uav_manager/~/takeoff
// ---------------------------------------------------------------------------

class TakeoffAction : public BT::RosActionNode<peregrine_interfaces::action::Takeoff>
{
public:
  using BT::RosActionNode<peregrine_interfaces::action::Takeoff>::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("altitude_m", 5.0, "Target takeoff altitude"),
      BT::InputPort<double>("climb_velocity_mps", 1.0, "Climb velocity"),
      BT::OutputPort<double>("final_altitude_m", "Achieved altitude"),
    });
  }

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};

// ---------------------------------------------------------------------------
// LandAction — calls uav_manager/~/land
// ---------------------------------------------------------------------------

class LandAction : public BT::RosActionNode<peregrine_interfaces::action::Land>
{
public:
  using BT::RosActionNode<peregrine_interfaces::action::Land>::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("descent_velocity_mps", 0.8, "Descent velocity"),
    });
  }

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};

// ---------------------------------------------------------------------------
// ExecuteTrajectoryAction — calls trajectory_manager/~/execute_trajectory
// ---------------------------------------------------------------------------

class ExecuteTrajectoryAction
  : public BT::RosActionNode<peregrine_interfaces::action::ExecuteTrajectory>
{
public:
  using BT::RosActionNode<peregrine_interfaces::action::ExecuteTrajectory>::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("trajectory_type", "Type name (circle, figure8, lemniscate, etc.)"),
      BT::InputPort<std::string>("params", "Comma-separated parameter values"),
    });
  }

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};

// ---------------------------------------------------------------------------
// GoToAction — calls trajectory_manager/~/go_to
// ---------------------------------------------------------------------------

class GoToAction : public BT::RosActionNode<peregrine_interfaces::action::GoTo>
{
public:
  using BT::RosActionNode<peregrine_interfaces::action::GoTo>::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("x", "Target X (ENU meters)"),
      BT::InputPort<double>("y", "Target Y (ENU meters)"),
      BT::InputPort<double>("z", "Target Z (ENU meters)"),
      BT::InputPort<double>("yaw", 0.0, "Target yaw (radians, ENU)"),
      BT::InputPort<double>("velocity_mps", 1.0, "Cruise velocity"),
    });
  }

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};

}  // namespace peregrine_bt
