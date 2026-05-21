#pragma once

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>
#include <peregrine_interfaces/msg/safety_status.hpp>
#include <peregrine_interfaces/msg/gps_status.hpp>
#include <peregrine_interfaces/msg/px4_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>

namespace peregrine_bt
{

// ---------------------------------------------------------------------------
// UAV state conditions (subscribe to "uav_state")
// ---------------------------------------------------------------------------

class IsArmed : public BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::UAVState>& msg) override;
};

class IsFlying : public BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::UAVState>& msg) override;
};

class IsLanded : public BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::UAVState>& msg) override;
};

class IsDependenciesReady : public BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::UAVState>& msg) override;
};

class IsEmergency : public BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::UAVState>& msg) override;
};

class IsConnected : public BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::UAVState>& msg) override;
};

class IsOffboard : public BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::UAVState>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::UAVState>& msg) override;
};

// ---------------------------------------------------------------------------
// Safety conditions (subscribe to "safety_status")
// ---------------------------------------------------------------------------

class IsSafetyNominal : public BT::RosTopicSubNode<peregrine_interfaces::msg::SafetyStatus>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::SafetyStatus>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::SafetyStatus>& msg) override;
};

class IsSafetyAtLeast : public BT::RosTopicSubNode<peregrine_interfaces::msg::SafetyStatus>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::SafetyStatus>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<unsigned>("max_level", 1, "Maximum tolerable safety level (0=NOMINAL, 1=WARNING, 2=CRITICAL)"),
    });
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::SafetyStatus>& msg) override;
};

// ---------------------------------------------------------------------------
// Sensor conditions
// ---------------------------------------------------------------------------

class IsBatteryAbove : public BT::RosTopicSubNode<peregrine_interfaces::msg::PX4Status>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::PX4Status>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("threshold_pct", 0.2, "Minimum battery fraction (0.0-1.0)"),
    });
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::PX4Status>& msg) override;
};

class IsGpsHealthy : public BT::RosTopicSubNode<peregrine_interfaces::msg::GpsStatus>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::GpsStatus>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<unsigned>("min_fix_type", 3, "Minimum GPS fix type (3=3D)"),
      BT::InputPort<double>("max_hdop", 5.0, "Maximum acceptable HDOP"),
      BT::InputPort<double>("max_vdop", 5.0, "Maximum acceptable VDOP"),
    });
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::GpsStatus>& msg) override;
};

// ---------------------------------------------------------------------------
// Position conditions (subscribe to "state" — estimated state)
// ---------------------------------------------------------------------------

class HasValidState : public BT::RosTopicSubNode<peregrine_interfaces::msg::State>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::State>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("max_age_s", 1.0, "Maximum age of estimated state in seconds"),
    });
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::State>& msg) override;
};

class IsAboveAltitude : public BT::RosTopicSubNode<peregrine_interfaces::msg::State>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::State>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("min_alt_m", "Minimum altitude in meters"),
    });
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::State>& msg) override;
};

class IsBelowAltitude : public BT::RosTopicSubNode<peregrine_interfaces::msg::State>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::State>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("max_alt_m", "Maximum altitude in meters"),
    });
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::State>& msg) override;
};

class IsAtPosition : public BT::RosTopicSubNode<peregrine_interfaces::msg::State>
{
public:
  using BT::RosTopicSubNode<peregrine_interfaces::msg::State>::RosTopicSubNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("target_x", "Target X position (ENU meters)"),
      BT::InputPort<double>("target_y", "Target Y position (ENU meters)"),
      BT::InputPort<double>("target_z", "Target Z position (ENU meters)"),
      BT::InputPort<double>("tolerance_m", 0.5, "Position tolerance in meters"),
    });
  }

  BT::NodeStatus onTick(const std::shared_ptr<peregrine_interfaces::msg::State>& msg) override;
};

}  // namespace peregrine_bt
