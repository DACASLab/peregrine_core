#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <peregrine_interfaces/action/execute_trajectory.hpp>
#include <peregrine_interfaces/action/go_to.hpp>
#include <peregrine_interfaces/action/land.hpp>
#include <peregrine_interfaces/action/takeoff.hpp>
#include <peregrine_interfaces/msg/safety_status.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>
#include <peregrine_interfaces/srv/clear_emergency.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <optional>

namespace mission_executor
{

class MissionExecutorNode : public rclcpp::Node
{
public:
  explicit MissionExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void registerNodes();
  void tick();

  bool waitForReady(double timeout_s);
  bool isMotionAuthorized() const;
  bool isAirborne() const;
  bool isFaultLatched() const;
  bool isSafetyNominal() const;

  bool sendTakeoff(double altitude_m, double climb_velocity_mps);
  bool sendLand(double descent_velocity_mps);
  bool sendGoTo(double x, double y, double z, double yaw, double velocity_mps, double acceptance_radius_m);
  bool sendTrajectory(const std::string & type, const std::vector<double> & params);
  bool callClearEmergency();

  template<typename ActionT>
  bool sendActionGoal(
    rclcpp_action::Client<ActionT> & client,
    typename ActionT::Goal goal,
    const std::string & name,
    double timeout_s = 60.0);

  rclcpp::Subscription<peregrine_interfaces::msg::UAVState>::SharedPtr uavStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::SafetyStatus>::SharedPtr safetySub_;

  std::optional<peregrine_interfaces::msg::UAVState> latestUavState_;
  std::optional<peregrine_interfaces::msg::SafetyStatus> latestSafety_;

  rclcpp_action::Client<peregrine_interfaces::action::Takeoff>::SharedPtr takeoffClient_;
  rclcpp_action::Client<peregrine_interfaces::action::Land>::SharedPtr landClient_;
  rclcpp_action::Client<peregrine_interfaces::action::GoTo>::SharedPtr goToClient_;
  rclcpp_action::Client<peregrine_interfaces::action::ExecuteTrajectory>::SharedPtr executeClient_;
  rclcpp::Client<peregrine_interfaces::srv::ClearEmergency>::SharedPtr clearEmergencyClient_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr tickTimer_;
};

}  // namespace mission_executor
