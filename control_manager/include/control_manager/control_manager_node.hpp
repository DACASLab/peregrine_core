/**
 * @file control_manager_node.hpp
 * @brief Control manager ROS2 component.
 */

#pragma once

#include <control_manager/controller_base.hpp>

#include <peregrine_interfaces/msg/control_output.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>

namespace control_manager
{

/**
 * @class ControlManagerNode
 * @brief Publishes control output and controller health status.
 */
class ControlManagerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructs the control manager component.
   */
  explicit ControlManagerNode(const rclcpp::NodeOptions& options);

private:
  /**
   * @brief Stores the latest estimated state from estimation_manager.
   */
  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);

  /**
   * @brief Stores the latest trajectory setpoint from trajectory_manager.
   */
  void onTrajectorySetpoint(const peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg);

  /**
   * @brief Publishes control output at the configured control rate.
   */
  void publishControlOutput();

  /**
   * @brief Publishes controller health/status diagnostics.
   */
  void publishStatus();

  /**
   * @brief Converts a positive frequency in Hz to a timer period.
   */
  static std::chrono::nanoseconds periodFromHz(double hz);

  /**
   * @brief Builds a hold-position setpoint from the latest estimated state.
   */
  static peregrine_interfaces::msg::TrajectorySetpoint makeHoldSetpoint(
      const peregrine_interfaces::msg::State& state);

  /**
   * @brief Extracts ENU yaw from a quaternion.
   */
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

  std::unique_ptr<ControllerBase> controller_;

  double publishRateHz_{250.0};
  double statusRateHz_{10.0};
  double stateTimeoutS_{0.5};

  mutable std::mutex dataMutex_;
  std::optional<peregrine_interfaces::msg::State> latestState_;
  std::optional<peregrine_interfaces::msg::TrajectorySetpoint> latestSetpoint_;
  rclcpp::Time lastStateTime_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::TrajectorySetpoint>::SharedPtr trajectorySetpointSub_;

  rclcpp::Publisher<peregrine_interfaces::msg::ControlOutput>::SharedPtr controlOutputPub_;
  rclcpp::Publisher<peregrine_interfaces::msg::ManagerStatus>::SharedPtr statusPub_;

  rclcpp::TimerBase::SharedPtr publishTimer_;
  rclcpp::TimerBase::SharedPtr statusTimer_;
};

}  // namespace control_manager
