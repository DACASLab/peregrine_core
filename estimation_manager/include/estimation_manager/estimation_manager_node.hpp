/**
 * @file estimation_manager_node.hpp
 * @brief Estimation manager ROS2 component.
 */

#pragma once

#include <estimation_manager/estimator_base.hpp>

#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

namespace estimation_manager
{

/**
 * @class EstimationManagerNode
 * @brief Publishes estimator output and estimator health status.
 */
class EstimationManagerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructs the estimation manager component.
   */
  explicit EstimationManagerNode(const rclcpp::NodeOptions& options);

private:
  /**
   * @brief Handles incoming state messages from hardware_abstraction.
   */
  void onState(const peregrine_interfaces::msg::State::SharedPtr msg);

  /**
   * @brief Publishes the latest estimated state at the configured rate.
   */
  void publishEstimatedState();

  /**
   * @brief Publishes estimator health/status diagnostics.
   */
  void publishStatus();

  /**
   * @brief Converts a positive frequency in Hz to a timer period.
   */
  static std::chrono::nanoseconds periodFromHz(double hz);

  std::unique_ptr<EstimatorBase> estimator_;

  double publishRateHz_{250.0};
  double statusRateHz_{10.0};
  double stateTimeoutS_{0.5};

  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr stateSub_;
  rclcpp::Publisher<peregrine_interfaces::msg::State>::SharedPtr estimatedStatePub_;
  rclcpp::Publisher<peregrine_interfaces::msg::ManagerStatus>::SharedPtr statusPub_;

  rclcpp::TimerBase::SharedPtr publishTimer_;
  rclcpp::TimerBase::SharedPtr statusTimer_;
};

}  // namespace estimation_manager
