/**
 * @note C++ Primer for Python ROS2 readers
 *
 * This file follows a few recurring C++ patterns:
 * - Ownership is explicit: `std::unique_ptr` means single owner, `std::shared_ptr` means shared ownership.
 * - References (`T&`) and `const` are used to avoid unnecessary copies and make mutation intent explicit.
 * - RAII is used for resource safety: objects such as locks clean themselves up automatically at scope exit.
 * - ROS2 callbacks may run concurrently depending on executor/callback-group setup, so shared state is guarded.
 * - Templates (for example `create_subscription<MsgT>`) are compile-time type binding, not runtime reflection.
 */
/**
 * @file estimation_manager_node.hpp
 * @brief Estimation manager ROS2 lifecycle component.
 *
 * Pipeline position: hardware_abstraction -> [estimation_manager] -> control_manager,
 *                                                                    trajectory_manager,
 *                                                                    uav_manager
 *
 * This is the first lifecycle manager in the data pipeline. It subscribes to raw state
 * messages from hardware_abstraction (already converted to ENU/FLU), runs an estimator
 * backend, and publishes estimated_state for all downstream consumers.
 *
 * The estimated_state topic is the single source of truth for vehicle state throughout
 * the manager stack. It is consumed by:
 *  - control_manager: for feedback control (or passthrough)
 *  - trajectory_manager: for trajectory tracking and completion detection
 *  - uav_manager: for altitude/position feedback in action results
 *
 * The current estimator backend (Px4PassthroughEstimator) forwards PX4's onboard EKF2
 * output without modification. Future backends could implement sensor fusion, filtering,
 * or outlier rejection at this layer.
 */

#pragma once

#include <estimation_manager/estimator_base.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <chrono>
#include <memory>

namespace estimation_manager
{

/**
 * @class EstimationManagerNode
 * @brief Publishes estimator output and estimator health status.
 */
class EstimationManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructs the estimation manager lifecycle component.
   */
  explicit EstimationManagerNode(const rclcpp::NodeOptions & options);

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Allocates ROS interfaces and estimator backend.
   *
   * Also enforces startup dependency ordering by waiting for an upstream
   * publisher on topic `state` before succeeding.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Enables publishers and periodic timers.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Stops timers and deactivates lifecycle publishers.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Releases subscriptions, publishers, timers, and estimator resources.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reuses cleanup path during final shutdown.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Best-effort error handler that halts periodic activity.
   */
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

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

  /// Active estimator backend.
  std::unique_ptr<EstimatorBase> estimator_;

  /// Output publication frequency for estimated_state.
  double publishRateHz_{250.0};
  /// Status publication frequency for estimation_status.
  double statusRateHz_{10.0};
  /// Max accepted estimate age before reporting unhealthy.
  double stateTimeoutS_{0.5};
  /// Configure-time wait bound for required upstream topic discovery.
  double dependencyStartupTimeoutS_{2.0};

  /// True once configure has successfully allocated runtime resources.
  bool configured_{false};
  /// True while node is lifecycle-active.
  bool active_{false};

  /// Upstream state input from hardware abstraction.
  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr stateSub_;
  /// Lifecycle-gated estimate output for downstream managers.
  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::State>::SharedPtr
    estimatedStatePub_;
  /// Lifecycle-gated manager status output.
  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::ManagerStatus>::SharedPtr
    statusPub_;

  /// Periodic estimate publication timer.
  rclcpp::TimerBase::SharedPtr publishTimer_;
  /// Periodic status publication timer.
  rclcpp::TimerBase::SharedPtr statusTimer_;
};

}  // namespace estimation_manager
