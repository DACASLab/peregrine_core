/**
 * @file control_manager_node.hpp
 * @brief Control manager ROS2 lifecycle component.
 *
 * Pipeline position: estimation_manager -> [control_manager] -> hardware_abstraction
 *
 * This node sits between the trajectory planner and the hardware bridge. It receives
 * the vehicle's estimated state from estimation_manager and desired setpoints from
 * trajectory_manager, runs a controller backend, and publishes ControlOutput messages
 * that hardware_abstraction converts into PX4-compatible setpoints.
 *
 * Key design choices:
 *  - Lifecycle node: resources (subs, pubs, timers) are only allocated during configure
 *    and only publish data during the ACTIVE state. This allows the lifecycle orchestrator
 *    to bring up the data pipeline in a deterministic order.
 *  - Composable node: registered via RCLCPP_COMPONENTS_REGISTER_NODE so it can be loaded
 *    into a component_container process, sharing memory with other managers and enabling
 *    intra-process zero-copy transport.
 *  - Decoupled from PX4: this node only knows about peregrine_interfaces messages, not
 *    px4_msgs. The ENU->NED conversion and PX4 protocol details are handled exclusively
 *    by hardware_abstraction.
 *
 * Threading: all callbacks run in the component_container's executor. The mutex_ protects
 * shared data (latest state/setpoint) between subscription callbacks (writers) and timer
 * callbacks (readers). The lock is held only for copies, not during compute().
 */

#pragma once

#include <control_manager/controller_base.hpp>
#include <control_manager/se3_controller.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <peregrine_interfaces/msg/control_output.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

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
class ControlManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructs the control manager lifecycle component.
   */
  explicit ControlManagerNode(const rclcpp::NodeOptions & options);

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Allocates controller backend and ROS interfaces.
   *
   * Enforces startup ordering by waiting for upstream `estimated_state`
   * publishers before completing configuration.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Enables output/status publishers and periodic timers.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Stops timers and deactivates lifecycle publishers.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Releases subscriptions, publishers, timers, and cached inputs.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reuses cleanup path during node shutdown.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Best-effort error callback that halts periodic work.
   */
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

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
    const peregrine_interfaces::msg::State & state);

  /**
   * @brief Extracts ENU yaw from a quaternion.
   */
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

  /// Active controller backend; swapped at configure time based on `controller_type` parameter.
  std::unique_ptr<ControllerBase> controller_;

  /// Selected controller type ("passthrough" or "se3").
  std::string controllerType_{"passthrough"};

  /// Frame names captured from first estimated_state message.
  std::string odomFrame_{"odom"};
  std::string baseLinkFrame_{"base_link"};

  /// Control output publication frequency.
  double publishRateHz_{250.0};
  /// Manager status publication frequency.
  double statusRateHz_{10.0};
  /// Maximum accepted estimated_state age before unhealthy status.
  double stateTimeoutS_{0.5};
  /// Configure-time wait bound for upstream topic discovery.
  double dependencyStartupTimeoutS_{2.0};

  /// True once configure has successfully allocated runtime resources.
  bool configured_{false};
  /// True while node is lifecycle-active.
  bool active_{false};

  /// Protects cached state/setpoint and last sample timestamps.
  mutable std::mutex dataMutex_;

  /// Latest estimated state sample; empty until the first message arrives.
  std::optional<peregrine_interfaces::msg::State> latestState_;
  /// Latest desired trajectory setpoint.
  std::optional<peregrine_interfaces::msg::TrajectorySetpoint> latestSetpoint_;
  /// Effective receive time used for freshness checks.
  rclcpp::Time lastStateTime_{0, 0, RCL_ROS_TIME};

  /// Estimated state input from estimation_manager.
  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  /// Setpoint input from trajectory_manager.
  rclcpp::Subscription<peregrine_interfaces::msg::TrajectorySetpoint>::SharedPtr
    trajectorySetpointSub_;

  /// Lifecycle-gated control output for hardware abstraction.
  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::ControlOutput>::SharedPtr
    controlOutputPub_;
  /// Lifecycle-gated manager status output.
  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::ManagerStatus>::SharedPtr
    statusPub_;

  /// Periodic control publication timer.
  rclcpp::TimerBase::SharedPtr publishTimer_;
  /// Periodic status publication timer.
  rclcpp::TimerBase::SharedPtr statusTimer_;

  /// When true, node self-transitions through configure -> activate on startup.
  bool autoStart_{true};
  /// One-shot timer that drives the auto-start sequence.
  rclcpp::TimerBase::SharedPtr startupTimer_;
};

}  // namespace control_manager
