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

  // `std::unique_ptr<ControllerBase>` holds a pointer to a ControllerBase-derived
  // object (currently Px4PassthroughController). unique_ptr enforces single
  // ownership: only one unique_ptr can own the object at a time. When the
  // unique_ptr is destroyed (e.g., during on_cleanup), the controller is
  // automatically deleted. This is equivalent to Python's typical ownership
  // model where `self.controller = Px4PassthroughController()` holds the only
  // reference and the GC cleans up when `self` dies.
  //
  // The pointer is to the BASE class (ControllerBase), but it points to a
  // DERIVED object (Px4PassthroughController). This is "polymorphism" — calling
  // `controller_->compute()` dispatches to the derived class's implementation.
  // In Python, this happens automatically via duck typing; in C++, it requires
  // the `virtual` keyword on the base class method.
  /// Active controller backend implementation.
  std::unique_ptr<ControllerBase> controller_;

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

  // `std::optional<T>` is a container that either holds a value of type T or is
  // empty (has no value). This is the C++ equivalent of Python's `Optional[T]`
  // or a variable that can be `None`. You check if it has a value with
  // `.has_value()` (like `x is not None` in Python) and access the value with
  // `*opt` or `opt.value()`. Using optional makes it explicit that these fields
  // start empty and are only populated once the first message arrives.
  /// Latest estimated state sample.
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
