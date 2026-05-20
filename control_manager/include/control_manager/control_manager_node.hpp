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
 * Threading: subscriber callbacks store immutable snapshots via atomic shared_ptr
 * operations (lock-free on most platforms). Timer callbacks load snapshots without
 * blocking subscriber writes.
 */

#pragma once

#include <control_manager/controller_base.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <peregrine_interfaces/msg/control_output.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/trajectory_setpoint.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

namespace control_manager
{

struct CachedState
{
  peregrine_interfaces::msg::State state;
  rclcpp::Time receiveTime{0, 0, RCL_ROS_TIME};
  std::string odomFrame{"odom"};
  std::string baseLinkFrame{"base_link"};
};

/**
 * @class ControlManagerNode
 * @brief Publishes control output and controller health status.
 */
class ControlManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ControlManagerNode(const rclcpp::NodeOptions & options);

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  void stopPublishing();
  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);
  void onTrajectorySetpoint(const peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg);
  void publishControlOutput();
  void publishStatus();

  static std::chrono::nanoseconds periodFromHz(double hz);
  static peregrine_interfaces::msg::TrajectorySetpoint makeHoldSetpoint(
    const peregrine_interfaces::msg::State & state);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

  std::unique_ptr<pluginlib::ClassLoader<ControllerBase>> pluginLoader_;
  pluginlib::UniquePtr<ControllerBase> controller_;
  std::string controllerType_{"control_manager::Px4PassthroughController"};

  double publishRateHz_{250.0};
  double statusRateHz_{10.0};
  double stateTimeoutS_{0.5};

  bool configured_{false};
  bool active_{false};

  std::shared_ptr<const CachedState> cachedState_;
  std::shared_ptr<const peregrine_interfaces::msg::TrajectorySetpoint> cachedSetpoint_;

  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::TrajectorySetpoint>::SharedPtr
    trajectorySetpointSub_;

  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::ControlOutput>::SharedPtr
    controlOutputPub_;
  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::ManagerStatus>::SharedPtr
    statusPub_;

  rclcpp::TimerBase::SharedPtr publishTimer_;
  rclcpp::TimerBase::SharedPtr statusTimer_;

  bool autoStart_{true};
  rclcpp::TimerBase::SharedPtr startupTimer_;
};

}  // namespace control_manager
