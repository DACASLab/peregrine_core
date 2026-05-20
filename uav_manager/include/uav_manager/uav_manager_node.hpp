/**
 * @file uav_manager_node.hpp
 * @brief UAV manager ROS2 lifecycle component — state gateway for BT-driven flight.
 *
 * Pipeline position: Top-level supervisor node, above all other managers.
 *   Consumes status from: hardware_abstraction, estimation_manager, control_manager,
 *                          trajectory_manager, safety_monitor
 *   Commands:              hardware_abstraction (arm/set_mode services)
 *
 * This node is a pure state machine and safety gateway. It owns the vehicle's
 * high-level flight phase (Idle, Armed, TakingOff, Hovering, Flying, Landing,
 * Landed, Emergency) and exposes only three flight actions:
 *   - takeoff:  arm -> offboard -> execute takeoff trajectory -> hover
 *   - land:     switch PX4 to land mode -> wait for auto-disarm
 *   - arm:      (via service) arm/disarm the vehicle
 *
 * Flight maneuvers (go_to, circle, figure8, etc.) are handled directly by the
 * Behavior Tree calling trajectory_manager. uav_manager does not proxy them.
 *
 * Emergencies are latched faults. The operator or BT must explicitly call the
 * ClearEmergency service to reset — there is no automatic timer-based clearance.
 *
 * Threading model (requires MultiThreadedExecutor / component_container_mt):
 *   - Default group (MutuallyExclusive): subscriptions + status timer
 *   - actionCbGroup_ (Reentrant): action server accepted callbacks (may block)
 *   - serviceCbGroup_ (MutuallyExclusive): service/action client responses
 */

#pragma once

#include <uav_manager/action_orchestrator.hpp>
#include <uav_manager/health_aggregator.hpp>
#include <uav_manager/supervisor_state_machine.hpp>
#include <uav_manager/transition_guard.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <peregrine_interfaces/action/execute_trajectory.hpp>
#include <peregrine_interfaces/action/land.hpp>
#include <peregrine_interfaces/action/takeoff.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/px4_status.hpp>
#include <peregrine_interfaces/msg/safety_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>
#include <peregrine_interfaces/srv/arm.hpp>
#include <peregrine_interfaces/srv/clear_emergency.hpp>
#include <peregrine_interfaces/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace uav_manager
{

class UavManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Takeoff = peregrine_interfaces::action::Takeoff;
  using Land = peregrine_interfaces::action::Land;
  using ExecuteTrajectory = peregrine_interfaces::action::ExecuteTrajectory;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;

  explicit UavManagerNode(const rclcpp::NodeOptions & options);

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);
  void onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg);
  void onEstimationStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  void onControlStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  void onTrajectoryStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  void onSafetyStatus(const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg);

  void publishUavState();

  rclcpp_action::GoalResponse onTakeoffGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Takeoff::Goal> goal);
  rclcpp_action::CancelResponse onTakeoffCancel(
    const std::shared_ptr<GoalHandleTakeoff> goalHandle);
  void onTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goalHandle);

  rclcpp_action::GoalResponse onLandGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal);
  rclcpp_action::CancelResponse onLandCancel(const std::shared_ptr<GoalHandleLand> goalHandle);
  void onLandAccepted(const std::shared_ptr<GoalHandleLand> goalHandle);

  bool reserveActionSlot();
  void releaseActionSlot();

  TransitionOutcome applyEvent(SupervisorEvent event);
  bool isEmergency() const;

  bool createActionServers();
  void destroyActionServers();

  /// Handles ClearEmergency service requests.
  void onClearEmergency(
    const std::shared_ptr<peregrine_interfaces::srv::ClearEmergency::Request> request,
    std::shared_ptr<peregrine_interfaces::srv::ClearEmergency::Response> response);

  StepResult callArmService(bool arm);
  StepResult callSetModeService(const std::string & mode);

  StepResult pollUntil(
    const std::string & timeoutCode,
    std::chrono::milliseconds timeout,
    const std::function<bool()> & condition,
    const std::function<bool()> & preempted,
    const std::function<bool()> & emergency) const;

  StepResult forwardTakeoffTrajectory(
    const ExecuteTrajectory::Goal & goal,
    std::function<void(const ExecuteTrajectory::Feedback &)> feedbackCallback,
    const std::function<bool()> & preempted,
    const std::function<bool()> & emergency,
    ExecuteTrajectory::Result * resultOut) const;

  double latestAltitudeM() const;

  static std::chrono::nanoseconds periodFromHz(double hz);
  static uint8_t toUavStateCode(SupervisorState state);
  std::chrono::steady_clock::time_point nowSteady() const;

  mutable std::mutex mutex_;
  std::optional<peregrine_interfaces::msg::State> latestState_;
  std::optional<peregrine_interfaces::msg::PX4Status> latestPx4Status_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestEstimationStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestControlStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestTrajectoryStatus_;

  SupervisorStateMachine supervisor_;
  std::string lastTransitionReason_{"BOOT"};

  std::atomic<bool> actionSlotReserved_{false};

  FreshnessConfig freshnessConfig_;
  std::unique_ptr<HealthAggregator> healthAggregator_;
  TransitionGuard transitionGuard_;

  bool configured_{false};
  std::atomic<bool> active_{false};

  double statusRateHz_{10.0};
  double serviceTimeoutS_{3.0};

  rclcpp::CallbackGroup::SharedPtr actionCbGroup_;
  rclcpp::CallbackGroup::SharedPtr serviceCbGroup_;

  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::PX4Status>::SharedPtr px4StatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr estimationStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr controlStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr trajectoryStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::SafetyStatus>::SharedPtr safetyStatusSub_;

  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::UAVState>::SharedPtr uavStatePub_;

  rclcpp::Client<peregrine_interfaces::srv::Arm>::SharedPtr armClient_;
  rclcpp::Client<peregrine_interfaces::srv::SetMode>::SharedPtr setModeClient_;
  /// Internal client for takeoff trajectory dispatch to trajectory_manager.
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr trajectoryExecuteClient_;

  rclcpp_action::Server<Takeoff>::SharedPtr takeoffServer_;
  rclcpp_action::Server<Land>::SharedPtr landServer_;

  /// ClearEmergency service server — operator/BT must explicitly clear latched faults.
  rclcpp::Service<peregrine_interfaces::srv::ClearEmergency>::SharedPtr clearEmergencyService_;

  rclcpp::TimerBase::SharedPtr statusTimer_;

  bool requireExternalSafety_{false};
  uint8_t latestSafetyLevel_{0};

  bool autoStart_{true};
  int dataReadinessPollMs_{200};
  rclcpp::TimerBase::SharedPtr startupTimer_;
  rclcpp::TimerBase::SharedPtr readinessTimer_;
};

}  // namespace uav_manager
