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
 * @file uav_manager_node.hpp
 * @brief UAV manager ROS2 lifecycle component.
 *
 * Pipeline position: Top-level supervisor node, above all other managers.
 *   Consumes status from: hardware_abstraction, estimation_manager, control_manager,
 *                          trajectory_manager
 *   Commands:              hardware_abstraction (arm/set_mode services)
 *   Delegates to:          trajectory_manager (go_to, execute_trajectory action servers)
 *
 * This node is the user-facing entry point for high-level flight operations. External
 * clients (Python demo scripts, ground stations, mission planners) interact with the
 * peregrine stack exclusively through this node's action interfaces:
 *   - takeoff:  arm -> offboard -> execute takeoff trajectory -> hover
 *   - land:     switch PX4 to land mode -> wait for auto-disarm
 *   - go_to:    forward to trajectory_manager -> wait for completion
 *   - execute_trajectory: forward to trajectory_manager -> wait for completion
 *
 * Internally, the node is composed of several subsystems:
 *   - SupervisorStateMachine: table-driven FSM tracking high-level flight phase
 *   - HealthAggregator: freshness-based readiness monitoring of all dependencies
 *   - TransitionGuard: policy checks evaluated before each FSM transition
 *   - ActionOrchestrator: preemption-aware wait/step execution utilities
 *
 * Threading model (requires MultiThreadedExecutor / component_container_mt):
 *   - Default group (MutuallyExclusive): subscriptions + status timer (fast, non-blocking)
 *   - actionCbGroup_ (Reentrant): action server accepted callbacks (may block for minutes)
 *   - serviceCbGroup_ (MutuallyExclusive): service/action client responses (prevents deadlock
 *     when a service response arrives while an accepted callback is blocked)
 *
 * Single-flight policy: only one high-level action runs at a time. The action slot mutex
 * prevents concurrent takeoff + land, or two go_to goals running simultaneously.
 */

#pragma once

#include <uav_manager/action_orchestrator.hpp>
#include <uav_manager/health_aggregator.hpp>
#include <uav_manager/supervisor_state_machine.hpp>
#include <uav_manager/transition_guard.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <peregrine_interfaces/action/execute_trajectory.hpp>
#include <peregrine_interfaces/action/go_to.hpp>
#include <peregrine_interfaces/action/land.hpp>
#include <peregrine_interfaces/action/takeoff.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/px4_status.hpp>
#include <peregrine_interfaces/msg/safety_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>
#include <peregrine_interfaces/srv/arm.hpp>
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

/**
 * @class UavManagerNode
 * @brief Coordinates arm/offboard/trajectory/land operations via service and action APIs.
 */
class UavManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  // `using` type aliases are the C++ equivalent of Python's type aliases:
  //   Takeoff = peregrine_interfaces.action.Takeoff
  // They shorten deeply nested template types that appear throughout action
  // server/client code. Without these, signatures like
  // rclcpp_action::ServerGoalHandle<peregrine_interfaces::action::Takeoff> would
  // repeat in every goal/cancel/accepted callback declaration.
  using Takeoff = peregrine_interfaces::action::Takeoff;
  using Land = peregrine_interfaces::action::Land;
  using GoTo = peregrine_interfaces::action::GoTo;
  using ExecuteTrajectory = peregrine_interfaces::action::ExecuteTrajectory;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;
  using GoalHandleGoTo = rclcpp_action::ServerGoalHandle<GoTo>;
  using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

  /**
   * @brief Constructs the UAV manager lifecycle component.
   */
  explicit UavManagerNode(const rclcpp::NodeOptions & options);

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Lifecycle state machine callbacks. The managed lifecycle transitions are:
  //   UNCONFIGURED --on_configure--> INACTIVE --on_activate--> ACTIVE
  //   ACTIVE --on_deactivate--> INACTIVE --on_cleanup--> UNCONFIGURED
  // on_shutdown can be called from any primary state; on_error handles faults.
  // Each callback returns SUCCESS to proceed or FAILURE to stay/transition to error.

  /**
   * @brief Allocates ROS interfaces and resets supervisor/runtime state.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
    * @brief Activates status publication and exposes action servers.
    *
    * This callback intentionally does NOT fail activation based on dependency readiness.
    * Readiness is enforced at goal execution time via explicit checks and reason codes
    * rather than making lifecycle activation timing-sensitive.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Stops status publication while keeping subscriptions alive.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Releases all ROS resources and clears cached health/state.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reuses cleanup path during final shutdown.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Best-effort error callback that stops periodic status publishing.
   */
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  /// Caches latest estimated_state and updates readiness freshness.
  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);
  /// Caches PX4 status, updates readiness snapshot, and injects failsafe event.
  void onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg);
  /// Caches estimation manager status and updates readiness snapshot.
  void onEstimationStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  /// Caches control manager status and updates readiness snapshot.
  void onControlStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  /// Caches trajectory manager status and updates readiness snapshot.
  void onTrajectoryStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  /// Caches safety status, updates readiness, and injects emergency event if needed.
  void onSafetyStatus(const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg);

  /// Publishes externally-consumed supervisor state/status.
  void publishUavState();

  /// Validates and accepts/rejects incoming takeoff goals.
  rclcpp_action::GoalResponse onTakeoffGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Takeoff::Goal> goal);
  /// Accepts cancel requests for active takeoff goals.
  rclcpp_action::CancelResponse onTakeoffCancel(
    const std::shared_ptr<GoalHandleTakeoff> goalHandle);
  /// Executes the full takeoff orchestration flow.
  void onTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goalHandle);

  /// Validates and accepts/rejects incoming land goals.
  rclcpp_action::GoalResponse onLandGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal);
  /// Accepts cancel requests for active land goals.
  rclcpp_action::CancelResponse onLandCancel(const std::shared_ptr<GoalHandleLand> goalHandle);
  /// Executes the full landing orchestration flow.
  void onLandAccepted(const std::shared_ptr<GoalHandleLand> goalHandle);

  /// Validates and accepts/rejects incoming go-to goals.
  rclcpp_action::GoalResponse onGoToGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoTo::Goal> goal);
  /// Accepts cancel requests for active go-to goals.
  rclcpp_action::CancelResponse onGoToCancel(const std::shared_ptr<GoalHandleGoTo> goalHandle);
  /// Forwards go-to execution to trajectory_manager with preemption control.
  void onGoToAccepted(const std::shared_ptr<GoalHandleGoTo> goalHandle);

  /// Validates and accepts/rejects execute-trajectory goals.
  rclcpp_action::GoalResponse onExecuteGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteTrajectory::Goal> goal);
  /// Accepts cancel requests for active execute-trajectory goals.
  rclcpp_action::CancelResponse onExecuteCancel(
    const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle);
  /// Forwards execute-trajectory to trajectory_manager with preemption control.
  void onExecuteAccepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle);

  /// Single-flight guard: only one high-level action may run at a time.
  bool reserveActionSlot();
  /// Releases single-flight action guard.
  void releaseActionSlot();

  /**
   * @brief Applies one explicit supervisor event with guard evaluation.
   *
   * Transition result is logged and `lastTransitionReason_` is updated.
   */
  TransitionOutcome applyEvent(SupervisorEvent event);
  /// Returns true when supervisor is in Emergency state.
  bool isEmergency() const;

  /// Creates (advertises) uav_manager action servers.
  bool createActionServers();
  /// Destroys uav_manager action servers (removes them from the ROS graph).
  void destroyActionServers();

  /// Ensures PX4 nav state is armable before arming.
  StepResult ensureArmableMode();
  /// Calls `arm` service with explicit timeout/error mapping.
  StepResult callArmService(bool arm);
  /// Calls `set_mode` service with explicit timeout/error mapping.
  StepResult callSetModeService(const std::string & mode);

  /// Waits until PX4 armed state matches expected value.
  StepResult waitForArmed(bool armed, std::chrono::milliseconds timeout) const;
  /// Waits until PX4 offboard state matches expected value.
  StepResult waitForOffboard(bool offboard, std::chrono::milliseconds timeout) const;
  /// Waits until PX4 nav_state matches expected value.
  StepResult waitForNavState(uint8_t navState, std::chrono::milliseconds timeout) const;
  /// Waits for healthy control output flow before requesting offboard.
  StepResult waitForControlSetpointFlow(std::chrono::milliseconds timeout) const;

  /**
   * @brief Forwards ExecuteTrajectory to trajectory_manager with bounded waits.
   *
   * All waits are preemption-aware and return machine-readable reason codes.
   */
  StepResult forwardExecuteTrajectory(
    const ExecuteTrajectory::Goal & goal,
    std::function<void(const ExecuteTrajectory::Feedback &)> feedbackCallback,
    const std::function<bool()> & preempted,
    const std::function<bool()> & emergency,
    ExecuteTrajectory::Result * resultOut) const;

  /**
   * @brief Forwards GoTo to trajectory_manager with bounded waits.
   *
   * All waits are preemption-aware and return machine-readable reason codes.
   */
  StepResult forwardGoTo(
    const GoTo::Goal & goal, std::function<void(const GoTo::Feedback &)> feedbackCallback,
    const std::function<bool()> & preempted,
    const std::function<bool()> & emergency,
    GoTo::Result * resultOut) const;

  /// Returns latest altitude from estimated_state, or 0.0 when unavailable.
  double latestAltitudeM() const;
  /// Returns latest position from estimated_state, or origin when unavailable.
  geometry_msgs::msg::Point latestPosition() const;

  /// Utility to convert frequency in Hz into timer period.
  static std::chrono::nanoseconds periodFromHz(double hz);
  /// Maps supervisor state enum to UAVState numeric code.
  static uint8_t toUavStateCode(SupervisorState state);

  /// Steady-clock "now" used for freshness tracking.
  std::chrono::steady_clock::time_point nowSteady() const;

  /// Protects cached state/status and supervisor updates.
  mutable std::mutex mutex_;
  /// Latest estimated_state message.
  std::optional<peregrine_interfaces::msg::State> latestState_;
  /// Latest PX4 bridge status message.
  std::optional<peregrine_interfaces::msg::PX4Status> latestPx4Status_;
  /// Latest estimation manager status message.
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestEstimationStatus_;
  /// Latest control manager status message.
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestControlStatus_;
  /// Latest trajectory manager status message.
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestTrajectoryStatus_;

  /// Pure table-driven supervisor FSM.
  SupervisorStateMachine supervisor_;
  /// Last transition reason exported in UAVState.detail.
  std::string lastTransitionReason_{"BOOT"};

  /// Protects the single-flight action slot.
  mutable std::mutex actionSlotMutex_;
  /// True when any high-level action is currently executing.
  bool actionSlotReserved_{false};

  /// Freshness thresholds for dependency/readiness evaluation.
  FreshnessConfig freshnessConfig_;
  /// Aggregates subsystem health into one snapshot for guard checks.
  std::unique_ptr<HealthAggregator> healthAggregator_;
  /// Deterministic guard evaluator for FSM transitions.
  TransitionGuard transitionGuard_;
  /// Orchestrates wait/timeout/preemption handling for action steps.
  std::unique_ptr<ActionOrchestrator> orchestrator_;

  /// True once lifecycle configure succeeded.
  bool configured_{false};
  // `std::atomic<bool>` is a thread-safe boolean that can be read/written from
  // multiple threads without a mutex. Normal booleans are NOT safe to share between
  // threads in C++ (unlike Python, where the GIL serializes access). std::atomic
  // provides lock-free, hardware-level synchronization for simple types. This is
  // used for `active_` because it is read from many different callback threads
  // (subscription callbacks, timer callbacks, action callbacks) and written
  // during lifecycle transitions.
  /// True while lifecycle state is active.
  std::atomic<bool> active_{false};

  /// UAV state publication frequency.
  double statusRateHz_{10.0};
  /// Activation-time wait bound for dependency readiness.
  double dependencyStartupTimeoutS_{2.0};
  /// Service availability wait timeout.
  double serviceWaitS_{3.0};
  /// Service response wait timeout.
  double serviceResponseWaitS_{5.0};
  /// Downstream action server availability timeout.
  double actionServerWaitS_{3.0};
  /// Downstream action result wait timeout.
  double actionResultWaitS_{180.0};
  /// PX4 offboard transition timeout.
  double offboardWaitS_{6.0};
  /// PX4 armed confirmation timeout.
  double armedWaitS_{6.0};

  /// Reentrant group for long-running action server callbacks.
  rclcpp::CallbackGroup::SharedPtr actionCbGroup_;
  /// Separate group for service/action clients and their responses.
  rclcpp::CallbackGroup::SharedPtr serviceCbGroup_;

  /// Estimated state subscription.
  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  /// PX4 bridge status subscription.
  rclcpp::Subscription<peregrine_interfaces::msg::PX4Status>::SharedPtr px4StatusSub_;
  /// Estimation manager status subscription.
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr estimationStatusSub_;
  /// Control manager status subscription.
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr controlStatusSub_;
  /// Trajectory manager status subscription.
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr trajectoryStatusSub_;
  /// Safety monitor status subscription.
  rclcpp::Subscription<peregrine_interfaces::msg::SafetyStatus>::SharedPtr safetyStatusSub_;

  /// Lifecycle-gated UAV supervisor status publisher.
  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::UAVState>::SharedPtr uavStatePub_;

  /// Client for arm/disarm requests.
  rclcpp::Client<peregrine_interfaces::srv::Arm>::SharedPtr armClient_;
  /// Client for PX4 mode changes.
  rclcpp::Client<peregrine_interfaces::srv::SetMode>::SharedPtr setModeClient_;
  /// Client for go-to forwarding into trajectory_manager.
  rclcpp_action::Client<GoTo>::SharedPtr trajectoryGoToClient_;
  /// Client for execute-trajectory forwarding into trajectory_manager.
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr trajectoryExecuteClient_;

  /// High-level takeoff action server exposed by uav_manager.
  rclcpp_action::Server<Takeoff>::SharedPtr takeoffServer_;
  /// High-level land action server exposed by uav_manager.
  rclcpp_action::Server<Land>::SharedPtr landServer_;
  /// High-level go-to action server exposed by uav_manager.
  rclcpp_action::Server<GoTo>::SharedPtr goToServer_;
  /// High-level execute-trajectory action server exposed by uav_manager.
  rclcpp_action::Server<ExecuteTrajectory>::SharedPtr executeServer_;

  /// Periodic UAVState publisher timer.
  rclcpp::TimerBase::SharedPtr statusTimer_;

  /// Whether safety monitor must be healthy for dependenciesReady().
  bool requireExternalSafety_{false};

  /// When true, node self-transitions through configure -> activate on startup.
  bool autoStart_{true};
  /// Timeout for data readiness polling between configure and activate.
  double dataReadinessTimeoutS_{30.0};
  /// Poll interval for data readiness check.
  int dataReadinessPollMs_{200};
  /// One-shot timer that drives the auto-start sequence (phase 1: configure).
  rclcpp::TimerBase::SharedPtr startupTimer_;
  /// Recurring timer that polls data readiness before activating (phase 2).
  rclcpp::TimerBase::SharedPtr readinessTimer_;
  /// Deadline for data readiness polling before activating anyway.
  std::chrono::steady_clock::time_point readinessDeadline_;
};

}  // namespace uav_manager
