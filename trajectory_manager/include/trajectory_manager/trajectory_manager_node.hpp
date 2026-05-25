/**
 * @file trajectory_manager_node.hpp
 * @brief Trajectory manager ROS2 lifecycle component.
 *
 * Pipeline position: estimation_manager -> [trajectory_manager] -> control_manager
 *                    uav_manager ─(action goals)──^
 *
 * This node produces trajectory setpoints at a fixed rate (50 Hz by default) from an
 * active trajectory generator. It serves two ROS2 action interfaces:
 *   - go_to: point-to-point flight with acceptance radius
 *   - execute_trajectory: named trajectory types (takeoff, circle, figure8, land, hold)
 *
 * Design: "timer-driven, action-accepting"
 *   Unlike uav_manager (where accepted callbacks block for goal duration), this node's
 *   accepted callbacks are NON-BLOCKING. They swap the active generator under lock and
 *   return immediately. All trajectory sampling, feedback emission, and goal completion
 *   are driven by the periodic publishTrajectorySetpoint timer callback. This means:
 *   - Setpoint cadence is guaranteed by the timer, not by action server timing
 *   - A MutuallyExclusive callback group is sufficient (no Reentrant needed)
 *   - Goal lifecycle is resolved outside the mutex to avoid deadlock
 *
 * Single-goal policy: only one trajectory (GoTo or ExecuteTrajectory) runs at a time.
 * New goals are rejected while another is active. The caller must cancel or wait for
 * completion before sending a new goal.
 *
 * When no active goal is running, a HoldPositionGenerator continuously publishes the
 * last known position to keep the PX4 offboard setpoint stream alive. Without this,
 * PX4 would exit offboard mode after ~500ms of setpoint silence.
 */

#pragma once

#include <trajectory_manager/trajectory_generator_base.hpp>
#include <trajectory_manager/trajectory_manager_parameters.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <peregrine_interfaces/action/execute_trajectory.hpp>
#include <peregrine_interfaces/action/go_to.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/trajectory_setpoint.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace trajectory_manager
{

/**
 * @class TrajectoryManagerNode
 * @brief Publishes trajectory setpoints and serves trajectory actions.
 */
class TrajectoryManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using GoTo = peregrine_interfaces::action::GoTo;
  using ExecuteTrajectory = peregrine_interfaces::action::ExecuteTrajectory;
  using GoalHandleGoTo = rclcpp_action::ServerGoalHandle<GoTo>;
  using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

  /**
   * @brief Constructs the trajectory manager lifecycle component.
   */
  explicit TrajectoryManagerNode(const rclcpp::NodeOptions & options);

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Tracks which action type currently owns `activeGenerator_`.
   *
   * GoTo and ExecuteTrajectory share the same activeGenerator_ slot, but they have
   * different result message types. This enum lets publishTrajectorySetpoint know which
   * goal handle to populate when completing or canceling a trajectory.
   */
  enum class ActiveGoalType
  {
    None,
    GoTo,
    ExecuteTrajectory
  };

  /**
   * @brief Allocates ROS interfaces, action servers, and timer resources.
   *
   * Enforces startup ordering by waiting for upstream `estimated_state`
   * publishers before succeeding.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Enables lifecycle publishers and periodic setpoint/status timers.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Stops timers, deactivates publishers, and aborts active goals.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Releases subscriptions, publishers, servers, and local generators.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reuses cleanup path during final shutdown.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Best-effort error callback that halts periodic activity.
   */
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Stores latest estimated state and initializes hold mode when needed.
   */
  void stopPublishing();
  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);
  void onUavState(const peregrine_interfaces::msg::UAVState::SharedPtr msg);

  /**
   * @brief Publishes trajectory setpoint at fixed rate.
   */
  void publishTrajectorySetpoint();

  /**
   * @brief Publishes manager health/status diagnostics.
   */
  void publishStatus();

  /**
   * @brief Handles new GoTo goals.
   */
  rclcpp_action::GoalResponse onGoToGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoTo::Goal> goal);

  /**
   * @brief Handles GoTo cancel requests.
   */
  rclcpp_action::CancelResponse onGoToCancel(const std::shared_ptr<GoalHandleGoTo> goalHandle);

  /**
   * @brief Activates GoTo goal execution.
   */
  void onGoToAccepted(const std::shared_ptr<GoalHandleGoTo> goalHandle);

  /**
   * @brief Handles new ExecuteTrajectory goals.
   */
  rclcpp_action::GoalResponse onExecuteGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteTrajectory::Goal> goal);

  /**
   * @brief Handles ExecuteTrajectory cancel requests.
   */
  rclcpp_action::CancelResponse onExecuteCancel(
    const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle);

  /**
   * @brief Activates ExecuteTrajectory goal execution.
   */
  void onExecuteAccepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle);

  /**
   * @brief Creates the selected generator from ExecuteTrajectory goal.
   */
  std::unique_ptr<TrajectoryGeneratorBase> createGeneratorForExecuteGoal(
    const ExecuteTrajectory::Goal & goal, const peregrine_interfaces::msg::State & state,
    const rclcpp::Time & startTime) const;

  /**
   * @brief Replaces active generator with a hold generator from latest state.
   */
  void switchToHoldFromState(const peregrine_interfaces::msg::State & state);

  /**
   * @brief Converts a positive frequency in Hz to timer period.
   */
  static std::chrono::nanoseconds periodFromHz(double hz);

  // mutex_ protects ALL mutable state listed below it: latestState_, lastStateTime_,
  // holdGenerator_, activeGenerator_, activeGoalType_, the active goal handles, and
  // activeModuleName_. Any access to these fields from subscription callbacks, timer
  // callbacks, or action server callbacks must be performed under this lock. The lock is
  // intentionally non-recursive (std::mutex) so that accidental re-entrant acquisitions
  // produce a deadlock rather than silently succeeding -- this is why goal results are
  // resolved outside the lock in publishTrajectorySetpoint.
  std::mutex mutex_;
  /// Latest estimated state sample from estimation_manager.
  std::optional<peregrine_interfaces::msg::State> latestState_;
  /// Effective receive time used for freshness checks in status output.
  rclcpp::Time lastStateTime_{0, 0, RCL_ROS_TIME};

  /// Default generator used when no explicit trajectory goal is active.
  std::unique_ptr<TrajectoryGeneratorBase> holdGenerator_;
  /// Active trajectory generator owned by current goal.
  std::unique_ptr<TrajectoryGeneratorBase> activeGenerator_;
  /// Generator validated in onExecuteGoal(), consumed by onExecuteAccepted().
  std::unique_ptr<TrajectoryGeneratorBase> pendingGenerator_;
  /// Type of goal currently driving `activeGenerator_`.
  ActiveGoalType activeGoalType_{ActiveGoalType::None};
  /// Active GoTo goal handle, if any.
  std::shared_ptr<GoalHandleGoTo> activeGoToGoal_;
  /// Active ExecuteTrajectory goal handle, if any.
  std::shared_ptr<GoalHandleExecuteTrajectory> activeExecuteGoal_;
  /// ManagerStatus.active_module value for observability.
  std::string activeModuleName_{"hold_position"};
  /// Odom frame name captured from first estimated_state message.
  std::string odomFrame_{"odom"};

  std::shared_ptr<trajectory_manager::ParamListener> paramListener_;
  trajectory_manager::Params params_;

  /// True once configure has successfully allocated runtime resources.
  bool configured_{false};
  /// True while node is lifecycle-active.
  bool active_{false};

  /// Tracks the last supervisor state to detect LANDED/IDLE transitions.
  uint8_t lastUavState_{peregrine_interfaces::msg::UAVState::STATE_IDLE};

  /// Estimated state input from estimation_manager.
  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::UAVState>::SharedPtr uavStateSub_;
  /// Lifecycle-gated trajectory setpoint output for control_manager.
  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::TrajectorySetpoint>::SharedPtr
    trajectorySetpointPub_;
  /// Lifecycle-gated manager status output.
  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::ManagerStatus>::SharedPtr
    statusPub_;

  /// Action server for point-to-point goals.
  rclcpp_action::Server<GoTo>::SharedPtr goToServer_;
  /// Action server for named trajectories (takeoff/land/circle/figure8/...).
  rclcpp_action::Server<ExecuteTrajectory>::SharedPtr executeServer_;

  /// Periodic setpoint publication timer.
  rclcpp::TimerBase::SharedPtr publishTimer_;
  /// Periodic status publication timer.
  rclcpp::TimerBase::SharedPtr statusTimer_;

  /// One-shot timer that drives the auto-start sequence.
  rclcpp::TimerBase::SharedPtr startupTimer_;
};

}  // namespace trajectory_manager
