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
#include <trajectory_manager/trajectory_manager_node.hpp>

#include <trajectory_manager/generators.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace trajectory_manager
{
namespace
{

// File-local constant with internal linkage (unnamed namespace).
constexpr char kManagerName[] = "trajectory_manager";

}  // namespace

TrajectoryManagerNode::TrajectoryManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kManagerName, options)
{
  publishRateHz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 10.0);
  stateTimeoutS_ = this->declare_parameter<double>("state_timeout_s", 0.5);
  dependencyStartupTimeoutS_ = this->declare_parameter<double>("dependency_startup_timeout_s", 2.0);
  autoStart_ = this->declare_parameter<bool>("auto_start", true);

  if (autoStart_) {
    startupTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() {
        startupTimer_->cancel();  // one-shot

        RCLCPP_INFO(get_logger(), "Auto-start: triggering configure");
        auto configResult = this->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        if (configResult.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          RCLCPP_ERROR(get_logger(), "Auto-configure failed (state=%s)", configResult.label().c_str());
          return;
        }

        RCLCPP_INFO(get_logger(), "Auto-start: triggering activate");
        auto activateResult = this->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        if (activateResult.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          RCLCPP_ERROR(get_logger(), "Auto-activate failed (state=%s)", activateResult.label().c_str());
          return;
        }

        RCLCPP_INFO(get_logger(), "Auto-start complete: ACTIVE");
      });
  }
}

TrajectoryManagerNode::CallbackReturn TrajectoryManagerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (publishRateHz_ <= 0.0 || statusRateHz_ <= 0.0 || stateTimeoutS_ <= 0.0 ||
    dependencyStartupTimeoutS_ <= 0.0)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "publish_rate_hz, status_rate_hz, state_timeout_s, and dependency_startup_timeout_s must be > 0");
    return CallbackReturn::FAILURE;
  }

  const auto startupDeadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(
    dependencyStartupTimeoutS_);
  // Deterministic startup gate: trajectory generation requires estimated_state input.
  while (std::chrono::steady_clock::now() < startupDeadline) {
    if (!this->get_publishers_info_by_topic("estimated_state").empty()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (this->get_publishers_info_by_topic("estimated_state").empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Cannot configure trajectory_manager: upstream topic 'estimated_state' not available");
    return CallbackReturn::FAILURE;
  }

  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

  // Template note:
  // `create_subscription<peregrine_interfaces::msg::State>(...)` specializes the
  // subscription type at compile time. This gives zero runtime type dispatch overhead.
  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    "estimated_state", qos,
    std::bind(&TrajectoryManagerNode::onEstimatedState, this, std::placeholders::_1));
  trajectorySetpointPub_ = this->create_publisher<peregrine_interfaces::msg::TrajectorySetpoint>(
    "trajectory_setpoint", qos);
  statusPub_ = this->create_publisher<peregrine_interfaces::msg::ManagerStatus>(
    "trajectory_status",
    statusQos);

  // Action servers use the default MutuallyExclusive callback group. This is safe because
  // the accepted callbacks (onGoToAccepted, onExecuteAccepted) are non-blocking: they just
  // swap the active generator under lock and return immediately. All trajectory computation,
  // feedback emission, and goal completion happen in the publishTrajectorySetpoint timer
  // callback, so there is no need for a Reentrant callback group (unlike uav_manager, whose
  // accepted callbacks block for the duration of the goal).
  goToServer_ = rclcpp_action::create_server<GoTo>(
    this, "~/go_to",
    std::bind(
      &TrajectoryManagerNode::onGoToGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&TrajectoryManagerNode::onGoToCancel, this, std::placeholders::_1),
    std::bind(&TrajectoryManagerNode::onGoToAccepted, this, std::placeholders::_1));

  executeServer_ = rclcpp_action::create_server<ExecuteTrajectory>(
    this, "~/execute_trajectory",
    std::bind(
      &TrajectoryManagerNode::onExecuteGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&TrajectoryManagerNode::onExecuteCancel, this, std::placeholders::_1),
    std::bind(&TrajectoryManagerNode::onExecuteAccepted, this, std::placeholders::_1));

  // Create-then-cancel pattern: timers must exist before on_activate, but should not
  // fire until the node transitions to ACTIVE. on_activate calls reset() to re-arm them.
  // This is the same lifecycle gate used by other managers in the peregrine stack.
  publishTimer_ = this->create_wall_timer(
    periodFromHz(publishRateHz_),
    std::bind(&TrajectoryManagerNode::publishTrajectorySetpoint, this));
  statusTimer_ =
    this->create_wall_timer(
    periodFromHz(statusRateHz_),
    std::bind(&TrajectoryManagerNode::publishStatus, this));

  publishTimer_->cancel();
  statusTimer_->cancel();

  {
    std::scoped_lock lock(mutex_);
    latestState_.reset();
    holdGenerator_.reset();
    activeGenerator_.reset();
    activeGoalType_ = ActiveGoalType::None;
    activeGoToGoal_.reset();
    activeExecuteGoal_.reset();
    activeModuleName_ = "hold_position";
    lastStateTime_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  configured_ = true;
  active_ = false;
  RCLCPP_INFO(
    this->get_logger(), "Configured trajectory_manager: publish_rate_hz=%.1f status_rate_hz=%.1f", publishRateHz_,
    statusRateHz_);
  return CallbackReturn::SUCCESS;
}

TrajectoryManagerNode::CallbackReturn TrajectoryManagerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!configured_) {
    RCLCPP_ERROR(this->get_logger(), "Cannot activate before configure");
    return CallbackReturn::FAILURE;
  }

  trajectorySetpointPub_->on_activate();
  statusPub_->on_activate();
  publishTimer_->reset();
  statusTimer_->reset();
  active_ = true;

  RCLCPP_INFO(this->get_logger(), "Activated trajectory_manager");
  return CallbackReturn::SUCCESS;
}

TrajectoryManagerNode::CallbackReturn TrajectoryManagerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;

  if (publishTimer_) {
    publishTimer_->cancel();
  }
  if (statusTimer_) {
    statusTimer_->cancel();
  }

  if (trajectorySetpointPub_) {
    trajectorySetpointPub_->on_deactivate();
  }
  if (statusPub_) {
    statusPub_->on_deactivate();
  }

  std::shared_ptr<GoalHandleGoTo> goToAbort;
  std::shared_ptr<GoalHandleExecuteTrajectory> executeAbort;
  {
    std::scoped_lock lock(mutex_);
    goToAbort = activeGoToGoal_;
    executeAbort = activeExecuteGoal_;
    activeGenerator_.reset();
    activeGoalType_ = ActiveGoalType::None;
    activeGoToGoal_.reset();
    activeExecuteGoal_.reset();
  }

  // Lifecycle deactivation preempts in-flight goals with explicit reason code.
  if (goToAbort) {
    auto result = std::make_shared<GoTo::Result>();
    result->success = false;
    result->message = "LIFECYCLE_DEACTIVATED";
    goToAbort->abort(result);
  }
  if (executeAbort) {
    auto result = std::make_shared<ExecuteTrajectory::Result>();
    result->success = false;
    result->message = "LIFECYCLE_DEACTIVATED";
    executeAbort->abort(result);
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated trajectory_manager");
  return CallbackReturn::SUCCESS;
}

TrajectoryManagerNode::CallbackReturn TrajectoryManagerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  configured_ = false;

  publishTimer_.reset();
  statusTimer_.reset();
  estimatedStateSub_.reset();
  trajectorySetpointPub_.reset();
  statusPub_.reset();
  goToServer_.reset();
  executeServer_.reset();

  {
    std::scoped_lock lock(mutex_);
    latestState_.reset();
    holdGenerator_.reset();
    activeGenerator_.reset();
    activeGoalType_ = ActiveGoalType::None;
    activeGoToGoal_.reset();
    activeExecuteGoal_.reset();
    activeModuleName_ = "hold_position";
    lastStateTime_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  RCLCPP_INFO(this->get_logger(), "Cleaned up trajectory_manager");
  return CallbackReturn::SUCCESS;
}

TrajectoryManagerNode::CallbackReturn TrajectoryManagerNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  (void)on_cleanup(this->get_current_state());
  RCLCPP_INFO(this->get_logger(), "Shut down trajectory_manager");
  return CallbackReturn::SUCCESS;
}

TrajectoryManagerNode::CallbackReturn TrajectoryManagerNode::on_error(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (publishTimer_) {
    publishTimer_->cancel();
  }
  if (statusTimer_) {
    statusTimer_->cancel();
  }
  RCLCPP_ERROR(this->get_logger(), "Error in trajectory_manager lifecycle; timers canceled");
  return CallbackReturn::SUCCESS;
}

void TrajectoryManagerNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  if (!configured_) {
    return;
  }

  std::scoped_lock lock(mutex_);
  latestState_ = *msg;
  // Keep freshness clock robust even when upstream stamp is unset.
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    lastStateTime_ = this->now();
  } else {
    lastStateTime_ = rclcpp::Time(msg->header.stamp);
  }

  // Lazy initialization: the hold generator is created on the first state message rather
  // than in on_configure. This avoids the need for state data during configuration and
  // guarantees the hold position is always a physically valid position (the vehicle's
  // last known location) rather than a default or zero value.
  if (!holdGenerator_) {
    holdGenerator_ = std::make_unique<HoldPositionGenerator>(*msg);
    activeModuleName_ = holdGenerator_->name();
  }
}

// Central trajectory tick: ALL trajectory computation, feedback, and goal lifecycle
// management happens in this single timer callback -- not in the action accepted callbacks.
// This design means: (1) setpoint publication cadence is owned by the timer, not the action
// server, (2) goal completion detection and feedback emission are co-located with sampling,
// and (3) cancellation is processed synchronously with the next sample. Because everything
// runs in the default MutuallyExclusive callback group on one executor thread, no concurrent
// access to generator state is possible and the lock is only needed to coordinate with the
// subscription and action callbacks.
void TrajectoryManagerNode::publishTrajectorySetpoint()
{
  if (!active_ || !trajectorySetpointPub_ || !trajectorySetpointPub_->is_activated()) {
    return;
  }

  // Goal handle copies are staged here so that succeed()/canceled() can be called OUTSIDE
  // the lock. See the comment near the end of this function for the deadlock rationale.
  std::shared_ptr<GoalHandleGoTo> goToGoalToSucceed;
  std::shared_ptr<GoalHandleGoTo> goToGoalToCancel;
  std::shared_ptr<GoalHandleExecuteTrajectory> executeGoalToSucceed;
  std::shared_ptr<GoalHandleExecuteTrajectory> executeGoalToCancel;
  std::shared_ptr<GoTo::Result> goToResult;
  std::shared_ptr<ExecuteTrajectory::Result> executeResult;

  peregrine_interfaces::msg::TrajectorySetpoint setpoint;
  const auto now = this->now();

  {
    std::scoped_lock lock(mutex_);
    if (!latestState_.has_value()) {
      return;
    }

    // Lazily initialize hold generator once state is available.
    if (!holdGenerator_) {
      holdGenerator_ = std::make_unique<HoldPositionGenerator>(*latestState_);
    }

    TrajectorySample sample;
    if (activeGenerator_) {
      // Cancellation is handled here so state switch and result reason are serialized.
      if (activeGoalType_ == ActiveGoalType::GoTo && activeGoToGoal_ &&
        activeGoToGoal_->is_canceling())
      {
        goToGoalToCancel = activeGoToGoal_;
        goToResult = std::make_shared<GoTo::Result>();
        goToResult->success = false;
        goToResult->message = "GOAL_CANCELED";
        goToResult->final_position = latestState_->pose.pose.position;
        switchToHoldFromState(*latestState_);
      } else if (activeGoalType_ == ActiveGoalType::ExecuteTrajectory && activeExecuteGoal_ &&
        activeExecuteGoal_->is_canceling())
      {
        executeGoalToCancel = activeExecuteGoal_;
        executeResult = std::make_shared<ExecuteTrajectory::Result>();
        executeResult->success = false;
        executeResult->message = "GOAL_CANCELED";
        switchToHoldFromState(*latestState_);
      }
    }

    if (activeGenerator_) {
      // Active goal path: sample trajectory, emit feedback, and detect completion.
      sample = activeGenerator_->sample(*latestState_, now);
      setpoint = sample.setpoint;

      if (activeGoalType_ == ActiveGoalType::GoTo && activeGoToGoal_) {
        auto feedback = std::make_shared<GoTo::Feedback>();
        feedback->distance_remaining_m = sample.distanceRemaining;
        activeGoToGoal_->publish_feedback(feedback);
      } else if (activeGoalType_ == ActiveGoalType::ExecuteTrajectory && activeExecuteGoal_) {
        auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
        feedback->progress = sample.progress;
        activeExecuteGoal_->publish_feedback(feedback);
      }

      if (sample.completed) {
        // Latch terminal result first, then switch back to hold generator.
        if (activeGoalType_ == ActiveGoalType::GoTo && activeGoToGoal_) {
          goToGoalToSucceed = activeGoToGoal_;
          goToResult = std::make_shared<GoTo::Result>();
          goToResult->success = true;
          goToResult->message = "GOAL_REACHED";
          goToResult->final_position = sample.setpoint.position;
        } else if (activeGoalType_ == ActiveGoalType::ExecuteTrajectory && activeExecuteGoal_) {
          executeGoalToSucceed = activeExecuteGoal_;
          executeResult = std::make_shared<ExecuteTrajectory::Result>();
          executeResult->success = true;
          executeResult->message = "TRAJECTORY_COMPLETED";
        }

        const double holdYaw = sample.setpoint.yaw;
        // Freeze final setpoint as new hold reference for smooth post-goal behavior.
        holdGenerator_ = std::make_unique<HoldPositionGenerator>(sample.setpoint.position, holdYaw);
        activeGenerator_.reset();
        activeGoalType_ = ActiveGoalType::None;
        activeGoToGoal_.reset();
        activeExecuteGoal_.reset();
        activeModuleName_ = holdGenerator_->name();
      }
    } else {
      // Idle path: keep publishing hold setpoints.
      sample = holdGenerator_->sample(*latestState_, now);
      setpoint = sample.setpoint;
      activeModuleName_ = holdGenerator_->name();
    }
  }

  setpoint.header.stamp = now;
  // Timestamp normalized at publish time so downstream controller sees consistent cadence.
  trajectorySetpointPub_->publish(setpoint);

  // Goal results (succeed/canceled) are resolved OUTSIDE the lock. Calling
  // goalHandle->succeed() or goalHandle->canceled() may synchronously invoke the action
  // server's goal or cancel callbacks (onGoToGoal, onGoToCancel, etc.), which themselves
  // acquire mutex_. If we called succeed() while holding the lock, the re-entrant lock
  // attempt would deadlock (std::mutex is non-recursive).
  if (goToGoalToCancel && goToResult) {
    goToGoalToCancel->canceled(goToResult);
  }
  if (executeGoalToCancel && executeResult) {
    executeGoalToCancel->canceled(executeResult);
  }
  if (goToGoalToSucceed && goToResult) {
    goToGoalToSucceed->succeed(goToResult);
  }
  if (executeGoalToSucceed && executeResult) {
    executeGoalToSucceed->succeed(executeResult);
  }
}

void TrajectoryManagerNode::publishStatus()
{
  if (!configured_ || !statusPub_ || !statusPub_->is_activated()) {
    return;
  }

  peregrine_interfaces::msg::ManagerStatus status;
  status.header.stamp = this->now();
  status.manager_name = kManagerName;
  status.output_rate_hz = static_cast<float>(publishRateHz_);
  status.active = active_;

  {
    std::scoped_lock lock(mutex_);
    status.active_module = activeModuleName_;
    if (!active_) {
      status.healthy = false;
      status.message = "LIFECYCLE_INACTIVE";
    } else if (!latestState_.has_value()) {
      status.healthy = false;
      status.message = "WAITING_FOR_ESTIMATED_STATE";
    } else {
      // Freshness check guards against stale estimated_state input.
      const double ageS = (this->now() - lastStateTime_).seconds();
      status.healthy = ageS <= stateTimeoutS_;
      status.message = status.healthy ? "OK" : "ESTIMATED_STATE_STALE";
    }
  }

  statusPub_->publish(status);
}

rclcpp_action::GoalResponse TrajectoryManagerNode::onGoToGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  const std::shared_ptr<const GoTo::Goal> goal)
{
  // Lock is required because publishTrajectorySetpoint may concurrently modify
  // activeGoalType_ (e.g., when a trajectory completes and resets to None).
  std::scoped_lock lock(mutex_);
  // Single-goal policy: only one trajectory can execute at a time. activeGoalType_ != None
  // means another trajectory (GoTo or ExecuteTrajectory) is already running. New goals are
  // rejected rather than queued; the caller must wait or cancel the current goal first.
  if (!active_) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!latestState_.has_value() || activeGoalType_ != ActiveGoalType::None) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->acceptance_radius_m <= 0.0) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryManagerNode::onGoToCancel(
  const std::shared_ptr<GoalHandleGoTo> goalHandle)
{
  std::scoped_lock lock(mutex_);
  if (activeGoalType_ == ActiveGoalType::GoTo && activeGoToGoal_ == goalHandle) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

// This accepted callback is intentionally non-blocking: it swaps activeGenerator_ under
// lock and returns immediately. The timer (publishTrajectorySetpoint) picks up the new
// generator on its next tick and begins sampling it. This is why a MutuallyExclusive
// callback group is sufficient -- there is no long-running work to block the executor.
void TrajectoryManagerNode::onGoToAccepted(const std::shared_ptr<GoalHandleGoTo> goalHandle)
{
  std::scoped_lock lock(mutex_);
  if (!active_ || !latestState_.has_value()) {
    auto result = std::make_shared<GoTo::Result>();
    result->success = false;
    result->message = "MISSING_ESTIMATED_STATE";
    goalHandle->abort(result);
    return;
  }

  const auto & goal = *goalHandle->get_goal();
  // If caller does not provide yaw, preserve current heading for smooth takeover.
  const double targetYaw =
    std::isfinite(goal.target_yaw) ? goal.target_yaw : yawFromQuaternion(
    latestState_->pose.pose.orientation);
  const double velocity = (goal.velocity_mps > 0.0) ? goal.velocity_mps : 1.0;
  activeGenerator_ = std::make_unique<LinearGoToGenerator>(
    *latestState_, goal.target_position, targetYaw, velocity,
    goal.acceptance_radius_m, this->now());
  activeGoalType_ = ActiveGoalType::GoTo;
  activeGoToGoal_ = goalHandle;
  activeExecuteGoal_.reset();
  activeModuleName_ = activeGenerator_->name();
}

rclcpp_action::GoalResponse TrajectoryManagerNode::onExecuteGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  const std::shared_ptr<const ExecuteTrajectory::Goal> goal)
{
  std::scoped_lock lock(mutex_);
  // Single-goal policy: reject when another generator already owns execution.
  if (!active_) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!latestState_.has_value() || activeGoalType_ != ActiveGoalType::None) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto generator = createGeneratorForExecuteGoal(*goal, *latestState_, this->now());
  if (!generator) {
    // Validation in goal callback provides early reject for malformed params.
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryManagerNode::onExecuteCancel(
  const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle)
{
  std::scoped_lock lock(mutex_);
  if (activeGoalType_ == ActiveGoalType::ExecuteTrajectory && activeExecuteGoal_ == goalHandle) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void TrajectoryManagerNode::onExecuteAccepted(
  const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle)
{
  std::scoped_lock lock(mutex_);
  if (!active_ || !latestState_.has_value()) {
    auto result = std::make_shared<ExecuteTrajectory::Result>();
    result->success = false;
    result->message = "MISSING_ESTIMATED_STATE";
    goalHandle->abort(result);
    return;
  }

  auto generator =
    createGeneratorForExecuteGoal(*goalHandle->get_goal(), *latestState_, this->now());
  if (!generator) {
    auto result = std::make_shared<ExecuteTrajectory::Result>();
    result->success = false;
    result->message = "INVALID_GOAL";
    goalHandle->abort(result);
    return;
  }

  activeGenerator_ = std::move(generator);
  // Ownership transfer marks this goal as the sole setpoint producer.
  // `std::move` converts `generator` into an rvalue so unique_ptr ownership can be
  // transferred. After move, `generator` is empty (nullptr by convention).
  activeGoalType_ = ActiveGoalType::ExecuteTrajectory;
  activeExecuteGoal_ = goalHandle;
  activeGoToGoal_.reset();
  activeModuleName_ = activeGenerator_->name();
}

std::unique_ptr<TrajectoryGeneratorBase> TrajectoryManagerNode::createGeneratorForExecuteGoal(
  const ExecuteTrajectory::Goal & goal, const peregrine_interfaces::msg::State & state,
  const rclcpp::Time & startTime) const
{
  // Each trajectory type has a fixed parameter count as its API contract. The params array
  // is positional (not named) -- ordering conventions are documented per trajectory type.
  // Validation happens both in onExecuteGoal (for early rejection before acceptance) and
  // here in onExecuteAccepted (defensive, in case state changed between goal and accepted).
  //
  // Parameter conventions:
  //   hold:     no params
  //   takeoff:  [target_altitude_m, climb_velocity_mps]
  //   land:     [descent_velocity_mps]
  //   circle:   [radius_m, angular_velocity_radps, num_loops]
  //   figure8:  [radius_m, angular_velocity_radps, num_loops]
  if (goal.trajectory_type == "hold") {
    return std::make_unique<HoldPositionGenerator>(state);
  }

  if (goal.trajectory_type == "takeoff") {
    if (goal.params.size() != 2U) {
      return nullptr;
    }
    return std::make_unique<TakeoffGenerator>(state, goal.params[0], goal.params[1], startTime);
  }

  if (goal.trajectory_type == "land") {
    if (goal.params.size() != 1U) {
      return nullptr;
    }
    return std::make_unique<LandGenerator>(state, goal.params[0], startTime);
  }

  if (goal.trajectory_type == "circle") {
    if (goal.params.size() != 3U) {
      return nullptr;
    }
    return std::make_unique<CircleGenerator>(
      state, goal.params[0], goal.params[1], goal.params[2],
      startTime);
  }

  if (goal.trajectory_type == "figure8") {
    if (goal.params.size() != 3U) {
      return nullptr;
    }
    return std::make_unique<FigureEightGenerator>(
      state, goal.params[0], goal.params[1],
      goal.params[2], startTime);
  }

  // Returning nullptr is the C++ "no object" idiom for pointer-returning APIs.
  // This mirrors Python returning `None` for unsupported trajectory types.
  return nullptr;
}

void TrajectoryManagerNode::switchToHoldFromState(const peregrine_interfaces::msg::State & state)
{
  // Common reset path used after cancel/complete to return to stable hold behavior.
  holdGenerator_ = std::make_unique<HoldPositionGenerator>(state);
  activeGenerator_.reset();
  activeGoalType_ = ActiveGoalType::None;
  activeGoToGoal_.reset();
  activeExecuteGoal_.reset();
  activeModuleName_ = holdGenerator_->name();
}

std::chrono::nanoseconds TrajectoryManagerNode::periodFromHz(const double hz)
{
  const auto period = std::chrono::duration<double>(1.0 / hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(period);
}

}  // namespace trajectory_manager

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_manager::TrajectoryManagerNode)
