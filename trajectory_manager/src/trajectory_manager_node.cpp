#include <trajectory_manager/trajectory_manager_node.hpp>

#include <trajectory_manager/generators.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace trajectory_manager
{
using namespace std::chrono_literals;

namespace
{

// File-local constant with internal linkage (unnamed namespace).
constexpr char kManagerName[] = "trajectory_manager";

}  // namespace

TrajectoryManagerNode::TrajectoryManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kManagerName, options)
{
  paramListener_ = std::make_shared<trajectory_manager::ParamListener>(
      get_node_parameters_interface());
  params_ = paramListener_->get_params();

  if (params_.auto_start) {
    startupTimer_ = this->create_wall_timer(
      200ms,
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
  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

  // Template note:
  // `create_subscription<peregrine_interfaces::msg::State>(...)` specializes the
  // subscription type at compile time. This gives zero runtime type dispatch overhead.
  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    "estimated_state", qos,
    [this](peregrine_interfaces::msg::State::SharedPtr msg) { onEstimatedState(msg); });
  uavStateSub_ = this->create_subscription<peregrine_interfaces::msg::UAVState>(
    "uav_state", statusQos,
    [this](peregrine_interfaces::msg::UAVState::SharedPtr msg) { onUavState(msg); });
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
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoTo::Goal> goal) {
      return onGoToGoal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleGoTo> goalHandle) { return onGoToCancel(goalHandle); },
    [this](const std::shared_ptr<GoalHandleGoTo> goalHandle) { onGoToAccepted(goalHandle); });

  executeServer_ = rclcpp_action::create_server<ExecuteTrajectory>(
    this, "~/execute_trajectory",
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteTrajectory::Goal> goal) {
      return onExecuteGoal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle) {
      return onExecuteCancel(goalHandle);
    },
    [this](const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle) {
      onExecuteAccepted(goalHandle);
    });

  // Create-then-cancel pattern: timers must exist before on_activate, but should not
  // fire until the node transitions to ACTIVE. on_activate calls reset() to re-arm them.
  // This is the same lifecycle gate used by other managers in the peregrine stack.
  publishTimer_ = this->create_wall_timer(
    periodFromHz(params_.publish_rate_hz),
    [this]() { publishTrajectorySetpoint(); });
  statusTimer_ =
    this->create_wall_timer(
    periodFromHz(params_.status_rate_hz),
    [this]() { publishStatus(); });

  publishTimer_->cancel();
  statusTimer_->cancel();

  {
    std::scoped_lock lock(mutex_);
    latestState_.reset();
    geometry_msgs::msg::Point origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
    holdGenerator_ = std::make_unique<HoldPositionGenerator>(origin, 0.0);
    activeGenerator_.reset();
    pendingGenerator_.reset();
    activeGoalType_ = ActiveGoalType::None;
    activeGoToGoal_.reset();
    activeExecuteGoal_.reset();
    activeModuleName_ = holdGenerator_->name();
    lastStateTime_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  configured_ = true;
  active_ = false;
  RCLCPP_INFO(
    this->get_logger(), "Configured trajectory_manager: publish_rate_hz=%.1f status_rate_hz=%.1f", params_.publish_rate_hz,
    params_.status_rate_hz);
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
  stopPublishing();

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
  uavStateSub_.reset();
  trajectorySetpointPub_.reset();
  statusPub_.reset();
  goToServer_.reset();
  executeServer_.reset();

  {
    std::scoped_lock lock(mutex_);
    latestState_.reset();
    holdGenerator_.reset();
    activeGenerator_.reset();
    pendingGenerator_.reset();
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
  stopPublishing();
  RCLCPP_ERROR(this->get_logger(), "Error in trajectory_manager lifecycle; timers canceled");
  return CallbackReturn::SUCCESS;
}

void TrajectoryManagerNode::stopPublishing()
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

  // Capture odom frame name from upstream estimated_state.
  if (!msg->header.frame_id.empty()) {
    odomFrame_ = msg->header.frame_id;
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

void TrajectoryManagerNode::onUavState(const peregrine_interfaces::msg::UAVState::SharedPtr msg)
{
  if (!configured_) {
    return;
  }

  const uint8_t prev = lastUavState_;
  lastUavState_ = msg->state;

  const bool landedOrIdle =
    msg->state == peregrine_interfaces::msg::UAVState::STATE_LANDED ||
    msg->state == peregrine_interfaces::msg::UAVState::STATE_IDLE;
  const bool wasFlightState =
    prev != peregrine_interfaces::msg::UAVState::STATE_LANDED &&
    prev != peregrine_interfaces::msg::UAVState::STATE_IDLE;

  if (landedOrIdle && wasFlightState) {
    std::scoped_lock lock(mutex_);
    if (latestState_.has_value() && activeGoalType_ == ActiveGoalType::None) {
      holdGenerator_ = std::make_unique<HoldPositionGenerator>(*latestState_);
      activeModuleName_ = holdGenerator_->name();
      RCLCPP_INFO(get_logger(), "Reset hold to current position on supervisor %s",
        msg->state == peregrine_interfaces::msg::UAVState::STATE_LANDED ? "LANDED" : "IDLE");
    }
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

  // Phase 1: Sample under lock — copy state, run generator, detect cancel/completion.
  // Phase 2: Publish setpoint and feedback outside lock.
  // Phase 3: Resolve goal handles outside lock (avoids deadlock with action server callbacks).
  std::shared_ptr<GoalHandleGoTo> goToGoalToSucceed;
  std::shared_ptr<GoalHandleGoTo> goToGoalToCancel;
  std::shared_ptr<GoalHandleExecuteTrajectory> executeGoalToSucceed;
  std::shared_ptr<GoalHandleExecuteTrajectory> executeGoalToCancel;
  std::shared_ptr<GoTo::Result> goToResult;
  std::shared_ptr<ExecuteTrajectory::Result> executeResult;

  std::shared_ptr<GoalHandleGoTo> goToFeedbackHandle;
  std::shared_ptr<GoalHandleExecuteTrajectory> executeFeedbackHandle;
  TrajectorySample sample;

  peregrine_interfaces::msg::TrajectorySetpoint setpoint;
  const auto now = this->now();
  bool haveSample = false;

  {
    std::scoped_lock lock(mutex_);
    if (!latestState_.has_value()) {
      return;
    }

    if (!holdGenerator_) {
      holdGenerator_ = std::make_unique<HoldPositionGenerator>(*latestState_);
    }

    if (activeGenerator_) {
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
      sample = activeGenerator_->sample(now);
      setpoint = sample.setpoint;
      haveSample = true;

      if (activeGoalType_ == ActiveGoalType::GoTo && activeGoToGoal_) {
        goToFeedbackHandle = activeGoToGoal_;
      } else if (activeGoalType_ == ActiveGoalType::ExecuteTrajectory && activeExecuteGoal_) {
        executeFeedbackHandle = activeExecuteGoal_;
      }

      if (sample.progress >= 1.0F) {
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
        holdGenerator_ = std::make_unique<HoldPositionGenerator>(sample.setpoint.position, holdYaw);
        activeGenerator_.reset();
        activeGoalType_ = ActiveGoalType::None;
        activeGoToGoal_.reset();
        activeExecuteGoal_.reset();
        activeModuleName_ = holdGenerator_->name();
      }
    } else {
      sample = holdGenerator_->sample(now);
      setpoint = sample.setpoint;
      activeModuleName_ = holdGenerator_->name();
    }
  }

  // Phase 2: publish setpoint and feedback outside the lock.
  setpoint.header.stamp = now;
  setpoint.header.frame_id = odomFrame_;
  trajectorySetpointPub_->publish(setpoint);

  if (haveSample && goToFeedbackHandle) {
    auto feedback = std::make_shared<GoTo::Feedback>();
    feedback->progress = sample.progress;
    goToFeedbackHandle->publish_feedback(feedback);
  } else if (haveSample && executeFeedbackHandle) {
    auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
    feedback->progress = sample.progress;
    executeFeedbackHandle->publish_feedback(feedback);
  }

  // Phase 3: resolve goal handles outside the lock. Calling succeed()/canceled() may
  // synchronously invoke action server callbacks that acquire mutex_.
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
  status.output_rate_hz = static_cast<float>(params_.publish_rate_hz);
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
      status.healthy = ageS <= params_.state_timeout_s;
      status.message = status.healthy ? "OK" : "ESTIMATED_STATE_STALE";
    }
  }

  statusPub_->publish(status);
}

rclcpp_action::GoalResponse TrajectoryManagerNode::onGoToGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  const std::shared_ptr<const GoTo::Goal> /*goal*/)
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
    *latestState_, goal.target_position, targetYaw, velocity, this->now());
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
  if (!active_) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!latestState_.has_value() || activeGoalType_ != ActiveGoalType::None) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  auto generator = createGeneratorForExecuteGoal(*goal, *latestState_, this->now());
  if (!generator) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  pendingGenerator_ = std::move(generator);
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
  if (!active_ || !latestState_.has_value() || !pendingGenerator_) {
    auto result = std::make_shared<ExecuteTrajectory::Result>();
    result->success = false;
    result->message = pendingGenerator_ ? "MISSING_ESTIMATED_STATE" : "NO_PENDING_GENERATOR";
    pendingGenerator_.reset();
    goalHandle->abort(result);
    return;
  }

  activeGenerator_ = std::move(pendingGenerator_);
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
  //   step_response:
  //             [dx_m, dy_m, dz_m, dyaw_rad, pre_step_hold_s, post_step_hold_s]
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

  if (goal.trajectory_type == "step_response") {
    if (goal.params.size() != 6U) {
      return nullptr;
    }
    geometry_msgs::msg::Point stepOffset;
    stepOffset.x = goal.params[0];
    stepOffset.y = goal.params[1];
    stepOffset.z = goal.params[2];
    return std::make_unique<StepResponseGenerator>(
      state, stepOffset, goal.params[3], goal.params[4], goal.params[5], startTime);
  }

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
