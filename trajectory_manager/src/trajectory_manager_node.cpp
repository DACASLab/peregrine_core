#include <trajectory_manager/trajectory_manager_node.hpp>

#include <trajectory_manager/coverage_planner.hpp>
#include <trajectory_manager/generators.hpp>
#include <trajectory_manager/path_smoothing.hpp>

#include <frame_transforms/conversions.hpp>
#include <frame_transforms/viz_colormap.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace trajectory_manager
{
using namespace std::chrono_literals;

namespace
{

// File-local constant with internal linkage (unnamed namespace).
constexpr char kManagerName[] = "trajectory_manager";

// Parses the trailing integer of a UAV namespace ("/uav3" -> 3) for deterministic viz
// coloring. Returns 0 when no digits are present (single-UAV / root namespace).
int uavIndexFromNamespace(const std::string & ns)
{
  std::string digits;
  for (auto it = ns.rbegin(); it != ns.rend(); ++it) {
    if (std::isdigit(static_cast<unsigned char>(*it))) {
      digits.insert(digits.begin(), *it);
    } else if (!digits.empty()) {
      break;
    }
  }
  return digits.empty() ? 0 : std::stoi(digits);
}

std_msgs::msg::ColorRGBA makeColorRgba(const std::array<float, 3> & rgb, float alpha)
{
  std_msgs::msg::ColorRGBA color;
  color.r = rgb[0];
  color.g = rgb[1];
  color.b = rgb[2];
  color.a = alpha;
  return color;
}

}  // namespace

TrajectoryManagerNode::TrajectoryManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kManagerName, options)
{
  paramListener_ = std::make_shared<trajectory_manager::ParamListener>(
      get_node_parameters_interface());
  params_ = paramListener_->get_params();

  uavColorIndex_ = uavIndexFromNamespace(this->get_namespace());

  // Latched (transient_local) viz outputs so a late-joining RViz in the GCS immediately
  // receives the current coverage plan. Depth 1: only the latest plan matters.
  const auto latchedQos = rclcpp::QoS(1).reliable().transient_local();
  coverageGridPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "viz/coverage_grid", latchedQos);
  plannedPathPub_ = this->create_publisher<nav_msgs::msg::Path>("viz/planned_path", latchedQos);
  coverageSwathPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "viz/coverage_swath", latchedQos);

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
  actionCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

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

  rcl_action_server_options_t actionOpts = rcl_action_server_get_default_options();

  goToServer_ = rclcpp_action::create_server<GoTo>(
    this, "~/go_to",
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoTo::Goal> goal) {
      return onGoToGoal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleGoTo> goalHandle) { return onGoToCancel(goalHandle); },
    [this](const std::shared_ptr<GoalHandleGoTo> goalHandle) { onGoToAccepted(goalHandle); },
    actionOpts, actionCbGroup_);

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
    },
    actionOpts, actionCbGroup_);

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
  actionCbGroup_.reset();

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

  // Edge-triggered health-transition logging. This drives the TUI "trajectory BAD" badge, so a
  // logged transition here is the authoritative record of when/why the trajectory pipeline lost
  // its estimated_state input during a flight.
  if (status.message != prevHealthMessage_) {
    if (status.healthy) {
      RCLCPP_INFO(get_logger(), "trajectory health -> OK (was: %s)",
                  prevHealthMessage_.empty() ? "init" : prevHealthMessage_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "trajectory health -> UNHEALTHY: %s", status.message.c_str());
    }
    prevHealthMessage_ = status.message;
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
// generator on its next tick and begins sampling it. The action callback group is
// Reentrant so action goal/cancel/accepted callbacks are not serialized with the
// default subscription and timer callbacks; mutex_ still serializes shared state.
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
  //   coverage_sweep:
  //             [grid_origin_lat, grid_origin_lon, cell_size, n_cols, n_rows,
  //              altitude, velocity, footprint_w, N, cell_0, ..., cell_N-1]
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

  if (goal.trajectory_type == "coverage_sweep") {
    if (goal.params.size() < 10U) {
      return nullptr;
    }

    const double origin_lat = goal.params[0];
    const double origin_lon = goal.params[1];
    const double cell_size = goal.params[2];
    const int n_cols = static_cast<int>(goal.params[3]);
    const int n_rows = static_cast<int>(goal.params[4]);
    const double altitude = goal.params[5];
    const double velocity = goal.params[6];
    const double footprint_w = goal.params[7];
    const int n_cells = static_cast<int>(goal.params[8]);

    if (goal.params.size() != static_cast<size_t>(9 + n_cells)) {
      return nullptr;
    }

    std::vector<int> cell_seq;
    cell_seq.reserve(n_cells);
    for (int i = 0; i < n_cells; ++i) {
      cell_seq.push_back(static_cast<int>(goal.params[9 + i]));
    }

    // Convert grid origin from GPS to local ENU.
    const double home_lat = params_.home_lat_deg;
    const double home_lon = params_.home_lon_deg;
    auto enu_offset = frame_transforms::geodeticToEnu(
        home_lat, home_lon, 0.0, origin_lat, origin_lon, 0.0);
    const double origin_e = enu_offset.x();
    const double origin_n = enu_offset.y();

    auto grid = buildGrid(origin_e, origin_n, cell_size, n_cols, n_rows);
    auto boundary_pts = buildBoundaryPoints(grid.cell_polys, 3);

    Point start_pos(state.pose.pose.position.x, state.pose.pose.position.y);

    const std::vector<double> thetas = {0.0, M_PI / 2.0, M_PI / 4.0, 3.0 * M_PI / 4.0};
    auto plan_result = planSequence(
        grid.cells, cell_seq, footprint_w, thetas, boundary_pts, start_pos);

    if (plan_result.full_path.empty()) {
      return nullptr;
    }

    // Build cell bounds map for shrinking.
    std::map<int, std::array<double, 4>> cell_bounds;
    for (const auto & [id, poly] : grid.cells) {
      double xmin = poly[0].x(), xmax = poly[0].x();
      double ymin = poly[0].y(), ymax = poly[0].y();
      for (const auto & p : poly) {
        xmin = std::min(xmin, p.x());
        xmax = std::max(xmax, p.x());
        ymin = std::min(ymin, p.y());
        ymax = std::max(ymax, p.y());
      }
      cell_bounds[id] = {xmin, xmax, ymin, ymax};
    }

    // Shrink per-cell sub-paths inside cell bounds, then concatenate.
    SmoothingParams smooth_params;
    std::vector<Eigen::Vector2d> concatenated;

    for (size_t c = 0; c < plan_result.cell_point_ranges.size(); ++c) {
      auto [start_idx, end_idx] = plan_result.cell_point_ranges[c];
      int cid = plan_result.cell_ids[c];

      // Add transit points (from end of previous cell to start of this cell).
      if (c == 0 && start_idx > 0) {
        for (size_t i = 0; i < start_idx; ++i) {
          concatenated.push_back(plan_result.full_path[i]);
        }
      }

      // Extract cell sub-path and shrink.
      std::vector<Eigen::Vector2d> cell_pts(
          plan_result.full_path.begin() + start_idx,
          plan_result.full_path.begin() + end_idx);

      auto shrunk = shrinkPathInsideCell(cell_pts, cell_bounds[cid], smooth_params.inside_margin);
      for (const auto & p : shrunk) {
        if (concatenated.empty() || (p - concatenated.back()).norm() > 1e-9) {
          concatenated.push_back(p);
        }
      }

      // Add transit to next cell.
      if (c + 1 < plan_result.cell_point_ranges.size()) {
        size_t next_start = plan_result.cell_point_ranges[c + 1].first;
        for (size_t i = end_idx; i < next_start; ++i) {
          if (concatenated.empty() ||
              (plan_result.full_path[i] - concatenated.back()).norm() > 1e-9)
          {
            concatenated.push_back(plan_result.full_path[i]);
          }
        }
      }
    }

    if (concatenated.size() < 2) {
      return nullptr;
    }

    // Smooth the full path (only modifies corners).
    auto smooth_path = smoothPolylineWithQuinticFillets(concatenated, smooth_params);

    if (smooth_path.size() < 2) {
      return nullptr;
    }

    auto yaw = computeForwardYaw(smooth_path);

    // Publish RViz overlays for this plan. odomFrame_ is the frame the setpoints are stamped
    // with (world-anchored), so the grid/path/swath overlay exactly on the flown trail.
    publishCoverageViz(
      grid.cells, concatenated, smooth_path, altitude, footprint_w, odomFrame_);

    RCLCPP_INFO(get_logger(),
        "coverage_sweep: %zu cells, %zu C0 pts -> %zu smooth pts, path_len=%.1fm",
        cell_seq.size(), concatenated.size(), smooth_path.size(),
        [&]() {
          double len = 0.0;
          for (size_t i = 1; i < smooth_path.size(); ++i) {
            len += (smooth_path[i] - smooth_path[i - 1]).norm();
          }
          return len;
        }());

    return std::make_unique<WaypointTrajectoryGenerator>(
        smooth_path, yaw, altitude, velocity, startTime);
  }

  return nullptr;
}

void TrajectoryManagerNode::publishCoverageViz(
  const std::map<int, std::vector<Eigen::Vector2d>> & cells,
  const std::vector<Eigen::Vector2d> & reference_path,
  const std::vector<Eigen::Vector2d> & smooth_path,
  double altitude, double footprint_w, const std::string & frame_id) const
{
  const auto now = this->now();
  const auto rgb = frame_transforms::viz::colorForUav(uavColorIndex_);

  auto point_at = [](double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  };

  // ---- Grid: the shared ground survey region. Drawn at z=0 in neutral colors and labeling
  // ALL cells, so every UAV publishes an identical grid that coincides visually (instead of
  // stacking per-UAV colored copies). Outlines/labels are opaque so overlapping duplicates
  // from N UAVs look like a single grid. Per-UAV coverage is conveyed by the swath/paths.
  constexpr double kGridZ = 0.0;
  visualization_msgs::msg::MarkerArray grid_markers;

  visualization_msgs::msg::Marker outline;
  outline.header.frame_id = frame_id;
  outline.header.stamp = now;
  outline.ns = "coverage_grid";
  outline.id = 0;
  outline.type = visualization_msgs::msg::Marker::LINE_LIST;
  outline.action = visualization_msgs::msg::Marker::ADD;
  outline.scale.x = 0.15;  // thicker cell edges
  outline.color = makeColorRgba({0.85F, 0.85F, 0.90F}, 1.0F);
  outline.pose.orientation.w = 1.0;
  for (const auto & [id, poly] : cells) {
    (void)id;
    for (std::size_t i = 0; i < poly.size(); ++i) {
      const auto & a = poly[i];
      const auto & b = poly[(i + 1) % poly.size()];
      outline.points.push_back(point_at(a.x(), a.y(), kGridZ));
      outline.points.push_back(point_at(b.x(), b.y(), kGridZ));
    }
  }
  grid_markers.markers.push_back(std::move(outline));

  int marker_id = 1;
  for (const auto & [id, poly] : cells) {
    if (poly.empty()) {
      continue;
    }
    double xmin = poly[0].x();
    double xmax = poly[0].x();
    double ymin = poly[0].y();
    double ymax = poly[0].y();
    for (const auto & p : poly) {
      xmin = std::min(xmin, p.x());
      xmax = std::max(xmax, p.x());
      ymin = std::min(ymin, p.y());
      ymax = std::max(ymax, p.y());
    }
    const double cx = 0.5 * (xmin + xmax);
    const double cy = 0.5 * (ymin + ymax);

    visualization_msgs::msg::Marker label;
    label.header.frame_id = frame_id;
    label.header.stamp = now;
    label.ns = "coverage_cell_ids";
    label.id = marker_id++;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position = point_at(cx, cy, kGridZ + 0.3);
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.6;
    label.color = makeColorRgba({0.95F, 0.95F, 0.95F}, 1.0F);
    label.text = std::to_string(id);
    grid_markers.markers.push_back(std::move(label));
  }
  coverageGridPub_->publish(grid_markers);

  // ---- Reference path: raw straight-line waypoints (pre-smoothing), at survey altitude.
  nav_msgs::msg::Path ref;
  ref.header.frame_id = frame_id;
  ref.header.stamp = now;
  ref.poses.reserve(reference_path.size());
  for (const auto & p : reference_path) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = ref.header;
    ps.pose.position = point_at(p.x(), p.y(), altitude);
    ps.pose.orientation.w = 1.0;
    ref.poses.push_back(std::move(ps));
  }
  plannedPathPub_->publish(ref);

  // ---- Swath: footprint_w-wide ribbon swept along the smoothed centerline.
  visualization_msgs::msg::MarkerArray swath_markers;
  visualization_msgs::msg::Marker ribbon;
  ribbon.header.frame_id = frame_id;
  ribbon.header.stamp = now;
  ribbon.ns = "coverage_swath";
  ribbon.id = 0;
  ribbon.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  ribbon.action = visualization_msgs::msg::Marker::ADD;
  ribbon.scale.x = 1.0;
  ribbon.scale.y = 1.0;
  ribbon.scale.z = 1.0;
  ribbon.pose.orientation.w = 1.0;
  ribbon.color = makeColorRgba(rgb, 0.12F);
  const double half = 0.5 * footprint_w;
  const double swath_z = altitude - 0.03;
  for (std::size_t i = 0; i + 1 < smooth_path.size(); ++i) {
    Eigen::Vector2d a = smooth_path[i];
    Eigen::Vector2d b = smooth_path[i + 1];
    Eigen::Vector2d d = b - a;
    const double n = d.norm();
    if (n < 1e-6) {
      continue;
    }
    d /= n;
    const Eigen::Vector2d perp(-d.y(), d.x());
    const Eigen::Vector2d al = a + perp * half;
    const Eigen::Vector2d ar = a - perp * half;
    const Eigen::Vector2d bl = b + perp * half;
    const Eigen::Vector2d br = b - perp * half;
    ribbon.points.push_back(point_at(al.x(), al.y(), swath_z));
    ribbon.points.push_back(point_at(ar.x(), ar.y(), swath_z));
    ribbon.points.push_back(point_at(bl.x(), bl.y(), swath_z));
    ribbon.points.push_back(point_at(ar.x(), ar.y(), swath_z));
    ribbon.points.push_back(point_at(br.x(), br.y(), swath_z));
    ribbon.points.push_back(point_at(bl.x(), bl.y(), swath_z));
  }
  if (!ribbon.points.empty()) {
    swath_markers.markers.push_back(std::move(ribbon));
  }
  coverageSwathPub_->publish(swath_markers);
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
