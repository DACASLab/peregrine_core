#include <trajectory_manager/trajectory_manager_node.hpp>

#include <trajectory_manager/generators.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace trajectory_manager
{
namespace
{

constexpr char kManagerName[] = "trajectory_manager";

}  // namespace

TrajectoryManagerNode::TrajectoryManagerNode(const rclcpp::NodeOptions& options)
: Node(kManagerName, options)
{
  publishRateHz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 10.0);
  stateTimeoutS_ = this->declare_parameter<double>("state_timeout_s", 0.5);

  if (publishRateHz_ <= 0.0 || statusRateHz_ <= 0.0 || stateTimeoutS_ <= 0.0)
  {
    throw std::runtime_error("publish_rate_hz, status_rate_hz, and state_timeout_s must be > 0");
  }

  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
      "estimated_state", qos, std::bind(&TrajectoryManagerNode::onEstimatedState, this, std::placeholders::_1));
  trajectorySetpointPub_ =
      this->create_publisher<peregrine_interfaces::msg::TrajectorySetpoint>("trajectory_setpoint", qos);
  statusPub_ = this->create_publisher<peregrine_interfaces::msg::ManagerStatus>("trajectory_status", statusQos);

  goToServer_ = rclcpp_action::create_server<GoTo>(
      this, "~/go_to", std::bind(&TrajectoryManagerNode::onGoToGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryManagerNode::onGoToCancel, this, std::placeholders::_1),
      std::bind(&TrajectoryManagerNode::onGoToAccepted, this, std::placeholders::_1));

  executeServer_ = rclcpp_action::create_server<ExecuteTrajectory>(
      this, "~/execute_trajectory",
      std::bind(&TrajectoryManagerNode::onExecuteGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryManagerNode::onExecuteCancel, this, std::placeholders::_1),
      std::bind(&TrajectoryManagerNode::onExecuteAccepted, this, std::placeholders::_1));

  publishTimer_ = this->create_wall_timer(periodFromHz(publishRateHz_),
                                          std::bind(&TrajectoryManagerNode::publishTrajectorySetpoint, this));
  statusTimer_ = this->create_wall_timer(periodFromHz(statusRateHz_), std::bind(&TrajectoryManagerNode::publishStatus, this));

  RCLCPP_INFO(this->get_logger(), "trajectory_manager started: publish_rate_hz=%.1f status_rate_hz=%.1f",
              publishRateHz_, statusRateHz_);
}

void TrajectoryManagerNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestState_ = *msg;
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
  {
    lastStateTime_ = this->now();
  }
  else
  {
    lastStateTime_ = rclcpp::Time(msg->header.stamp);
  }
  if (!holdGenerator_)
  {
    holdGenerator_ = std::make_unique<HoldPositionGenerator>(*msg);
    activeModuleName_ = holdGenerator_->name();
  }
}

void TrajectoryManagerNode::publishTrajectorySetpoint()
{
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
    if (!latestState_.has_value())
    {
      return;
    }

    if (!holdGenerator_)
    {
      holdGenerator_ = std::make_unique<HoldPositionGenerator>(*latestState_);
    }

    TrajectorySample sample;
    if (activeGenerator_)
    {
      if (activeGoalType_ == ActiveGoalType::GoTo && activeGoToGoal_ && activeGoToGoal_->is_canceling())
      {
        goToGoalToCancel = activeGoToGoal_;
        goToResult = std::make_shared<GoTo::Result>();
        goToResult->success = false;
        goToResult->message = "goal canceled";
        goToResult->final_position = latestState_->pose.pose.position;
        switchToHoldFromState(*latestState_);
      }
      else if (activeGoalType_ == ActiveGoalType::ExecuteTrajectory && activeExecuteGoal_ && activeExecuteGoal_->is_canceling())
      {
        executeGoalToCancel = activeExecuteGoal_;
        executeResult = std::make_shared<ExecuteTrajectory::Result>();
        executeResult->success = false;
        executeResult->message = "goal canceled";
        switchToHoldFromState(*latestState_);
      }
    }

    if (activeGenerator_)
    {
      sample = activeGenerator_->sample(*latestState_, now);
      setpoint = sample.setpoint;

      if (activeGoalType_ == ActiveGoalType::GoTo && activeGoToGoal_)
      {
        auto feedback = std::make_shared<GoTo::Feedback>();
        feedback->distance_remaining_m = sample.distanceRemaining;
        activeGoToGoal_->publish_feedback(feedback);
      }
      else if (activeGoalType_ == ActiveGoalType::ExecuteTrajectory && activeExecuteGoal_)
      {
        auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
        feedback->progress = sample.progress;
        activeExecuteGoal_->publish_feedback(feedback);
      }

      if (sample.completed)
      {
        if (activeGoalType_ == ActiveGoalType::GoTo && activeGoToGoal_)
        {
          goToGoalToSucceed = activeGoToGoal_;
          goToResult = std::make_shared<GoTo::Result>();
          goToResult->success = true;
          goToResult->message = "goal reached";
          goToResult->final_position = sample.setpoint.position;
        }
        else if (activeGoalType_ == ActiveGoalType::ExecuteTrajectory && activeExecuteGoal_)
        {
          executeGoalToSucceed = activeExecuteGoal_;
          executeResult = std::make_shared<ExecuteTrajectory::Result>();
          executeResult->success = true;
          executeResult->message = "trajectory completed";
        }

        const double holdYaw = sample.setpoint.yaw;
        holdGenerator_ = std::make_unique<HoldPositionGenerator>(sample.setpoint.position, holdYaw);
        activeGenerator_.reset();
        activeGoalType_ = ActiveGoalType::None;
        activeGoToGoal_.reset();
        activeExecuteGoal_.reset();
        activeModuleName_ = holdGenerator_->name();
      }
    }
    else
    {
      sample = holdGenerator_->sample(*latestState_, now);
      setpoint = sample.setpoint;
      activeModuleName_ = holdGenerator_->name();
    }
  }

  setpoint.header.stamp = now;
  trajectorySetpointPub_->publish(setpoint);

  if (goToGoalToCancel && goToResult)
  {
    goToGoalToCancel->canceled(goToResult);
  }
  if (executeGoalToCancel && executeResult)
  {
    executeGoalToCancel->canceled(executeResult);
  }
  if (goToGoalToSucceed && goToResult)
  {
    goToGoalToSucceed->succeed(goToResult);
  }
  if (executeGoalToSucceed && executeResult)
  {
    executeGoalToSucceed->succeed(executeResult);
  }
}

void TrajectoryManagerNode::publishStatus()
{
  peregrine_interfaces::msg::ManagerStatus status;
  status.header.stamp = this->now();
  status.manager_name = kManagerName;
  status.active_module = activeModuleName_;
  status.output_rate_hz = static_cast<float>(publishRateHz_);

  {
    std::scoped_lock lock(mutex_);
    status.active = latestState_.has_value();
    if (!status.active)
    {
      status.healthy = false;
      status.message = "waiting_for_estimated_state";
    }
    else
    {
      const double ageS = (this->now() - lastStateTime_).seconds();
      status.healthy = ageS <= stateTimeoutS_;
      status.message = status.healthy ? "ok" : "estimated_state_timeout";
      status.active_module = activeModuleName_;
    }
  }

  statusPub_->publish(status);
}

rclcpp_action::GoalResponse TrajectoryManagerNode::onGoToGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                              const std::shared_ptr<const GoTo::Goal> goal)
{
  std::scoped_lock lock(mutex_);
  if (!latestState_.has_value() || activeGoalType_ != ActiveGoalType::None)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->acceptance_radius_m <= 0.0)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryManagerNode::onGoToCancel(const std::shared_ptr<GoalHandleGoTo> goalHandle)
{
  std::scoped_lock lock(mutex_);
  if (activeGoalType_ == ActiveGoalType::GoTo && activeGoToGoal_ == goalHandle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void TrajectoryManagerNode::onGoToAccepted(const std::shared_ptr<GoalHandleGoTo> goalHandle)
{
  std::scoped_lock lock(mutex_);
  if (!latestState_.has_value())
  {
    auto result = std::make_shared<GoTo::Result>();
    result->success = false;
    result->message = "missing_estimated_state";
    goalHandle->abort(result);
    return;
  }

  const auto& goal = *goalHandle->get_goal();
  const double targetYaw = std::isfinite(goal.target_yaw) ? goal.target_yaw
                                                           : yawFromQuaternion(latestState_->pose.pose.orientation);
  const double velocity = (goal.velocity_mps > 0.0) ? goal.velocity_mps : 1.0;
  activeGenerator_ = std::make_unique<LinearGoToGenerator>(*latestState_, goal.target_position, targetYaw, velocity,
                                                            goal.acceptance_radius_m, this->now());
  activeGoalType_ = ActiveGoalType::GoTo;
  activeGoToGoal_ = goalHandle;
  activeExecuteGoal_.reset();
  activeModuleName_ = activeGenerator_->name();
}

rclcpp_action::GoalResponse TrajectoryManagerNode::onExecuteGoal(
    const rclcpp_action::GoalUUID& /*uuid*/, const std::shared_ptr<const ExecuteTrajectory::Goal> goal)
{
  std::scoped_lock lock(mutex_);
  if (!latestState_.has_value() || activeGoalType_ != ActiveGoalType::None)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto generator = createGeneratorForExecuteGoal(*goal, *latestState_, this->now());
  if (!generator)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryManagerNode::onExecuteCancel(
    const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle)
{
  std::scoped_lock lock(mutex_);
  if (activeGoalType_ == ActiveGoalType::ExecuteTrajectory && activeExecuteGoal_ == goalHandle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void TrajectoryManagerNode::onExecuteAccepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle)
{
  std::scoped_lock lock(mutex_);
  if (!latestState_.has_value())
  {
    auto result = std::make_shared<ExecuteTrajectory::Result>();
    result->success = false;
    result->message = "missing_estimated_state";
    goalHandle->abort(result);
    return;
  }

  auto generator = createGeneratorForExecuteGoal(*goalHandle->get_goal(), *latestState_, this->now());
  if (!generator)
  {
    auto result = std::make_shared<ExecuteTrajectory::Result>();
    result->success = false;
    result->message = "invalid_goal";
    goalHandle->abort(result);
    return;
  }

  activeGenerator_ = std::move(generator);
  activeGoalType_ = ActiveGoalType::ExecuteTrajectory;
  activeExecuteGoal_ = goalHandle;
  activeGoToGoal_.reset();
  activeModuleName_ = activeGenerator_->name();
}

std::unique_ptr<TrajectoryGeneratorBase> TrajectoryManagerNode::createGeneratorForExecuteGoal(
    const ExecuteTrajectory::Goal& goal, const peregrine_interfaces::msg::State& state, const rclcpp::Time& startTime) const
{
  if (goal.trajectory_type == "hold")
  {
    return std::make_unique<HoldPositionGenerator>(state);
  }

  if (goal.trajectory_type == "takeoff")
  {
    if (goal.params.size() != 2U)
    {
      return nullptr;
    }
    return std::make_unique<TakeoffGenerator>(state, goal.params[0], goal.params[1], startTime);
  }

  if (goal.trajectory_type == "land")
  {
    if (goal.params.size() != 1U)
    {
      return nullptr;
    }
    return std::make_unique<LandGenerator>(state, goal.params[0], startTime);
  }

  if (goal.trajectory_type == "circle")
  {
    if (goal.params.size() != 3U)
    {
      return nullptr;
    }
    return std::make_unique<CircleGenerator>(state, goal.params[0], goal.params[1], goal.params[2], startTime);
  }

  if (goal.trajectory_type == "figure8")
  {
    if (goal.params.size() != 3U)
    {
      return nullptr;
    }
    return std::make_unique<FigureEightGenerator>(state, goal.params[0], goal.params[1], goal.params[2], startTime);
  }

  return nullptr;
}

void TrajectoryManagerNode::switchToHoldFromState(const peregrine_interfaces::msg::State& state)
{
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
