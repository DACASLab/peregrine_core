#include <uav_manager/uav_manager_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <future>
#include <stdexcept>
#include <thread>
#include <utility>

namespace uav_manager
{
namespace
{

constexpr char kNodeName[] = "uav_manager";

constexpr double kServiceWaitS = 3.0;
constexpr double kServiceResponseWaitS = 5.0;
constexpr double kActionServerWaitS = 3.0;
constexpr double kActionResultWaitS = 180.0;
constexpr double kArmAcquireTimeoutS = 45.0;
constexpr double kArmAcquirePollS = 0.25;
constexpr double kArmStateWaitPerAttemptS = 2.0;

// PX4 nav_state constants from px4_msgs/VehicleStatus.
constexpr uint8_t kNavStatePosCtl = 2;
constexpr uint8_t kNavStateAutoMission = 3;
constexpr uint8_t kNavStateAutoRtl = 5;      // Return mode.
constexpr uint8_t kNavStateAutoTakeoff = 17;
constexpr uint8_t kNavStateAutoLand = 18;

bool navStatePreventsArming(const uint8_t navState)
{
  // Mission/return/takeoff/land are not valid arm-entry modes in this manager flow.
  return navState == kNavStateAutoMission || navState == kNavStateAutoRtl || navState == kNavStateAutoTakeoff ||
         navState == kNavStateAutoLand;
}

}  // namespace

UavManagerNode::UavManagerNode(const rclcpp::NodeOptions& options)
: Node(kNodeName, options)
{
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 10.0);
  if (statusRateHz_ <= 0.0)
  {
    throw std::runtime_error("status_rate_hz must be > 0");
  }

  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
      "estimated_state", qos, std::bind(&UavManagerNode::onEstimatedState, this, std::placeholders::_1));
  px4StatusSub_ = this->create_subscription<peregrine_interfaces::msg::PX4Status>(
      "status", statusQos, std::bind(&UavManagerNode::onPx4Status, this, std::placeholders::_1));
  estimationStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
      "estimation_status", statusQos, std::bind(&UavManagerNode::onEstimationStatus, this, std::placeholders::_1));
  controlStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
      "control_status", statusQos, std::bind(&UavManagerNode::onControlStatus, this, std::placeholders::_1));
  trajectoryStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
      "trajectory_status", statusQos, std::bind(&UavManagerNode::onTrajectoryStatus, this, std::placeholders::_1));

  uavStatePub_ = this->create_publisher<peregrine_interfaces::msg::UAVState>("uav_state", statusQos);

  armClient_ = this->create_client<peregrine_interfaces::srv::Arm>("arm");
  setModeClient_ = this->create_client<peregrine_interfaces::srv::SetMode>("set_mode");
  trajectoryGoToClient_ = rclcpp_action::create_client<GoTo>(this, "trajectory_manager/go_to");
  trajectoryExecuteClient_ =
      rclcpp_action::create_client<ExecuteTrajectory>(this, "trajectory_manager/execute_trajectory");

  takeoffServer_ = rclcpp_action::create_server<Takeoff>(
      this, "~/takeoff", std::bind(&UavManagerNode::onTakeoffGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UavManagerNode::onTakeoffCancel, this, std::placeholders::_1),
      std::bind(&UavManagerNode::onTakeoffAccepted, this, std::placeholders::_1));

  landServer_ = rclcpp_action::create_server<Land>(
      this, "~/land", std::bind(&UavManagerNode::onLandGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UavManagerNode::onLandCancel, this, std::placeholders::_1),
      std::bind(&UavManagerNode::onLandAccepted, this, std::placeholders::_1));

  goToServer_ = rclcpp_action::create_server<GoTo>(
      this, "~/go_to", std::bind(&UavManagerNode::onGoToGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UavManagerNode::onGoToCancel, this, std::placeholders::_1),
      std::bind(&UavManagerNode::onGoToAccepted, this, std::placeholders::_1));

  executeServer_ = rclcpp_action::create_server<ExecuteTrajectory>(
      this, "~/execute_trajectory",
      std::bind(&UavManagerNode::onExecuteGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UavManagerNode::onExecuteCancel, this, std::placeholders::_1),
      std::bind(&UavManagerNode::onExecuteAccepted, this, std::placeholders::_1));

  statusTimer_ = this->create_wall_timer(periodFromHz(statusRateHz_), std::bind(&UavManagerNode::publishUavState, this));

  RCLCPP_INFO(this->get_logger(), "uav_manager started: status_rate_hz=%.1f", statusRateHz_);
}

void UavManagerNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestState_ = *msg;
}

void UavManagerNode::onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestPx4Status_ = *msg;
  }
  if (msg->failsafe)
  {
    setFsmState(FsmState::Emergency, "px4_failsafe");
  }
}

void UavManagerNode::onEstimationStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestEstimationStatus_ = *msg;
}

void UavManagerNode::onControlStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestControlStatus_ = *msg;
}

void UavManagerNode::onTrajectoryStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestTrajectoryStatus_ = *msg;
}

void UavManagerNode::publishUavState()
{
  peregrine_interfaces::msg::UAVState output;
  output.header.stamp = this->now();

  {
    std::scoped_lock lock(mutex_);
    switch (fsmState_)
    {
      case FsmState::Idle:
        output.state = peregrine_interfaces::msg::UAVState::STATE_IDLE;
        break;
      case FsmState::Armed:
        output.state = peregrine_interfaces::msg::UAVState::STATE_ARMED;
        break;
      case FsmState::TakingOff:
        output.state = peregrine_interfaces::msg::UAVState::STATE_TAKING_OFF;
        break;
      case FsmState::Hovering:
        output.state = peregrine_interfaces::msg::UAVState::STATE_HOVERING;
        break;
      case FsmState::Flying:
        output.state = peregrine_interfaces::msg::UAVState::STATE_FLYING;
        break;
      case FsmState::Landing:
        output.state = peregrine_interfaces::msg::UAVState::STATE_LANDING;
        break;
      case FsmState::Landed:
        output.state = peregrine_interfaces::msg::UAVState::STATE_LANDED;
        break;
      case FsmState::Emergency:
      default:
        output.state = peregrine_interfaces::msg::UAVState::STATE_EMERGENCY;
        break;
    }

    output.mode = stateName(fsmState_);
    output.detail = fsmDetail_;

    if (latestPx4Status_.has_value())
    {
      output.armed = latestPx4Status_->armed;
      output.offboard = latestPx4Status_->offboard;
      output.connected = latestPx4Status_->connected;
      output.failsafe = latestPx4Status_->failsafe;
    }
    else
    {
      output.armed = false;
      output.offboard = false;
      output.connected = false;
      output.failsafe = false;
    }
  }

  uavStatePub_->publish(output);
}

rclcpp_action::GoalResponse UavManagerNode::onTakeoffGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                          const std::shared_ptr<const Takeoff::Goal> goal)
{
  std::string reason;
  if (goal->target_altitude_m <= 0.0)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!takeoffStateAllowed(&reason))
  {
    RCLCPP_WARN(this->get_logger(), "Rejecting takeoff goal: %s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!reserveActionSlot())
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UavManagerNode::onTakeoffCancel(const std::shared_ptr<GoalHandleTakeoff> /*goalHandle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UavManagerNode::onTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goalHandle)
{
  std::thread(
      [this, goalHandle]()
      {
        auto result = std::make_shared<Takeoff::Result>();
        std::string error;

        if (goalHandle->is_canceling())
        {
          result->success = false;
          result->message = "goal canceled";
          result->final_altitude_m = latestAltitudeM();
          goalHandle->canceled(result);
          releaseActionSlot();
          return;
        }

        std::string preflightError;
        if (!preflightReady(&preflightError))
        {
          result->success = false;
          result->message = "preflight_failed: " + preflightError;
          result->final_altitude_m = latestAltitudeM();
          goalHandle->abort(result);
          releaseActionSlot();
          return;
        }

        if (!ensureArmableMode(&error))
        {
          result->success = false;
          result->message = "arm_mode_prepare_failed: " + error;
          result->final_altitude_m = latestAltitudeM();
          setFsmState(FsmState::Idle, "arm_mode_prepare_failed");
          goalHandle->abort(result);
          releaseActionSlot();
          return;
        }

        // Arm can transiently fail during early bringup; retry in a bounded window.
        const auto armDeadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(kArmAcquireTimeoutS);
        bool armed = false;
        while (rclcpp::ok() && std::chrono::steady_clock::now() < armDeadline)
        {
          std::string attemptError;
          const bool commandAccepted = callArmService(true, &attemptError);
          if (commandAccepted && waitForArmed(true, kArmStateWaitPerAttemptS))
          {
            armed = true;
            break;
          }

          if (!attemptError.empty())
          {
            error = attemptError;
          }
          std::this_thread::sleep_for(std::chrono::duration<double>(kArmAcquirePollS));
        }

        if (!armed)
        {
          result->success = false;
          result->message = error.empty() ? "arm_timeout" : ("arm_timeout: " + error);
          result->final_altitude_m = latestAltitudeM();
          setFsmState(FsmState::Idle, "arm_timeout");
          goalHandle->abort(result);
          releaseActionSlot();
          return;
        }

        setFsmState(FsmState::Armed, "armed");

        if (!waitForCondition([this]() { return controlSetpointFlowing(); }, 4.0))
        {
          result->success = false;
          result->message = "setpoint_flow_missing";
          result->final_altitude_m = latestAltitudeM();
          setFsmState(FsmState::Armed, "setpoint_flow_missing");
          goalHandle->abort(result);
          releaseActionSlot();
          return;
        }

        if (!callSetModeService("offboard", &error))
        {
          result->success = false;
          result->message = "set_mode_failed: " + error;
          result->final_altitude_m = latestAltitudeM();
          setFsmState(FsmState::Armed, "offboard_set_mode_failed");
          goalHandle->abort(result);
          releaseActionSlot();
          return;
        }

        if (!waitForOffboard(true, 6.0))
        {
          result->success = false;
          result->message = "offboard_timeout";
          result->final_altitude_m = latestAltitudeM();
          setFsmState(FsmState::Armed, "offboard_timeout");
          goalHandle->abort(result);
          releaseActionSlot();
          return;
        }

        setFsmState(FsmState::TakingOff, "takeoff_trajectory");

        ExecuteTrajectory::Goal executeGoal;
        executeGoal.trajectory_type = "takeoff";
        executeGoal.params = {goalHandle->get_goal()->target_altitude_m, goalHandle->get_goal()->climb_velocity_mps};

        ExecuteTrajectory::Result executeResult;
        const bool trajectoryOk =
            forwardExecuteTrajectory(
                executeGoal,
                [this, goalHandle](const ExecuteTrajectory::Feedback& feedback)
                {
                  auto fb = std::make_shared<Takeoff::Feedback>();
                  fb->progress = feedback.progress;
                  fb->current_altitude_m = latestAltitudeM();
                  goalHandle->publish_feedback(fb);
                },
                &executeResult, &error);

        if (!trajectoryOk || !executeResult.success)
        {
          result->success = false;
          result->message = trajectoryOk ? executeResult.message : ("takeoff_trajectory_failed: " + error);
          result->final_altitude_m = latestAltitudeM();
          setFsmState(FsmState::Armed, "takeoff_trajectory_failed");
          goalHandle->abort(result);
          releaseActionSlot();
          return;
        }

        setFsmState(FsmState::Hovering, "takeoff_complete");
        result->success = true;
        result->message = "takeoff_complete";
        result->final_altitude_m = latestAltitudeM();
        goalHandle->succeed(result);
        releaseActionSlot();
      })
      .detach();
}

rclcpp_action::GoalResponse UavManagerNode::onLandGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                       const std::shared_ptr<const Land::Goal> /*goal*/)
{
  std::string reason;
  if (!landStateAllowed(&reason))
  {
    RCLCPP_WARN(this->get_logger(), "Rejecting land goal: %s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!reserveActionSlot())
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UavManagerNode::onLandCancel(const std::shared_ptr<GoalHandleLand> /*goalHandle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UavManagerNode::onLandAccepted(const std::shared_ptr<GoalHandleLand> goalHandle)
{
  std::thread(
      [this, goalHandle]()
      {
        auto result = std::make_shared<Land::Result>();
        std::string error;
        FsmState fallbackState = FsmState::Hovering;
        {
          std::scoped_lock lock(mutex_);
          if (fsmState_ == FsmState::Armed)
          {
            fallbackState = FsmState::Armed;
          }
        }
        setFsmState(FsmState::Landing, "land_mode");

        if (!callSetModeService("land", &error))
        {
          result->success = false;
          result->message = "set_land_mode_failed: " + error;
          goalHandle->abort(result);
          setFsmState(fallbackState, "land_mode_failed");
          releaseActionSlot();
          return;
        }

        if (!waitForOffboard(false, 5.0))
        {
          result->success = false;
          result->message = "land_mode_not_active";
          goalHandle->abort(result);
          setFsmState(fallbackState, "land_mode_wait_timeout");
          releaseActionSlot();
          return;
        }

        if (!waitForArmed(false, 90.0))
        {
          result->success = false;
          result->message = "auto_disarm_timeout";
          goalHandle->abort(result);
          setFsmState(FsmState::Landing, "waiting_auto_disarm_timeout");
          releaseActionSlot();
          return;
        }

        setFsmState(FsmState::Landed, "landed");

        // Keep post-land behavior explicit: transition PX4 to hold mode when mission is complete.
        std::string holdError;
        if (!callSetModeService("hold", &holdError))
        {
          RCLCPP_WARN(this->get_logger(), "Post-land set_mode(hold) failed: %s", holdError.c_str());
        }

        result->success = true;
        result->message = "land_complete";
        goalHandle->succeed(result);
        setFsmState(FsmState::Idle, "idle");
        releaseActionSlot();
      })
      .detach();
}

rclcpp_action::GoalResponse UavManagerNode::onGoToGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                       const std::shared_ptr<const GoTo::Goal> goal)
{
  std::string reason;
  if (goal->acceptance_radius_m <= 0.0)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!flightActionStateAllowed(&reason))
  {
    RCLCPP_WARN(this->get_logger(), "Rejecting go_to goal: %s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!reserveActionSlot())
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UavManagerNode::onGoToCancel(const std::shared_ptr<GoalHandleGoTo> /*goalHandle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UavManagerNode::onGoToAccepted(const std::shared_ptr<GoalHandleGoTo> goalHandle)
{
  std::thread(
      [this, goalHandle]()
      {
        auto result = std::make_shared<GoTo::Result>();
        std::string error;

        if (!callSetModeService("offboard", &error))
        {
          result->success = false;
          result->message = "set_mode_failed: " + error;
          result->final_position = latestPosition();
          goalHandle->abort(result);
          setFsmState(FsmState::Hovering, "go_to_offboard_failed");
          releaseActionSlot();
          return;
        }

        if (!waitForOffboard(true, 6.0))
        {
          result->success = false;
          result->message = "offboard_timeout";
          result->final_position = latestPosition();
          goalHandle->abort(result);
          setFsmState(FsmState::Hovering, "go_to_offboard_timeout");
          releaseActionSlot();
          return;
        }

        setFsmState(FsmState::Flying, "go_to");

        GoTo::Result forwardResult;
        const bool ok = forwardGoTo(
            *goalHandle->get_goal(),
            [goalHandle](const GoTo::Feedback& feedback)
            {
              auto fb = std::make_shared<GoTo::Feedback>();
              *fb = feedback;
              goalHandle->publish_feedback(fb);
            },
            &forwardResult, &error);

        if (!ok || !forwardResult.success)
        {
          result->success = false;
          result->message = ok ? forwardResult.message : ("go_to_forward_failed: " + error);
          result->final_position = latestPosition();
          setFsmState(FsmState::Hovering, "go_to_failed");
          goalHandle->abort(result);
          releaseActionSlot();
          return;
        }

        setFsmState(FsmState::Hovering, "go_to_complete");
        *result = forwardResult;
        goalHandle->succeed(result);
        releaseActionSlot();
      })
      .detach();
}

rclcpp_action::GoalResponse UavManagerNode::onExecuteGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                          const std::shared_ptr<const ExecuteTrajectory::Goal> goal)
{
  std::string reason;
  if (goal->trajectory_type.empty())
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!flightActionStateAllowed(&reason))
  {
    RCLCPP_WARN(this->get_logger(), "Rejecting execute_trajectory goal: %s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!reserveActionSlot())
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UavManagerNode::onExecuteCancel(
    const std::shared_ptr<GoalHandleExecuteTrajectory> /*goalHandle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UavManagerNode::onExecuteAccepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle)
{
  std::thread(
      [this, goalHandle]()
      {
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        std::string error;

        if (!callSetModeService("offboard", &error))
        {
          result->success = false;
          result->message = "set_mode_failed: " + error;
          goalHandle->abort(result);
          setFsmState(FsmState::Hovering, "execute_offboard_failed");
          releaseActionSlot();
          return;
        }

        if (!waitForOffboard(true, 6.0))
        {
          result->success = false;
          result->message = "offboard_timeout";
          goalHandle->abort(result);
          setFsmState(FsmState::Hovering, "execute_offboard_timeout");
          releaseActionSlot();
          return;
        }

        setFsmState(FsmState::Flying, "execute_trajectory");

        ExecuteTrajectory::Result forwardResult;
        const bool ok = forwardExecuteTrajectory(
            *goalHandle->get_goal(),
            [goalHandle](const ExecuteTrajectory::Feedback& feedback)
            {
              auto fb = std::make_shared<ExecuteTrajectory::Feedback>();
              *fb = feedback;
              goalHandle->publish_feedback(fb);
            },
            &forwardResult, &error);

        if (!ok || !forwardResult.success)
        {
          result->success = false;
          result->message = ok ? forwardResult.message : ("execute_forward_failed: " + error);
          setFsmState(FsmState::Hovering, "execute_failed");
          goalHandle->abort(result);
          releaseActionSlot();
          return;
        }

        setFsmState(FsmState::Hovering, "execute_complete");
        *result = forwardResult;
        goalHandle->succeed(result);
        releaseActionSlot();
      })
      .detach();
}

bool UavManagerNode::reserveActionSlot()
{
  std::scoped_lock lock(actionSlotMutex_);
  if (actionSlotReserved_)
  {
    return false;
  }
  actionSlotReserved_ = true;
  return true;
}

void UavManagerNode::releaseActionSlot()
{
  std::scoped_lock lock(actionSlotMutex_);
  actionSlotReserved_ = false;
}

bool UavManagerNode::setFsmState(const FsmState state, const std::string& detail)
{
  std::scoped_lock lock(mutex_);
  if (fsmState_ == state)
  {
    fsmDetail_ = detail;
    return true;
  }

  bool allowed = false;
  if (state == FsmState::Emergency)
  {
    allowed = true;
  }
  else
  {
    switch (fsmState_)
    {
      case FsmState::Idle:
        allowed = state == FsmState::Armed;
        break;
      case FsmState::Armed:
        allowed = state == FsmState::Idle || state == FsmState::TakingOff || state == FsmState::Landing;
        break;
      case FsmState::TakingOff:
        allowed = state == FsmState::Armed || state == FsmState::Hovering || state == FsmState::Landing;
        break;
      case FsmState::Hovering:
        allowed = state == FsmState::Flying || state == FsmState::Landing || state == FsmState::Idle;
        break;
      case FsmState::Flying:
        allowed = state == FsmState::Hovering || state == FsmState::Landing;
        break;
      case FsmState::Landing:
        allowed = state == FsmState::Landed || state == FsmState::Hovering || state == FsmState::Armed;
        break;
      case FsmState::Landed:
        allowed = state == FsmState::Idle || state == FsmState::Armed;
        break;
      case FsmState::Emergency:
        allowed = state == FsmState::Idle;
        break;
      default:
        allowed = false;
        break;
    }
  }

  if (!allowed)
  {
    RCLCPP_WARN(this->get_logger(), "Rejected invalid FSM transition: %s -> %s", stateName(fsmState_).c_str(),
                stateName(state).c_str());
    return false;
  }

  fsmState_ = state;
  fsmDetail_ = detail;
  return true;
}

bool UavManagerNode::takeoffStateAllowed(std::string* reasonOut) const
{
  std::scoped_lock lock(mutex_);
  if (fsmState_ == FsmState::Emergency)
  {
    if (reasonOut)
    {
      *reasonOut = "fsm_emergency";
    }
    return false;
  }
  if (fsmState_ != FsmState::Idle && fsmState_ != FsmState::Armed && fsmState_ != FsmState::Landed)
  {
    if (reasonOut)
    {
      *reasonOut = "fsm_state_" + stateName(fsmState_) + "_not_takeoff_ready";
    }
    return false;
  }
  return true;
}

bool UavManagerNode::landStateAllowed(std::string* reasonOut) const
{
  std::scoped_lock lock(mutex_);
  if (fsmState_ == FsmState::Emergency)
  {
    if (reasonOut)
    {
      *reasonOut = "fsm_emergency";
    }
    return false;
  }
  if (fsmState_ != FsmState::Armed && fsmState_ != FsmState::TakingOff && fsmState_ != FsmState::Hovering &&
      fsmState_ != FsmState::Flying)
  {
    if (reasonOut)
    {
      *reasonOut = "fsm_state_" + stateName(fsmState_) + "_not_land_ready";
    }
    return false;
  }
  return true;
}

bool UavManagerNode::flightActionStateAllowed(std::string* reasonOut) const
{
  std::scoped_lock lock(mutex_);
  if (fsmState_ == FsmState::Emergency)
  {
    if (reasonOut)
    {
      *reasonOut = "fsm_emergency";
    }
    return false;
  }
  if (fsmState_ != FsmState::Hovering && fsmState_ != FsmState::Flying)
  {
    if (reasonOut)
    {
      *reasonOut = "fsm_state_" + stateName(fsmState_) + "_not_flight_action_ready";
    }
    return false;
  }
  if (!latestPx4Status_.has_value() || !latestPx4Status_->armed)
  {
    if (reasonOut)
    {
      *reasonOut = "vehicle_not_armed";
    }
    return false;
  }
  return true;
}

bool UavManagerNode::preflightReady(std::string* reasonOut) const
{
  std::scoped_lock lock(mutex_);
  if (!latestPx4Status_.has_value())
  {
    if (reasonOut)
    {
      *reasonOut = "missing_px4_status";
    }
    return false;
  }
  if (!latestPx4Status_->connected)
  {
    if (reasonOut)
    {
      *reasonOut = "px4_disconnected";
    }
    return false;
  }
  if (latestPx4Status_->failsafe)
  {
    if (reasonOut)
    {
      *reasonOut = "px4_failsafe";
    }
    return false;
  }
  if (!latestState_.has_value())
  {
    if (reasonOut)
    {
      *reasonOut = "missing_estimated_state";
    }
    return false;
  }
  if (!latestEstimationStatus_.has_value() || !latestEstimationStatus_->active || !latestEstimationStatus_->healthy)
  {
    if (reasonOut)
    {
      *reasonOut = "estimation_not_ready";
    }
    return false;
  }
  if (!latestControlStatus_.has_value() || !latestControlStatus_->active || !latestControlStatus_->healthy)
  {
    if (reasonOut)
    {
      *reasonOut = "control_not_ready";
    }
    return false;
  }
  if (!latestTrajectoryStatus_.has_value() || !latestTrajectoryStatus_->active || !latestTrajectoryStatus_->healthy)
  {
    if (reasonOut)
    {
      *reasonOut = "trajectory_not_ready";
    }
    return false;
  }
  return true;
}

bool UavManagerNode::ensureArmableMode(std::string* errorOut)
{
  std::optional<peregrine_interfaces::msg::PX4Status> status;
  {
    std::scoped_lock lock(mutex_);
    status = latestPx4Status_;
  }

  if (!status.has_value())
  {
    if (errorOut)
    {
      *errorOut = "missing_px4_status";
    }
    return false;
  }

  if (!status->connected)
  {
    if (errorOut)
    {
      *errorOut = "px4_disconnected";
    }
    return false;
  }

  if (status->armed || !navStatePreventsArming(status->nav_state))
  {
    return true;
  }

  RCLCPP_INFO(this->get_logger(), "Current nav_state=%u blocks arming; switching to position mode first.",
              static_cast<unsigned>(status->nav_state));

  std::string modeError;
  if (!callSetModeService("position", &modeError))
  {
    if (errorOut)
    {
      *errorOut = "set_position_failed: " + modeError;
    }
    return false;
  }

  if (!waitForNavState(kNavStatePosCtl, 6.0))
  {
    if (errorOut)
    {
      *errorOut = "position_mode_timeout";
    }
    return false;
  }

  return true;
}

bool UavManagerNode::controlSetpointFlowing() const
{
  std::scoped_lock lock(mutex_);
  return latestControlStatus_.has_value() && latestControlStatus_->active && latestControlStatus_->healthy;
}

bool UavManagerNode::callArmService(const bool arm, std::string* errorOut)
{
  if (!armClient_->wait_for_service(std::chrono::duration<double>(kServiceWaitS)))
  {
    if (errorOut)
    {
      *errorOut = "arm_service_unavailable";
    }
    return false;
  }

  auto request = std::make_shared<peregrine_interfaces::srv::Arm::Request>();
  request->arm = arm;
  auto future = armClient_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(kServiceResponseWaitS)) != std::future_status::ready)
  {
    if (errorOut)
    {
      *errorOut = "arm_service_timeout";
    }
    return false;
  }

  const auto response = future.get();
  if (!response->success)
  {
    if (errorOut)
    {
      *errorOut = response->message;
    }
    return false;
  }
  return true;
}

bool UavManagerNode::callSetModeService(const std::string& mode, std::string* errorOut)
{
  if (!setModeClient_->wait_for_service(std::chrono::duration<double>(kServiceWaitS)))
  {
    if (errorOut)
    {
      *errorOut = "set_mode_service_unavailable";
    }
    return false;
  }

  auto request = std::make_shared<peregrine_interfaces::srv::SetMode::Request>();
  request->mode = mode;
  auto future = setModeClient_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(kServiceResponseWaitS)) != std::future_status::ready)
  {
    if (errorOut)
    {
      *errorOut = "set_mode_service_timeout";
    }
    return false;
  }

  const auto response = future.get();
  if (!response->success)
  {
    if (errorOut)
    {
      *errorOut = response->message;
    }
    return false;
  }
  return true;
}

bool UavManagerNode::waitForArmed(const bool armed, const double timeoutS) const
{
  return waitForCondition(
      [this, armed]()
      {
        std::scoped_lock lock(mutex_);
        return latestPx4Status_.has_value() && latestPx4Status_->armed == armed;
      },
      timeoutS);
}

bool UavManagerNode::waitForOffboard(const bool offboard, const double timeoutS) const
{
  return waitForCondition(
      [this, offboard]()
      {
        std::scoped_lock lock(mutex_);
        return latestPx4Status_.has_value() && latestPx4Status_->offboard == offboard;
      },
      timeoutS);
}

bool UavManagerNode::waitForNavState(const uint8_t navState, const double timeoutS) const
{
  return waitForCondition(
      [this, navState]()
      {
        std::scoped_lock lock(mutex_);
        return latestPx4Status_.has_value() && latestPx4Status_->nav_state == navState;
      },
      timeoutS);
}

bool UavManagerNode::waitForCondition(const std::function<bool()>& condition, const double timeoutS) const
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutS);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (condition())
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return condition();
}

bool UavManagerNode::forwardExecuteTrajectory(const ExecuteTrajectory::Goal& goal,
                                              std::function<void(const ExecuteTrajectory::Feedback&)> feedbackCallback,
                                              ExecuteTrajectory::Result* resultOut, std::string* errorOut)
{
  if (!trajectoryExecuteClient_->wait_for_action_server(std::chrono::duration<double>(kActionServerWaitS)))
  {
    if (errorOut)
    {
      *errorOut = "trajectory_execute_server_unavailable";
    }
    return false;
  }

  rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions options;
  options.feedback_callback =
      [feedbackCallback = std::move(feedbackCallback)](auto, const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback)
  {
    if (feedbackCallback && feedback)
    {
      feedbackCallback(*feedback);
    }
  };

  auto goalHandleFuture = trajectoryExecuteClient_->async_send_goal(goal, options);
  if (goalHandleFuture.wait_for(std::chrono::duration<double>(kServiceResponseWaitS)) != std::future_status::ready)
  {
    if (errorOut)
    {
      *errorOut = "trajectory_execute_goal_timeout";
    }
    return false;
  }

  auto goalHandle = goalHandleFuture.get();
  if (!goalHandle)
  {
    if (errorOut)
    {
      *errorOut = "trajectory_execute_goal_rejected";
    }
    return false;
  }

  auto resultFuture = trajectoryExecuteClient_->async_get_result(goalHandle);
  if (resultFuture.wait_for(std::chrono::duration<double>(kActionResultWaitS)) != std::future_status::ready)
  {
    if (errorOut)
    {
      *errorOut = "trajectory_execute_result_timeout";
    }
    return false;
  }

  const auto wrappedResult = resultFuture.get();
  if (resultOut)
  {
    *resultOut = *wrappedResult.result;
  }
  if (wrappedResult.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    if (errorOut)
    {
      *errorOut = "trajectory_execute_result_code_not_succeeded";
    }
    return false;
  }
  return true;
}

bool UavManagerNode::forwardGoTo(const GoTo::Goal& goal, std::function<void(const GoTo::Feedback&)> feedbackCallback,
                                 GoTo::Result* resultOut, std::string* errorOut)
{
  if (!trajectoryGoToClient_->wait_for_action_server(std::chrono::duration<double>(kActionServerWaitS)))
  {
    if (errorOut)
    {
      *errorOut = "trajectory_goto_server_unavailable";
    }
    return false;
  }

  rclcpp_action::Client<GoTo>::SendGoalOptions options;
  options.feedback_callback = [feedbackCallback = std::move(feedbackCallback)](auto,
                                                                                const std::shared_ptr<const GoTo::Feedback> feedback)
  {
    if (feedbackCallback && feedback)
    {
      feedbackCallback(*feedback);
    }
  };

  auto goalHandleFuture = trajectoryGoToClient_->async_send_goal(goal, options);
  if (goalHandleFuture.wait_for(std::chrono::duration<double>(kServiceResponseWaitS)) != std::future_status::ready)
  {
    if (errorOut)
    {
      *errorOut = "trajectory_goto_goal_timeout";
    }
    return false;
  }

  auto goalHandle = goalHandleFuture.get();
  if (!goalHandle)
  {
    if (errorOut)
    {
      *errorOut = "trajectory_goto_goal_rejected";
    }
    return false;
  }

  auto resultFuture = trajectoryGoToClient_->async_get_result(goalHandle);
  if (resultFuture.wait_for(std::chrono::duration<double>(kActionResultWaitS)) != std::future_status::ready)
  {
    if (errorOut)
    {
      *errorOut = "trajectory_goto_result_timeout";
    }
    return false;
  }

  const auto wrappedResult = resultFuture.get();
  if (resultOut)
  {
    *resultOut = *wrappedResult.result;
  }
  if (wrappedResult.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    if (errorOut)
    {
      *errorOut = "trajectory_goto_result_code_not_succeeded";
    }
    return false;
  }
  return true;
}

double UavManagerNode::latestAltitudeM() const
{
  std::scoped_lock lock(mutex_);
  if (!latestState_.has_value())
  {
    return 0.0;
  }
  return latestState_->pose.pose.position.z;
}

geometry_msgs::msg::Point UavManagerNode::latestPosition() const
{
  std::scoped_lock lock(mutex_);
  if (!latestState_.has_value())
  {
    geometry_msgs::msg::Point origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
    return origin;
  }
  return latestState_->pose.pose.position;
}

std::chrono::nanoseconds UavManagerNode::periodFromHz(const double hz)
{
  const auto period = std::chrono::duration<double>(1.0 / hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(period);
}

std::string UavManagerNode::stateName(const FsmState state)
{
  switch (state)
  {
    case FsmState::Idle:
      return "IDLE";
    case FsmState::Armed:
      return "ARMED";
    case FsmState::TakingOff:
      return "TAKING_OFF";
    case FsmState::Hovering:
      return "HOVERING";
    case FsmState::Flying:
      return "FLYING";
    case FsmState::Landing:
      return "LANDING";
    case FsmState::Landed:
      return "LANDED";
    case FsmState::Emergency:
    default:
      return "EMERGENCY";
  }
}

}  // namespace uav_manager

RCLCPP_COMPONENTS_REGISTER_NODE(uav_manager::UavManagerNode)
