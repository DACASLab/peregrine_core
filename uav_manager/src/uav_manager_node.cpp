#include <uav_manager/uav_manager_node.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <future>
#include <utility>

namespace uav_manager
{
using namespace std::chrono_literals;

namespace
{

constexpr char kNodeName[] = "uav_manager";

constexpr uint8_t kNavStateManual = 0;
constexpr uint8_t kNavStatePosCtl = 2;
constexpr uint8_t kNavStateAutoMission = 3;
constexpr uint8_t kNavStateAutoRtl = 5;
constexpr uint8_t kNavStateOffboard = 14;
constexpr uint8_t kNavStateAutoTakeoff = 17;
constexpr uint8_t kNavStateAutoLand = 18;

bool navStatePreventsArming(const uint8_t navState)
{
  return navState == kNavStateAutoMission || navState == kNavStateAutoRtl ||
         navState == kNavStateAutoTakeoff || navState == kNavStateAutoLand;
}

std::string px4ModeString(const peregrine_interfaces::msg::PX4Status & status)
{
  if (status.failsafe) {
    return "FAILSAFE";
  }
  if (status.offboard || status.nav_state == kNavStateOffboard) {
    return "OFFBOARD";
  }
  switch (status.nav_state) {
    case kNavStateManual:
      return "MANUAL";
    case kNavStatePosCtl:
      return "POSCTL";
    case kNavStateAutoMission:
      return "AUTO_MISSION";
    case kNavStateAutoRtl:
      return "AUTO_RTL";
    case kNavStateAutoTakeoff:
      return "AUTO_TAKEOFF";
    case kNavStateAutoLand:
      return "AUTO_LAND";
    default:
      return "NAV_" + std::to_string(status.nav_state);
  }
}

std::chrono::milliseconds secondsToMillis(const double seconds)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(seconds));
}

}  // namespace

UavManagerNode::UavManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kNodeName, options)
{
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 5.0);
  dependencyStartupTimeoutS_ = this->declare_parameter<double>("dependency_startup_timeout_s", 2.0);
  serviceWaitS_ = this->declare_parameter<double>("service_wait_s", 3.0);
  serviceResponseWaitS_ = this->declare_parameter<double>("service_response_wait_s", 5.0);
  actionServerWaitS_ = this->declare_parameter<double>("action_server_wait_s", 3.0);
  actionResultWaitS_ = this->declare_parameter<double>("action_result_wait_s", 180.0);
  offboardWaitS_ = this->declare_parameter<double>("offboard_wait_s", 6.0);
  armedWaitS_ = this->declare_parameter<double>("armed_wait_s", 10.0);
  estimatedStateDeadlineS_ = this->declare_parameter<double>("estimated_state_deadline_s", 1.0);
  px4StatusDeadlineS_ = this->declare_parameter<double>("px4_status_deadline_s", 1.0);
  batteryDeadlineS_ = this->declare_parameter<double>("battery_deadline_s", 1.0);
  requireExternalSafety_ = this->declare_parameter<bool>("require_external_safety", false);

  autoStart_ = this->declare_parameter<bool>("auto_start", true);
  dataReadinessTimeoutS_ = this->declare_parameter<double>("data_readiness_timeout_s", 30.0);
  dataReadinessPollMs_ = this->declare_parameter<int>("data_readiness_poll_ms", 200);

  if (autoStart_) {
    startupTimer_ = this->create_wall_timer(
      200ms,
      [this]() {
        startupTimer_->cancel();
        RCLCPP_INFO(get_logger(), "Auto-start: triggering configure");
        const auto configResult = this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        if (configResult.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          RCLCPP_ERROR(get_logger(), "Auto-configure failed (state=%s)", configResult.label().c_str());
          return;
        }

        readinessDeadline_ = this->now() + rclcpp::Duration::from_seconds(dataReadinessTimeoutS_);
        readinessTimer_ = this->create_wall_timer(
          std::chrono::milliseconds(dataReadinessPollMs_),
          [this]() {
            const bool ready = dependenciesReady();
            const bool timedOut = this->now() >= readinessDeadline_;
            if (!ready && !timedOut) {
              return;
            }

            if (!ready) {
              RCLCPP_WARN(
                get_logger(),
                "Auto-start readiness timeout; activating anyway (%s)",
                dependenciesReason().c_str());
            }
            readinessTimer_->cancel();

            const auto activateResult = this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            if (activateResult.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
              RCLCPP_ERROR(get_logger(), "Auto-activate failed (state=%s)", activateResult.label().c_str());
              return;
            }
            RCLCPP_INFO(get_logger(), "Auto-start complete: ACTIVE");
          });
      });
  }
}

UavManagerNode::CallbackReturn UavManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  if (statusRateHz_ <= 0.0 || dependencyStartupTimeoutS_ <= 0.0 || serviceWaitS_ <= 0.0 ||
      serviceResponseWaitS_ <= 0.0 || actionServerWaitS_ <= 0.0 || actionResultWaitS_ <= 0.0 ||
      offboardWaitS_ <= 0.0 || armedWaitS_ <= 0.0 || estimatedStateDeadlineS_ <= 0.0 ||
      px4StatusDeadlineS_ <= 0.0 || batteryDeadlineS_ <= 0.0)
  {
    RCLCPP_ERROR(this->get_logger(), "Lifecycle parameters must all be > 0");
    return CallbackReturn::FAILURE;
  }

  actionCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  serviceCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto stateQos = rclcpp::QoS(20).reliable();
  stateQos.deadline(rclcpp::Duration::from_seconds(estimatedStateDeadlineS_));
  auto px4Qos = rclcpp::QoS(10).reliable();
  px4Qos.deadline(rclcpp::Duration::from_seconds(px4StatusDeadlineS_));
  auto batteryQos = rclcpp::QoS(10).reliable();
  batteryQos.deadline(rclcpp::Duration::from_seconds(batteryDeadlineS_));
  const auto statusQos = rclcpp::QoS(10).reliable();

  rclcpp::SubscriptionOptions stateOptions;
  stateOptions.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &) {
    estimatedStateFresh_.store(false, std::memory_order_release);
  };
  rclcpp::SubscriptionOptions px4Options;
  px4Options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &) {
    px4StatusFresh_.store(false, std::memory_order_release);
  };
  rclcpp::SubscriptionOptions batteryOptions;
  batteryOptions.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &) {
    batteryFresh_.store(false, std::memory_order_release);
  };

  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    "estimated_state", stateQos,
    [this](const peregrine_interfaces::msg::State::SharedPtr msg) { onEstimatedState(msg); }, stateOptions);
  px4StatusSub_ = this->create_subscription<peregrine_interfaces::msg::PX4Status>(
    "status", px4Qos,
    [this](const peregrine_interfaces::msg::PX4Status::SharedPtr msg) { onPx4Status(msg); }, px4Options);
  batterySub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery", batteryQos,
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) { onBattery(msg); }, batteryOptions);
  estimationStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    "estimation_status", statusQos,
    [this](const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg) { onEstimationStatus(msg); });
  controlStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    "control_status", statusQos,
    [this](const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg) { onControlStatus(msg); });
  trajectoryStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    "trajectory_status", statusQos,
    [this](const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg) { onTrajectoryStatus(msg); });
  safetyStatusSub_ = this->create_subscription<peregrine_interfaces::msg::SafetyStatus>(
    "safety_status", statusQos,
    [this](const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg) { onSafetyStatus(msg); });

  uavStatePub_ = this->create_publisher<peregrine_interfaces::msg::UAVState>("uav_state", statusQos);

  armClient_ = this->create_client<peregrine_interfaces::srv::Arm>("arm", rmw_qos_profile_services_default, serviceCbGroup_);
  setModeClient_ = this->create_client<peregrine_interfaces::srv::SetMode>("set_mode", rmw_qos_profile_services_default, serviceCbGroup_);
  trajectoryGoToClient_ = rclcpp_action::create_client<GoTo>(this, "trajectory_manager/go_to", serviceCbGroup_);
  trajectoryExecuteClient_ = rclcpp_action::create_client<ExecuteTrajectory>(
    this, "trajectory_manager/execute_trajectory", serviceCbGroup_);

  statusTimer_ = this->create_wall_timer(periodFromHz(statusRateHz_), [this]() { publishUavState(); });
  statusTimer_->cancel();

  {
    std::scoped_lock lock(mutex_);
    latestState_.reset();
    latestPx4Status_.reset();
    latestEstimationStatus_.reset();
    latestControlStatus_.reset();
    latestTrajectoryStatus_.reset();
    latestSafetyLevel_ = 0;
    lastTransitionReason_ = "CONFIGURED";
  }

  estimatedStateFresh_.store(false, std::memory_order_release);
  px4StatusFresh_.store(false, std::memory_order_release);
  batteryFresh_.store(false, std::memory_order_release);
  estimationManagerReady_.store(false, std::memory_order_release);
  controlManagerReady_.store(false, std::memory_order_release);
  trajectoryManagerReady_.store(false, std::memory_order_release);
  safetyReady_.store(!requireExternalSafety_, std::memory_order_release);
  emergency_.store(false, std::memory_order_release);
  actionSlotReserved_.store(false, std::memory_order_release);
  activeAction_.store(ActionKind::None, std::memory_order_release);

  configured_.store(true, std::memory_order_release);
  active_.store(false, std::memory_order_release);
  RCLCPP_INFO(this->get_logger(), "Configured uav_manager lifecycle");
  return CallbackReturn::SUCCESS;
}

bool UavManagerNode::createActionServers()
{
  if (takeoffServer_ || landServer_ || goToServer_ || executeServer_) {
    return true;
  }
  if (!actionCbGroup_) {
    RCLCPP_ERROR(this->get_logger(), "Action callback group not initialized");
    return false;
  }

  try {
    rcl_action_server_options_t actionOpts = rcl_action_server_get_default_options();
    takeoffServer_ = rclcpp_action::create_server<Takeoff>(
      this, "~/takeoff",
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Takeoff::Goal> goal) {
        return onTakeoffGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleTakeoff> goalHandle) { return onTakeoffCancel(goalHandle); },
      [this](const std::shared_ptr<GoalHandleTakeoff> goalHandle) { onTakeoffAccepted(goalHandle); },
      actionOpts, actionCbGroup_);

    landServer_ = rclcpp_action::create_server<Land>(
      this, "~/land",
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Land::Goal> goal) {
        return onLandGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleLand> goalHandle) { return onLandCancel(goalHandle); },
      [this](const std::shared_ptr<GoalHandleLand> goalHandle) { onLandAccepted(goalHandle); },
      actionOpts, actionCbGroup_);

    goToServer_ = rclcpp_action::create_server<GoTo>(
      this, "~/go_to",
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoTo::Goal> goal) {
        return onGoToGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleGoTo> goalHandle) { return onGoToCancel(goalHandle); },
      [this](const std::shared_ptr<GoalHandleGoTo> goalHandle) { onGoToAccepted(goalHandle); },
      actionOpts, actionCbGroup_);

    executeServer_ = rclcpp_action::create_server<ExecuteTrajectory>(
      this, "~/execute_trajectory",
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteTrajectory::Goal> goal) {
        return onExecuteGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle) { return onExecuteCancel(goalHandle); },
      [this](const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle) { onExecuteAccepted(goalHandle); },
      actionOpts, actionCbGroup_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create action servers: %s", e.what());
    destroyActionServers();
    return false;
  }

  return true;
}

void UavManagerNode::destroyActionServers()
{
  takeoffServer_.reset();
  landServer_.reset();
  goToServer_.reset();
  executeServer_.reset();
}

UavManagerNode::CallbackReturn UavManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  if (!configured_.load(std::memory_order_acquire) || !uavStatePub_ || !statusTimer_) {
    return CallbackReturn::FAILURE;
  }

  if (!createActionServers()) {
    return CallbackReturn::FAILURE;
  }
  active_.store(true, std::memory_order_release);

  uavStatePub_->on_activate();
  statusTimer_->reset();
  RCLCPP_INFO(this->get_logger(), "Activated uav_manager");
  return CallbackReturn::SUCCESS;
}

UavManagerNode::CallbackReturn UavManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  destroyActionServers();
  if (statusTimer_) {
    statusTimer_->cancel();
  }
  if (uavStatePub_) {
    uavStatePub_->on_deactivate();
  }

  {
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = "DEACTIVATED";
  }
  activeAction_.store(ActionKind::None, std::memory_order_release);
  statusCv_.notify_all();
  RCLCPP_INFO(this->get_logger(), "Deactivated uav_manager");
  return CallbackReturn::SUCCESS;
}

UavManagerNode::CallbackReturn UavManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  configured_.store(false, std::memory_order_release);

  statusTimer_.reset();
  estimatedStateSub_.reset();
  px4StatusSub_.reset();
  batterySub_.reset();
  estimationStatusSub_.reset();
  controlStatusSub_.reset();
  trajectoryStatusSub_.reset();
  safetyStatusSub_.reset();
  uavStatePub_.reset();
  armClient_.reset();
  setModeClient_.reset();
  trajectoryGoToClient_.reset();
  trajectoryExecuteClient_.reset();
  destroyActionServers();
  actionCbGroup_.reset();
  serviceCbGroup_.reset();

  {
    std::scoped_lock lock(mutex_);
    latestState_.reset();
    latestPx4Status_.reset();
    latestEstimationStatus_.reset();
    latestControlStatus_.reset();
    latestTrajectoryStatus_.reset();
    latestSafetyLevel_ = 0;
    lastTransitionReason_ = "CLEANUP";
  }

  estimatedStateFresh_.store(false, std::memory_order_release);
  px4StatusFresh_.store(false, std::memory_order_release);
  batteryFresh_.store(false, std::memory_order_release);
  estimationManagerReady_.store(false, std::memory_order_release);
  controlManagerReady_.store(false, std::memory_order_release);
  trajectoryManagerReady_.store(false, std::memory_order_release);
  safetyReady_.store(!requireExternalSafety_, std::memory_order_release);
  emergency_.store(false, std::memory_order_release);
  actionSlotReserved_.store(false, std::memory_order_release);
  activeAction_.store(ActionKind::None, std::memory_order_release);
  statusCv_.notify_all();

  RCLCPP_INFO(this->get_logger(), "Cleaned up uav_manager");
  return CallbackReturn::SUCCESS;
}

UavManagerNode::CallbackReturn UavManagerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  (void)on_cleanup(this->get_current_state());
  RCLCPP_INFO(this->get_logger(), "Shut down uav_manager");
  return CallbackReturn::SUCCESS;
}

UavManagerNode::CallbackReturn UavManagerNode::on_error(const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  destroyActionServers();
  if (statusTimer_) {
    statusTimer_->cancel();
  }
  if (uavStatePub_ && uavStatePub_->is_activated()) {
    uavStatePub_->on_deactivate();
  }
  RCLCPP_ERROR(this->get_logger(), "Error in uav_manager lifecycle");
  return CallbackReturn::SUCCESS;
}

void UavManagerNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestState_ = *msg;
  }
  estimatedStateFresh_.store(true, std::memory_order_release);
  statusCv_.notify_all();
}

void UavManagerNode::onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestPx4Status_ = *msg;
  }

  px4StatusFresh_.store(true, std::memory_order_release);
  if (msg->failsafe) {
    emergency_.store(true, std::memory_order_release);
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = "PX4_FAILSAFE";
  } else if (!msg->armed && latestSafetyLevel_ <= peregrine_interfaces::msg::SafetyStatus::LEVEL_WARNING) {
    emergency_.store(false, std::memory_order_release);
  }
  statusCv_.notify_all();
}

void UavManagerNode::onBattery(const sensor_msgs::msg::BatteryState::SharedPtr)
{
  batteryFresh_.store(true, std::memory_order_release);
}

void UavManagerNode::onEstimationStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestEstimationStatus_ = *msg;
  }
  estimationManagerReady_.store(msg->active && msg->healthy, std::memory_order_release);
  statusCv_.notify_all();
}

void UavManagerNode::onControlStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestControlStatus_ = *msg;
  }
  controlManagerReady_.store(msg->active && msg->healthy, std::memory_order_release);
  statusCv_.notify_all();
}

void UavManagerNode::onTrajectoryStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestTrajectoryStatus_ = *msg;
  }
  trajectoryManagerReady_.store(msg->active && msg->healthy, std::memory_order_release);
  statusCv_.notify_all();
}

void UavManagerNode::onSafetyStatus(const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestSafetyLevel_ = msg->level;
  }

  const bool ready = msg->level <= peregrine_interfaces::msg::SafetyStatus::LEVEL_WARNING;
  safetyReady_.store(ready, std::memory_order_release);

  if (msg->level >= peregrine_interfaces::msg::SafetyStatus::LEVEL_EMERGENCY) {
    emergency_.store(true, std::memory_order_release);
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = "SAFETY_EMERGENCY";
  }
}

void UavManagerNode::publishUavState()
{
  if (!active_.load(std::memory_order_acquire) || !uavStatePub_ || !uavStatePub_->is_activated()) {
    return;
  }

  peregrine_interfaces::msg::UAVState output;
  output.header.stamp = this->now();

  std::optional<peregrine_interfaces::msg::PX4Status> px4Status;
  bool hasState = false;
  {
    std::scoped_lock lock(mutex_);
    px4Status = latestPx4Status_;
    hasState = latestState_.has_value();
    output.detail = lastTransitionReason_;
  }

  const bool emergencyNow = isEmergency();
  output.state = toUavStateCode(activeAction_.load(std::memory_order_acquire), emergencyNow, px4Status);
  output.mode = px4Status.has_value() ? px4ModeString(*px4Status) : "PX4_UNKNOWN";

  if (px4Status.has_value()) {
    output.armed = px4Status->armed;
    output.offboard = px4Status->offboard;
    output.connected = px4Status->connected;
    output.failsafe = px4Status->failsafe;
  } else {
    output.armed = false;
    output.offboard = false;
    output.connected = false;
    output.failsafe = false;
  }

  const bool px4Ready = px4StatusFresh_.load(std::memory_order_acquire) && px4Status.has_value() &&
                        px4Status->connected && !px4Status->failsafe;
  const bool estimatedStateReady = estimatedStateFresh_.load(std::memory_order_acquire) && hasState;
  const bool estimationReady = estimationManagerReady_.load(std::memory_order_acquire);
  const bool controlReady = controlManagerReady_.load(std::memory_order_acquire);
  const bool trajectoryReady = trajectoryManagerReady_.load(std::memory_order_acquire);
  const bool safetyReady = !requireExternalSafety_ || safetyReady_.load(std::memory_order_acquire);
  const bool batteryReady = batteryFresh_.load(std::memory_order_acquire);

  output.px4_ready = px4Ready;
  output.estimated_state_ready = estimatedStateReady;
  output.estimation_ready = estimationReady;
  output.control_ready = controlReady;
  output.trajectory_ready = trajectoryReady;
  output.safety_ready = safetyReady;
  output.dependencies_ready = px4Ready && estimatedStateReady && batteryReady &&
                              estimationReady && controlReady && trajectoryReady && safetyReady;
  output.readiness_detail = dependenciesReason();

  uavStatePub_->publish(output);
}

rclcpp_action::GoalResponse UavManagerNode::onTakeoffGoal(
  const rclcpp_action::GoalUUID &, const std::shared_ptr<const Takeoff::Goal> goal)
{
  if (!active_.load(std::memory_order_acquire) || isEmergency() || goal->target_altitude_m <= 0.0) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!reserveActionSlot()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UavManagerNode::onTakeoffCancel(const std::shared_ptr<GoalHandleTakeoff>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UavManagerNode::onTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goalHandle)
{
  ActionSlotGuard slotGuard(this);
  activeAction_.store(ActionKind::Takeoff, std::memory_order_release);
  {
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = "TAKEOFF_ACTIVE";
  }

  auto result = std::make_shared<Takeoff::Result>();
  const auto preempted = [this, goalHandle]() {
    return !active_.load(std::memory_order_acquire) || goalHandle->is_canceling();
  };
  const auto emergencyCb = [this]() { return isEmergency(); };

  auto failGoal = [&](const std::string & reason) {
    result->success = false;
    result->message = reason;
    result->final_altitude_m = latestAltitudeM();
    if (reason == "GOAL_PREEMPTED") {
      goalHandle->canceled(result);
    } else {
      goalHandle->abort(result);
    }
    activeAction_.store(ActionKind::None, std::memory_order_release);
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = reason;
  };

  if (emergencyCb()) {
    failGoal("EMERGENCY_PREEMPT");
    return;
  }
  if (preempted()) {
    failGoal("GOAL_PREEMPTED");
    return;
  }
  if (!dependenciesReady()) {
    failGoal(dependenciesReason());
    return;
  }

  bool alreadyArmed = false;
  {
    std::scoped_lock lock(mutex_);
    alreadyArmed = latestPx4Status_.has_value() && latestPx4Status_->armed;
  }

  if (!alreadyArmed) {
    StepResult step = ensureArmableMode(preempted);
    if (!step.success) {
      failGoal(step.describe());
      return;
    }
    step = callArmService(true, preempted);
    if (!step.success) {
      failGoal(step.describe());
      return;
    }
    step = waitForArmed(true, secondsToMillis(armedWaitS_), preempted);
    if (!step.success) {
      failGoal(step.describe());
      return;
    }
  }

  StepResult step = waitForControlSetpointFlow(4s, preempted);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  step = callSetModeService("offboard", preempted);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  step = waitForOffboard(true, secondsToMillis(offboardWaitS_), preempted);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  ExecuteTrajectory::Goal executeGoal;
  executeGoal.trajectory_type = "takeoff";
  executeGoal.params = {goalHandle->get_goal()->target_altitude_m, goalHandle->get_goal()->climb_velocity_mps};

  ExecuteTrajectory::Result executeResult;
  step = forwardExecuteTrajectory(
    executeGoal,
    [this, goalHandle](const ExecuteTrajectory::Feedback & feedback) {
      auto fb = std::make_shared<Takeoff::Feedback>();
      fb->progress = feedback.progress;
      fb->current_altitude_m = latestAltitudeM();
      goalHandle->publish_feedback(fb);
    },
    preempted, emergencyCb, &executeResult);

  if (!step.success || !executeResult.success) {
    failGoal(step.success ? executeResult.message : step.describe());
    return;
  }

  result->success = true;
  result->message = "TAKEOFF_COMPLETE";
  result->final_altitude_m = latestAltitudeM();
  goalHandle->succeed(result);

  activeAction_.store(ActionKind::None, std::memory_order_release);
  {
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = "TAKEOFF_COMPLETE";
  }
}

rclcpp_action::GoalResponse UavManagerNode::onLandGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Land::Goal>)
{
  if (!active_.load(std::memory_order_acquire) || isEmergency()) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::optional<peregrine_interfaces::msg::PX4Status> status;
  {
    std::scoped_lock lock(mutex_);
    status = latestPx4Status_;
  }
  if (!status.has_value() || !status->armed) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!reserveActionSlot()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UavManagerNode::onLandCancel(const std::shared_ptr<GoalHandleLand>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UavManagerNode::onLandAccepted(const std::shared_ptr<GoalHandleLand> goalHandle)
{
  ActionSlotGuard slotGuard(this);
  activeAction_.store(ActionKind::Land, std::memory_order_release);
  {
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = "LAND_ACTIVE";
  }

  auto result = std::make_shared<Land::Result>();
  const auto preempted = [this, goalHandle]() {
    return !active_.load(std::memory_order_acquire) || goalHandle->is_canceling();
  };
  const auto emergencyCb = [this]() { return isEmergency(); };

  auto failGoal = [&](const std::string & reason) {
    result->success = false;
    result->message = reason;
    if (reason == "GOAL_PREEMPTED") {
      goalHandle->canceled(result);
    } else {
      goalHandle->abort(result);
    }
    activeAction_.store(ActionKind::None, std::memory_order_release);
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = reason;
  };

  if (emergencyCb()) {
    failGoal("EMERGENCY_PREEMPT");
    return;
  }
  if (preempted()) {
    failGoal("GOAL_PREEMPTED");
    return;
  }

  StepResult step = callSetModeService("land", preempted);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  step = waitForOffboard(false, 5s, preempted);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  step = waitForArmed(false, 90s, preempted);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  result->success = true;
  result->message = "LAND_COMPLETE";
  goalHandle->succeed(result);

  activeAction_.store(ActionKind::None, std::memory_order_release);
  {
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = "LAND_COMPLETE";
  }
}

rclcpp_action::GoalResponse UavManagerNode::onGoToGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const GoTo::Goal> goal)
{
  if (!active_.load(std::memory_order_acquire) || isEmergency() || goal->acceptance_radius_m <= 0.0) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!reserveActionSlot()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UavManagerNode::onGoToCancel(const std::shared_ptr<GoalHandleGoTo>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UavManagerNode::onGoToAccepted(const std::shared_ptr<GoalHandleGoTo> goalHandle)
{
  ActionSlotGuard slotGuard(this);
  activeAction_.store(ActionKind::GoTo, std::memory_order_release);
  {
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = "GO_TO_ACTIVE";
  }

  auto result = std::make_shared<GoTo::Result>();
  const auto preempted = [this, goalHandle]() {
    return !active_.load(std::memory_order_acquire) || goalHandle->is_canceling();
  };
  const auto emergencyCb = [this]() { return isEmergency(); };

  auto failGoal = [&](const std::string & reason) {
    result->success = false;
    result->message = reason;
    result->final_position = latestPosition();
    if (reason == "GOAL_PREEMPTED") {
      goalHandle->canceled(result);
    } else {
      goalHandle->abort(result);
    }
    activeAction_.store(ActionKind::None, std::memory_order_release);
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = reason;
  };

  if (emergencyCb()) {
    failGoal("EMERGENCY_PREEMPT");
    return;
  }
  if (preempted()) {
    failGoal("GOAL_PREEMPTED");
    return;
  }
  if (!dependenciesReady()) {
    failGoal(dependenciesReason());
    return;
  }

  StepResult step = callSetModeService("offboard", preempted);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }
  step = waitForOffboard(true, secondsToMillis(offboardWaitS_), preempted);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  GoTo::Result forwardResult;
  step = forwardGoTo(
    *goalHandle->get_goal(),
    [goalHandle](const GoTo::Feedback & feedback) {
      auto fb = std::make_shared<GoTo::Feedback>();
      *fb = feedback;
      goalHandle->publish_feedback(fb);
    },
    preempted, emergencyCb, &forwardResult);

  if (!step.success || !forwardResult.success) {
    failGoal(step.success ? forwardResult.message : step.describe());
    return;
  }

  *result = forwardResult;
  goalHandle->succeed(result);
  activeAction_.store(ActionKind::None, std::memory_order_release);
  {
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = "GO_TO_COMPLETE";
  }
}

rclcpp_action::GoalResponse UavManagerNode::onExecuteGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTrajectory::Goal> goal)
{
  if (!active_.load(std::memory_order_acquire) || isEmergency() || goal->trajectory_type.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!reserveActionSlot()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UavManagerNode::onExecuteCancel(
  const std::shared_ptr<GoalHandleExecuteTrajectory>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UavManagerNode::onExecuteAccepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle)
{
  ActionSlotGuard slotGuard(this);
  activeAction_.store(ActionKind::ExecuteTrajectory, std::memory_order_release);
  {
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = "EXECUTE_ACTIVE";
  }

  auto result = std::make_shared<ExecuteTrajectory::Result>();
  const auto preempted = [this, goalHandle]() {
    return !active_.load(std::memory_order_acquire) || goalHandle->is_canceling();
  };
  const auto emergencyCb = [this]() { return isEmergency(); };

  auto failGoal = [&](const std::string & reason) {
    result->success = false;
    result->message = reason;
    if (reason == "GOAL_PREEMPTED") {
      goalHandle->canceled(result);
    } else {
      goalHandle->abort(result);
    }
    activeAction_.store(ActionKind::None, std::memory_order_release);
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = reason;
  };

  if (emergencyCb()) {
    failGoal("EMERGENCY_PREEMPT");
    return;
  }
  if (preempted()) {
    failGoal("GOAL_PREEMPTED");
    return;
  }
  if (!dependenciesReady()) {
    failGoal(dependenciesReason());
    return;
  }

  StepResult step = callSetModeService("offboard", preempted);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }
  step = waitForOffboard(true, secondsToMillis(offboardWaitS_), preempted);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  ExecuteTrajectory::Result forwardResult;
  step = forwardExecuteTrajectory(
    *goalHandle->get_goal(),
    [goalHandle](const ExecuteTrajectory::Feedback & feedback) {
      auto fb = std::make_shared<ExecuteTrajectory::Feedback>();
      *fb = feedback;
      goalHandle->publish_feedback(fb);
    },
    preempted, emergencyCb, &forwardResult);

  if (!step.success || !forwardResult.success) {
    failGoal(step.success ? forwardResult.message : step.describe());
    return;
  }

  *result = forwardResult;
  goalHandle->succeed(result);
  activeAction_.store(ActionKind::None, std::memory_order_release);
  {
    std::scoped_lock lock(mutex_);
    lastTransitionReason_ = "EXECUTE_COMPLETE";
  }
}

bool UavManagerNode::reserveActionSlot()
{
  bool expected = false;
  return actionSlotReserved_.compare_exchange_strong(expected, true, std::memory_order_acq_rel);
}

void UavManagerNode::releaseActionSlot()
{
  actionSlotReserved_.store(false, std::memory_order_release);
}

bool UavManagerNode::isEmergency() const
{
  return emergency_.load(std::memory_order_acquire);
}

bool UavManagerNode::dependenciesReady() const
{
  std::optional<peregrine_interfaces::msg::PX4Status> px4;
  bool hasState = false;
  {
    std::scoped_lock lock(mutex_);
    px4 = latestPx4Status_;
    hasState = latestState_.has_value();
  }

  const bool px4Ready = px4StatusFresh_.load(std::memory_order_acquire) && px4.has_value() &&
                        px4->connected && !px4->failsafe;
  const bool estimatedStateReady = estimatedStateFresh_.load(std::memory_order_acquire) && hasState;
  const bool batteryReady = batteryFresh_.load(std::memory_order_acquire);
  const bool estimationReady = estimationManagerReady_.load(std::memory_order_acquire);
  const bool controlReady = controlManagerReady_.load(std::memory_order_acquire);
  const bool trajectoryReady = trajectoryManagerReady_.load(std::memory_order_acquire);
  const bool safetyReady = !requireExternalSafety_ || safetyReady_.load(std::memory_order_acquire);

  return px4Ready && estimatedStateReady && batteryReady &&
         estimationReady && controlReady && trajectoryReady && safetyReady;
}

std::string UavManagerNode::dependenciesReason() const
{
  std::optional<peregrine_interfaces::msg::PX4Status> px4;
  bool hasState = false;
  {
    std::scoped_lock lock(mutex_);
    px4 = latestPx4Status_;
    hasState = latestState_.has_value();
  }

  const bool px4Ready = px4StatusFresh_.load(std::memory_order_acquire) && px4.has_value() &&
                        px4->connected && !px4->failsafe;
  const bool estimatedStateReady = estimatedStateFresh_.load(std::memory_order_acquire) && hasState;
  const bool batteryReady = batteryFresh_.load(std::memory_order_acquire);
  const bool estimationReady = estimationManagerReady_.load(std::memory_order_acquire);
  const bool controlReady = controlManagerReady_.load(std::memory_order_acquire);
  const bool trajectoryReady = trajectoryManagerReady_.load(std::memory_order_acquire);
  const bool safetyReady = !requireExternalSafety_ || safetyReady_.load(std::memory_order_acquire);

  return "px4=" + std::string(px4Ready ? "OK" : "BAD") +
         " est_state=" + std::string(estimatedStateReady ? "OK" : "BAD") +
         " battery=" + std::string(batteryReady ? "OK" : "BAD") +
         " est_mgr=" + std::string(estimationReady ? "OK" : "BAD") +
         " ctrl_mgr=" + std::string(controlReady ? "OK" : "BAD") +
         " traj_mgr=" + std::string(trajectoryReady ? "OK" : "BAD") +
         " safety=" + std::string(safetyReady ? "OK" : "BAD");
}

StepResult UavManagerNode::ensureArmableMode(const std::function<bool()> & preempted)
{
  std::optional<peregrine_interfaces::msg::PX4Status> status;
  {
    std::scoped_lock lock(mutex_);
    status = latestPx4Status_;
  }
  if (!status.has_value()) {
    return StepResult::fail(StepCode::Px4StatusMissing);
  }
  if (!status->connected) {
    return StepResult::fail(StepCode::Px4Disconnected);
  }
  if (status->armed || !navStatePreventsArming(status->nav_state)) {
    return StepResult::ok();
  }

  StepResult step = callSetModeService("position", preempted);
  if (!step.success) {
    return StepResult::fail(StepCode::SetPositionFailed, step.describe());
  }
  step = waitForNavState(kNavStatePosCtl, 6s, preempted);
  if (!step.success) {
    return StepResult::fail(StepCode::PositionModeTimeout, step.describe());
  }
  return StepResult::ok();
}

StepResult UavManagerNode::callArmService(const bool arm, const std::function<bool()> & preempted)
{
  if (!armClient_) {
    return StepResult::fail(StepCode::ArmServiceUnavailable);
  }

  const auto serviceDeadline = std::chrono::steady_clock::now() + secondsToMillis(serviceWaitS_);
  while (std::chrono::steady_clock::now() < serviceDeadline) {
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (isEmergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (armClient_->wait_for_service(200ms)) {
      break;
    }
  }
  if (!armClient_->service_is_ready()) {
    return StepResult::fail(StepCode::ArmServiceUnavailable);
  }

  auto request = std::make_shared<peregrine_interfaces::srv::Arm::Request>();
  request->arm = arm;
  auto future = armClient_->async_send_request(request);

  const auto responseDeadline = std::chrono::steady_clock::now() + secondsToMillis(serviceResponseWaitS_);
  while (future.wait_for(50ms) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= responseDeadline) {
      return StepResult::fail(StepCode::ArmServiceTimeout);
    }
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (isEmergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
  }

  const auto response = future.get();
  using ArmResponse = peregrine_interfaces::srv::Arm::Response;
  if (response->result_code == ArmResponse::RESULT_ACCEPTED || response->success) {
    return StepResult::ok();
  }
  if (response->result_code == ArmResponse::RESULT_TIMEOUT) {
    return StepResult::fail(StepCode::ArmServiceTimeout, response->message);
  }
  return StepResult::fail(StepCode::ArmServiceRejected, response->message);
}

StepResult UavManagerNode::callSetModeService(const std::string & mode, const std::function<bool()> & preempted)
{
  if (!setModeClient_) {
    return StepResult::fail(StepCode::SetModeServiceUnavailable);
  }

  const auto serviceDeadline = std::chrono::steady_clock::now() + secondsToMillis(serviceWaitS_);
  while (std::chrono::steady_clock::now() < serviceDeadline) {
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (isEmergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (setModeClient_->wait_for_service(200ms)) {
      break;
    }
  }
  if (!setModeClient_->service_is_ready()) {
    return StepResult::fail(StepCode::SetModeServiceUnavailable);
  }

  auto request = std::make_shared<peregrine_interfaces::srv::SetMode::Request>();
  request->mode = mode;
  auto future = setModeClient_->async_send_request(request);

  const auto responseDeadline = std::chrono::steady_clock::now() + secondsToMillis(serviceResponseWaitS_);
  while (future.wait_for(50ms) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= responseDeadline) {
      return StepResult::fail(StepCode::SetModeServiceTimeout);
    }
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (isEmergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
  }

  const auto response = future.get();
  using SetModeResponse = peregrine_interfaces::srv::SetMode::Response;
  if (response->result_code == SetModeResponse::RESULT_ACCEPTED || response->success) {
    return StepResult::ok();
  }
  if (response->result_code == SetModeResponse::RESULT_TIMEOUT) {
    return StepResult::fail(StepCode::SetModeServiceTimeout, response->message);
  }
  return StepResult::fail(StepCode::SetModeRejected, response->message);
}

StepResult UavManagerNode::waitForArmed(
  const bool armed,
  const std::chrono::milliseconds timeout,
  const std::function<bool()> & preempted) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (isEmergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (latestPx4Status_.has_value() && latestPx4Status_->armed == armed) {
      return StepResult::ok();
    }
    if (statusCv_.wait_until(lock, deadline) == std::cv_status::timeout) {
      return StepResult::failCustom("ARMED_TIMEOUT");
    }
  }
}

StepResult UavManagerNode::waitForOffboard(
  const bool offboard,
  const std::chrono::milliseconds timeout,
  const std::function<bool()> & preempted) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (isEmergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (latestPx4Status_.has_value() && latestPx4Status_->offboard == offboard) {
      return StepResult::ok();
    }
    if (statusCv_.wait_until(lock, deadline) == std::cv_status::timeout) {
      return StepResult::failCustom("OFFBOARD_TIMEOUT");
    }
  }
}

StepResult UavManagerNode::waitForNavState(
  const uint8_t navState,
  const std::chrono::milliseconds timeout,
  const std::function<bool()> & preempted) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (isEmergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (latestPx4Status_.has_value() && latestPx4Status_->nav_state == navState) {
      return StepResult::ok();
    }
    if (statusCv_.wait_until(lock, deadline) == std::cv_status::timeout) {
      return StepResult::failCustom("NAV_STATE_TIMEOUT");
    }
  }
}

StepResult UavManagerNode::waitForControlSetpointFlow(
  const std::chrono::milliseconds timeout,
  const std::function<bool()> & preempted) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (isEmergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (latestControlStatus_.has_value() && latestControlStatus_->active && latestControlStatus_->healthy) {
      return StepResult::ok();
    }
    if (statusCv_.wait_until(lock, deadline) == std::cv_status::timeout) {
      return StepResult::fail(StepCode::ControlSetpointTimeout);
    }
  }
}

StepResult UavManagerNode::forwardExecuteTrajectory(
  const ExecuteTrajectory::Goal & goal,
  std::function<void(const ExecuteTrajectory::Feedback &)> feedbackCallback,
  const std::function<bool()> & preempted,
  const std::function<bool()> & emergencyCb,
  ExecuteTrajectory::Result * resultOut) const
{
  const auto serverDeadline = std::chrono::steady_clock::now() + secondsToMillis(actionServerWaitS_);
  while (std::chrono::steady_clock::now() < serverDeadline) {
    if (emergencyCb()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (trajectoryExecuteClient_ && trajectoryExecuteClient_->wait_for_action_server(200ms)) {
      break;
    }
  }

  if (!trajectoryExecuteClient_ || !trajectoryExecuteClient_->action_server_is_ready()) {
    return StepResult::fail(StepCode::TrajectoryExecuteServerUnavailable);
  }

  rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions options;
  options.feedback_callback =
    [feedbackCallback = std::move(feedbackCallback)](
    auto, const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback) {
      if (feedbackCallback && feedback) {
        feedbackCallback(*feedback);
      }
    };

  auto goalHandleFuture = trajectoryExecuteClient_->async_send_goal(goal, options);
  const auto goalDeadline = std::chrono::steady_clock::now() + secondsToMillis(serviceResponseWaitS_);
  while (goalHandleFuture.wait_for(50ms) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= goalDeadline) {
      return StepResult::fail(StepCode::TrajectoryExecuteGoalTimeout);
    }
    if (emergencyCb()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
  }

  auto goalHandle = goalHandleFuture.get();
  if (!goalHandle) {
    return StepResult::fail(StepCode::TrajectoryExecuteGoalRejected);
  }

  auto resultFuture = trajectoryExecuteClient_->async_get_result(goalHandle);
  const auto resultDeadline = std::chrono::steady_clock::now() + secondsToMillis(actionResultWaitS_);
  while (resultFuture.wait_for(50ms) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= resultDeadline) {
      return StepResult::fail(StepCode::TrajectoryExecuteResultTimeout);
    }
    if (emergencyCb()) {
      trajectoryExecuteClient_->async_cancel_goal(goalHandle);
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (preempted()) {
      trajectoryExecuteClient_->async_cancel_goal(goalHandle);
      return StepResult::fail(StepCode::GoalPreempted);
    }
  }

  const auto wrappedResult = resultFuture.get();
  if (resultOut) {
    *resultOut = *wrappedResult.result;
  }
  if (wrappedResult.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return StepResult::fail(StepCode::TrajectoryExecuteResultNotSucceeded);
  }
  return StepResult::ok();
}

StepResult UavManagerNode::forwardGoTo(
  const GoTo::Goal & goal,
  std::function<void(const GoTo::Feedback &)> feedbackCallback,
  const std::function<bool()> & preempted,
  const std::function<bool()> & emergencyCb,
  GoTo::Result * resultOut) const
{
  const auto serverDeadline = std::chrono::steady_clock::now() + secondsToMillis(actionServerWaitS_);
  while (std::chrono::steady_clock::now() < serverDeadline) {
    if (emergencyCb()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (trajectoryGoToClient_ && trajectoryGoToClient_->wait_for_action_server(200ms)) {
      break;
    }
  }

  if (!trajectoryGoToClient_ || !trajectoryGoToClient_->action_server_is_ready()) {
    return StepResult::fail(StepCode::TrajectoryGotoServerUnavailable);
  }

  rclcpp_action::Client<GoTo>::SendGoalOptions options;
  options.feedback_callback =
    [feedbackCallback = std::move(feedbackCallback)](
    auto, const std::shared_ptr<const GoTo::Feedback> feedback) {
      if (feedbackCallback && feedback) {
        feedbackCallback(*feedback);
      }
    };

  auto goalHandleFuture = trajectoryGoToClient_->async_send_goal(goal, options);
  const auto goalDeadline = std::chrono::steady_clock::now() + secondsToMillis(serviceResponseWaitS_);
  while (goalHandleFuture.wait_for(50ms) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= goalDeadline) {
      return StepResult::fail(StepCode::TrajectoryGotoGoalTimeout);
    }
    if (emergencyCb()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
  }

  auto goalHandle = goalHandleFuture.get();
  if (!goalHandle) {
    return StepResult::fail(StepCode::TrajectoryGotoGoalRejected);
  }

  auto resultFuture = trajectoryGoToClient_->async_get_result(goalHandle);
  const auto resultDeadline = std::chrono::steady_clock::now() + secondsToMillis(actionResultWaitS_);
  while (resultFuture.wait_for(50ms) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= resultDeadline) {
      return StepResult::fail(StepCode::TrajectoryGotoResultTimeout);
    }
    if (emergencyCb()) {
      trajectoryGoToClient_->async_cancel_goal(goalHandle);
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (preempted()) {
      trajectoryGoToClient_->async_cancel_goal(goalHandle);
      return StepResult::fail(StepCode::GoalPreempted);
    }
  }

  const auto wrappedResult = resultFuture.get();
  if (resultOut) {
    *resultOut = *wrappedResult.result;
  }
  if (wrappedResult.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return StepResult::fail(StepCode::TrajectoryGotoResultNotSucceeded);
  }
  return StepResult::ok();
}

double UavManagerNode::latestAltitudeM() const
{
  std::scoped_lock lock(mutex_);
  if (!latestState_.has_value()) {
    return 0.0;
  }
  return latestState_->pose.pose.position.z;
}

geometry_msgs::msg::Point UavManagerNode::latestPosition() const
{
  std::scoped_lock lock(mutex_);
  if (!latestState_.has_value()) {
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
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));
}

uint8_t UavManagerNode::toUavStateCode(
  const ActionKind activeAction,
  const bool emergency,
  const std::optional<peregrine_interfaces::msg::PX4Status> & px4)
{
  if (emergency) {
    return peregrine_interfaces::msg::UAVState::STATE_EMERGENCY;
  }
  switch (activeAction) {
    case ActionKind::Takeoff:
      return peregrine_interfaces::msg::UAVState::STATE_TAKING_OFF;
    case ActionKind::Land:
      return peregrine_interfaces::msg::UAVState::STATE_LANDING;
    case ActionKind::GoTo:
    case ActionKind::ExecuteTrajectory:
      return peregrine_interfaces::msg::UAVState::STATE_FLYING;
    case ActionKind::None:
    default:
      break;
  }

  if (px4.has_value() && px4->armed) {
    if (px4->offboard) {
      return peregrine_interfaces::msg::UAVState::STATE_HOVERING;
    }
    return peregrine_interfaces::msg::UAVState::STATE_ARMED;
  }
  return peregrine_interfaces::msg::UAVState::STATE_IDLE;
}

std::chrono::steady_clock::time_point UavManagerNode::nowSteady() const
{
  return std::chrono::steady_clock::now();
}

}  // namespace uav_manager

RCLCPP_COMPONENTS_REGISTER_NODE(uav_manager::UavManagerNode)
