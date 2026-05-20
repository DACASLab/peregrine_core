#include <uav_manager/uav_manager_node.hpp>

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
constexpr uint8_t kNavStateAltCtl = 1;
constexpr uint8_t kNavStatePosCtl = 2;
constexpr uint8_t kNavStateAutoMission = 3;
constexpr uint8_t kNavStateAutoLoiter = 4;
constexpr uint8_t kNavStateAutoRtl = 5;
constexpr uint8_t kNavStateAcro = 10;
constexpr uint8_t kNavStateDescend = 12;
constexpr uint8_t kNavStateTermination = 13;
constexpr uint8_t kNavStateOffboard = 14;
constexpr uint8_t kNavStateStab = 15;
constexpr uint8_t kNavStateAutoTakeoff = 17;
constexpr uint8_t kNavStateAutoLand = 18;
constexpr uint8_t kNavStateAutoFollowTarget = 19;
constexpr uint8_t kNavStateAutoPrecland = 20;
constexpr uint8_t kNavStateOrbit = 21;
constexpr uint8_t kNavStateAutoVtolTakeoff = 22;

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
    case kNavStateAltCtl:
      return "ALTCTL";
    case kNavStatePosCtl:
      return "POSCTL";
    case kNavStateAutoMission:
      return "AUTO_MISSION";
    case kNavStateAutoLoiter:
      return "AUTO_LOITER";
    case kNavStateAutoRtl:
      return "AUTO_RTL";
    case kNavStateAcro:
      return "ACRO";
    case kNavStateDescend:
      return "DESCEND";
    case kNavStateTermination:
      return "TERMINATION";
    case kNavStateStab:
      return "STAB";
    case kNavStateAutoTakeoff:
      return "AUTO_TAKEOFF";
    case kNavStateAutoLand:
      return "AUTO_LAND";
    case kNavStateAutoFollowTarget:
      return "AUTO_FOLLOW_TARGET";
    case kNavStateAutoPrecland:
      return "AUTO_PRECLAND";
    case kNavStateOrbit:
      return "ORBIT";
    case kNavStateAutoVtolTakeoff:
      return "AUTO_VTOL_TAKEOFF";
    default:
      return "NAV_" + std::to_string(status.nav_state);
  }
}

std::chrono::milliseconds secondsToMillis(const double seconds)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(
      seconds));
}

}  // namespace

UavManagerNode::UavManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kNodeName, options)
{
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 5.0);

  freshnessConfig_.px4StatusTimeout =
    secondsToMillis(this->declare_parameter<double>("px4_status_timeout_s", 1.0));
  freshnessConfig_.managerStatusTimeout =
    secondsToMillis(this->declare_parameter<double>("manager_status_timeout_s", 1.0));
  freshnessConfig_.estimatedStateTimeout =
    secondsToMillis(this->declare_parameter<double>("estimated_state_timeout_s", 1.0));
  freshnessConfig_.safetyStatusTimeout =
    secondsToMillis(this->declare_parameter<double>("safety_status_timeout_s", 2.0));
  requireExternalSafety_ = this->declare_parameter<bool>("require_external_safety", false);

  serviceTimeoutS_ = this->declare_parameter<double>("service_timeout_s", 3.0);

  autoStart_ = this->declare_parameter<bool>("auto_start", true);
  dataReadinessPollMs_ = this->declare_parameter<int>("data_readiness_poll_ms", 200);

  healthAggregator_ = std::make_unique<HealthAggregator>(freshnessConfig_);
  healthAggregator_->setRequireExternalSafety(requireExternalSafety_);

  if (autoStart_) {
    startupTimer_ = this->create_wall_timer(
      200ms,
      [this]() {
        startupTimer_->cancel();

        RCLCPP_INFO(get_logger(), "Auto-start: triggering configure");
        auto configResult = this->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        if (configResult.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          RCLCPP_ERROR(get_logger(), "Auto-configure failed (state=%s)",
            configResult.label().c_str());
          return;
        }

        RCLCPP_INFO(get_logger(), "Auto-start: waiting for data readiness");

        readinessTimer_ = this->create_wall_timer(
          std::chrono::milliseconds(dataReadinessPollMs_),
          [this]() {
            auto snap = healthAggregator_->snapshot(nowSteady());

            if (!snap.dependenciesReady()) {
              return;
            }

            RCLCPP_INFO(get_logger(), "Auto-start: data readiness satisfied");
            readinessTimer_->cancel();

            auto activateResult = this->trigger_transition(
              lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            if (activateResult.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
              RCLCPP_ERROR(get_logger(), "Auto-activate failed (state=%s)",
                activateResult.label().c_str());
              return;
            }
            RCLCPP_INFO(get_logger(), "Auto-start complete: ACTIVE");
          });
      });
  }
}

UavManagerNode::CallbackReturn UavManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  if (statusRateHz_ <= 0.0 || serviceTimeoutS_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "status_rate_hz and service_timeout_s must be > 0");
    return CallbackReturn::FAILURE;
  }

  actionCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  serviceCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    "estimated_state", qos,
    [this](peregrine_interfaces::msg::State::SharedPtr msg) { onEstimatedState(msg); });
  px4StatusSub_ = this->create_subscription<peregrine_interfaces::msg::PX4Status>(
    "status", statusQos, [this](peregrine_interfaces::msg::PX4Status::SharedPtr msg) { onPx4Status(msg); });
  estimationStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    "estimation_status", statusQos,
    [this](peregrine_interfaces::msg::ManagerStatus::SharedPtr msg) { onEstimationStatus(msg); });
  controlStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    "control_status", statusQos,
    [this](peregrine_interfaces::msg::ManagerStatus::SharedPtr msg) { onControlStatus(msg); });
  trajectoryStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    "trajectory_status", statusQos,
    [this](peregrine_interfaces::msg::ManagerStatus::SharedPtr msg) { onTrajectoryStatus(msg); });
  safetyStatusSub_ = this->create_subscription<peregrine_interfaces::msg::SafetyStatus>(
    "safety_status", statusQos,
    [this](peregrine_interfaces::msg::SafetyStatus::SharedPtr msg) { onSafetyStatus(msg); });

  uavStatePub_ =
    this->create_publisher<peregrine_interfaces::msg::UAVState>("uav_state", statusQos);

  armClient_ = this->create_client<peregrine_interfaces::srv::Arm>("arm", rmw_qos_profile_services_default, serviceCbGroup_);
  setModeClient_ = this->create_client<peregrine_interfaces::srv::SetMode>("set_mode", rmw_qos_profile_services_default, serviceCbGroup_);

  trajectoryExecuteClient_ = rclcpp_action::create_client<ExecuteTrajectory>(
    this,
    "trajectory_manager/execute_trajectory", serviceCbGroup_);

  clearEmergencyService_ = this->create_service<peregrine_interfaces::srv::ClearEmergency>(
    "~/clear_emergency",
    [this](
      const std::shared_ptr<peregrine_interfaces::srv::ClearEmergency::Request> request,
      std::shared_ptr<peregrine_interfaces::srv::ClearEmergency::Response> response) {
      onClearEmergency(request, response);
    });

  statusTimer_ =
    this->create_wall_timer(
    periodFromHz(statusRateHz_),
    [this]() { publishUavState(); });
  statusTimer_->cancel();

  {
    std::scoped_lock lock(mutex_);
    latestState_.reset();
    latestPx4Status_.reset();
    latestEstimationStatus_.reset();
    latestControlStatus_.reset();
    latestTrajectoryStatus_.reset();
    supervisor_ = SupervisorStateMachine();
    lastTransitionReason_ = "CONFIGURED";
    latestSafetyLevel_ = 0;
  }

  configured_ = true;
  active_ = false;

  RCLCPP_INFO(this->get_logger(), "Configured uav_manager lifecycle");
  return CallbackReturn::SUCCESS;
}

bool UavManagerNode::createActionServers()
{
  if (takeoffServer_ || landServer_) {
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
        return onTakeoffGoal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleTakeoff> goalHandle) {
        return onTakeoffCancel(goalHandle);
      },
      [this](const std::shared_ptr<GoalHandleTakeoff> goalHandle) { onTakeoffAccepted(goalHandle); },
      actionOpts, actionCbGroup_);

    landServer_ = rclcpp_action::create_server<Land>(
      this, "~/land",
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Land::Goal> goal) {
        return onLandGoal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleLand> goalHandle) {
        return onLandCancel(goalHandle);
      },
      [this](const std::shared_ptr<GoalHandleLand> goalHandle) { onLandAccepted(goalHandle); },
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
}

UavManagerNode::CallbackReturn UavManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  if (!configured_ || !uavStatePub_ || !statusTimer_) {
    return CallbackReturn::FAILURE;
  }

  active_ = true;

  if (!createActionServers()) {
    active_ = false;
    return CallbackReturn::FAILURE;
  }

  uavStatePub_->on_activate();
  statusTimer_->reset();

  if (healthAggregator_) {
    const auto snapshot = healthAggregator_->snapshot(nowSteady());
    if (!snapshot.dependenciesReady()) {
      RCLCPP_WARN(this->get_logger(), "Activated uav_manager before dependencies are ready");
    }
  }

  RCLCPP_INFO(this->get_logger(), "Activated uav_manager");
  return CallbackReturn::SUCCESS;
}

UavManagerNode::CallbackReturn UavManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;

  destroyActionServers();

  if (statusTimer_) {
    statusTimer_->cancel();
  }
  if (uavStatePub_) {
    uavStatePub_->on_deactivate();
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated uav_manager");
  return CallbackReturn::SUCCESS;
}

UavManagerNode::CallbackReturn UavManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_ = false;
  configured_ = false;

  statusTimer_.reset();

  estimatedStateSub_.reset();
  px4StatusSub_.reset();
  estimationStatusSub_.reset();
  controlStatusSub_.reset();
  trajectoryStatusSub_.reset();
  safetyStatusSub_.reset();

  uavStatePub_.reset();

  armClient_.reset();
  setModeClient_.reset();
  trajectoryExecuteClient_.reset();

  takeoffServer_.reset();
  landServer_.reset();
  clearEmergencyService_.reset();

  actionCbGroup_.reset();
  serviceCbGroup_.reset();

  {
    std::scoped_lock lock(mutex_);
    latestState_.reset();
    latestPx4Status_.reset();
    latestEstimationStatus_.reset();
    latestControlStatus_.reset();
    latestTrajectoryStatus_.reset();
    supervisor_ = SupervisorStateMachine();
    lastTransitionReason_ = "CLEANUP";
    latestSafetyLevel_ = 0;
  }

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
  active_ = false;
  destroyActionServers();
  if (statusTimer_) {
    statusTimer_->cancel();
  }
  if (uavStatePub_ && uavStatePub_->is_activated()) {
    uavStatePub_->on_deactivate();
  }
  RCLCPP_ERROR(this->get_logger(), "Error in uav_manager lifecycle; timers canceled");
  return CallbackReturn::SUCCESS;
}

void UavManagerNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestState_ = *msg;
  }

  if (healthAggregator_) {
    healthAggregator_->updateEstimatedState(nowSteady());
  }
}

void UavManagerNode::onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestPx4Status_ = *msg;
  }

  if (healthAggregator_) {
    Px4StatusInput input;
    input.connected = msg->connected;
    input.armed = msg->armed;
    input.offboard = msg->offboard;
    input.failsafe = msg->failsafe;
    input.navState = msg->nav_state;
    healthAggregator_->updatePx4Status(nowSteady(), input);
  }

  if (msg->failsafe) {
    (void)applyEvent(SupervisorEvent::FailsafeDetected);
  }
}

void UavManagerNode::onEstimationStatus(
  const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestEstimationStatus_ = *msg;
  }

  if (healthAggregator_) {
    ManagerStatusInput input;
    input.active = msg->active;
    input.healthy = msg->healthy;
    healthAggregator_->updateEstimationStatus(nowSteady(), input);
  }
}

void UavManagerNode::onControlStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestControlStatus_ = *msg;
  }

  if (healthAggregator_) {
    ManagerStatusInput input;
    input.active = msg->active;
    input.healthy = msg->healthy;
    healthAggregator_->updateControlStatus(nowSteady(), input);
  }
}

void UavManagerNode::onTrajectoryStatus(
  const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    latestTrajectoryStatus_ = *msg;
  }

  if (healthAggregator_) {
    ManagerStatusInput input;
    input.active = msg->active;
    input.healthy = msg->healthy;
    healthAggregator_->updateTrajectoryStatus(nowSteady(), input);
  }
}

void UavManagerNode::onSafetyStatus(
  const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg)
{
  if (healthAggregator_) {
    SafetyStatusInput input;
    input.level = msg->level;
    healthAggregator_->updateSafetyStatus(nowSteady(), input);
  }

  {
    std::scoped_lock lock(mutex_);
    latestSafetyLevel_ = msg->level;
  }

  if (msg->level >= peregrine_interfaces::msg::SafetyStatus::LEVEL_EMERGENCY) {
    (void)applyEvent(SupervisorEvent::FailsafeDetected);
  }
}

void UavManagerNode::publishUavState()
{
  if (!active_ || !uavStatePub_ || !uavStatePub_->is_activated()) {
    return;
  }

  peregrine_interfaces::msg::UAVState output;
  output.header.stamp = this->now();

  {
    std::scoped_lock lock(mutex_);
    output.state = toUavStateCode(supervisor_.state());
    output.mode = latestPx4Status_.has_value() ? px4ModeString(*latestPx4Status_) : "PX4_UNKNOWN";
    output.detail = lastTransitionReason_;

    if (latestPx4Status_.has_value()) {
      output.armed = latestPx4Status_->armed;
      output.offboard = latestPx4Status_->offboard;
      output.connected = latestPx4Status_->connected;
      output.failsafe = latestPx4Status_->failsafe;
    } else {
      output.armed = false;
      output.offboard = false;
      output.connected = false;
      output.failsafe = false;
    }
  }

  if (healthAggregator_) {
    auto snap = healthAggregator_->snapshot(nowSteady());
    output.dependencies_ready = snap.dependenciesReady();
    output.px4_ready = snap.px4.ready();
    output.estimation_ready = snap.estimationManager.ready();
    output.control_ready = snap.controlManager.ready();
    output.trajectory_ready = snap.trajectoryManager.ready();
    output.estimated_state_ready = snap.estimatedState.ready();
    output.safety_ready = snap.safetyMonitor.ready();
    output.readiness_detail =
      "px4=" + snap.px4.reasonCode +
      " est_state=" + snap.estimatedState.reasonCode +
      " est_mgr=" + snap.estimationManager.reasonCode +
      " ctrl_mgr=" + snap.controlManager.reasonCode +
      " traj_mgr=" + snap.trajectoryManager.reasonCode +
      " safety=" + snap.safetyMonitor.reasonCode;
  }

  uavStatePub_->publish(output);
}

void UavManagerNode::onClearEmergency(
  const std::shared_ptr<peregrine_interfaces::srv::ClearEmergency::Request>,
  std::shared_ptr<peregrine_interfaces::srv::ClearEmergency::Response> response)
{
  if (!isEmergency()) {
    response->success = false;
    response->message = "NOT_IN_EMERGENCY";
    return;
  }

  const TransitionOutcome outcome = applyEvent(SupervisorEvent::EmergencyCleared);
  response->success = outcome.accepted;
  response->message = outcome.reasonCode;
}

rclcpp_action::GoalResponse UavManagerNode::onTakeoffGoal(
  const rclcpp_action::GoalUUID &,
  const std::shared_ptr<const Takeoff::Goal> goal)
{
  if (!active_ || isEmergency()) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->target_altitude_m <= 0.0) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::scoped_lock lock(mutex_);
    const SupervisorState state = supervisor_.state();
    if (state != SupervisorState::Idle && state != SupervisorState::Armed &&
      state != SupervisorState::Landed)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  if (!reserveActionSlot()) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UavManagerNode::onTakeoffCancel(
  const std::shared_ptr<GoalHandleTakeoff>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UavManagerNode::onTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goalHandle)
{
  const auto release = std::shared_ptr<void>(nullptr, [this](void *) {releaseActionSlot();});
  (void)release;

  auto result = std::make_shared<Takeoff::Result>();
  const auto preempted = [this, goalHandle]() {return !active_ || goalHandle->is_canceling();};
  const auto emergency = [this]() {return isEmergency();};

  auto failGoal = [&](const std::string & reason)
  {
    bool needsDisarmTransition = false;
    {
      std::scoped_lock lock(mutex_);
      needsDisarmTransition =
        supervisor_.state() == SupervisorState::Armed &&
        latestPx4Status_.has_value() &&
        !latestPx4Status_->armed;
    }
    if (needsDisarmTransition) {
      (void)applyEvent(SupervisorEvent::DisarmCompleted);
    }

    result->success = false;
    result->message = reason;
    result->final_altitude_m = latestAltitudeM();
    if (reason == "GOAL_PREEMPTED") {
      goalHandle->canceled(result);
    } else {
      goalHandle->abort(result);
    }
  };

  if (emergency()) {
    failGoal("EMERGENCY_PREEMPT");
    return;
  }
  if (preempted()) {
    failGoal("GOAL_PREEMPTED");
    return;
  }

  SupervisorState currentState = SupervisorState::Idle;
  {
    std::scoped_lock lock(mutex_);
    currentState = supervisor_.state();
  }

  const auto svcTimeout = secondsToMillis(serviceTimeoutS_);

  if (currentState != SupervisorState::Armed) {
    const TransitionOutcome armTransition = applyEvent(SupervisorEvent::ArmRequested);
    if (!armTransition.accepted) {
      failGoal(armTransition.reasonCode);
      return;
    }

    if (emergency()) { failGoal("EMERGENCY_PREEMPT"); return; }
    if (preempted()) { failGoal("GOAL_PREEMPTED"); return; }

    StepResult step = callArmService(true);
    if (!step.success) {
      failGoal(step.describe());
      return;
    }

    step = pollUntil(
      "ARMED_TIMEOUT", svcTimeout,
      [this]()
      {
        std::scoped_lock lock(mutex_);
        return latestPx4Status_.has_value() && latestPx4Status_->armed;
      },
      preempted, emergency);
    if (!step.success) {
      failGoal(step.describe());
      return;
    }
  }

  StepResult step = pollUntil(
    "SETPOINT_FLOW_TIMEOUT", svcTimeout,
    [this]()
    {
      std::scoped_lock lock(mutex_);
      return latestControlStatus_.has_value() && latestControlStatus_->active && latestControlStatus_->healthy;
    },
    preempted, emergency);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  if (emergency()) { failGoal("EMERGENCY_PREEMPT"); return; }
  if (preempted()) { failGoal("GOAL_PREEMPTED"); return; }

  step = callSetModeService("offboard");
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  step = pollUntil(
    "OFFBOARD_TIMEOUT", svcTimeout,
    [this]()
    {
      std::scoped_lock lock(mutex_);
      return latestPx4Status_.has_value() && latestPx4Status_->offboard;
    },
    preempted, emergency);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  const TransitionOutcome takeoffStart = applyEvent(SupervisorEvent::TakeoffRequested);
  if (!takeoffStart.accepted) {
    failGoal(takeoffStart.reasonCode);
    return;
  }

  ExecuteTrajectory::Goal executeGoal;
  executeGoal.trajectory_type = "takeoff";
  executeGoal.params = {goalHandle->get_goal()->target_altitude_m,
    goalHandle->get_goal()->climb_velocity_mps};

  ExecuteTrajectory::Result executeResult;
  step = forwardTakeoffTrajectory(
    executeGoal,
    [this, goalHandle](const ExecuteTrajectory::Feedback & feedback)
    {
      auto fb = std::make_shared<Takeoff::Feedback>();
      fb->progress = feedback.progress;
      fb->current_altitude_m = latestAltitudeM();
      goalHandle->publish_feedback(fb);
    },
    preempted, emergency, &executeResult);

  if (!step.success || !executeResult.success) {
    (void)applyEvent(SupervisorEvent::TakeoffFailed);
    failGoal(step.success ? executeResult.message : step.describe());
    return;
  }

  const TransitionOutcome takeoffComplete = applyEvent(SupervisorEvent::TakeoffCompleted);
  if (!takeoffComplete.accepted) {
    failGoal(takeoffComplete.reasonCode);
    return;
  }

  result->success = true;
  result->message = "TAKEOFF_COMPLETE";
  result->final_altitude_m = latestAltitudeM();
  goalHandle->succeed(result);
}

rclcpp_action::GoalResponse UavManagerNode::onLandGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const Land::Goal>)
{
  if (!active_ || isEmergency()) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::scoped_lock lock(mutex_);
    const SupervisorState state = supervisor_.state();
    if (state != SupervisorState::Armed && state != SupervisorState::TakingOff &&
      state != SupervisorState::Hovering &&
      state != SupervisorState::Flying)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
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
  const auto release = std::shared_ptr<void>(nullptr, [this](void *) {releaseActionSlot();});
  (void)release;

  auto result = std::make_shared<Land::Result>();
  const auto preempted = [this, goalHandle]() {return !active_ || goalHandle->is_canceling();};
  const auto emergency = [this]() {return isEmergency();};

  auto failGoal = [&](const std::string & reason)
  {
    result->success = false;
    result->message = reason;
    if (reason == "GOAL_PREEMPTED") {
      goalHandle->canceled(result);
    } else {
      goalHandle->abort(result);
    }
  };

  if (emergency()) {
    failGoal("EMERGENCY_PREEMPT");
    return;
  }
  if (preempted()) {
    failGoal("GOAL_PREEMPTED");
    return;
  }

  const auto svcTimeout = secondsToMillis(serviceTimeoutS_);

  const TransitionOutcome landStart = applyEvent(SupervisorEvent::LandRequested);
  if (!landStart.accepted) {
    failGoal(landStart.reasonCode);
    return;
  }

  if (emergency()) { (void)applyEvent(SupervisorEvent::LandFailed); failGoal("EMERGENCY_PREEMPT"); return; }
  if (preempted()) { (void)applyEvent(SupervisorEvent::LandFailed); failGoal("GOAL_PREEMPTED"); return; }

  StepResult step = callSetModeService("land");
  if (!step.success) {
    (void)applyEvent(SupervisorEvent::LandFailed);
    failGoal(step.describe());
    return;
  }

  step = pollUntil(
    "LAND_MODE_TIMEOUT", svcTimeout,
    [this]()
    {
      std::scoped_lock lock(mutex_);
      return latestPx4Status_.has_value() && !latestPx4Status_->offboard;
    },
    preempted, emergency);
  if (!step.success) {
    (void)applyEvent(SupervisorEvent::LandFailed);
    failGoal(step.describe());
    return;
  }

  step = pollUntil(
    "AUTO_DISARM_TIMEOUT", 30s,
    [this]()
    {
      std::scoped_lock lock(mutex_);
      return latestPx4Status_.has_value() && !latestPx4Status_->armed;
    },
    preempted, emergency);
  if (!step.success) {
    (void)applyEvent(SupervisorEvent::LandFailed);
    failGoal(step.describe());
    return;
  }

  (void)applyEvent(SupervisorEvent::LandCompleted);
  (void)applyEvent(SupervisorEvent::DisarmCompleted);

  result->success = true;
  result->message = "LAND_COMPLETE";
  goalHandle->succeed(result);
}

bool UavManagerNode::reserveActionSlot()
{
  bool expected = false;
  return actionSlotReserved_.compare_exchange_strong(expected, true);
}

void UavManagerNode::releaseActionSlot()
{
  actionSlotReserved_.store(false);
}

TransitionOutcome UavManagerNode::applyEvent(const SupervisorEvent event)
{
  HealthSnapshot snapshot;
  if (healthAggregator_) {
    snapshot = healthAggregator_->snapshot(nowSteady());
  }

  TransitionOutcome outcome;
  {
    std::scoped_lock lock(mutex_);
    outcome = supervisor_.apply(event, transitionGuard_, snapshot);
    lastTransitionReason_ = outcome.reasonCode;
  }

  if (outcome.accepted) {
    RCLCPP_INFO(
      this->get_logger(),
      "FSM transition accepted: from=%s event=%s to=%s reason=%s",
      SupervisorStateMachine::toString(outcome.from),
      SupervisorStateMachine::toString(outcome.event),
      SupervisorStateMachine::toString(outcome.to), outcome.reasonCode.c_str());
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "FSM transition rejected: from=%s event=%s to=%s reason=%s",
      SupervisorStateMachine::toString(outcome.from),
      SupervisorStateMachine::toString(outcome.event),
      SupervisorStateMachine::toString(outcome.to), outcome.reasonCode.c_str());
  }

  return outcome;
}

bool UavManagerNode::isEmergency() const
{
  std::scoped_lock lock(mutex_);
  return supervisor_.state() == SupervisorState::Emergency;
}

StepResult UavManagerNode::callArmService(const bool arm)
{
  if (!armClient_ || !armClient_->service_is_ready()) {
    return StepResult::fail(StepCode::ArmServiceUnavailable);
  }

  auto request = std::make_shared<peregrine_interfaces::srv::Arm::Request>();
  request->arm = arm;
  auto future = armClient_->async_send_request(request);

  const auto deadline = this->now() + rclcpp::Duration(secondsToMillis(serviceTimeoutS_));
  while (future.wait_for(50ms) != std::future_status::ready) {
    if (this->now() >= deadline) {
      return StepResult::fail(StepCode::ArmServiceTimeout);
    }
    if (!active_ || isEmergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
  }

  const auto response = future.get();
  if (!response->success) {
    return StepResult::fail(
      StepCode::ArmServiceRejected,
      response->message.empty() ? std::string{} : response->message);
  }

  return StepResult::ok();
}

StepResult UavManagerNode::callSetModeService(const std::string & mode)
{
  if (!setModeClient_ || !setModeClient_->service_is_ready()) {
    return StepResult::fail(StepCode::SetModeServiceUnavailable);
  }

  auto request = std::make_shared<peregrine_interfaces::srv::SetMode::Request>();
  request->mode = mode;
  auto future = setModeClient_->async_send_request(request);

  const auto deadline = this->now() + rclcpp::Duration(secondsToMillis(serviceTimeoutS_));
  while (future.wait_for(50ms) != std::future_status::ready) {
    if (this->now() >= deadline) {
      return StepResult::fail(StepCode::SetModeServiceTimeout);
    }
    if (!active_ || isEmergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
  }

  const auto response = future.get();
  if (!response->success) {
    return StepResult::fail(
      StepCode::SetModeRejected,
      response->message.empty() ? std::string{} : response->message);
  }

  return StepResult::ok();
}

StepResult UavManagerNode::pollUntil(
  const std::string & timeoutCode,
  const std::chrono::milliseconds timeout,
  const std::function<bool()> & condition,
  const std::function<bool()> & preempted,
  const std::function<bool()> & emergency) const
{
  rclcpp::Rate rate(20.0);
  const auto deadline = this->now() + rclcpp::Duration(timeout);
  while (rclcpp::ok() && this->now() < deadline) {
    if (emergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (condition()) {
      return StepResult::ok();
    }
    rate.sleep();
  }
  if (condition()) {
    return StepResult::ok();
  }
  return StepResult::failCustom(timeoutCode);
}

StepResult UavManagerNode::forwardTakeoffTrajectory(
  const ExecuteTrajectory::Goal & goal,
  std::function<void(const ExecuteTrajectory::Feedback &)> feedbackCallback,
  const std::function<bool()> & preempted,
  const std::function<bool()> & emergency, ExecuteTrajectory::Result * resultOut) const
{
  if (!trajectoryExecuteClient_ || !trajectoryExecuteClient_->action_server_is_ready()) {
    return StepResult::fail(StepCode::TrajectoryExecuteServerUnavailable);
  }

  rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions options;
  options.feedback_callback =
    [feedbackCallback = std::move(feedbackCallback)](auto,
      const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback)
    {
      if (feedbackCallback && feedback) {
        feedbackCallback(*feedback);
      }
    };

  auto goalHandleFuture = trajectoryExecuteClient_->async_send_goal(goal, options);
  const auto goalDeadline = this->now() + rclcpp::Duration(secondsToMillis(serviceTimeoutS_));
  while (goalHandleFuture.wait_for(50ms) != std::future_status::ready) {
    if (this->now() >= goalDeadline) {
      return StepResult::fail(StepCode::TrajectoryExecuteGoalTimeout);
    }
    if (emergency()) {
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
  const auto resultDeadline = this->now() + rclcpp::Duration(30s);
  while (resultFuture.wait_for(50ms) != std::future_status::ready) {
    if (this->now() >= resultDeadline) {
      trajectoryExecuteClient_->async_cancel_goal(goalHandle);
      return StepResult::fail(StepCode::TrajectoryExecuteResultTimeout);
    }
    if (emergency()) {
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

double UavManagerNode::latestAltitudeM() const
{
  std::scoped_lock lock(mutex_);
  if (!latestState_.has_value()) {
    return 0.0;
  }
  return latestState_->pose.pose.position.z;
}

std::chrono::nanoseconds UavManagerNode::periodFromHz(const double hz)
{
  const auto period = std::chrono::duration<double>(1.0 / hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(period);
}

uint8_t UavManagerNode::toUavStateCode(const SupervisorState state)
{
  switch (state) {
    case SupervisorState::Idle:
      return peregrine_interfaces::msg::UAVState::STATE_IDLE;
    case SupervisorState::Armed:
      return peregrine_interfaces::msg::UAVState::STATE_ARMED;
    case SupervisorState::TakingOff:
      return peregrine_interfaces::msg::UAVState::STATE_TAKING_OFF;
    case SupervisorState::Hovering:
      return peregrine_interfaces::msg::UAVState::STATE_HOVERING;
    case SupervisorState::Flying:
      return peregrine_interfaces::msg::UAVState::STATE_FLYING;
    case SupervisorState::Landing:
      return peregrine_interfaces::msg::UAVState::STATE_LANDING;
    case SupervisorState::Landed:
      return peregrine_interfaces::msg::UAVState::STATE_LANDED;
    case SupervisorState::Emergency:
    default:
      return peregrine_interfaces::msg::UAVState::STATE_EMERGENCY;
  }
}

std::chrono::steady_clock::time_point UavManagerNode::nowSteady() const
{
  return std::chrono::steady_clock::now();
}

}  // namespace uav_manager

RCLCPP_COMPONENTS_REGISTER_NODE(uav_manager::UavManagerNode)
