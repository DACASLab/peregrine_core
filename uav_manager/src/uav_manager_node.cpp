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
#include <uav_manager/uav_manager_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <future>
#include <utility>

namespace uav_manager
{
namespace
{

constexpr char kNodeName[] = "uav_manager";

// PX4 nav_state values from px4_msgs::msg::VehicleStatus. These are raw uint8 constants
// because the PX4Status bridge message exposes nav_state as a plain integer.
// `constexpr` makes these compile-time constants with zero runtime initialization cost.
constexpr uint8_t kNavStatePosCtl = 2;
constexpr uint8_t kNavStateAutoMission = 3;
constexpr uint8_t kNavStateAutoRtl = 5;
constexpr uint8_t kNavStateAutoTakeoff = 17;
constexpr uint8_t kNavStateAutoLand = 18;

// PX4 firmware rejects arm commands while in certain autonomous nav states
// (e.g. AUTO_MISSION, AUTO_RTL). Before arming we must detect these and
// switch to a permissive mode like POSCTL first. See ensureArmableMode().
bool navStatePreventsArming(const uint8_t navState)
{
  return navState == kNavStateAutoMission || navState == kNavStateAutoRtl ||
         navState == kNavStateAutoTakeoff ||
         navState == kNavStateAutoLand;
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
  // All timeouts are configurable ROS parameters so they can be tuned per deployment.
  // SITL environments typically use shorter timeouts than real hardware because
  // simulated PX4 responds faster and network latency is negligible.
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 10.0);
  dependencyStartupTimeoutS_ = this->declare_parameter<double>("dependency_startup_timeout_s", 2.0);

  // Freshness thresholds determine how stale a subscription message can be before the
  // health aggregator marks that dependency as unhealthy.
  freshnessConfig_.px4StatusTimeout =
    secondsToMillis(this->declare_parameter<double>("px4_status_timeout_s", 1.0));
  freshnessConfig_.managerStatusTimeout =
    secondsToMillis(this->declare_parameter<double>("manager_status_timeout_s", 1.0));
  freshnessConfig_.estimatedStateTimeout =
    secondsToMillis(this->declare_parameter<double>("estimated_state_timeout_s", 1.0));

  serviceWaitS_ = this->declare_parameter<double>("service_wait_s", 3.0);
  serviceResponseWaitS_ = this->declare_parameter<double>("service_response_wait_s", 5.0);
  actionServerWaitS_ = this->declare_parameter<double>("action_server_wait_s", 3.0);
  actionResultWaitS_ = this->declare_parameter<double>("action_result_wait_s", 180.0);
  offboardWaitS_ = this->declare_parameter<double>("offboard_wait_s", 6.0);
  armedWaitS_ = this->declare_parameter<double>("armed_wait_s", 6.0);

  const auto pollMs = this->declare_parameter<int>("orchestrator_poll_ms", 50);
  OrchestratorConfig orchestratorConfig;
  orchestratorConfig.pollPeriod = std::chrono::milliseconds(std::max<int64_t>(10, pollMs));

  healthAggregator_ = std::make_unique<HealthAggregator>(freshnessConfig_);
  orchestrator_ = std::make_unique<ActionOrchestrator>(orchestratorConfig);
}

UavManagerNode::CallbackReturn UavManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  if (statusRateHz_ <= 0.0 || dependencyStartupTimeoutS_ <= 0.0 || serviceWaitS_ <= 0.0 ||
    serviceResponseWaitS_ <= 0.0 ||
    actionServerWaitS_ <= 0.0 || actionResultWaitS_ <= 0.0 || offboardWaitS_ <= 0.0 ||
    armedWaitS_ <= 0.0)
  {
    RCLCPP_ERROR(this->get_logger(), "Lifecycle parameters must be > 0");
    return CallbackReturn::FAILURE;
  }

  // Three callback groups implement a deliberate threading architecture for
  // MultiThreadedExecutor:
  //
  // 1) Default MutuallyExclusive group (implicit, no group specified):
  //    Used by subscriptions and the status timer. These callbacks are quick
  //    (lock, cache message, unlock) so a single thread is sufficient.
  //
  // 2) Reentrant group (actionCbGroup_):
  //    Used by action servers. Accepted callbacks (onTakeoffAccepted, etc.)
  //    block for minutes during flight operations. Reentrant allows the
  //    executor to dispatch new goal/cancel callbacks concurrently even while
  //    a previous accepted callback is still running.
  //
  // 3) Separate MutuallyExclusive group (serviceCbGroup_):
  //    Used by service and action clients. This prevents deadlock where a
  //    service response callback cannot be delivered because the executor
  //    thread is blocked inside an action accepted callback from group 2.
  // Callback groups control how the ROS2 executor schedules callbacks. In Python
  // rclpy, the GIL naturally serializes callbacks. In C++ with a MultiThreadedExecutor,
  // multiple callbacks can run truly in parallel on different OS threads. Callback
  // groups let you control this:
  //   - MutuallyExclusive: only one callback in this group runs at a time (like a lock)
  //   - Reentrant: multiple callbacks in this group can run simultaneously
  actionCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  serviceCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

  // Subscriptions and timer use the default MutuallyExclusive callback group (no group specified).
  // They are all non-blocking and can safely share a single executor thread.
  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    "estimated_state", qos, std::bind(
      &UavManagerNode::onEstimatedState, this,
      std::placeholders::_1));
  px4StatusSub_ = this->create_subscription<peregrine_interfaces::msg::PX4Status>(
    "status", statusQos, std::bind(&UavManagerNode::onPx4Status, this, std::placeholders::_1));
  estimationStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    "estimation_status", statusQos,
    std::bind(&UavManagerNode::onEstimationStatus, this, std::placeholders::_1));
  controlStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    "control_status", statusQos,
    std::bind(&UavManagerNode::onControlStatus, this, std::placeholders::_1));
  trajectoryStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    "trajectory_status", statusQos,
    std::bind(&UavManagerNode::onTrajectoryStatus, this, std::placeholders::_1));

  uavStatePub_ =
    this->create_publisher<peregrine_interfaces::msg::UAVState>("uav_state", statusQos);

  // Service clients in their own group so response callbacks are dispatched independently
  // of the blocking action callbacks in actionCbGroup_.
  armClient_ = this->create_client<peregrine_interfaces::srv::Arm>("arm", rmw_qos_profile_services_default, serviceCbGroup_);
  setModeClient_ = this->create_client<peregrine_interfaces::srv::SetMode>("set_mode", rmw_qos_profile_services_default, serviceCbGroup_);

  // Action clients use the service callback group for their internal response handling.
  trajectoryGoToClient_ = rclcpp_action::create_client<GoTo>(this, "trajectory_manager/go_to", serviceCbGroup_);
  trajectoryExecuteClient_ = rclcpp_action::create_client<ExecuteTrajectory>(
    this,
    "trajectory_manager/execute_trajectory", serviceCbGroup_);

  statusTimer_ =
    this->create_wall_timer(
    periodFromHz(statusRateHz_),
    std::bind(&UavManagerNode::publishUavState, this));
  statusTimer_->cancel();

  // Defensive reset: clear all cached data and re-initialize the FSM to ensure clean
  // state when on_configure is invoked after on_cleanup (lifecycle re-configuration).
  // Without this, stale messages from a previous activation could satisfy readiness
  // checks prematurely during the next on_activate.
  {
    // The bare `{` opens a new scope block. When this block closes at `}`, any
    // local variables (including `lock`) are destroyed. Since std::scoped_lock
    // releases the mutex in its destructor, this pattern gives precise control over
    // how long the lock is held. This is a common C++ idiom with no Python equivalent
    // - Python's `with` block serves a similar purpose but is syntactically different.
    std::scoped_lock lock(mutex_);
    // `.reset()` on std::optional clears the contained value, making it empty
    // (equivalent to setting a Python variable to None).
    latestState_.reset();
    latestPx4Status_.reset();
    latestEstimationStatus_.reset();
    latestControlStatus_.reset();
    latestTrajectoryStatus_.reset();
    supervisor_ = SupervisorStateMachine();
    lastTransitionReason_ = "CONFIGURED";
  }

  configured_ = true;
  active_ = false;

  RCLCPP_INFO(this->get_logger(), "Configured uav_manager lifecycle");
  return CallbackReturn::SUCCESS;
}

bool UavManagerNode::createActionServers()
{
  if (takeoffServer_ || landServer_ || goToServer_ || executeServer_) {
    // Idempotent: servers already exist.
    return true;
  }
  if (!actionCbGroup_) {
    RCLCPP_ERROR(this->get_logger(), "Action callback group not initialized");
    return false;
  }

  try {
    // rcl_action_server_options_t is needed to pass a callback group to
    // rclcpp_action::create_server (the rclcpp_action API requires it as a
    // parameter before the callback group). Using default options here.
    // The "~/" prefix resolves under the node's namespace, so "~/takeoff"
    // becomes e.g. /uav_manager/takeoff in the ROS graph.
    rcl_action_server_options_t actionOpts = rcl_action_server_get_default_options();

    takeoffServer_ = rclcpp_action::create_server<Takeoff>(
      this, "~/takeoff",
      std::bind(&UavManagerNode::onTakeoffGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UavManagerNode::onTakeoffCancel, this, std::placeholders::_1),
      std::bind(&UavManagerNode::onTakeoffAccepted, this, std::placeholders::_1),
      actionOpts, actionCbGroup_);

    landServer_ = rclcpp_action::create_server<Land>(
      this, "~/land",
      std::bind(&UavManagerNode::onLandGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UavManagerNode::onLandCancel, this, std::placeholders::_1),
      std::bind(&UavManagerNode::onLandAccepted, this, std::placeholders::_1),
      actionOpts, actionCbGroup_);

    goToServer_ = rclcpp_action::create_server<GoTo>(
      this, "~/go_to",
      std::bind(&UavManagerNode::onGoToGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UavManagerNode::onGoToCancel, this, std::placeholders::_1),
      std::bind(&UavManagerNode::onGoToAccepted, this, std::placeholders::_1),
      actionOpts, actionCbGroup_);

    executeServer_ = rclcpp_action::create_server<ExecuteTrajectory>(
      this, "~/execute_trajectory",
      std::bind(&UavManagerNode::onExecuteGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&UavManagerNode::onExecuteCancel, this, std::placeholders::_1),
      std::bind(&UavManagerNode::onExecuteAccepted, this, std::placeholders::_1),
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
  // Resetting the shared_ptr tears down the underlying rcl action server and removes it
  // from discovery. This makes action-server visibility match lifecycle operational state.
  takeoffServer_.reset();
  landServer_.reset();
  goToServer_.reset();
  executeServer_.reset();
}

UavManagerNode::CallbackReturn UavManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  if (!configured_ || !uavStatePub_ || !statusTimer_) {
    return CallbackReturn::FAILURE;
  }

  // Activation is intentionally not gated on dependency readiness. Readiness is enforced
  // by explicit checks at goal execution time (and surfaced via UAVState.detail) rather
  // than causing lifecycle activation to be timing-sensitive.
  active_ = true;

  if (!createActionServers()) {
    active_ = false;
    return CallbackReturn::FAILURE;
  }

  uavStatePub_->on_activate();
  statusTimer_->reset();

  // Optional: log a warning if dependencies are not yet ready at activation time.
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

  // Tear down action servers first so external clients do not discover/target this node
  // while it is inactive.
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

  uavStatePub_.reset();

  armClient_.reset();
  setModeClient_.reset();
  trajectoryGoToClient_.reset();
  trajectoryExecuteClient_.reset();

  takeoffServer_.reset();
  landServer_.reset();
  goToServer_.reset();
  executeServer_.reset();

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
    // Failsafe detection preempts supervisor regardless of current mission phase.
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
    output.mode = SupervisorStateMachine::toString(supervisor_.state());
    // `detail` carries the latest transition/guard reason for operator visibility.
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

  uavStatePub_->publish(output);
}

// Multi-layer rejection logic filters goals before they reach the expensive
// accepted callback. Each layer catches a different failure class:
//   1) Lifecycle gate  -- node not active (still configuring or shutting down)
//   2) Emergency gate  -- FSM is in emergency state (failsafe detected)
//   3) Parameter check -- reject obviously invalid goal fields early
//   4) FSM state check -- only valid pre-flight states allow takeoff
//   5) Action slot     -- single-flight mutex prevents concurrent operations
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

// Core takeoff orchestration flow. Phases:
//   Phase 1: Arm sequence   -- ensure armable mode, call arm service, wait for armed confirmation
//   Phase 2: Offboard entry  -- switch PX4 to offboard mode before trajectory dispatch
//   Phase 3: Trajectory exec -- forward takeoff trajectory to trajectory_manager, relay feedback
//   Phase 4: FSM finalization -- advance supervisor to Hovering on success
void UavManagerNode::onTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goalHandle)
{
  // Runs on an executor thread from actionCbGroup_ (Reentrant).
  // No manual std::thread -- the MultiThreadedExecutor manages concurrency.

  // RAII (Resource Acquisition Is Initialization) cleanup guard. This is a C++
  // pattern with no direct Python equivalent, though Python's try/finally or
  // contextlib.ExitStack serve a similar purpose.
  //
  // `std::shared_ptr<void>(nullptr, deleter)` creates a shared_ptr that owns
  // nothing (nullptr) but has a custom "deleter" function. When this shared_ptr
  // is destroyed (when `release` goes out of scope at the end of this function),
  // the deleter runs - calling releaseActionSlot(). This guarantees the action
  // slot is released on ANY exit path: normal return, early error return, or
  // exception. The equivalent Python pattern would be:
  //   try:
  //       ... all the logic ...
  //   finally:
  //       self.release_action_slot()
  //
  // `(void)release;` suppresses the "unused variable" compiler warning. The
  // variable IS used - its destructor side-effect is the entire point - but the
  // compiler doesn't know that without the cast.
  const auto release = std::shared_ptr<void>(nullptr, [this](void *) {releaseActionSlot();});
  (void)release;

  auto result = std::make_shared<Takeoff::Result>();
  // These lambdas capture external variables for use inside the function body:
  //   - `[this, goalHandle]` captures the object pointer AND the goal handle
  //     shared_ptr by copy. The lambda owns its own copy of the shared_ptr,
  //     keeping the goal handle alive as long as the lambda exists.
  //   - `[this]` captures only the object pointer.
  //   - `[&]` (used below on failGoal) captures ALL local variables by reference,
  //     meaning the lambda accesses the same variables, not copies. This is
  //     efficient but the lambda must not outlive the captured variables.
  const auto preempted = [this, goalHandle]() {return !active_ || goalHandle->is_canceling();};
  const auto emergency = [this]() {return isEmergency();};

  // `[&]` captures `result`, `goalHandle`, and all other locals by reference.
  // This is safe here because `failGoal` never outlives this function scope.
  auto failGoal = [&](const std::string & reason)
  {
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

  if (currentState != SupervisorState::Armed) {
    // Phase 1: move supervisor to Armed and synchronize PX4 arming state.
    const TransitionOutcome armTransition = applyEvent(SupervisorEvent::ArmRequested);
    if (!armTransition.accepted) {
      failGoal(armTransition.reasonCode);
      return;
    }

    StepResult step = orchestrator_->callStep(
      "ENSURE_ARMABLE_MODE", [this]() {return ensureArmableMode();}, preempted,
      emergency);
    if (!step.success) {
        failGoal(step.describe());
      return;
    }

    step =
    orchestrator_->callStep(
      "ARM_SERVICE",
      [this]() {return callArmService(true);}, preempted, emergency);
    if (!step.success) {
      failGoal(step.describe());
      return;
    }

    step = orchestrator_->waitForCondition(
      "ARMED_TIMEOUT", secondsToMillis(armedWaitS_),
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

  StepResult step = waitForControlSetpointFlow(std::chrono::seconds(4));
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  // Phase 2: enter offboard before dispatching trajectory_manager takeoff.
  step = orchestrator_->callStep(
    "SET_MODE_OFFBOARD", [this]() {return callSetModeService("offboard");}, preempted,
    emergency);
  if (!step.success) {
    failGoal(step.describe());
    return;
  }

  step = orchestrator_->waitForCondition(
    "OFFBOARD_TIMEOUT", secondsToMillis(offboardWaitS_),
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

  // Phase 3: execute takeoff trajectory and proxy progress feedback upstream.
  ExecuteTrajectory::Goal executeGoal;
  executeGoal.trajectory_type = "takeoff";
  executeGoal.params = {goalHandle->get_goal()->target_altitude_m,
    goalHandle->get_goal()->climb_velocity_mps};

  ExecuteTrajectory::Result executeResult;
  step = forwardExecuteTrajectory(
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

  // Phase 4: finalize FSM state once trajectory_manager reports completion.
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

// Landing strategy: instead of executing a trajectory-based descent, we delegate to
// PX4's native land mode via set_mode("land"). PX4's own landing controller handles
// descent rate, ground detection, and auto-disarm after touchdown. The sequence is:
//   1) set_mode("land")          -- hand control to PX4's landing controller
//   2) wait for !offboard        -- confirms PX4 accepted the mode switch away from offboard
//   3) wait for !armed           -- PX4 auto-disarms after detecting ground contact
// The 90s timeout on auto-disarm is generous because PX4 may hover-check or perform
// a slow final descent before touchdown, especially in windy conditions.
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

  const TransitionOutcome landStart = applyEvent(SupervisorEvent::LandRequested);
  if (!landStart.accepted) {
    failGoal(landStart.reasonCode);
    return;
  }

  StepResult step = orchestrator_->callStep(
    "SET_MODE_LAND", [this]() {return callSetModeService("land");}, preempted,
    emergency);
  if (!step.success) {
    (void)applyEvent(SupervisorEvent::LandFailed);
    failGoal(step.describe());
    return;
  }

  step = orchestrator_->waitForCondition(
    "LAND_MODE_TIMEOUT", std::chrono::seconds(5),
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

  step = orchestrator_->waitForCondition(
    "AUTO_DISARM_TIMEOUT", std::chrono::seconds(90),
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

rclcpp_action::GoalResponse UavManagerNode::onGoToGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const GoTo::Goal> goal)
{
  if (!active_ || isEmergency()) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->acceptance_radius_m <= 0.0) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::scoped_lock lock(mutex_);
    const SupervisorState state = supervisor_.state();
    if (state != SupervisorState::Hovering && state != SupervisorState::Flying) {
      return rclcpp_action::GoalResponse::REJECT;
    }
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
  const auto release = std::shared_ptr<void>(nullptr, [this](void *) {releaseActionSlot();});
  (void)release;

  auto result = std::make_shared<GoTo::Result>();
  const auto preempted = [this, goalHandle]() {return !active_ || goalHandle->is_canceling();};
  const auto emergency = [this]() {return isEmergency();};

  auto failGoal = [&](const std::string & reason)
  {
    result->success = false;
    result->message = reason;
    result->final_position = latestPosition();
    if (reason == "GOAL_PREEMPTED") {
      goalHandle->canceled(result);
    } else {
      goalHandle->abort(result);
    }
  };

  const TransitionOutcome flightStart = applyEvent(SupervisorEvent::FlightActionRequested);
  if (!flightStart.accepted) {
    failGoal(flightStart.reasonCode);
    return;
  }

  // Ensure PX4 remains in offboard before delegating to trajectory_manager/go_to.
  StepResult step = orchestrator_->callStep(
    "SET_MODE_OFFBOARD", [this]() {return callSetModeService("offboard");},
    preempted, emergency);
  if (!step.success) {
    (void)applyEvent(SupervisorEvent::FlightActionFailed);
    failGoal(step.describe());
    return;
  }

  step = orchestrator_->waitForCondition(
    "OFFBOARD_TIMEOUT", secondsToMillis(offboardWaitS_),
    [this]()
    {
      std::scoped_lock lock(mutex_);
      return latestPx4Status_.has_value() && latestPx4Status_->offboard;
    },
    preempted, emergency);
  if (!step.success) {
    (void)applyEvent(SupervisorEvent::FlightActionFailed);
    failGoal(step.describe());
    return;
  }

  GoTo::Result forwardResult;
  // Forward goal and pass through downstream feedback/result while preserving preemption checks.
  step = forwardGoTo(
    *goalHandle->get_goal(),
    [goalHandle](const GoTo::Feedback & feedback)
    {
      auto fb = std::make_shared<GoTo::Feedback>();
      *fb = feedback;
      goalHandle->publish_feedback(fb);
    },
    preempted, emergency, &forwardResult);

  if (!step.success || !forwardResult.success) {
    (void)applyEvent(SupervisorEvent::FlightActionFailed);
    failGoal(step.success ? forwardResult.message : step.describe());
    return;
  }

  (void)applyEvent(SupervisorEvent::FlightActionCompleted);

  *result = forwardResult;
  goalHandle->succeed(result);
}

rclcpp_action::GoalResponse UavManagerNode::onExecuteGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ExecuteTrajectory::Goal> goal)
{
  if (!active_ || isEmergency()) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->trajectory_type.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::scoped_lock lock(mutex_);
    const SupervisorState state = supervisor_.state();
    if (state != SupervisorState::Hovering && state != SupervisorState::Flying) {
      return rclcpp_action::GoalResponse::REJECT;
    }
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

void UavManagerNode::onExecuteAccepted(
  const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle)
{
  const auto release = std::shared_ptr<void>(nullptr, [this](void *) {releaseActionSlot();});
  (void)release;

  auto result = std::make_shared<ExecuteTrajectory::Result>();
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

  const TransitionOutcome flightStart = applyEvent(SupervisorEvent::FlightActionRequested);
  if (!flightStart.accepted) {
    failGoal(flightStart.reasonCode);
    return;
  }

  // Ensure PX4 remains in offboard before delegating execute_trajectory downstream.
  StepResult step = orchestrator_->callStep(
    "SET_MODE_OFFBOARD", [this]() {return callSetModeService("offboard");},
    preempted, emergency);
  if (!step.success) {
    (void)applyEvent(SupervisorEvent::FlightActionFailed);
    failGoal(step.describe());
    return;
  }

  step = orchestrator_->waitForCondition(
    "OFFBOARD_TIMEOUT", secondsToMillis(offboardWaitS_),
    [this]()
    {
      std::scoped_lock lock(mutex_);
      return latestPx4Status_.has_value() && latestPx4Status_->offboard;
    },
    preempted, emergency);
  if (!step.success) {
    (void)applyEvent(SupervisorEvent::FlightActionFailed);
    failGoal(step.describe());
    return;
  }

  ExecuteTrajectory::Result forwardResult;
  // Forward goal and pass through downstream feedback/result while preserving preemption checks.
  step = forwardExecuteTrajectory(
    *goalHandle->get_goal(),
    [goalHandle](const ExecuteTrajectory::Feedback & feedback)
    {
      auto fb = std::make_shared<ExecuteTrajectory::Feedback>();
      *fb = feedback;
      goalHandle->publish_feedback(fb);
    },
    preempted, emergency, &forwardResult);

  if (!step.success || !forwardResult.success) {
    (void)applyEvent(SupervisorEvent::FlightActionFailed);
    failGoal(step.success ? forwardResult.message : step.describe());
    return;
  }

  (void)applyEvent(SupervisorEvent::FlightActionCompleted);

  *result = forwardResult;
  goalHandle->succeed(result);
}

// reserveActionSlot / releaseActionSlot implement a mutex-guarded boolean that
// enforces single-flight policy: only one high-level action (takeoff, land,
// go_to, execute_trajectory) may run at a time. This prevents conflicting PX4
// commands -- e.g. a land goal arriving while takeoff is in progress. The slot
// is reserved in onXxxGoal() and released via the RAII release guard in
// onXxxAccepted().
bool UavManagerNode::reserveActionSlot()
{
  std::scoped_lock lock(actionSlotMutex_);
  if (actionSlotReserved_) {
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

TransitionOutcome UavManagerNode::applyEvent(const SupervisorEvent event)
{
  // The health snapshot is captured ONCE outside the lock. This gives the guard
  // evaluator a consistent point-in-time view of all subsystem health. Both the
  // guard check and the FSM state mutation then happen under one lock acquisition,
  // preventing state/snapshot skew where the FSM advances based on a health
  // snapshot that has already changed.
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

// PX4 rejects arm commands when in certain autonomous nav states
// (AUTO_MISSION, AUTO_RTL, AUTO_TAKEOFF, AUTO_LAND). This helper detects
// those states and forces POSCTL first, which always allows arming.
// If the vehicle is already armed or in a permissive mode, this is a no-op.
StepResult UavManagerNode::ensureArmableMode()
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

  StepResult step = callSetModeService("position");
  if (!step.success) {
    return StepResult::fail(StepCode::SetPositionFailed, step.describe());
  }

  step = waitForNavState(kNavStatePosCtl, std::chrono::seconds(6));
  if (!step.success) {
    return StepResult::fail(StepCode::PositionModeTimeout, step.describe());
  }

  return StepResult::ok();
}

// Service call pattern: instead of blocking on wait_for_service() for the full
// timeout, we poll in 200ms increments. This allows checking emergency/active
// flags each iteration so the arm sequence can be aborted within ~200ms of a
// preemption event. The same pattern is used for the response future (50ms
// increments). Without this, a blocking wait_for() would prevent emergency
// preemption until the full timeout expired.
StepResult UavManagerNode::callArmService(const bool arm)
{
  if (!armClient_) {
    return StepResult::fail(StepCode::ArmServiceUnavailable);
  }

  {
    const auto deadline = std::chrono::steady_clock::now() + secondsToMillis(serviceWaitS_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (!active_ || isEmergency()) {
        return StepResult::fail(StepCode::EmergencyPreempt);
      }
      if (armClient_->wait_for_service(std::chrono::milliseconds(200))) {
        break;
      }
    }
    if (!armClient_->service_is_ready()) {
      return StepResult::fail(StepCode::ArmServiceUnavailable);
    }
  }

  auto request = std::make_shared<peregrine_interfaces::srv::Arm::Request>();
  request->arm = arm;
  auto future = armClient_->async_send_request(request);
  // `future` is std::shared_future<Response>:: a synchronization primitive representing
  // a value that will be produced asynchronously by the executor thread.

  // Poll the future with emergency/active checks.
  const auto deadline = std::chrono::steady_clock::now() + secondsToMillis(serviceResponseWaitS_);
  while (future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= deadline) {
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

// Same polling pattern as callArmService: bounded waits with periodic
// emergency/preemption checks on both service availability and response future.
StepResult UavManagerNode::callSetModeService(const std::string & mode)
{
  if (!setModeClient_) {
    return StepResult::fail(StepCode::SetModeServiceUnavailable);
  }

  {
    const auto deadline = std::chrono::steady_clock::now() + secondsToMillis(serviceWaitS_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (!active_ || isEmergency()) {
        return StepResult::fail(StepCode::EmergencyPreempt);
      }
      if (setModeClient_->wait_for_service(std::chrono::milliseconds(200))) {
        break;
      }
    }
    if (!setModeClient_->service_is_ready()) {
      return StepResult::fail(StepCode::SetModeServiceUnavailable);
    }
  }

  auto request = std::make_shared<peregrine_interfaces::srv::SetMode::Request>();
  request->mode = mode;
  auto future = setModeClient_->async_send_request(request);

  const auto deadline = std::chrono::steady_clock::now() + secondsToMillis(serviceResponseWaitS_);
  while (future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= deadline) {
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

// The waitForXxx helpers below pass empty (always-false) preempted/emergency
// lambdas. This is intentional: these helpers are building blocks called from
// ensureArmableMode() and similar utility functions that don't own a goal
// handle. The actual preemption is handled by the CALLER -- onTakeoffAccepted
// etc. wraps the call inside orchestrator_->callStep() which provides its own
// preempted/emergency lambdas at the outer orchestration level.
StepResult UavManagerNode::waitForArmed(
  const bool armed,
  const std::chrono::milliseconds timeout) const
{
  return orchestrator_->waitForCondition(
    "ARMED_TIMEOUT", timeout,
    [this, armed]()
    {
      std::scoped_lock lock(mutex_);
      return latestPx4Status_.has_value() && latestPx4Status_->armed == armed;
    },
    []() {return false;}, []() {return false;});
}

StepResult UavManagerNode::waitForOffboard(
  const bool offboard,
  const std::chrono::milliseconds timeout) const
{
  return orchestrator_->waitForCondition(
    "OFFBOARD_TIMEOUT", timeout,
    [this, offboard]()
    {
      std::scoped_lock lock(mutex_);
      return latestPx4Status_.has_value() && latestPx4Status_->offboard == offboard;
    },
    []() {return false;}, []() {return false;});
}

StepResult UavManagerNode::waitForNavState(
  const uint8_t navState,
  const std::chrono::milliseconds timeout) const
{
  return orchestrator_->waitForCondition(
    "NAV_STATE_TIMEOUT", timeout,
    [this, navState]()
    {
      std::scoped_lock lock(mutex_);
      return latestPx4Status_.has_value() && latestPx4Status_->nav_state == navState;
    },
    []() {return false;}, []() {return false;});
}

StepResult UavManagerNode::waitForControlSetpointFlow(const std::chrono::milliseconds timeout) const
{
  // Confirms control_manager has begun publishing healthy outputs before offboard entry.
  return orchestrator_->waitForCondition(
    "SETPOINT_FLOW_TIMEOUT", timeout,
    [this]()
    {
      std::scoped_lock lock(mutex_);
      return latestControlStatus_.has_value() && latestControlStatus_->active && latestControlStatus_->healthy;
    },
    []() {return false;}, []() {return false;});
}

// Forwards an ExecuteTrajectory goal to the downstream trajectory_manager action
// server. The 3-phase pattern is:
//   Phase 1: Wait for downstream action server availability with periodic
//            emergency/preemption checks (200ms poll intervals).
//   Phase 2: Send goal and poll for acceptance (50ms intervals). If the
//            downstream server rejects, we fail immediately.
//   Phase 3: Poll for result while checking preemption each iteration.
//            On preemption, cancel propagation is fire-and-forget via
//            async_cancel_goal() -- we return immediately without waiting for
//            the cancel acknowledgment, because the caller needs to finalize
//            the upstream goal handle promptly.
StepResult UavManagerNode::forwardExecuteTrajectory(
  const ExecuteTrajectory::Goal & goal,
  std::function<void(const ExecuteTrajectory::Feedback &)> feedbackCallback,
  const std::function<bool()> & preempted,
  const std::function<bool()> & emergency, ExecuteTrajectory::Result * resultOut) const
{
  const auto serverDeadline = std::chrono::steady_clock::now() +
    secondsToMillis(actionServerWaitS_);
  while (std::chrono::steady_clock::now() < serverDeadline) {
    if (emergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (trajectoryExecuteClient_ &&
      trajectoryExecuteClient_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
      break;
    }
  }

  if (!trajectoryExecuteClient_ || !trajectoryExecuteClient_->action_server_is_ready()) {
    return StepResult::fail(StepCode::TrajectoryExecuteServerUnavailable);
  }

  rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions options;
  options.feedback_callback =
    [feedbackCallback = std::move(feedbackCallback)](auto,
      const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback)
    {
      // Init-capture with `std::move(...)` transfers ownership of the std::function into
      // the lambda object, avoiding an extra heap allocation/copy for the callback wrapper.
      if (feedbackCallback && feedback) {
        feedbackCallback(*feedback);
      }
    };

  auto goalHandleFuture = trajectoryExecuteClient_->async_send_goal(goal, options);
  // Wait for goal acceptance using bounded polling to keep callback interruptible.
  const auto goalDeadline = std::chrono::steady_clock::now() +
    secondsToMillis(serviceResponseWaitS_);
  while (goalHandleFuture.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= goalDeadline) {
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
  // While waiting for result, propagate local preemption to downstream cancel.
  const auto resultDeadline = std::chrono::steady_clock::now() +
    secondsToMillis(actionResultWaitS_);
  while (resultFuture.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= resultDeadline) {
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

// Same 3-phase forward pattern as forwardExecuteTrajectory (wait for server,
// send goal with bounded acceptance poll, poll result with cancel propagation).
// See forwardExecuteTrajectory comments for detailed explanation.
StepResult UavManagerNode::forwardGoTo(
  const GoTo::Goal & goal,
  std::function<void(const GoTo::Feedback &)> feedbackCallback,
  const std::function<bool()> & preempted,
  const std::function<bool()> & emergency,
  GoTo::Result * resultOut) const
{
  const auto serverDeadline = std::chrono::steady_clock::now() +
    secondsToMillis(actionServerWaitS_);
  while (std::chrono::steady_clock::now() < serverDeadline) {
    if (emergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (trajectoryGoToClient_ &&
      trajectoryGoToClient_->wait_for_action_server(std::chrono::milliseconds(200)))
    {
      break;
    }
  }

  if (!trajectoryGoToClient_ || !trajectoryGoToClient_->action_server_is_ready()) {
    return StepResult::fail(StepCode::TrajectoryGotoServerUnavailable);
  }

  rclcpp_action::Client<GoTo>::SendGoalOptions options;
  options.feedback_callback =
    [feedbackCallback = std::move(feedbackCallback)](auto,
      const std::shared_ptr<const GoTo::Feedback> feedback)
    {
      if (feedbackCallback && feedback) {
        feedbackCallback(*feedback);
      }
    };

  auto goalHandleFuture = trajectoryGoToClient_->async_send_goal(goal, options);
  // Wait for goal acceptance using bounded polling to keep callback interruptible.
  const auto goalDeadline = std::chrono::steady_clock::now() +
    secondsToMillis(serviceResponseWaitS_);
  while (goalHandleFuture.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= goalDeadline) {
      return StepResult::fail(StepCode::TrajectoryGotoGoalTimeout);
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
    return StepResult::fail(StepCode::TrajectoryGotoGoalRejected);
  }

  auto resultFuture = trajectoryGoToClient_->async_get_result(goalHandle);
  // While waiting for result, propagate local preemption to downstream cancel.
  const auto resultDeadline = std::chrono::steady_clock::now() +
    secondsToMillis(actionResultWaitS_);
  while (resultFuture.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() >= resultDeadline) {
      return StepResult::fail(StepCode::TrajectoryGotoResultTimeout);
    }
    if (emergency()) {
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
  // steady_clock is monotonic and never jumps backward/forward due to NTP or sim-time.
  // Use it for freshness/timeout math; use ROS time only for message timestamps.
  return std::chrono::steady_clock::now();
}

}  // namespace uav_manager

// Registers UavManagerNode as a composable component so it can be loaded into a
// component_container at runtime via launch files. This node requires
// component_container_mt (MultiThreadedExecutor) because the Reentrant action
// callback group needs multiple executor threads to dispatch concurrent
// goal/cancel callbacks while an accepted callback is blocked.
RCLCPP_COMPONENTS_REGISTER_NODE(uav_manager::UavManagerNode)
