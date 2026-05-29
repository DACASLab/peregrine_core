#include <tui_status/tui_node.hpp>

#include <ncurses.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace tui_status
{
using namespace std::chrono_literals;

namespace
{

template<typename T>
double radToDeg(const T radians)
{
  return radians * 180.0 / M_PI;
}

std::string uavStateToString(const uint8_t state)
{
  switch (state) {
    case peregrine_interfaces::msg::UAVState::STATE_IDLE:
      return "IDLE";
    case peregrine_interfaces::msg::UAVState::STATE_ARMED:
      return "ARMED";
    case peregrine_interfaces::msg::UAVState::STATE_TAKING_OFF:
      return "TAKING_OFF";
    case peregrine_interfaces::msg::UAVState::STATE_HOVERING:
      return "HOVERING";
    case peregrine_interfaces::msg::UAVState::STATE_FLYING:
      return "FLYING";
    case peregrine_interfaces::msg::UAVState::STATE_LANDING:
      return "LANDING";
    case peregrine_interfaces::msg::UAVState::STATE_LANDED:
      return "LANDED";
    case peregrine_interfaces::msg::UAVState::STATE_EMERGENCY:
      return "EMERGENCY";
    default:
      return "UNKNOWN";
  }
}

bool isFlightState(const uint8_t state)
{
  return state == peregrine_interfaces::msg::UAVState::STATE_TAKING_OFF ||
         state == peregrine_interfaces::msg::UAVState::STATE_HOVERING ||
         state == peregrine_interfaces::msg::UAVState::STATE_FLYING ||
         state == peregrine_interfaces::msg::UAVState::STATE_LANDING;
}

}  // namespace

TuiNode::TuiNode(const std::shared_ptr<Renderer> & renderer)
: Node("tui_status_node")
, paramListener_(std::make_shared<tui_status_node::ParamListener>(
    this->get_node_parameters_interface()))
, params_(paramListener_->get_params())
, renderer_(renderer)
, alertBuffer_(static_cast<std::size_t>(std::max(int64_t{1}, params_.alert_buffer_size)))

{

  const double refreshRateHz = params_.refresh_rate_hz;

  const auto qos = rclcpp::QoS(10).reliable();

  uavStateSub_ = this->create_subscription<peregrine_interfaces::msg::UAVState>(
    topicName("uav_state"), qos, [this](peregrine_interfaces::msg::UAVState::SharedPtr msg) { onUavState(msg); });

  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    topicName("estimated_state"), qos,
    [this](peregrine_interfaces::msg::State::SharedPtr msg) { onEstimatedState(msg); });

  safetyStatusSub_ = this->create_subscription<peregrine_interfaces::msg::SafetyStatus>(
    topicName("safety_status"), qos,
    [this](peregrine_interfaces::msg::SafetyStatus::SharedPtr msg) { onSafetyStatus(msg); });

  estimationStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    topicName("estimation_status"), qos,
    [this](peregrine_interfaces::msg::ManagerStatus::SharedPtr msg) {
      onManagerStatus(msg, hadFirstEstimationStatus_, lastEstimationHealthy_, latestEstimationStatus_, "EST");
    });

  controlStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    topicName("control_status"), qos,
    [this](peregrine_interfaces::msg::ManagerStatus::SharedPtr msg) {
      onManagerStatus(msg, hadFirstControlStatus_, lastControlHealthy_, latestControlStatus_, "CTL");
    });

  trajectoryStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    topicName("trajectory_status"), qos,
    [this](peregrine_interfaces::msg::ManagerStatus::SharedPtr msg) {
      onManagerStatus(msg, hadFirstTrajectoryStatus_, lastTrajectoryHealthy_, latestTrajectoryStatus_, "TRJ");
    });

  px4StatusSub_ = this->create_subscription<peregrine_interfaces::msg::PX4Status>(
    topicName("status"), qos, [this](peregrine_interfaces::msg::PX4Status::SharedPtr msg) { onPx4Status(msg); });

  gpsStatusSub_ = this->create_subscription<peregrine_interfaces::msg::GpsStatus>(
    topicName("gps_status"), qos, [this](peregrine_interfaces::msg::GpsStatus::SharedPtr msg) { onGpsStatus(msg); });

  computeStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ComputeStatus>(
    topicName("compute_status"), qos,
    [this](peregrine_interfaces::msg::ComputeStatus::SharedPtr msg) { onComputeStatus(msg); });

  // Command clients
  armClient_ = this->create_client<peregrine_interfaces::srv::Arm>(topicName("arm"));
  clearEmergencyClient_ = this->create_client<peregrine_interfaces::srv::ClearEmergency>(
    topicName("uav_manager/clear_emergency"));
  takeoffClient_ = rclcpp_action::create_client<peregrine_interfaces::action::Takeoff>(
    this, topicName("uav_manager/takeoff"));
  landClient_ = rclcpp_action::create_client<peregrine_interfaces::action::Land>(
    this, topicName("uav_manager/land"));

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, refreshRateHz));
  refreshTimer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { onTimer(); });

  RCLCPP_INFO(
    this->get_logger(),
    "tui_status started (refresh=%.1fHz, namespace='%s')",
    refreshRateHz, params_.uav_namespace.c_str());
}

bool TuiNode::shouldExit() const
{
  std::scoped_lock lock(mutex_);
  return exitRequested_;
}

std::string TuiNode::topicName(const std::string & base_topic) const
{
  if (params_.uav_namespace.empty() || params_.uav_namespace == "/") {
    return base_topic;
  }

  std::string ns = params_.uav_namespace;
  if (ns.front() != '/') {
    ns = "/" + ns;
  }
  if (ns.back() == '/') {
    ns.pop_back();
  }
  return ns + "/" + base_topic;
}

void TuiNode::onTimer()
{
  if (!renderer_ || !renderer_->initialized()) {
    return;
  }

  // Expire arm/disarm confirmation windows on each tick, not just on keypress
  {
    const auto now = this->now();
    if (armConfirmPending_ && now > confirmDeadline_) {
      armConfirmPending_ = false;
      setCommandStatus("ARM", "confirmation timed out");
    }
    if (disarmConfirmPending_ && now > confirmDeadline_) {
      disarmConfirmPending_ = false;
      setCommandStatus("DISARM", "confirmation timed out");
    }
  }

  for (int key = renderer_->pollKey(); key != ERR; key = renderer_->pollKey()) {
    handleKeyInput(key);
    if (shouldExit()) {
      return;
    }
  }

  StatusSnapshot snapshot = buildSnapshot();
  std::vector<AlertEntry> alerts = alertBuffer_.snapshot();

  std::size_t scroll = 0;
  {
    std::scoped_lock lock(mutex_);
    if (alerts.empty()) {
      alertScroll_ = 0;
    } else {
      alertScroll_ = std::min(alertScroll_, alerts.size() - 1);
    }
    scroll = alertScroll_;
  }

  renderer_->render(snapshot, alerts, scroll, params_.uav_namespace);
}

void TuiNode::handleKeyInput(int key)
{
  const auto now = this->now();

  if (key == 'q' || key == 'Q') {
    {
      std::scoped_lock lock(mutex_);
      exitRequested_ = true;
    }
    rclcpp::shutdown();
    return;
  }

  if (key == 'c' || key == 'C') {
    alertBuffer_.clear();
    std::scoped_lock lock(mutex_);
    alertScroll_ = 0;
    return;
  }

  if (key == KEY_UP) {
    const std::size_t count = alertBuffer_.size();
    std::scoped_lock lock(mutex_);
    if (alertScroll_ + 1 < count) {
      ++alertScroll_;
    }
    return;
  }

  if (key == KEY_DOWN) {
    std::scoped_lock lock(mutex_);
    if (alertScroll_ > 0) {
      --alertScroll_;
    }
    return;
  }

  if (key == 'a') {
    if (armConfirmPending_) {
      armConfirmPending_ = false;
      sendArm(true);
    } else {
      armConfirmPending_ = true;
      disarmConfirmPending_ = false;
      confirmDeadline_ = now + rclcpp::Duration(2, 0);
      setCommandStatus("ARM", "press 'a' again to confirm");
    }
    return;
  }

  if (key == 'd') {
    if (disarmConfirmPending_) {
      disarmConfirmPending_ = false;
      sendArm(false);
    } else {
      disarmConfirmPending_ = true;
      armConfirmPending_ = false;
      confirmDeadline_ = now + rclcpp::Duration(2, 0);
      setCommandStatus("DISARM", "press 'd' again to confirm");
    }
    return;
  }

  // Any other command key clears pending arm/disarm
  armConfirmPending_ = false;
  disarmConfirmPending_ = false;

  if (key == 't') {
    sendTakeoff();
    return;
  }

  if (key == 'l') {
    sendLand();
    return;
  }

  if (key == 'e') {
    sendClearEmergency();
    return;
  }
}

void TuiNode::setCommandStatus(const std::string & command, const std::string & result, bool pending)
{
  std::scoped_lock lock(mutex_);
  lastCommand_ = command;
  lastCommandResult_ = result;
  commandPending_ = pending;
}

void TuiNode::sendArm(bool arm)
{
  if (!armClient_->service_is_ready()) {
    setCommandStatus(arm ? "ARM" : "DISARM", "service not available");
    alertBuffer_.push(AlertSeverity::Warning, std::string(arm ? "ARM" : "DISARM") + " service not available");
    return;
  }

  auto request = std::make_shared<peregrine_interfaces::srv::Arm::Request>();
  request->arm = arm;
  setCommandStatus(arm ? "ARM" : "DISARM", "sending...", true);
  alertBuffer_.push(AlertSeverity::Info, std::string("Sending ") + (arm ? "ARM" : "DISARM") + " command");

  armClient_->async_send_request(
    request,
    [this, arm](rclcpp::Client<peregrine_interfaces::srv::Arm>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        setCommandStatus(arm ? "ARM" : "DISARM", "SUCCESS");
      } else {
        setCommandStatus(arm ? "ARM" : "DISARM", "FAILED: " + response->message);
        alertBuffer_.push(AlertSeverity::Error,
          std::string(arm ? "ARM" : "DISARM") + " failed: " + response->message);
      }
    });
}

void TuiNode::sendTakeoff()
{
  if (!takeoffClient_->action_server_is_ready()) {
    setCommandStatus("TAKEOFF", "action not available");
    alertBuffer_.push(AlertSeverity::Warning, "Takeoff action server not available");
    return;
  }

  auto goal = peregrine_interfaces::action::Takeoff::Goal();
  goal.target_altitude_m = 5.0;
  goal.climb_velocity_mps = 1.0;
  setCommandStatus("TAKEOFF", "sending goal...", true);
  alertBuffer_.push(AlertSeverity::Info, "Sending takeoff to 5.0m");

  auto options = rclcpp_action::Client<peregrine_interfaces::action::Takeoff>::SendGoalOptions();
  options.feedback_callback =
    [this](auto, const auto & feedback) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "alt=%.1fm progress=%.0f%%",
        feedback->current_altitude_m, feedback->progress * 100.0F);
      setCommandStatus("TAKEOFF", buf, true);
    };
  options.result_callback =
    [this](const auto & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "SUCCESS at %.1fm", result.result->final_altitude_m);
        setCommandStatus("TAKEOFF", buf);
        alertBuffer_.push(AlertSeverity::Info, "Takeoff complete");
      } else {
        setCommandStatus("TAKEOFF", "FAILED: " + result.result->message);
        alertBuffer_.push(AlertSeverity::Error, "Takeoff failed: " + result.result->message);
      }
    };

  takeoffClient_->async_send_goal(goal, options);
}

void TuiNode::sendLand()
{
  if (!landClient_->action_server_is_ready()) {
    setCommandStatus("LAND", "action not available");
    alertBuffer_.push(AlertSeverity::Warning, "Land action server not available");
    return;
  }

  auto goal = peregrine_interfaces::action::Land::Goal();
  goal.descent_velocity_mps = 0.8;
  setCommandStatus("LAND", "sending goal...", true);
  alertBuffer_.push(AlertSeverity::Info, "Sending land command");

  auto options = rclcpp_action::Client<peregrine_interfaces::action::Land>::SendGoalOptions();
  options.feedback_callback =
    [this](auto, const auto & feedback) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "alt=%.1fm progress=%.0f%%",
        feedback->current_altitude_m, feedback->progress * 100.0F);
      setCommandStatus("LAND", buf, true);
    };
  options.result_callback =
    [this](const auto & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        setCommandStatus("LAND", "SUCCESS");
        alertBuffer_.push(AlertSeverity::Info, "Landing complete");
      } else {
        setCommandStatus("LAND", "FAILED: " + result.result->message);
        alertBuffer_.push(AlertSeverity::Error, "Landing failed: " + result.result->message);
      }
    };

  landClient_->async_send_goal(goal, options);
}

void TuiNode::sendClearEmergency()
{
  if (!clearEmergencyClient_->service_is_ready()) {
    setCommandStatus("CLEAR EMERGENCY", "service not available");
    alertBuffer_.push(AlertSeverity::Warning, "Clear emergency service not available");
    return;
  }

  auto request = std::make_shared<peregrine_interfaces::srv::ClearEmergency::Request>();
  setCommandStatus("CLEAR EMERGENCY", "sending...", true);
  alertBuffer_.push(AlertSeverity::Info, "Sending clear emergency");

  clearEmergencyClient_->async_send_request(
    request,
    [this](rclcpp::Client<peregrine_interfaces::srv::ClearEmergency>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        setCommandStatus("CLEAR EMERGENCY", "SUCCESS");
        alertBuffer_.push(AlertSeverity::Info, "Emergency cleared");
      } else {
        setCommandStatus("CLEAR EMERGENCY", "FAILED: " + response->message);
        alertBuffer_.push(AlertSeverity::Error, "Clear emergency failed: " + response->message);
      }
    });
}

void TuiNode::onUavState(const peregrine_interfaces::msg::UAVState::SharedPtr msg)
{
  const std::string newState = uavStateToString(msg->state);

  std::scoped_lock lock(mutex_);

  // Transition alerts
  if (hadFirstUavState_) {
    if (newState != lastUavStateStr_) {
      alertBuffer_.push(
        AlertSeverity::Info,
        "State: " + lastUavStateStr_ + " -> " + newState);
    }
    if (msg->armed != lastArmed_) {
      alertBuffer_.push(
        msg->armed ? AlertSeverity::Warning : AlertSeverity::Info,
        msg->armed ? "Vehicle ARMED" : "Vehicle DISARMED");
    }
  }

  const auto now = this->now();

  // UAV uptime: time since first uav_state message
  if (!hadFirstUavState_) {
    firstUavStateTime_ = now;
  }

  // Armed time: accumulate while armed
  if (msg->armed && !wasArmed_) {
    armStartTime_ = now;
  } else if (!msg->armed && wasArmed_) {
    accumulatedArmedS_ += (now - armStartTime_).seconds();
  }
  wasArmed_ = msg->armed;

  // Flight timer: accumulate across takeoff/land cycles
  if (isFlightState(msg->state) && !inFlight_) {
    inFlight_ = true;
    flightStartTime_ = now;
  } else if (!isFlightState(msg->state) && inFlight_) {
    accumulatedFlightS_ += (now - flightStartTime_).seconds();
    inFlight_ = false;
  }

  lastUavStateStr_ = newState;
  lastArmed_ = msg->armed;
  hadFirstUavState_ = true;

  lastUavStateTime_ = this->now();
  receivedUavState_ = true;
  latestUavState_ = *msg;
}

void TuiNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  lastEstimatedStateTime_ = this->now();
  receivedEstimatedState_ = true;
  latestEstimatedState_ = *msg;
}

void TuiNode::onSafetyStatus(const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
    lastSafetyStatusTime_ = this->now();
    receivedSafetyStatus_ = true;
    latestSafetyStatus_ = *msg;
  }

  if (lastSafetyLevel_ != static_cast<int>(msg->level)) {
    AlertSeverity severity = AlertSeverity::Info;
    if (msg->level >= peregrine_interfaces::msg::SafetyStatus::LEVEL_CRITICAL) {
      severity = AlertSeverity::Error;
    } else if (msg->level == peregrine_interfaces::msg::SafetyStatus::LEVEL_WARNING) {
      severity = AlertSeverity::Warning;
    }

    alertBuffer_.push(
      severity,
      "Safety level changed to " + std::to_string(msg->level) + " reason=" + msg->reason);
    lastSafetyLevel_ = static_cast<int>(msg->level);
  }
}

void TuiNode::onManagerStatus(
  const peregrine_interfaces::msg::ManagerStatus::SharedPtr & msg,
  bool & hadFirst,
  bool & lastHealthy,
  std::optional<peregrine_interfaces::msg::ManagerStatus> & latestStatus,
  const char * label)
{
  std::scoped_lock lock(mutex_);

  if (hadFirst && msg->healthy != lastHealthy) {
    alertBuffer_.push(
      msg->healthy ? AlertSeverity::Info : AlertSeverity::Error,
      std::string(label) + " health: " + (msg->healthy ? "OK" : "BAD"));
  }
  lastHealthy = msg->healthy;
  hadFirst = true;

  latestStatus = *msg;
}

void TuiNode::onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);

  if (hadFirstPx4Status_) {
    if (msg->connected != lastConnected_) {
      alertBuffer_.push(
        msg->connected ? AlertSeverity::Info : AlertSeverity::Error,
        msg->connected ? "PX4 CONNECTED" : "PX4 DISCONNECTED");
    }
    if (msg->failsafe != lastFailsafe_) {
      alertBuffer_.push(
        msg->failsafe ? AlertSeverity::Error : AlertSeverity::Info,
        msg->failsafe ? "FAILSAFE ACTIVATED" : "Failsafe cleared");
    }
  }
  lastConnected_ = msg->connected;
  lastFailsafe_ = msg->failsafe;
  hadFirstPx4Status_ = true;

  lastPx4StatusTime_ = this->now();
  receivedPx4Status_ = true;
  latestPx4Status_ = *msg;
}

void TuiNode::onGpsStatus(const peregrine_interfaces::msg::GpsStatus::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  lastGpsStatusTime_ = this->now();
  receivedGpsStatus_ = true;
  latestGpsStatus_ = *msg;
}

void TuiNode::onComputeStatus(const peregrine_interfaces::msg::ComputeStatus::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestComputeStatus_ = *msg;
}

StatusSnapshot TuiNode::buildSnapshot() const
{
  StatusSnapshot snapshot;

  std::scoped_lock lock(mutex_);

  const rclcpp::Time now = this->now();

  // UAV uptime (time since first uav_state message from onboard stack)
  if (hadFirstUavState_) {
    snapshot.uav_uptime_s = (now - firstUavStateTime_).seconds();
  }

  // Armed time (accumulated + current arm session if still armed)
  snapshot.armed_time_s = accumulatedArmedS_;
  if (wasArmed_) {
    snapshot.armed_time_s += (now - armStartTime_).seconds();
  }

  // Flight time (accumulated + current flight if still in air)
  snapshot.flight_time_s = accumulatedFlightS_;
  if (inFlight_) {
    snapshot.flight_time_s += (now - flightStartTime_).seconds();
  }

  // Staleness
  if (receivedUavState_) {
    snapshot.uav_state_age_s = (now - lastUavStateTime_).seconds();
  }
  if (receivedEstimatedState_) {
    snapshot.estimated_state_age_s = (now - lastEstimatedStateTime_).seconds();
  }
  if (receivedSafetyStatus_) {
    snapshot.safety_status_age_s = (now - lastSafetyStatusTime_).seconds();
  }
  if (receivedPx4Status_) {
    snapshot.px4_status_age_s = (now - lastPx4StatusTime_).seconds();
  }
  if (receivedGpsStatus_) {
    snapshot.gps_status_age_s = (now - lastGpsStatusTime_).seconds();
  }

  if (latestUavState_.has_value()) {
    snapshot.state = uavStateToString(latestUavState_->state);
    snapshot.mode = latestUavState_->mode;
    snapshot.armed = latestUavState_->armed;
    snapshot.offboard = latestUavState_->offboard;
    snapshot.connected = latestUavState_->connected;
    snapshot.failsafe = latestUavState_->failsafe;
    snapshot.dependencies_ready = latestUavState_->dependencies_ready;
    snapshot.safety_ready = latestUavState_->safety_ready;
    snapshot.readiness_detail = latestUavState_->readiness_detail;
  }

  if (latestEstimatedState_.has_value()) {
    snapshot.has_pose = true;
    snapshot.x_m = latestEstimatedState_->pose.pose.position.x;
    snapshot.y_m = latestEstimatedState_->pose.pose.position.y;
    snapshot.z_m = latestEstimatedState_->pose.pose.position.z;

    snapshot.has_velocity = true;
    snapshot.vx_mps = latestEstimatedState_->twist.twist.linear.x;
    snapshot.vy_mps = latestEstimatedState_->twist.twist.linear.y;
    snapshot.vz_mps = latestEstimatedState_->twist.twist.linear.z;

    const auto & q = latestEstimatedState_->pose.pose.orientation;
    const double sinr = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double roll = std::atan2(sinr, cosr);

    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    const double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny, cosy);

    snapshot.roll_deg = radToDeg(roll);
    snapshot.pitch_deg = radToDeg(pitch);
    snapshot.yaw_deg = radToDeg(yaw);
  }

  if (latestEstimationStatus_.has_value()) {
    snapshot.estimation_module = latestEstimationStatus_->active_module;
    snapshot.estimation_healthy = latestEstimationStatus_->healthy;
    snapshot.estimation_rate_hz = latestEstimationStatus_->output_rate_hz;
  }

  if (latestControlStatus_.has_value()) {
    snapshot.control_module = latestControlStatus_->active_module;
    snapshot.control_healthy = latestControlStatus_->healthy;
    snapshot.control_rate_hz = latestControlStatus_->output_rate_hz;
  }

  if (latestTrajectoryStatus_.has_value()) {
    snapshot.trajectory_module = latestTrajectoryStatus_->active_module;
    snapshot.trajectory_healthy = latestTrajectoryStatus_->healthy;
    snapshot.trajectory_rate_hz = latestTrajectoryStatus_->output_rate_hz;
  }

  if (latestPx4Status_.has_value()) {
    snapshot.battery_percent = latestPx4Status_->battery_remaining;
    snapshot.battery_voltage = latestPx4Status_->battery_voltage;
    bool anyMotor = false;
    for (int i = 0; i < 4; ++i) {
      snapshot.motor_output[i] = latestPx4Status_->motor_output[i];
      if (snapshot.motor_output[i] > 0.0F) {
        anyMotor = true;
      }
    }
    snapshot.has_motor_data = anyMotor;
  }

  if (latestGpsStatus_.has_value()) {
    snapshot.gps_fix_type = latestGpsStatus_->fix_type;
    snapshot.gps_satellites = latestGpsStatus_->satellites_used;
    snapshot.gps_hdop = latestGpsStatus_->hdop;
    snapshot.gps_vdop = latestGpsStatus_->vdop;
    snapshot.gps_eph = latestGpsStatus_->eph;
    snapshot.gps_epv = latestGpsStatus_->epv;
  }

  if (latestSafetyStatus_.has_value()) {
    snapshot.has_safety_status = true;
    snapshot.safety_level = latestSafetyStatus_->level;
    snapshot.safety_reason = latestSafetyStatus_->reason;

    snapshot.checker_levels.clear();
    for (const auto & checker : latestSafetyStatus_->checker_results) {
      snapshot.checker_levels.push_back(SafetyCheckerView{checker.checker_name, checker.level, checker.reason});
    }
  } else {
    snapshot.has_safety_status = false;
    snapshot.safety_reason = "waiting_safety_status";
  }

  if (latestComputeStatus_.has_value()) {
    snapshot.has_compute_status = true;
    snapshot.compute.cpu_percent = latestComputeStatus_->cpu_percent;
    snapshot.compute.cpu_freq_ghz = latestComputeStatus_->cpu_freq_ghz;
    snapshot.compute.cpu_temp_c = latestComputeStatus_->cpu_temp_c;
    snapshot.compute.ram_used_gb = latestComputeStatus_->ram_used_gb;
    snapshot.compute.ram_total_gb = latestComputeStatus_->ram_total_gb;
  }

  snapshot.last_command = lastCommand_;
  snapshot.last_command_result = lastCommandResult_;
  snapshot.command_pending = commandPending_;

  return snapshot;
}

}  // namespace tui_status
