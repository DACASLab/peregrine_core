#include <tui_status/tui_node.hpp>

#include <ncurses.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace tui_status
{
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

}  // namespace

TuiNode::TuiNode(const std::shared_ptr<Renderer> & renderer)
: Node("tui_status_node")
, renderer_(renderer)
, alertBuffer_(
    static_cast<std::size_t>(
      std::max<int64_t>(
        1,
        this->declare_parameter<int>("alert_buffer_size", 100))))
{
  const double refreshRateHz = this->declare_parameter<double>("refresh_rate_hz", 10.0);
  uavNamespace_ = this->declare_parameter<std::string>("uav_namespace", "");

  const auto qos = rclcpp::QoS(10).reliable();

  uavStateSub_ = this->create_subscription<peregrine_interfaces::msg::UAVState>(
    topicName("uav_state"), qos, std::bind(&TuiNode::onUavState, this, std::placeholders::_1));

  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    topicName("estimated_state"), qos,
    std::bind(&TuiNode::onEstimatedState, this, std::placeholders::_1));

  safetyStatusSub_ = this->create_subscription<peregrine_interfaces::msg::SafetyStatus>(
    topicName("safety_status"), qos,
    std::bind(&TuiNode::onSafetyStatus, this, std::placeholders::_1));

  estimationStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    topicName("estimation_status"), qos,
    std::bind(&TuiNode::onEstimationStatus, this, std::placeholders::_1));

  controlStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    topicName("control_status"), qos,
    std::bind(&TuiNode::onControlStatus, this, std::placeholders::_1));

  trajectoryStatusSub_ = this->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    topicName("trajectory_status"), qos,
    std::bind(&TuiNode::onTrajectoryStatus, this, std::placeholders::_1));

  px4StatusSub_ = this->create_subscription<peregrine_interfaces::msg::PX4Status>(
    topicName("status"), qos, std::bind(&TuiNode::onPx4Status, this, std::placeholders::_1));

  gpsStatusSub_ = this->create_subscription<peregrine_interfaces::msg::GpsStatus>(
    topicName("gps_status"), qos, std::bind(&TuiNode::onGpsStatus, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, refreshRateHz));
  refreshTimer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TuiNode::onTimer, this));

  RCLCPP_INFO(
    this->get_logger(),
    "tui_status started (refresh=%.1fHz, namespace='%s')",
    refreshRateHz, uavNamespace_.c_str());
}

bool TuiNode::shouldExit() const
{
  std::scoped_lock lock(mutex_);
  return exitRequested_;
}

std::string TuiNode::topicName(const std::string & base_topic) const
{
  if (uavNamespace_.empty() || uavNamespace_ == "/") {
    return base_topic;
  }

  std::string ns = uavNamespace_;
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

  for (int key = renderer_->pollKey(); key != ERR; key = renderer_->pollKey()) {
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
      continue;
    }

    if (key == KEY_UP) {
      const std::size_t count = alertBuffer_.size();
      std::scoped_lock lock(mutex_);
      if (alertScroll_ + 1 < count) {
        ++alertScroll_;
      }
      continue;
    }

    if (key == KEY_DOWN) {
      std::scoped_lock lock(mutex_);
      if (alertScroll_ > 0) {
        --alertScroll_;
      }
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

  renderer_->render(snapshot, alerts, scroll, uavNamespace_);
}

void TuiNode::onUavState(const peregrine_interfaces::msg::UAVState::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestUavState_ = *msg;
}

void TuiNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestEstimatedState_ = *msg;
}

void TuiNode::onSafetyStatus(const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg)
{
  {
    std::scoped_lock lock(mutex_);
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

void TuiNode::onEstimationStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestEstimationStatus_ = *msg;
}

void TuiNode::onControlStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestControlStatus_ = *msg;
}

void TuiNode::onTrajectoryStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestTrajectoryStatus_ = *msg;
}

void TuiNode::onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestPx4Status_ = *msg;
}

void TuiNode::onGpsStatus(const peregrine_interfaces::msg::GpsStatus::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestGpsStatus_ = *msg;
}

StatusSnapshot TuiNode::buildSnapshot() const
{
  StatusSnapshot snapshot;

  std::scoped_lock lock(mutex_);

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
  }

  if (latestControlStatus_.has_value()) {
    snapshot.control_module = latestControlStatus_->active_module;
    snapshot.control_healthy = latestControlStatus_->healthy;
  }

  if (latestTrajectoryStatus_.has_value()) {
    snapshot.trajectory_module = latestTrajectoryStatus_->active_module;
    snapshot.trajectory_healthy = latestTrajectoryStatus_->healthy;
  }

  if (latestPx4Status_.has_value()) {
    snapshot.battery_percent = latestPx4Status_->battery_remaining;
    snapshot.battery_voltage = latestPx4Status_->battery_voltage;
  }

  if (latestGpsStatus_.has_value()) {
    snapshot.gps_fix_type = latestGpsStatus_->fix_type;
    snapshot.gps_satellites = latestGpsStatus_->satellites_used;
    snapshot.gps_hdop = latestGpsStatus_->hdop;
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

  return snapshot;
}

}  // namespace tui_status
