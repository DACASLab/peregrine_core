#include <safety_monitor/safety_monitor_node.hpp>

#include <safety_monitor/battery_checker.hpp>
#include <safety_monitor/envelope_checker.hpp>
#include <safety_monitor/geofence_checker.hpp>
#include <safety_monitor/gps_checker.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <chrono>

namespace safety_monitor
{
namespace
{

constexpr char kNodeName[] = "safety_monitor";

}  // namespace

SafetyMonitorNode::SafetyMonitorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kNodeName, options)
{
  evaluateRateHz_ = this->declare_parameter<double>("evaluate_rate_hz", 2.0);
  commandLandEnabled_ = this->declare_parameter<bool>("command_land_enabled", false);
  gpsFreshnessTimeoutS_ = this->declare_parameter<double>("rules.gps.freshness_timeout_s", 2.0);
  autoStart_ = this->declare_parameter<bool>("auto_start", true);
  batteryTopic_ = this->declare_parameter<std::string>("battery_topic", "battery");
  gpsStatusTopic_ = this->declare_parameter<std::string>("gps_status_topic", "gps_status");
  estimatedStateTopic_ =
    this->declare_parameter<std::string>("estimated_state_topic", "estimated_state");
  px4StatusTopic_ = this->declare_parameter<std::string>("px4_status_topic", "status");

  // Battery checker params
  this->declare_parameter<bool>("rules.battery.enabled", true);
  this->declare_parameter<double>("rules.battery.warn_pct", 0.25);
  this->declare_parameter<double>("rules.battery.critical_pct", 0.15);
  this->declare_parameter<double>("rules.battery.emergency_pct", 0.10);
  this->declare_parameter<double>("rules.battery.min_voltage", 10.0);
  this->declare_parameter<double>("rules.battery.warn_grace_s", 5.0);
  this->declare_parameter<double>("rules.battery.critical_grace_s", 2.0);

  // GPS checker params
  this->declare_parameter<bool>("rules.gps.enabled", true);
  this->declare_parameter<int>("rules.gps.min_fix_type", 3);
  this->declare_parameter<double>("rules.gps.max_hdop", 5.0);
  this->declare_parameter<double>("rules.gps.max_vdop", 5.0);
  this->declare_parameter<int>("rules.gps.min_satellites", 6);
  this->declare_parameter<double>("rules.gps.warn_grace_s", 5.0);
  this->declare_parameter<double>("rules.gps.critical_grace_s", 3.0);

  // Geofence checker params
  this->declare_parameter<bool>("rules.geofence.enabled", true);
  this->declare_parameter<double>("rules.geofence.max_radius_m", 500.0);
  this->declare_parameter<double>("rules.geofence.max_altitude_m", 120.0);
  this->declare_parameter<double>("rules.geofence.min_altitude_m", -5.0);
  this->declare_parameter<double>("rules.geofence.warn_grace_s", 3.0);
  this->declare_parameter<double>("rules.geofence.critical_grace_s", 1.0);

  // Envelope checker params
  this->declare_parameter<bool>("rules.envelope.enabled", true);
  this->declare_parameter<double>("rules.envelope.max_velocity_ms", 15.0);
  this->declare_parameter<double>("rules.envelope.max_altitude_m", 120.0);
  this->declare_parameter<double>("rules.envelope.max_tilt_rad", 0.7);
  this->declare_parameter<double>("rules.envelope.warn_grace_s", 3.0);
  this->declare_parameter<double>("rules.envelope.critical_grace_s", 1.0);

  // Global params
  this->declare_parameter<double>("global.healthy_auto_clear_s", 3.0);
  this->declare_parameter<double>("global.land_command_timeout_s", 5.0);
  this->declare_parameter<int>("global.land_command_retry_count", 3);

  if (autoStart_) {
    startupTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
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
        auto activateResult = this->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        if (activateResult.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          RCLCPP_ERROR(get_logger(), "Auto-activate failed (state=%s)",
            activateResult.label().c_str());
          return;
        }
        RCLCPP_INFO(get_logger(), "Auto-start complete: ACTIVE");
      });
  }
}

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (evaluateRateHz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "evaluate_rate_hz must be > 0");
    return CallbackReturn::FAILURE;
  }

  // Build rule engine
  RuleEngineConfig engineConfig;
  engineConfig.healthy_auto_clear_s = this->get_parameter("global.healthy_auto_clear_s").as_double();
  ruleEngine_ = std::make_unique<RuleEngine>(engineConfig);

  // Battery checker
  if (this->get_parameter("rules.battery.enabled").as_bool()) {
    BatteryCheckerConfig cfg;
    cfg.warn_pct = static_cast<float>(this->get_parameter("rules.battery.warn_pct").as_double());
    cfg.critical_pct = static_cast<float>(this->get_parameter("rules.battery.critical_pct").as_double());
    cfg.emergency_pct = static_cast<float>(this->get_parameter("rules.battery.emergency_pct").as_double());
    cfg.min_voltage = static_cast<float>(this->get_parameter("rules.battery.min_voltage").as_double());
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = this->get_parameter("rules.battery.warn_grace_s").as_double();
    rule.critical_grace_s = this->get_parameter("rules.battery.critical_grace_s").as_double();
    ruleEngine_->addChecker(std::make_shared<BatteryChecker>(cfg), rule);
  }

  // GPS checker
  if (this->get_parameter("rules.gps.enabled").as_bool()) {
    GpsCheckerConfig cfg;
    cfg.min_fix_type = this->get_parameter("rules.gps.min_fix_type").as_int();
    cfg.max_hdop = static_cast<float>(this->get_parameter("rules.gps.max_hdop").as_double());
    cfg.max_vdop = static_cast<float>(this->get_parameter("rules.gps.max_vdop").as_double());
    cfg.min_satellites = this->get_parameter("rules.gps.min_satellites").as_int();
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = this->get_parameter("rules.gps.warn_grace_s").as_double();
    rule.critical_grace_s = this->get_parameter("rules.gps.critical_grace_s").as_double();
    ruleEngine_->addChecker(std::make_shared<GpsChecker>(cfg), rule);
  }

  // Geofence checker
  if (this->get_parameter("rules.geofence.enabled").as_bool()) {
    GeofenceCheckerConfig cfg;
    cfg.max_radius_m = this->get_parameter("rules.geofence.max_radius_m").as_double();
    cfg.max_altitude_m = this->get_parameter("rules.geofence.max_altitude_m").as_double();
    cfg.min_altitude_m = this->get_parameter("rules.geofence.min_altitude_m").as_double();
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = this->get_parameter("rules.geofence.warn_grace_s").as_double();
    rule.critical_grace_s = this->get_parameter("rules.geofence.critical_grace_s").as_double();
    ruleEngine_->addChecker(std::make_shared<GeofenceChecker>(cfg), rule);
  }

  // Envelope checker
  if (this->get_parameter("rules.envelope.enabled").as_bool()) {
    EnvelopeCheckerConfig cfg;
    cfg.max_velocity_ms = this->get_parameter("rules.envelope.max_velocity_ms").as_double();
    cfg.max_altitude_m = this->get_parameter("rules.envelope.max_altitude_m").as_double();
    cfg.max_tilt_rad = this->get_parameter("rules.envelope.max_tilt_rad").as_double();
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = this->get_parameter("rules.envelope.warn_grace_s").as_double();
    rule.critical_grace_s = this->get_parameter("rules.envelope.critical_grace_s").as_double();
    ruleEngine_->addChecker(std::make_shared<EnvelopeChecker>(cfg), rule);
  }

  // Subscriptions
  const auto qos = rclcpp::QoS(20).reliable();
  batterySub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    batteryTopic_, qos, std::bind(&SafetyMonitorNode::onBattery, this, std::placeholders::_1));
  gpsStatusSub_ = this->create_subscription<peregrine_interfaces::msg::GpsStatus>(
    gpsStatusTopic_, qos, std::bind(&SafetyMonitorNode::onGpsStatus, this, std::placeholders::_1));
  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    estimatedStateTopic_, qos,
    std::bind(&SafetyMonitorNode::onEstimatedState, this, std::placeholders::_1));
  px4StatusSub_ = this->create_subscription<peregrine_interfaces::msg::PX4Status>(
    px4StatusTopic_, rclcpp::QoS(10).reliable(),
    std::bind(&SafetyMonitorNode::onPx4Status, this, std::placeholders::_1));

  // Publisher
  safetyStatusPub_ = this->create_publisher<peregrine_interfaces::msg::SafetyStatus>(
    "safety_status", rclcpp::QoS(10).reliable());

  // Set mode client for land commands
  setModeClient_ = this->create_client<peregrine_interfaces::srv::SetMode>("set_mode");

  SafetyActionConfig actionConfig;
  actionConfig.land_command_timeout_s = this->get_parameter("global.land_command_timeout_s").as_double();
  actionConfig.land_command_retry_count = this->get_parameter("global.land_command_retry_count").as_int();
  actionExecutor_ = std::make_unique<SafetyActionExecutor>(
    setModeClient_, get_logger(), actionConfig);

  // Evaluation timer (created but not started until activate)
  const auto period = std::chrono::duration<double>(1.0 / evaluateRateHz_);
  evaluateTimer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SafetyMonitorNode::evaluateAndPublish, this));
  evaluateTimer_->cancel();

  RCLCPP_INFO(
    get_logger(),
    "Configured safety_monitor (rate=%.1fHz, land_enabled=%s, battery_topic=%s, gps_topic=%s, "
    "state_topic=%s, px4_topic=%s)",
    evaluateRateHz_, commandLandEnabled_ ? "true" : "false", batteryTopic_.c_str(),
    gpsStatusTopic_.c_str(), estimatedStateTopic_.c_str(), px4StatusTopic_.c_str());
  return CallbackReturn::SUCCESS;
}

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!safetyStatusPub_ || !evaluateTimer_) {
    return CallbackReturn::FAILURE;
  }
  safetyStatusPub_->on_activate();
  evaluateTimer_->reset();
  RCLCPP_INFO(get_logger(), "Activated safety_monitor");
  return CallbackReturn::SUCCESS;
}

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (evaluateTimer_) {
    evaluateTimer_->cancel();
  }
  if (safetyStatusPub_) {
    safetyStatusPub_->on_deactivate();
  }
  RCLCPP_INFO(get_logger(), "Deactivated safety_monitor");
  return CallbackReturn::SUCCESS;
}

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  evaluateTimer_.reset();
  batterySub_.reset();
  gpsStatusSub_.reset();
  estimatedStateSub_.reset();
  px4StatusSub_.reset();
  safetyStatusPub_.reset();
  setModeClient_.reset();
  ruleEngine_.reset();
  actionExecutor_.reset();

  {
    std::scoped_lock lock(mutex_);
    latestBattery_.reset();
    latestGps_.reset();
    latestPosition_.reset();
    latestPx4_.reset();
  }

  RCLCPP_INFO(get_logger(), "Cleaned up safety_monitor");
  return CallbackReturn::SUCCESS;
}

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  (void)on_cleanup(this->get_current_state());
  return CallbackReturn::SUCCESS;
}

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_error(
  const rclcpp_lifecycle::State &)
{
  if (evaluateTimer_) {
    evaluateTimer_->cancel();
  }
  if (safetyStatusPub_ && safetyStatusPub_->is_activated()) {
    safetyStatusPub_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

void SafetyMonitorNode::onBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  BatteryData data;
  data.percentage = msg->percentage;
  data.voltage = msg->voltage;
  latestBattery_ = data;
}

void SafetyMonitorNode::onGpsStatus(const peregrine_interfaces::msg::GpsStatus::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  GpsData data;
  data.fix_type = msg->fix_type;
  data.hdop = msg->hdop;
  data.vdop = msg->vdop;
  data.eph = msg->eph;
  data.epv = msg->epv;
  data.satellites_used = msg->satellites_used;
  latestGps_ = data;
  lastGpsTime_ = std::chrono::steady_clock::now();
}

void SafetyMonitorNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  PositionData data;
  data.x = msg->pose.pose.position.x;
  data.y = msg->pose.pose.position.y;
  data.z = msg->pose.pose.position.z;
  data.vx = msg->twist.twist.linear.x;
  data.vy = msg->twist.twist.linear.y;
  data.vz = msg->twist.twist.linear.z;

  // Extract roll and pitch from quaternion
  const auto & q = msg->pose.pose.orientation;
  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  data.roll = std::atan2(sinr_cosp, cosr_cosp);
  // Pitch (y-axis rotation)
  double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  data.pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

  latestPosition_ = data;
}

void SafetyMonitorNode::onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  Px4Data data;
  data.connected = msg->connected;
  data.armed = msg->armed;
  data.failsafe = msg->failsafe;
  latestPx4_ = data;
}

CheckerContext SafetyMonitorNode::buildContext() const
{
  // Must be called with mutex_ held
  CheckerContext ctx;
  ctx.battery = latestBattery_;
  ctx.position = latestPosition_;
  ctx.px4 = latestPx4_;

  // Check GPS freshness
  if (latestGps_.has_value()) {
    auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - lastGpsTime_).count();
    if (elapsed <= gpsFreshnessTimeoutS_) {
      ctx.gps = latestGps_;
    }
    // else: stale GPS treated as missing
  }

  return ctx;
}

void SafetyMonitorNode::evaluateAndPublish()
{
  if (!ruleEngine_ || !safetyStatusPub_ || !safetyStatusPub_->is_activated()) {
    return;
  }

  CheckerContext ctx;
  {
    std::scoped_lock lock(mutex_);
    ctx = buildContext();
  }

  const auto now = std::chrono::steady_clock::now();
  auto evalResult = ruleEngine_->evaluateDetailed(ctx, now);

  // Build and publish SafetyStatus
  peregrine_interfaces::msg::SafetyStatus statusMsg;
  statusMsg.header.stamp = this->now();
  statusMsg.level = static_cast<uint8_t>(evalResult.overallLevel);

  std::string overallReason;
  for (size_t i = 0; i < evalResult.results.size(); ++i) {
    peregrine_interfaces::msg::SafetyCheckerResult checkerResult;
    checkerResult.level = static_cast<uint8_t>(evalResult.results[i].level);
    checkerResult.reason = evalResult.results[i].reason;

    // Use the checker name from the rule order
    // We store results in the same order as rules were added
    checkerResult.checker_name = evalResult.results[i].reason.empty()
      ? "unknown" : evalResult.results[i].reason.substr(0, evalResult.results[i].reason.find('='));

    statusMsg.checker_results.push_back(checkerResult);

    if (evalResult.results[i].level != SafetyLevel::Nominal) {
      if (!overallReason.empty()) {
        overallReason += "; ";
      }
      overallReason += evalResult.results[i].reason;
    }
  }

  statusMsg.reason = overallReason.empty() ? "nominal" : overallReason;
  safetyStatusPub_->publish(statusMsg);

  // Action execution
  if (commandLandEnabled_ && actionExecutor_) {
    if (evalResult.overallLevel >= SafetyLevel::Critical) {
      actionExecutor_->requestLand(overallReason);
    }
    actionExecutor_->tick(now);
  }
}

}  // namespace safety_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(safety_monitor::SafetyMonitorNode)
