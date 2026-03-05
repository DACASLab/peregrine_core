#include <safety_monitor/safety_monitor_node.hpp>

#include <safety_monitor/battery_checker.hpp>
#include <safety_monitor/envelope_checker.hpp>
#include <safety_monitor/geofence_checker.hpp>
#include <safety_monitor/gps_checker.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <map>

namespace safety_monitor
{
using namespace std::chrono_literals;

namespace
{

constexpr char kNodeName[] = "safety_monitor";

}  // namespace

SafetyMonitorNode::SafetyMonitorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kNodeName, options)
{
  const std::map<std::string, rclcpp::ParameterValue> parameterDefaults = {
    {"evaluate_rate_hz", rclcpp::ParameterValue(2.0)},
    {"command_land_enabled", rclcpp::ParameterValue(true)},
    {"gps.freshness_timeout_s", rclcpp::ParameterValue(2.0)},
    {"auto_start", rclcpp::ParameterValue(true)},
    {"battery_topic", rclcpp::ParameterValue(std::string("battery"))},
    {"gps_status_topic", rclcpp::ParameterValue(std::string("gps_status"))},
    {"estimated_state_topic", rclcpp::ParameterValue(std::string("estimated_state"))},
    {"px4_status_topic", rclcpp::ParameterValue(std::string("status"))},
    {"battery.enabled", rclcpp::ParameterValue(true)},
    {"battery.warn_pct", rclcpp::ParameterValue(0.25)},
    {"battery.critical_pct", rclcpp::ParameterValue(0.15)},
    {"battery.emergency_pct", rclcpp::ParameterValue(0.10)},
    {"battery.min_voltage", rclcpp::ParameterValue(10.0)},
    {"battery.warn_grace_s", rclcpp::ParameterValue(5.0)},
    {"battery.critical_grace_s", rclcpp::ParameterValue(2.0)},
    {"gps.enabled", rclcpp::ParameterValue(true)},
    {"gps.min_fix_type", rclcpp::ParameterValue(3)},
    {"gps.max_hdop", rclcpp::ParameterValue(5.0)},
    {"gps.max_vdop", rclcpp::ParameterValue(5.0)},
    {"gps.min_satellites", rclcpp::ParameterValue(6)},
    {"gps.warn_grace_s", rclcpp::ParameterValue(5.0)},
    {"gps.critical_grace_s", rclcpp::ParameterValue(3.0)},
    {"geofence.enabled", rclcpp::ParameterValue(true)},
    {"geofence.max_radius_m", rclcpp::ParameterValue(500.0)},
    {"geofence.max_altitude_m", rclcpp::ParameterValue(120.0)},
    {"geofence.min_altitude_m", rclcpp::ParameterValue(-5.0)},
    {"geofence.warn_grace_s", rclcpp::ParameterValue(3.0)},
    {"geofence.critical_grace_s", rclcpp::ParameterValue(1.0)},
    {"envelope.enabled", rclcpp::ParameterValue(true)},
    {"envelope.max_velocity_ms", rclcpp::ParameterValue(15.0)},
    {"envelope.max_altitude_m", rclcpp::ParameterValue(120.0)},
    {"envelope.max_tilt_rad", rclcpp::ParameterValue(0.7)},
    {"envelope.warn_grace_s", rclcpp::ParameterValue(3.0)},
    {"envelope.critical_grace_s", rclcpp::ParameterValue(1.0)},
    {"healthy_auto_clear_s", rclcpp::ParameterValue(3.0)},
    {"land_command_timeout_s", rclcpp::ParameterValue(5.0)},
    {"land_command_retry_count", rclcpp::ParameterValue(3)},
  };

  for (const auto & [name, value] : parameterDefaults) {
    this->declare_parameter(name, value);
  }

  evaluateRateHz_ = this->get_parameter("evaluate_rate_hz").as_double();
  commandLandEnabled_ = this->get_parameter("command_land_enabled").as_bool();
  gpsFreshnessTimeoutS_ = this->get_parameter("gps.freshness_timeout_s").as_double();
  autoStart_ = this->get_parameter("auto_start").as_bool();
  batteryTopic_ = this->get_parameter("battery_topic").as_string();
  gpsStatusTopic_ = this->get_parameter("gps_status_topic").as_string();
  estimatedStateTopic_ = this->get_parameter("estimated_state_topic").as_string();
  px4StatusTopic_ = this->get_parameter("px4_status_topic").as_string();

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

  geofenceChecker_.reset();

  // Build rule engine
  RuleEngineConfig engineConfig;
  engineConfig.healthy_auto_clear_s = this->get_parameter("healthy_auto_clear_s").as_double();
  ruleEngine_ = std::make_unique<RuleEngine>(engineConfig);

  // Battery checker
  if (this->get_parameter("battery.enabled").as_bool()) {
    BatteryCheckerConfig cfg;
    cfg.warn_pct = static_cast<float>(this->get_parameter("battery.warn_pct").as_double());
    cfg.critical_pct = static_cast<float>(this->get_parameter("battery.critical_pct").as_double());
    cfg.emergency_pct = static_cast<float>(this->get_parameter("battery.emergency_pct").as_double());
    cfg.min_voltage = static_cast<float>(this->get_parameter("battery.min_voltage").as_double());
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = this->get_parameter("battery.warn_grace_s").as_double();
    rule.critical_grace_s = this->get_parameter("battery.critical_grace_s").as_double();
    ruleEngine_->addChecker(std::make_shared<BatteryChecker>(cfg), rule);
  }

  // GPS checker
  if (this->get_parameter("gps.enabled").as_bool()) {
    GpsCheckerConfig cfg;
    cfg.min_fix_type = this->get_parameter("gps.min_fix_type").as_int();
    cfg.max_hdop = static_cast<float>(this->get_parameter("gps.max_hdop").as_double());
    cfg.max_vdop = static_cast<float>(this->get_parameter("gps.max_vdop").as_double());
    cfg.min_satellites = this->get_parameter("gps.min_satellites").as_int();
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = this->get_parameter("gps.warn_grace_s").as_double();
    rule.critical_grace_s = this->get_parameter("gps.critical_grace_s").as_double();
    ruleEngine_->addChecker(std::make_shared<GpsChecker>(cfg), rule);
  }

  // Geofence checker
  if (this->get_parameter("geofence.enabled").as_bool()) {
    GeofenceCheckerConfig cfg;
    cfg.max_radius_m = this->get_parameter("geofence.max_radius_m").as_double();
    cfg.max_altitude_m = this->get_parameter("geofence.max_altitude_m").as_double();
    cfg.min_altitude_m = this->get_parameter("geofence.min_altitude_m").as_double();
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = this->get_parameter("geofence.warn_grace_s").as_double();
    rule.critical_grace_s = this->get_parameter("geofence.critical_grace_s").as_double();
    geofenceChecker_ = std::make_shared<GeofenceChecker>(cfg);
    ruleEngine_->addChecker(geofenceChecker_, rule);
  }

  // Envelope checker
  if (this->get_parameter("envelope.enabled").as_bool()) {
    EnvelopeCheckerConfig cfg;
    cfg.max_velocity_ms = this->get_parameter("envelope.max_velocity_ms").as_double();
    cfg.max_altitude_m = this->get_parameter("envelope.max_altitude_m").as_double();
    cfg.max_tilt_rad = this->get_parameter("envelope.max_tilt_rad").as_double();
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = this->get_parameter("envelope.warn_grace_s").as_double();
    rule.critical_grace_s = this->get_parameter("envelope.critical_grace_s").as_double();
    ruleEngine_->addChecker(std::make_shared<EnvelopeChecker>(cfg), rule);
  }

  // TF2 buffer and listener for odom→map frame transforms (geofence checks).
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  // Subscriptions
  const auto qos = rclcpp::QoS(20).reliable();
  batterySub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    batteryTopic_, qos, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) { onBattery(msg); });
  gpsStatusSub_ = this->create_subscription<peregrine_interfaces::msg::GpsStatus>(
    gpsStatusTopic_, qos, [this](const peregrine_interfaces::msg::GpsStatus::SharedPtr msg) { onGpsStatus(msg); });
  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    estimatedStateTopic_, qos, [this](const peregrine_interfaces::msg::State::SharedPtr msg) { onEstimatedState(msg); });
  px4StatusSub_ = this->create_subscription<peregrine_interfaces::msg::PX4Status>(
    px4StatusTopic_, rclcpp::QoS(10).reliable(),
    [this](const peregrine_interfaces::msg::PX4Status::SharedPtr msg) { onPx4Status(msg); });

  // Publisher
  safetyStatusPub_ = this->create_publisher<peregrine_interfaces::msg::SafetyStatus>(
    "safety_status", rclcpp::QoS(10).reliable());

  // Set mode client for land commands
  setModeClient_ = this->create_client<peregrine_interfaces::srv::SetMode>("set_mode");

  SafetyActionConfig actionConfig;
  actionConfig.land_command_timeout_s = this->get_parameter("land_command_timeout_s").as_double();
  actionConfig.land_command_retry_count = this->get_parameter("land_command_retry_count").as_int();
  actionExecutor_ = std::make_unique<SafetyActionExecutor>(
    setModeClient_, get_logger(), actionConfig, this->get_clock());

  // Evaluation timer (created but not started until activate)
  const auto period = std::chrono::duration<double>(1.0 / evaluateRateHz_);
  evaluateTimer_ = this->create_wall_timer(
    period, [this]() { evaluateAndPublish(); });
  geofenceCacheTimer_ = this->create_wall_timer(1s, [this]() { updateGeofenceCache(); });

  evaluateTimer_->cancel();
  geofenceCacheTimer_->cancel();

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
  if (!safetyStatusPub_ || !evaluateTimer_ || !geofenceCacheTimer_) {
    return CallbackReturn::FAILURE;
  }
  safetyStatusPub_->on_activate();
  evaluateTimer_->reset();
  geofenceCacheTimer_->reset();
  updateGeofenceCache();
  RCLCPP_INFO(get_logger(), "Activated safety_monitor");
  return CallbackReturn::SUCCESS;
}

SafetyMonitorNode::CallbackReturn SafetyMonitorNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (evaluateTimer_) {
    evaluateTimer_->cancel();
  }
  if (geofenceCacheTimer_) {
    geofenceCacheTimer_->cancel();
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
  geofenceCacheTimer_.reset();
  batterySub_.reset();
  gpsStatusSub_.reset();
  estimatedStateSub_.reset();
  px4StatusSub_.reset();
  safetyStatusPub_.reset();
  setModeClient_.reset();
  ruleEngine_.reset();
  geofenceChecker_.reset();
  actionExecutor_.reset();
  tfListener_.reset();
  tfBuffer_.reset();

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
  lastGpsTime_ = this->now();
}

void SafetyMonitorNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  if (!msg->header.frame_id.empty()) {
    odomFrame_ = msg->header.frame_id;
  }

  PositionData data;
  data.x = msg->pose.pose.position.x;
  data.y = msg->pose.pose.position.y;
  data.z = msg->pose.pose.position.z;
  // Velocity magnitude is frame-invariant; no rotation needed for envelope speed checks.
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
    auto elapsed = (this->now() - lastGpsTime_).seconds();
    if (elapsed <= gpsFreshnessTimeoutS_) {
      ctx.gps = latestGps_;
    }
    // else: stale GPS treated as missing
  }

  return ctx;
}

void SafetyMonitorNode::updateGeofenceCache()
{
  if (!geofenceChecker_ || !tfBuffer_ || odomFrame_.empty()) {
    return;
  }

  try {
    geometry_msgs::msg::PointStamped centerMap;
    centerMap.header.stamp = this->now();
    centerMap.header.frame_id = mapFrame_;
    centerMap.point.x = 0.0;
    centerMap.point.y = 0.0;
    centerMap.point.z = 0.0;

    geometry_msgs::msg::PointStamped minAltMap = centerMap;
    geometry_msgs::msg::PointStamped maxAltMap = centerMap;
    minAltMap.point.z = this->get_parameter("geofence.min_altitude_m").as_double();
    maxAltMap.point.z = this->get_parameter("geofence.max_altitude_m").as_double();

    const auto centerOdom = tfBuffer_->transform(centerMap, odomFrame_, tf2::durationFromSec(0.0));
    const auto minAltOdom = tfBuffer_->transform(minAltMap, odomFrame_, tf2::durationFromSec(0.0));
    const auto maxAltOdom = tfBuffer_->transform(maxAltMap, odomFrame_, tf2::durationFromSec(0.0));

    GeofenceBoundary boundary;
    boundary.center_x_m = centerOdom.point.x;
    boundary.center_y_m = centerOdom.point.y;
    boundary.max_radius_m = this->get_parameter("geofence.max_radius_m").as_double();
    boundary.min_altitude_m = std::min(minAltOdom.point.z, maxAltOdom.point.z);
    boundary.max_altitude_m = std::max(minAltOdom.point.z, maxAltOdom.point.z);
    geofenceChecker_->updateBoundary(boundary);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Failed to refresh geofence map->%s transform: %s", odomFrame_.c_str(), ex.what());
  }
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

  const auto steadyNow = std::chrono::steady_clock::now();
  auto evalResult = ruleEngine_->evaluateDetailed(ctx, steadyNow);

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

  // Action execution — only request land when vehicle is armed (in flight or
  // about to fly). Sending land to a grounded vehicle causes PX4 health issues
  // that block subsequent arming attempts.
  const bool vehicleArmed = ctx.px4.has_value() && ctx.px4->armed;
  if (commandLandEnabled_ && actionExecutor_) {
    if (evalResult.overallLevel >= SafetyLevel::Critical && vehicleArmed) {
      actionExecutor_->requestLand(overallReason);
    } else if (evalResult.overallLevel < SafetyLevel::Critical
      && !vehicleArmed
      && actionExecutor_->state() != LandCommandState::Idle)
    {
      // Vehicle is disarmed and situation recovered below Critical — reset
      // executor so it can handle future critical events. Without this, the
      // executor stays latched in Sent/Timeout/Rejected and ignores subsequent
      // requestLand() calls.
      RCLCPP_INFO(get_logger(), "Safety level nominal and vehicle disarmed, resetting action executor");
      actionExecutor_->reset();
    }
    actionExecutor_->tick(this->now());
  }
}

}  // namespace safety_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(safety_monitor::SafetyMonitorNode)
