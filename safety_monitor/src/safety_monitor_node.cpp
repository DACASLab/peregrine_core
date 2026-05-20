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

#include <cmath>
#include <chrono>

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
  paramListener_ = std::make_shared<safety_monitor::ParamListener>(
      get_node_parameters_interface());
  params_ = paramListener_->get_params();

  if (params_.auto_start) {
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
  // Build rule engine
  RuleEngineConfig engineConfig;
  engineConfig.healthy_auto_clear_s = params_.healthy_auto_clear_s;
  ruleEngine_ = std::make_unique<RuleEngine>(engineConfig);

  // Battery checker
  if (params_.battery.enabled) {
    BatteryCheckerConfig cfg;
    cfg.warn_pct = static_cast<float>(params_.battery.warn_pct);
    cfg.critical_pct = static_cast<float>(params_.battery.critical_pct);
    cfg.emergency_pct = static_cast<float>(params_.battery.emergency_pct);
    cfg.min_voltage = static_cast<float>(params_.battery.min_voltage);
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = params_.battery.warn_grace_s;
    rule.critical_grace_s = params_.battery.critical_grace_s;
    ruleEngine_->addChecker(std::make_shared<BatteryChecker>(cfg), rule);
  }

  // GPS checker
  if (params_.gps.enabled) {
    GpsCheckerConfig cfg;
    cfg.min_fix_type = params_.gps.min_fix_type;
    cfg.max_hdop = static_cast<float>(params_.gps.max_hdop);
    cfg.max_vdop = static_cast<float>(params_.gps.max_vdop);
    cfg.min_satellites = params_.gps.min_satellites;
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = params_.gps.warn_grace_s;
    rule.critical_grace_s = params_.gps.critical_grace_s;
    ruleEngine_->addChecker(std::make_shared<GpsChecker>(cfg), rule);
  }

  // Geofence checker
  if (params_.geofence.enabled) {
    GeofenceCheckerConfig cfg;
    cfg.max_radius_m = params_.geofence.max_radius_m;
    cfg.max_altitude_m = params_.geofence.max_altitude_m;
    cfg.min_altitude_m = params_.geofence.min_altitude_m;
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = params_.geofence.warn_grace_s;
    rule.critical_grace_s = params_.geofence.critical_grace_s;
    ruleEngine_->addChecker(std::make_shared<GeofenceChecker>(cfg), rule);
  }

  // Envelope checker
  if (params_.envelope.enabled) {
    EnvelopeCheckerConfig cfg;
    cfg.max_velocity_ms = params_.envelope.max_velocity_ms;
    cfg.max_altitude_m = params_.envelope.max_altitude_m;
    cfg.max_tilt_rad = params_.envelope.max_tilt_rad;
    RuleConfig rule;
    rule.enabled = true;
    rule.warn_grace_s = params_.envelope.warn_grace_s;
    rule.critical_grace_s = params_.envelope.critical_grace_s;
    ruleEngine_->addChecker(std::make_shared<EnvelopeChecker>(cfg), rule);
  }

  // TF2 buffer and listener for odom→map frame transforms (geofence checks).
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  // Subscriptions
  const auto qos = rclcpp::QoS(20).reliable();
  batterySub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery", qos, [this](sensor_msgs::msg::BatteryState::SharedPtr msg) { onBattery(msg); });
  gpsStatusSub_ = this->create_subscription<peregrine_interfaces::msg::GpsStatus>(
    "gps_status", qos, [this](peregrine_interfaces::msg::GpsStatus::SharedPtr msg) { onGpsStatus(msg); });
  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    "estimated_state", qos,
    [this](peregrine_interfaces::msg::State::SharedPtr msg) { onEstimatedState(msg); });
  px4StatusSub_ = this->create_subscription<peregrine_interfaces::msg::PX4Status>(
    "status", rclcpp::QoS(10).reliable(),
    [this](peregrine_interfaces::msg::PX4Status::SharedPtr msg) { onPx4Status(msg); });

  // Publisher
  safetyStatusPub_ = this->create_publisher<peregrine_interfaces::msg::SafetyStatus>(
    "safety_status", rclcpp::QoS(10).reliable());

  // Set mode client for land commands
  setModeClient_ = this->create_client<peregrine_interfaces::srv::SetMode>("set_mode");

  SafetyActionConfig actionConfig;
  actionConfig.land_command_timeout_s = params_.land_command_timeout_s;
  actionConfig.land_command_retry_count = params_.land_command_retry_count;
  actionExecutor_ = std::make_unique<SafetyActionExecutor>(
    setModeClient_, get_logger(), actionConfig, this->get_clock());

  // Evaluation timer (created but not started until activate)
  const auto period = std::chrono::duration<double>(1.0 / params_.evaluate_rate_hz);
  evaluateTimer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { evaluateAndPublish(); });
  evaluateTimer_->cancel();

  RCLCPP_INFO(
    get_logger(),
    "Configured safety_monitor (rate=%.1fHz, land_enabled=%s)",
    params_.evaluate_rate_hz, params_.command_land_enabled ? "true" : "false");
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
  // Transform position from odom frame to map frame for geofence checking.
  double px = msg->pose.pose.position.x;
  double py = msg->pose.pose.position.y;
  double pz = msg->pose.pose.position.z;

  if (tfBuffer_ && !msg->header.frame_id.empty()) {
    try {
      geometry_msgs::msg::PointStamped odomPoint;
      odomPoint.header = msg->header;
      odomPoint.point = msg->pose.pose.position;
      auto mapPoint = tfBuffer_->transform(odomPoint, std::string("map"), tf2::durationFromSec(0.0));
      px = mapPoint.point.x;
      py = mapPoint.point.y;
      pz = mapPoint.point.z;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "TF2 odom->map lookup failed, using raw odom position for geofence: %s", ex.what());
    }
  }

  std::scoped_lock lock(mutex_);
  PositionData data;
  data.x = px;
  data.y = py;
  data.z = pz;
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
    if (elapsed <= params_.gps.freshness_timeout_s) {
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
  if (params_.command_land_enabled && actionExecutor_) {
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
