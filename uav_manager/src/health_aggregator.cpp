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
#include <uav_manager/health_aggregator.hpp>

namespace uav_manager
{

std::string toReasonCode(const ReadinessCode code)
{
  // Central mapping keeps reason tokens stable for logs/tests/operators.
  switch (code) {
    case ReadinessCode::Ok:
      return "OK";
    case ReadinessCode::Missing:
      return "MISSING";
    case ReadinessCode::Inactive:
      return "INACTIVE";
    case ReadinessCode::Unhealthy:
      return "UNHEALTHY";
    case ReadinessCode::Stale:
      return "STALE";
    case ReadinessCode::Disconnected:
      return "DISCONNECTED";
    case ReadinessCode::Failsafe:
      return "FAILSAFE";
    default:
      return "UNKNOWN";
  }
}

HealthAggregator::HealthAggregator(const FreshnessConfig config)
: config_(config)
{
  // Constructor argument is intentionally passed by value and copied into config_.
  // For tiny structs like FreshnessConfig this is cheap and keeps call sites simple.
}

void HealthAggregator::updateEstimatedState(const std::chrono::steady_clock::time_point now)
{
  std::scoped_lock lock(mutex_);
  // Only freshness matters for estimated_state readiness.
  lastEstimatedState_ = now;
}

void HealthAggregator::updatePx4Status(
  const std::chrono::steady_clock::time_point now,
  const Px4StatusInput & status)
{
  std::scoped_lock lock(mutex_);
  // Store value + receive time together to avoid stale/value skew.
  px4Status_ = TimedPx4Status{status, now};
}

void HealthAggregator::updateEstimationStatus(
  const std::chrono::steady_clock::time_point now,
  const ManagerStatusInput & status)
{
  std::scoped_lock lock(mutex_);
  // Last sample wins; manager statuses are treated as periodic heartbeats.
  estimationStatus_ = TimedManagerStatus{status, now};
}

void HealthAggregator::updateControlStatus(
  const std::chrono::steady_clock::time_point now,
  const ManagerStatusInput & status)
{
  std::scoped_lock lock(mutex_);
  // Last sample wins; manager statuses are treated as periodic heartbeats.
  controlStatus_ = TimedManagerStatus{status, now};
}

void HealthAggregator::updateTrajectoryStatus(
  const std::chrono::steady_clock::time_point now,
  const ManagerStatusInput & status)
{
  std::scoped_lock lock(mutex_);
  // Last sample wins; manager statuses are treated as periodic heartbeats.
  trajectoryStatus_ = TimedManagerStatus{status, now};
}

void HealthAggregator::updateSafetyStatus(
  const std::chrono::steady_clock::time_point now,
  const SafetyStatusInput & status)
{
  std::scoped_lock lock(mutex_);
  safetyStatus_ = TimedSafetyStatus{status, now};
}

void HealthAggregator::setRequireExternalSafety(bool require)
{
  std::scoped_lock lock(mutex_);
  requireExternalSafety_ = require;
}

// Evaluates a single manager dependency using a priority ladder:
//   Missing > Stale > Inactive > Unhealthy > Ok
// Stale is checked before active/healthy because stale data cannot be trusted --
// even if the last sample said "active + healthy", that was N seconds ago and the
// manager may have crashed since. The prefix parameter creates namespaced reason
// codes (e.g., "ESTIMATION_STALE") for operator-facing diagnostics.
DependencyReadiness HealthAggregator::evaluateManager(
  const std::optional<TimedManagerStatus> & status,
  const std::chrono::milliseconds timeout,
  const std::chrono::steady_clock::time_point now,
  const std::string & prefix) const
{
  // `const T&` parameters avoid copying while guaranteeing read-only access.
  // This pattern is common in C++ for larger objects (optional/string/status structs).
  DependencyReadiness readiness;
  if (!status.has_value()) {
    // Missing beats stale/inactive/unhealthy because there is no usable sample yet.
    readiness.code = ReadinessCode::Missing;
    readiness.reasonCode = prefix + "_MISSING";
    return readiness;
  }

  readiness.present = true;
  readiness.active = status->status.active;
  readiness.healthy = status->status.healthy;
  readiness.fresh = (now - status->receivedAt) <= timeout;

  // Ordering matters: stale data is treated as unusable before active/healthy checks.
  if (!readiness.fresh) {
    readiness.code = ReadinessCode::Stale;
    readiness.reasonCode = prefix + "_STALE";
    return readiness;
  }

  if (!readiness.active) {
    readiness.code = ReadinessCode::Inactive;
    readiness.reasonCode = prefix + "_INACTIVE";
    return readiness;
  }

  if (!readiness.healthy) {
    readiness.code = ReadinessCode::Unhealthy;
    readiness.reasonCode = prefix + "_UNHEALTHY";
    return readiness;
  }

  readiness.code = ReadinessCode::Ok;
  readiness.reasonCode = prefix + "_OK";
  return readiness;
}

// estimated_state is a raw data stream (e.g., pose/twist from the estimator), not
// a managed status with active/healthy fields. Readiness is therefore determined
// solely by presence and freshness. active and healthy are hardcoded to true so
// that DependencyReadiness::ready() works uniformly across all dependency types.
DependencyReadiness HealthAggregator::evaluateState(const std::chrono::steady_clock::time_point now)
const
{
  DependencyReadiness readiness;
  if (!lastEstimatedState_.has_value()) {
    readiness.code = ReadinessCode::Missing;
    readiness.reasonCode = "ESTIMATED_STATE_MISSING";
    return readiness;
  }

  readiness.present = true;
  readiness.active = true;
  readiness.healthy = true;
  readiness.fresh = (now - *lastEstimatedState_) <= config_.estimatedStateTimeout;
  if (!readiness.fresh) {
    readiness.code = ReadinessCode::Stale;
    readiness.reasonCode = "ESTIMATED_STATE_STALE";
    return readiness;
  }

  readiness.code = ReadinessCode::Ok;
  readiness.reasonCode = "ESTIMATED_STATE_OK";
  return readiness;
}

// PX4 evaluation chain: Missing > Stale > Disconnected > Failsafe > Ok.
// Stale is checked before connected/failsafe because stale data could show
// connected=true from 5 seconds ago when the link is actually dead. Acting on
// that stale "connected" status could lead to sending commands into the void.
Px4Readiness HealthAggregator::evaluatePx4(const std::chrono::steady_clock::time_point now) const
{
  Px4Readiness readiness;
  if (!px4Status_.has_value()) {
    readiness.code = ReadinessCode::Missing;
    readiness.reasonCode = "PX4_STATUS_MISSING";
    return readiness;
  }

  readiness.present = true;
  readiness.connected = px4Status_->status.connected;
  readiness.armed = px4Status_->status.armed;
  readiness.offboard = px4Status_->status.offboard;
  readiness.failsafe = px4Status_->status.failsafe;
  readiness.navState = px4Status_->status.navState;
  readiness.fresh = (now - px4Status_->receivedAt) <= config_.px4StatusTimeout;

  // PX4 freshness is checked before connected/failsafe to avoid acting on stale link state.
  if (!readiness.fresh) {
    readiness.code = ReadinessCode::Stale;
    readiness.reasonCode = "PX4_STATUS_STALE";
    return readiness;
  }

  if (!readiness.connected) {
    readiness.code = ReadinessCode::Disconnected;
    readiness.reasonCode = "PX4_DISCONNECTED";
    return readiness;
  }

  if (readiness.failsafe) {
    readiness.code = ReadinessCode::Failsafe;
    readiness.reasonCode = "PX4_FAILSAFE";
    return readiness;
  }

  readiness.code = ReadinessCode::Ok;
  readiness.reasonCode = "PX4_OK";
  return readiness;
}

// Safety monitor evaluation: Missing > Stale > Unhealthy (level >= CRITICAL) > Ok
DependencyReadiness HealthAggregator::evaluateSafety(
  const std::chrono::steady_clock::time_point now) const
{
  DependencyReadiness readiness;
  if (!safetyStatus_.has_value()) {
    readiness.code = ReadinessCode::Missing;
    readiness.reasonCode = "SAFETY_MISSING";
    return readiness;
  }

  readiness.present = true;
  readiness.active = true;  // Safety monitor is a data stream, not lifecycle-managed from this view
  readiness.fresh = (now - safetyStatus_->receivedAt) <= config_.safetyStatusTimeout;

  if (!readiness.fresh) {
    readiness.code = ReadinessCode::Stale;
    readiness.reasonCode = "SAFETY_STALE";
    return readiness;
  }

  // Safety level >= CRITICAL (2) means unhealthy
  readiness.healthy = safetyStatus_->status.level < 2;  // LEVEL_CRITICAL = 2
  if (!readiness.healthy) {
    readiness.code = ReadinessCode::Unhealthy;
    readiness.reasonCode = "SAFETY_UNHEALTHY";
    return readiness;
  }

  readiness.code = ReadinessCode::Ok;
  readiness.reasonCode = "SAFETY_OK";
  return readiness;
}

HealthSnapshot HealthAggregator::snapshot(const std::chrono::steady_clock::time_point now) const
{
  // All five dependencies are evaluated under a single mutex acquisition to
  // ensure the snapshot is temporally coherent. Without this, you could get
  // px4.ready()=true evaluated at T=0 but trajectory.ready()=false evaluated at
  // T=1 where PX4 went failsafe between the two checks, producing a snapshot
  // that never existed in reality.
  std::scoped_lock lock(mutex_);
  HealthSnapshot snapshot;
  snapshot.px4 = evaluatePx4(now);
  snapshot.estimatedState = evaluateState(now);
  snapshot.estimationManager = evaluateManager(
    estimationStatus_, config_.managerStatusTimeout, now,
    "ESTIMATION");
  snapshot.controlManager = evaluateManager(
    controlStatus_, config_.managerStatusTimeout, now,
    "CONTROL");
  snapshot.trajectoryManager = evaluateManager(
    trajectoryStatus_, config_.managerStatusTimeout, now,
    "TRAJECTORY");
  snapshot.safetyMonitor = evaluateSafety(now);
  snapshot.requireExternalSafety = requireExternalSafety_;
  // Return-by-value is efficient in modern C++: copy elision typically constructs
  // the object directly in the caller's destination storage.
  return snapshot;
}

}  // namespace uav_manager
