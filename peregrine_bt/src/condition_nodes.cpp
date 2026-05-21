#include "peregrine_bt/condition_nodes.hpp"
#include <cmath>

namespace peregrine_bt
{

using UAVState = peregrine_interfaces::msg::UAVState;

// ---------------------------------------------------------------------------
// UAV state conditions
// ---------------------------------------------------------------------------

BT::NodeStatus IsArmed::onTick(const std::shared_ptr<UAVState>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  return msg->armed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsFlying::onTick(const std::shared_ptr<UAVState>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  return (msg->state == UAVState::STATE_FLYING ||
          msg->state == UAVState::STATE_HOVERING)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsLanded::onTick(const std::shared_ptr<UAVState>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  return (msg->state == UAVState::STATE_IDLE ||
          msg->state == UAVState::STATE_LANDED)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsDependenciesReady::onTick(const std::shared_ptr<UAVState>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  return msg->dependencies_ready ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsEmergency::onTick(const std::shared_ptr<UAVState>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  return (msg->state == UAVState::STATE_EMERGENCY)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsConnected::onTick(const std::shared_ptr<UAVState>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  return msg->connected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsOffboard::onTick(const std::shared_ptr<UAVState>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  return msg->offboard ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// Safety conditions
// ---------------------------------------------------------------------------

BT::NodeStatus IsSafetyNominal::onTick(
  const std::shared_ptr<peregrine_interfaces::msg::SafetyStatus>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  return (msg->level == peregrine_interfaces::msg::SafetyStatus::LEVEL_NOMINAL)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsSafetyAtLeast::onTick(
  const std::shared_ptr<peregrine_interfaces::msg::SafetyStatus>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  unsigned max_level = 1;
  getInput("max_level", max_level);
  return (msg->level <= max_level) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// Sensor conditions
// ---------------------------------------------------------------------------

BT::NodeStatus IsBatteryAbove::onTick(
  const std::shared_ptr<peregrine_interfaces::msg::PX4Status>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  double threshold = 0.2;
  getInput("threshold_pct", threshold);
  return (msg->battery_remaining >= threshold)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsGpsHealthy::onTick(
  const std::shared_ptr<peregrine_interfaces::msg::GpsStatus>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  unsigned min_fix = 3;
  double max_hdop = 5.0;
  double max_vdop = 5.0;
  getInput("min_fix_type", min_fix);
  getInput("max_hdop", max_hdop);
  getInput("max_vdop", max_vdop);
  return (msg->fix_type >= min_fix && msg->hdop <= max_hdop && msg->vdop <= max_vdop)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// Position conditions
// ---------------------------------------------------------------------------

BT::NodeStatus HasValidState::onTick(
  const std::shared_ptr<peregrine_interfaces::msg::State>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  double max_age = 1.0;
  getInput("max_age_s", max_age);

  auto node = node_.lock();
  if (!node) return BT::NodeStatus::FAILURE;

  auto age = node->now() - msg->header.stamp;
  return (age.seconds() <= max_age) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsAboveAltitude::onTick(
  const std::shared_ptr<peregrine_interfaces::msg::State>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  double min_alt = 0.0;
  getInput("min_alt_m", min_alt);
  return (msg->pose.pose.position.z >= min_alt)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsBelowAltitude::onTick(
  const std::shared_ptr<peregrine_interfaces::msg::State>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;
  double max_alt = 0.0;
  getInput("max_alt_m", max_alt);
  return (msg->pose.pose.position.z <= max_alt)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsAtPosition::onTick(
  const std::shared_ptr<peregrine_interfaces::msg::State>& msg)
{
  if (!msg) return BT::NodeStatus::FAILURE;

  double tx = 0.0, ty = 0.0, tz = 0.0, tol = 0.5;
  getInput("target_x", tx);
  getInput("target_y", ty);
  getInput("target_z", tz);
  getInput("tolerance_m", tol);

  double dx = msg->pose.pose.position.x - tx;
  double dy = msg->pose.pose.position.y - ty;
  double dz = msg->pose.pose.position.z - tz;
  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  return (dist <= tol) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace peregrine_bt
