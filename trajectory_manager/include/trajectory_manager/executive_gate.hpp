#pragma once

#include <peregrine_interfaces/msg/uav_state.hpp>

#include <optional>
#include <string>

namespace trajectory_manager
{

inline bool executiveAllowsMotion(
  const std::optional<peregrine_interfaces::msg::UAVState> & state,
  std::string * reason = nullptr)
{
  if (!state.has_value()) {
    if (reason) *reason = "MISSING_EXECUTIVE_STATE";
    return false;
  }
  if (!state->dependencies_ready) {
    if (reason) *reason = "EXECUTIVE_NOT_READY";
    return false;
  }
  if (state->fault_latched) {
    if (reason) *reason = "EXECUTIVE_FAULT_LATCHED";
    return false;
  }
  if (state->failsafe) {
    if (reason) *reason = "EXECUTIVE_FAILSAFE";
    return false;
  }
  if (!state->motion_authorized) {
    if (reason) *reason = "EXECUTIVE_MOTION_REVOKED";
    return false;
  }
  if (reason) *reason = "OK";
  return true;
}

}  // namespace trajectory_manager
