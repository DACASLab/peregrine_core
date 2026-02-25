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
/**
 * @file transition_guard.hpp
 * @brief Pure guard checks over aggregated health snapshot.
 *
 * Guards are side-effect-free policy functions that inspect the HealthSnapshot and
 * return pass/fail with a machine-readable reason code. They are evaluated by the
 * supervisor FSM before each transition:
 *
 *   Guard hierarchy (most to least permissive):
 *     Always            - unconditional pass (for completion/failure events)
 *     LandingReady      - relaxed: only needs a live PX4 link, so landing is always
 *                         available even if estimation/control are degraded
 *     TakeoffReady      - strict: all 5 dependencies must be present, fresh, active,
 *                         and healthy before arming/takeoff is allowed
 *     FlightReady       - TakeoffReady + vehicle must be armed
 *     EmergencyClearReady - PX4 must be connected and out of failsafe before the
 *                           operator can leave Emergency state
 *
 * The reason codes (e.g., "PX4_DISCONNECTED", "ESTIMATION_STALE") propagate through
 * TransitionOutcome to the action result message, giving operators clear diagnostics
 * when a flight goal is rejected.
 */

#pragma once

#include <uav_manager/health_aggregator.hpp>

#include <cstdint>
#include <string>

namespace uav_manager
{

/**
 * @enum GuardId
 * @brief Identifier for each supervisor transition guard policy.
 */
enum class GuardId : uint8_t
{
  /// Unconditional pass.
  Always,
  /// Requires full dependency readiness for arming/takeoff.
  TakeoffReady,
  /// Requires full dependency readiness plus armed vehicle.
  FlightReady,
  /// Requires minimally valid PX4 link for landing request.
  LandingReady,
  /// Requires PX4 present, connected, and out of failsafe.
  EmergencyClearReady
};

/**
 * @struct GuardResult
 * @brief Boolean decision plus machine-readable reason code.
 */
struct GuardResult
{
  bool passed{false};
  std::string reasonCode{"GUARD_FAILED"};
};

/**
 * @class TransitionGuard
 * @brief Pure guard evaluator for FSM transitions.
 */
class TransitionGuard
{
public:
  /**
   * @brief Evaluates one guard policy against the latest health snapshot.
   */
  GuardResult evaluate(GuardId guard, const HealthSnapshot & health) const;

private:
  /// Convenience constructor for successful guard outcomes.
  static GuardResult pass();
  /// Convenience constructor for failed guard outcomes.
  static GuardResult fail(const std::string & reasonCode);
};

}  // namespace uav_manager
