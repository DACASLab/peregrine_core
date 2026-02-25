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
#include <uav_manager/transition_guard.hpp>

namespace uav_manager
{

GuardResult TransitionGuard::pass()
{
  return GuardResult{true, "OK"};
}

GuardResult TransitionGuard::fail(const std::string & reasonCode)
{
  return GuardResult{false, reasonCode};
}

// Guard hierarchy (from most to least permissive):
//   Always          - unconditional pass, used for completion/failure events
//   LandingReady    - intentionally relaxed: only needs a live PX4 link, so
//                     landing remains possible even if managers are degraded
//   TakeoffReady    - strictest pre-flight gate: all five deps must be ready
//   FlightReady     - TakeoffReady-like checks plus armed verification
//   EmergencyClearReady - strict re-entry: PX4 must be out of failsafe and
//                         connected before the operator can leave Emergency
GuardResult TransitionGuard::evaluate(const GuardId guard, const HealthSnapshot & health) const
{
  // Guards are intentionally side-effect free; they only inspect the provided snapshot.
  // The `switch` gives O(1) dispatch over enum values and keeps all guard logic in one
  // compile-time-checked branch table.
  switch (guard) {
    case GuardId::Always:
      // Unconditional pass -- used for completion events (TakeoffCompleted, etc.)
      // where the action itself already validated preconditions.
      return pass();

    case GuardId::TakeoffReady:
      // Strictest gate: all five dependencies (PX4, estimated state, estimation
      // manager, control manager, trajectory manager) must be present, fresh,
      // active, and healthy before we allow arming or takeoff.
      if (!health.px4.ready()) {
        return fail(health.px4.reasonCode);
      }
      if (!health.estimatedState.ready()) {
        return fail(health.estimatedState.reasonCode);
      }
      if (!health.estimationManager.ready()) {
        return fail(health.estimationManager.reasonCode);
      }
      if (!health.controlManager.ready()) {
        return fail(health.controlManager.reasonCode);
      }
      if (!health.trajectoryManager.ready()) {
        return fail(health.trajectoryManager.reasonCode);
      }
      return pass();

    case GuardId::FlightReady:
      // Flight actions additionally require vehicle to be armed.
      if (!health.px4.ready()) {
        return fail(health.px4.reasonCode);
      }
      if (!health.px4.armed) {
        return fail("VEHICLE_NOT_ARMED");
      }
      if (!health.estimatedState.ready()) {
        return fail(health.estimatedState.reasonCode);
      }
      if (!health.controlManager.ready()) {
        return fail(health.controlManager.reasonCode);
      }
      if (!health.trajectoryManager.ready()) {
        return fail(health.trajectoryManager.reasonCode);
      }
      return pass();

    case GuardId::LandingReady:
      // Landing is allowed with a minimal valid PX4 link, even if managers are degraded.
      if (!health.px4.present) {
        return fail("PX4_STATUS_MISSING");
      }
      if (!health.px4.connected) {
        return fail("PX4_DISCONNECTED");
      }
      if (!health.px4.fresh) {
        return fail("PX4_STATUS_STALE");
      }
      return pass();

    case GuardId::EmergencyClearReady:
      // Emergency clear is intentionally strict to avoid unsafe re-entry.
      if (!health.px4.present) {
        return fail("PX4_STATUS_MISSING");
      }
      if (health.px4.failsafe) {
        return fail("PX4_FAILSAFE_ACTIVE");
      }
      if (!health.px4.connected) {
        return fail("PX4_DISCONNECTED");
      }
      return pass();

    default:
      return fail("UNKNOWN_GUARD");
  }
}

}  // namespace uav_manager
