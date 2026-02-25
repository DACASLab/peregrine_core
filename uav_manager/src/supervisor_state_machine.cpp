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
#include <uav_manager/supervisor_state_machine.hpp>

namespace uav_manager
{

SupervisorStateMachine::SupervisorStateMachine() = default;

const std::vector<SupervisorStateMachine::TransitionRule> & SupervisorStateMachine::rules()
{
  // --- Transition table design ---
  // Each row is: (from_state, event, guard, target_state, success_reason).
  // The table is scanned linearly; the FIRST row matching (from, event) defines
  // the transition. If that row's guard fails, the transition is rejected -- it
  // does NOT fall through to later rows with the same (from, event) pair.
  // FailsafeDetected is handled outside this table as a hardcoded emergency
  // preemption (see apply()), so it has no row here.
  // `static` local here is initialized once (thread-safe since C++11) and then reused,
  // avoiding repeated table construction on every rules() call.
  static const std::vector<TransitionRule> kRules = {
    // --- Arming: allowed from Idle (cold start) and Landed (re-arm after landing) ---
    {SupervisorState::Idle, SupervisorEvent::ArmRequested, GuardId::TakeoffReady,
      SupervisorState::Armed, "ARM_ACCEPTED"},
    {SupervisorState::Landed, SupervisorEvent::ArmRequested, GuardId::TakeoffReady,
      SupervisorState::Armed, "ARM_ACCEPTED"},

    // --- Takeoff: Armed -> TakingOff, with completion/failure returning to Hovering/Armed ---
    {SupervisorState::Armed, SupervisorEvent::TakeoffRequested, GuardId::TakeoffReady,
      SupervisorState::TakingOff,
      "TAKEOFF_STARTED"},
    {SupervisorState::TakingOff, SupervisorEvent::TakeoffCompleted, GuardId::Always,
      SupervisorState::Hovering,
      "TAKEOFF_COMPLETED"},
    {SupervisorState::TakingOff, SupervisorEvent::TakeoffFailed, GuardId::Always,
      SupervisorState::Armed,
      "TAKEOFF_FAILED"},

    // --- Flight actions: Hovering->Flying starts an action, Flying->Flying chains actions ---
    {SupervisorState::Hovering, SupervisorEvent::FlightActionRequested, GuardId::FlightReady,
      SupervisorState::Flying,
      "FLIGHT_ACTION_STARTED"},
    // Flying->Flying allows chaining sequential actions (e.g., waypoint-to-waypoint)
    // without returning to Hovering between each one.
    {SupervisorState::Flying, SupervisorEvent::FlightActionRequested, GuardId::FlightReady,
      SupervisorState::Flying,
      "FLIGHT_ACTION_STARTED"},
    {SupervisorState::Flying, SupervisorEvent::FlightActionCompleted, GuardId::Always,
      SupervisorState::Hovering,
      "FLIGHT_ACTION_COMPLETED"},
    {SupervisorState::Flying, SupervisorEvent::FlightActionFailed, GuardId::Always,
      SupervisorState::Hovering,
      "FLIGHT_ACTION_FAILED"},

    // --- Landing: allowed from many states for safety (Hovering, Flying, Armed, TakingOff) ---
    // The LandingReady guard is intentionally relaxed (PX4 link only) so that
    // landing remains available even when other managers are degraded.
    {SupervisorState::Hovering, SupervisorEvent::LandRequested, GuardId::LandingReady,
      SupervisorState::Landing,
      "LANDING_STARTED"},
    {SupervisorState::Flying, SupervisorEvent::LandRequested, GuardId::LandingReady,
      SupervisorState::Landing,
      "LANDING_STARTED"},
    {SupervisorState::Armed, SupervisorEvent::LandRequested, GuardId::LandingReady,
      SupervisorState::Landing,
      "LANDING_STARTED"},
    {SupervisorState::TakingOff, SupervisorEvent::LandRequested, GuardId::LandingReady,
      SupervisorState::Landing,
      "LANDING_STARTED"},
    {SupervisorState::Landing, SupervisorEvent::LandCompleted, GuardId::Always,
      SupervisorState::Landed, "LAND_COMPLETED"},
    {SupervisorState::Landing, SupervisorEvent::LandFailed, GuardId::Always,
      SupervisorState::Hovering, "LAND_FAILED"},
    {SupervisorState::Landed, SupervisorEvent::DisarmCompleted, GuardId::Always,
      SupervisorState::Idle, "IDLE"},
    {SupervisorState::Armed, SupervisorEvent::DisarmCompleted, GuardId::Always,
      SupervisorState::Idle, "IDLE"},

    // --- Emergency clear: strict re-entry from Emergency back to Idle ---
    {SupervisorState::Emergency, SupervisorEvent::EmergencyCleared, GuardId::EmergencyClearReady,
      SupervisorState::Idle,
      "EMERGENCY_CLEARED"},
  };
  return kRules;
}

TransitionOutcome SupervisorStateMachine::apply(
  const SupervisorEvent event,
  const TransitionGuard & guard,
  const HealthSnapshot & health)
{
  TransitionOutcome outcome;
  outcome.from = state_;
  outcome.to = state_;
  outcome.event = event;

  if (event == SupervisorEvent::FailsafeDetected) {
    // FailsafeDetected unconditionally moves to Emergency regardless of current
    // state. A PX4 failsafe means the vehicle entered an unrecoverable autonomous
    // mode (e.g., RTL, auto-land) and the supervisor must not issue conflicting
    // commands. This bypasses the table entirely -- no guard is consulted.
    state_ = SupervisorState::Emergency;
    outcome.to = state_;
    outcome.accepted = true;
    outcome.reasonCode = "FAILSAFE_PREEMPT";
    return outcome;
  }

  // Linear scan with first-match semantics: if two rows share the same
  // (from, event), only the first is reachable. Guard failure on that row
  // rejects the transition outright rather than trying the next row.
  for (const auto & rule : rules()) {
    if (rule.from != state_ || rule.event != event) {
      continue;
    }

    // Guard failures reject transition with exact upstream reason code.
    const GuardResult guardResult = guard.evaluate(rule.guard, health);
    if (!guardResult.passed) {
      outcome.accepted = false;
      outcome.reasonCode = guardResult.reasonCode;
      return outcome;
    }

    state_ = rule.target;
    outcome.to = state_;
    outcome.accepted = true;
    outcome.reasonCode = rule.successReason;
    return outcome;
  }

  // No matching row in table means illegal transition for current state/event.
  outcome.accepted = false;
  outcome.reasonCode = "ILLEGAL_TRANSITION";
  return outcome;
}

SupervisorState SupervisorStateMachine::state() const
{
  return state_;
}

const char * SupervisorStateMachine::toString(const SupervisorState state)
{
  // String tokens are part of operator-facing diagnostics; keep them stable.
  switch (state) {
    case SupervisorState::Idle:
      return "IDLE";
    case SupervisorState::Armed:
      return "ARMED";
    case SupervisorState::TakingOff:
      return "TAKING_OFF";
    case SupervisorState::Hovering:
      return "HOVERING";
    case SupervisorState::Flying:
      return "FLYING";
    case SupervisorState::Landing:
      return "LANDING";
    case SupervisorState::Landed:
      return "LANDED";
    case SupervisorState::Emergency:
      return "EMERGENCY";
    default:
      return "UNKNOWN";
  }
}

const char * SupervisorStateMachine::toString(const SupervisorEvent event)
{
  // Event strings are used in transition logs and should remain machine-friendly.
  switch (event) {
    case SupervisorEvent::ArmRequested:
      return "arm_requested";
    case SupervisorEvent::TakeoffRequested:
      return "takeoff_requested";
    case SupervisorEvent::TakeoffCompleted:
      return "takeoff_completed";
    case SupervisorEvent::TakeoffFailed:
      return "takeoff_failed";
    case SupervisorEvent::FlightActionRequested:
      return "flight_action_requested";
    case SupervisorEvent::FlightActionCompleted:
      return "flight_action_completed";
    case SupervisorEvent::FlightActionFailed:
      return "flight_action_failed";
    case SupervisorEvent::LandRequested:
      return "land_requested";
    case SupervisorEvent::LandCompleted:
      return "land_completed";
    case SupervisorEvent::LandFailed:
      return "land_failed";
    case SupervisorEvent::DisarmCompleted:
      return "disarm_completed";
    case SupervisorEvent::FailsafeDetected:
      return "failsafe_detected";
    case SupervisorEvent::EmergencyCleared:
      return "emergency_cleared";
    default:
      return "unknown_event";
  }
}

}  // namespace uav_manager
