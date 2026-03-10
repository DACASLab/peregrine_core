/**
 * @file supervisor_state_machine.hpp
 * @brief Deterministic table-driven supervisor FSM.
 *
 * The supervisor FSM tracks the vehicle's high-level operational phase:
 *
 *   Idle ──(ArmRequested)──> Armed ──(TakeoffRequested)──> TakingOff
 *     ^                        │                              │
 *     │                        │                     TakeoffCompleted
 *     │                   LandRequested                       │
 *     │                        │                              v
 *     │                        v                           Hovering <──(FlightActionCompleted)
 *     │                     Landing                           │                │
 *     │                        │                    FlightActionRequested       │
 *     │                   LandCompleted                       │                │
 *     │                        │                              v                │
 *     │                        v                           Flying ─────────────┘
 *  (DisarmCompleted)        Landed
 *     │                        │
 *     └────────────────────────┘
 *
 *   Any state ──(FailsafeDetected)──> Emergency ──(EmergencyCleared)──> Idle
 *
 * Design:
 *  - Table-driven: transitions are defined in a static vector of (from, event, guard, target)
 *    rules, scanned linearly. First match wins, guard failure rejects the transition.
 *  - FailsafeDetected bypasses the table entirely and unconditionally moves to Emergency.
 *  - Guard policies (TakeoffReady, FlightReady, LandingReady, EmergencyClearReady) are
 *    evaluated by TransitionGuard against the HealthAggregator snapshot.
 *  - The FSM is purely synchronous: apply() returns immediately with the outcome.
 */

#pragma once

#include <uav_manager/health_aggregator.hpp>
#include <uav_manager/transition_guard.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace uav_manager
{

/**
 * @enum SupervisorState
 * @brief Safety-oriented supervisor states for mission orchestration.
 */
enum class SupervisorState : uint8_t
{
  Idle,
  Armed,
  TakingOff,
  Hovering,
  Flying,
  Landing,
  Landed,
  Emergency
};

/**
 * @enum SupervisorEvent
 * @brief Explicit event vocabulary accepted by the supervisor FSM.
 */
enum class SupervisorEvent : uint8_t
{
  ArmRequested,
  TakeoffRequested,
  TakeoffCompleted,
  TakeoffFailed,
  FlightActionRequested,
  FlightActionCompleted,
  FlightActionFailed,
  LandRequested,
  LandCompleted,
  LandFailed,
  DisarmCompleted,
  FailsafeDetected,
  EmergencyCleared
};

/**
 * @struct TransitionOutcome
 * @brief Full transition decision payload for logging and result mapping.
 */
struct TransitionOutcome
{
  SupervisorState from{SupervisorState::Idle};
  SupervisorState to{SupervisorState::Idle};
  SupervisorEvent event{SupervisorEvent::ArmRequested};
  bool accepted{false};
  std::string reasonCode{"ILLEGAL_TRANSITION"};
};

/**
 * @class SupervisorStateMachine
 * @brief Deterministic table-driven FSM for uav_manager high-level state.
 */
class SupervisorStateMachine
{
public:
  /// Constructs FSM with initial state `Idle`.
  SupervisorStateMachine();

  /**
   * @brief Applies one event from the current state.
   *
   * `FailsafeDetected` is handled as an unconditional emergency preemption.
   */
  TransitionOutcome apply(
    SupervisorEvent event, const TransitionGuard & guard,
    const HealthSnapshot & health);
  /// Returns current FSM state.
  SupervisorState state() const;

  /// String helper for state values used in logs/status.
  static const char * toString(SupervisorState state);
  /// String helper for event values used in logs/status.
  static const char * toString(SupervisorEvent event);

private:
  /// One row in the transition table.
  struct TransitionRule
  {
    SupervisorState from;
    SupervisorEvent event;
    GuardId guard;
    SupervisorState target;
    /// Reason string logged on successful transition; points to a string literal.
    const char * successReason;
  };

  /// Static transition table; initialized once and shared across all instances.
  static const std::vector<TransitionRule> & rules();

  /// Current FSM state.
  SupervisorState state_{SupervisorState::Idle};
};

}  // namespace uav_manager
