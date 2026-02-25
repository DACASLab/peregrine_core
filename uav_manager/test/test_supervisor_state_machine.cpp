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
 * @file test_supervisor_state_machine.cpp
 * @brief Unit tests for the table-driven supervisor FSM and its guard policies.
 *
 * The supervisor state machine governs all flight-phase transitions:
 *   Idle -> Armed -> TakingOff -> Hovering <-> Flying -> Landing -> Landed -> Idle
 *
 * Each test exercises a specific aspect of the FSM contract:
 *   - NominalTakeoffFlightLandFlow: golden-path lifecycle from idle to idle
 *   - RejectsIllegalTransition: events with no matching table row are rejected
 *   - RejectsGuardFailureWithReason: valid events blocked by guard policy propagate
 *     the guard's reason code (not a generic "REJECTED")
 *   - FailsafePreemptsFromAnyState: the FailsafeDetected event overrides any state
 *   - EmergencyClearRequiresGuard: leaving Emergency requires PX4 out of failsafe
 *
 * Tests are pure unit tests -- no ROS infrastructure needed. The FSM evaluates
 * transitions against a HealthSnapshot struct, and guards are side-effect-free
 * policy functions, so tests construct snapshots directly.
 */

#include <gtest/gtest.h>

#include <uav_manager/supervisor_state_machine.hpp>

namespace
{
// Unnamed namespace gives this helper internal linkage (file-local symbol), which
// avoids linker collisions if other test files define a function with the same name.

// Constructs a HealthSnapshot where every dependency is present, fresh, active,
// healthy, and PX4 is armed in offboard mode. This represents the "all systems go"
// state used as the baseline for tests that want to verify transition logic
// without hitting guard rejections.
uav_manager::HealthSnapshot readySnapshot()
{
  uav_manager::HealthSnapshot snapshot;
  snapshot.px4.present = true;
  snapshot.px4.connected = true;
  snapshot.px4.armed = true;
  snapshot.px4.offboard = true;
  snapshot.px4.failsafe = false;
  snapshot.px4.fresh = true;
  snapshot.px4.reasonCode = "PX4_OK";

  auto ready = [](uav_manager::DependencyReadiness * dep, const char * code)
    {
      dep->present = true;
      dep->active = true;
      dep->healthy = true;
      dep->fresh = true;
      dep->reasonCode = code;
    };

  ready(&snapshot.estimatedState, "ESTIMATED_STATE_OK");
  ready(&snapshot.estimationManager, "ESTIMATION_OK");
  ready(&snapshot.controlManager, "CONTROL_OK");
  ready(&snapshot.trajectoryManager, "TRAJECTORY_OK");
  return snapshot;
}

}  // namespace

// Exercises the complete nominal flight lifecycle: Idle -> Armed -> TakingOff ->
// Hovering -> Flying -> Hovering -> Landing -> Landed -> Idle. Each apply() call
// must return accepted=true and advance to the expected next state.
TEST(SupervisorStateMachineTest, NominalTakeoffFlightLandFlow)
{
  uav_manager::SupervisorStateMachine fsm;
  uav_manager::TransitionGuard guard;
  auto snapshot = readySnapshot();

  auto out = fsm.apply(uav_manager::SupervisorEvent::ArmRequested, guard, snapshot);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Armed);

  out = fsm.apply(uav_manager::SupervisorEvent::TakeoffRequested, guard, snapshot);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::TakingOff);

  out = fsm.apply(uav_manager::SupervisorEvent::TakeoffCompleted, guard, snapshot);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Hovering);

  out = fsm.apply(uav_manager::SupervisorEvent::FlightActionRequested, guard, snapshot);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Flying);

  out = fsm.apply(uav_manager::SupervisorEvent::FlightActionCompleted, guard, snapshot);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Hovering);

  out = fsm.apply(uav_manager::SupervisorEvent::LandRequested, guard, snapshot);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Landing);

  out = fsm.apply(uav_manager::SupervisorEvent::LandCompleted, guard, snapshot);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Landed);

  out = fsm.apply(uav_manager::SupervisorEvent::DisarmCompleted, guard, snapshot);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Idle);
}

// Verifies that events with no matching row in the transition table are rejected
// with ILLEGAL_TRANSITION. Here, LandRequested from Idle has no valid transition
// (you must arm and take off first). The FSM state must not change.
TEST(SupervisorStateMachineTest, RejectsIllegalTransition)
{
  uav_manager::SupervisorStateMachine fsm;
  uav_manager::TransitionGuard guard;

  const auto out = fsm.apply(uav_manager::SupervisorEvent::LandRequested, guard, readySnapshot());
  EXPECT_FALSE(out.accepted);
  EXPECT_EQ(out.reasonCode, "ILLEGAL_TRANSITION");
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Idle);
}

// Verifies that when a guard rejects a transition, the guard's specific reason
// code (e.g., "CONTROL_UNHEALTHY") propagates through TransitionOutcome rather
// than a generic rejection message. This is important for operator diagnostics --
// the action result message shows exactly which subsystem prevented the operation.
TEST(SupervisorStateMachineTest, RejectsGuardFailureWithReason)
{
  uav_manager::SupervisorStateMachine fsm;
  uav_manager::TransitionGuard guard;
  auto snapshot = readySnapshot();
  snapshot.controlManager.healthy = false;
  snapshot.controlManager.reasonCode = "CONTROL_UNHEALTHY";

  const auto out = fsm.apply(uav_manager::SupervisorEvent::ArmRequested, guard, snapshot);
  EXPECT_FALSE(out.accepted);
  EXPECT_EQ(out.reasonCode, "CONTROL_UNHEALTHY");
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Idle);
}

// Verifies that FailsafeDetected is a "wildcard" event that preempts from any
// non-Emergency state. The FSM transition table has rows matching (*, FailsafeDetected)
// with GuardId::Always, so this event always succeeds regardless of current state.
// This is the mechanism that triggers emergency handling when PX4 enters failsafe
// (e.g., GPS loss, RC loss, battery critical, geofence breach).
TEST(SupervisorStateMachineTest, FailsafePreemptsFromAnyState)
{
  uav_manager::SupervisorStateMachine fsm;
  uav_manager::TransitionGuard guard;
  auto snapshot = readySnapshot();

  EXPECT_TRUE(fsm.apply(uav_manager::SupervisorEvent::ArmRequested, guard, snapshot).accepted);
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Armed);

  const auto out = fsm.apply(uav_manager::SupervisorEvent::FailsafeDetected, guard, snapshot);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(out.reasonCode, "FAILSAFE_PREEMPT");
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Emergency);
}

// Verifies the two-step Emergency exit gate:
//   1. With failsafe still active -> EmergencyCleared is rejected (PX4_FAILSAFE_ACTIVE)
//   2. With failsafe cleared -> EmergencyCleared is accepted, returns to Idle
// This prevents the operator from exiting emergency mode while PX4 is still in
// a failsafe state, which could lead to unsafe re-arming attempts.
TEST(SupervisorStateMachineTest, EmergencyClearRequiresGuard)
{
  uav_manager::SupervisorStateMachine fsm;
  uav_manager::TransitionGuard guard;
  auto snapshot = readySnapshot();

  EXPECT_TRUE(fsm.apply(uav_manager::SupervisorEvent::FailsafeDetected, guard, snapshot).accepted);
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Emergency);

  snapshot.px4.failsafe = true;
  snapshot.px4.connected = true;
  snapshot.px4.present = true;
  snapshot.px4.fresh = true;
  auto out = fsm.apply(uav_manager::SupervisorEvent::EmergencyCleared, guard, snapshot);
  EXPECT_FALSE(out.accepted);
  EXPECT_EQ(out.reasonCode, "PX4_FAILSAFE_ACTIVE");
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Emergency);

  snapshot.px4.failsafe = false;
  out = fsm.apply(uav_manager::SupervisorEvent::EmergencyCleared, guard, snapshot);
  EXPECT_TRUE(out.accepted);
  EXPECT_EQ(fsm.state(), uav_manager::SupervisorState::Idle);
}
