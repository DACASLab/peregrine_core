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
 * @file action_orchestrator.hpp
 * @brief Orchestrates explicit waits/timeouts and preemption-aware steps.
 *
 * The orchestrator provides two building blocks for uav_manager's action server
 * accepted callbacks:
 *
 *   callStep():  Runs a function after checking emergency/preemption predicates.
 *                Used to wrap service calls (arm, set_mode) so they can be aborted
 *                immediately if a failsafe is detected mid-call.
 *
 *   waitForCondition():  Polls a boolean condition at a configurable rate (default 50ms)
 *                        with emergency/preemption checks on each tick. Used to wait for
 *                        PX4 state changes (armed confirmation, offboard entry, auto-disarm).
 *
 * Both methods return StepResult with machine-readable reason codes that propagate
 * through the action result. This gives the action client a deterministic explanation
 * for any failure (e.g., "ARMED_TIMEOUT", "EMERGENCY_PREEMPT", "GOAL_PREEMPTED").
 *
 * The polling approach (rather than blocking waits) is intentional: it ensures that
 * emergency and cancel events are detected within one poll period (~50ms) rather than
 * being delayed until a potentially long timeout expires.
 */

#pragma once

#include <chrono>
#include <functional>
#include <string>

namespace uav_manager
{

/**
 * @struct StepResult
 * @brief Standardized success/failure result for orchestration steps.
 */
struct StepResult
{
  /// True when the step completed successfully.
  bool success{false};
  /// Machine-readable reason code (`OK`, timeout code, preempt code, etc.).
  std::string reasonCode{"UNKNOWN"};

  static StepResult ok(const std::string & reason = "OK")
  {
    return StepResult{true, reason};
  }

  static StepResult fail(const std::string & reason)
  {
    return StepResult{false, reason};
  }
};

/**
 * @struct OrchestratorConfig
 * @brief Timing configuration for polling-based waits.
 */
struct OrchestratorConfig
{
  /// Poll period used by wait loops.
  std::chrono::milliseconds pollPeriod{50};
};

/**
 * @class ActionOrchestrator
 * @brief Runs preemption-aware action/service/wait steps with explicit reason codes.
 */
class ActionOrchestrator
{
public:
  /**
   * @brief Constructs the orchestrator with polling configuration.
   */
  explicit ActionOrchestrator(OrchestratorConfig config);

  /**
   * @brief Executes one step function after preemption checks.
   *
   * @param stepCode Stable step identifier used as fallback reason code.
   * @param fn Step function that returns detailed result.
   * @param preempted Predicate indicating action cancel/preempt.
   * @param emergency Predicate indicating emergency takeover.
   * @return Step result with machine-readable reason.
   */
  StepResult callStep(
    const std::string & stepCode,
    const std::function<StepResult()> & fn,
    const std::function<bool()> & preempted,
    const std::function<bool()> & emergency) const;

  /**
   * @brief Waits until a condition is true, preempted, or timed out.
   *
   * @param timeoutCode Reason code returned on timeout.
   * @param timeout Maximum wait duration.
   * @param condition Condition to poll.
   * @param preempted Predicate indicating action cancel/preempt.
   * @param emergency Predicate indicating emergency takeover.
   * @return Step result with machine-readable reason.
   */
  StepResult waitForCondition(
    const std::string & timeoutCode,
    std::chrono::milliseconds timeout,
    const std::function<bool()> & condition,
    const std::function<bool()> & preempted,
    const std::function<bool()> & emergency) const;

private:
  /// Polling/wait configuration.
  OrchestratorConfig config_;
};

}  // namespace uav_manager
