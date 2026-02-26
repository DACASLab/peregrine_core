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
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>

namespace uav_manager
{

/**
 * @enum StepCode
 * @brief Canonical reason codes for orchestration steps.
 *
 * This enum replaces scattered string literals ("ARM_SERVICE_TIMEOUT", etc.) with a
 * typed, refactor-safe set of reason codes. A StepResult may also carry a custom
 * string code (e.g. timeout identifiers passed through waitForCondition).
 */
enum class StepCode : uint16_t
{
  Ok = 0,
  Unknown,

  // Preemption / cancellation
  EmergencyPreempt,
  GoalPreempted,

  // PX4 / readiness prerequisites
  Px4StatusMissing,
  Px4Disconnected,
  ControlSetpointTimeout,

  // Service orchestration
  SetPositionFailed,
  PositionModeTimeout,
  ArmServiceUnavailable,
  ArmServiceTimeout,
  ArmServiceRejected,
  SetModeServiceUnavailable,
  SetModeServiceTimeout,
  SetModeRejected,

  // Downstream trajectory_manager forwarding
  TrajectoryExecuteServerUnavailable,
  TrajectoryExecuteGoalTimeout,
  TrajectoryExecuteGoalRejected,
  TrajectoryExecuteResultTimeout,
  TrajectoryExecuteResultNotSucceeded,
  TrajectoryGotoServerUnavailable,
  TrajectoryGotoGoalTimeout,
  TrajectoryGotoGoalRejected,
  TrajectoryGotoResultTimeout,
  TrajectoryGotoResultNotSucceeded,

  // Generic escape hatch for codes that are generated at runtime.
  Custom,
};

/// Maps StepCode to the corresponding stable machine-readable string.
constexpr std::string_view stepCodeToString(const StepCode code) noexcept
{
  switch (code) {
    case StepCode::Ok:
      return "OK";
    case StepCode::Unknown:
      return "UNKNOWN";
    case StepCode::EmergencyPreempt:
      return "EMERGENCY_PREEMPT";
    case StepCode::GoalPreempted:
      return "GOAL_PREEMPTED";
    case StepCode::Px4StatusMissing:
      return "PX4_STATUS_MISSING";
    case StepCode::Px4Disconnected:
      return "PX4_DISCONNECTED";
    case StepCode::ControlSetpointTimeout:
      return "CONTROL_SETPOINT_TIMEOUT";
    case StepCode::SetPositionFailed:
      return "SET_POSITION_FAILED";
    case StepCode::PositionModeTimeout:
      return "POSITION_MODE_TIMEOUT";
    case StepCode::ArmServiceUnavailable:
      return "ARM_SERVICE_UNAVAILABLE";
    case StepCode::ArmServiceTimeout:
      return "ARM_SERVICE_TIMEOUT";
    case StepCode::ArmServiceRejected:
      return "ARM_SERVICE_REJECTED";
    case StepCode::SetModeServiceUnavailable:
      return "SET_MODE_SERVICE_UNAVAILABLE";
    case StepCode::SetModeServiceTimeout:
      return "SET_MODE_SERVICE_TIMEOUT";
    case StepCode::SetModeRejected:
      return "SET_MODE_REJECTED";
    case StepCode::TrajectoryExecuteServerUnavailable:
      return "TRAJECTORY_EXECUTE_SERVER_UNAVAILABLE";
    case StepCode::TrajectoryExecuteGoalTimeout:
      return "TRAJECTORY_EXECUTE_GOAL_TIMEOUT";
    case StepCode::TrajectoryExecuteGoalRejected:
      return "TRAJECTORY_EXECUTE_GOAL_REJECTED";
    case StepCode::TrajectoryExecuteResultTimeout:
      return "TRAJECTORY_EXECUTE_RESULT_TIMEOUT";
    case StepCode::TrajectoryExecuteResultNotSucceeded:
      return "TRAJECTORY_EXECUTE_RESULT_NOT_SUCCEEDED";
    case StepCode::TrajectoryGotoServerUnavailable:
      return "TRAJECTORY_GOTO_SERVER_UNAVAILABLE";
    case StepCode::TrajectoryGotoGoalTimeout:
      return "TRAJECTORY_GOTO_GOAL_TIMEOUT";
    case StepCode::TrajectoryGotoGoalRejected:
      return "TRAJECTORY_GOTO_GOAL_REJECTED";
    case StepCode::TrajectoryGotoResultTimeout:
      return "TRAJECTORY_GOTO_RESULT_TIMEOUT";
    case StepCode::TrajectoryGotoResultNotSucceeded:
      return "TRAJECTORY_GOTO_RESULT_NOT_SUCCEEDED";
    case StepCode::Custom:
      return "CUSTOM";
  }
  return "UNKNOWN";
}

/**
 * @struct StepResult
 * @brief Standardized success/failure result for orchestration steps.
 */
struct StepResult
{
  /// True when the step completed successfully.
  bool success{false};
  /// Canonical reason code.
  StepCode code{StepCode::Unknown};
  /// Runtime-generated reason code when code==Custom (e.g. "ARMED_TIMEOUT").
  std::string customCode;
  /// Optional human-readable detail (e.g. service response message).
  std::string detail;

  /// Returns the machine-readable code string (customCode when code==Custom).
  std::string_view codeView() const
  {
    if (code == StepCode::Custom) {
      return customCode.empty() ? stepCodeToString(StepCode::Unknown) : std::string_view(customCode);
    }
    return stepCodeToString(code);
  }

  /// Returns a stable, operator-friendly description for logs/results.
  std::string describe() const
  {
    if (detail.empty()) {
      return std::string(codeView());
    }
    std::string out;
    out.reserve(codeView().size() + 2 + detail.size());
    out.append(codeView());
    out.append(": ");
    out.append(detail);
    return out;
  }

  static StepResult ok()
  {
    StepResult r;
    r.success = true;
    r.code = StepCode::Ok;
    return r;
  }

  static StepResult fail(const StepCode code)
  {
    StepResult r;
    r.success = false;
    r.code = code;
    return r;
  }

  static StepResult fail(const StepCode code, std::string detail)
  {
    StepResult r;
    r.success = false;
    r.code = code;
    r.detail = std::move(detail);
    return r;
  }

  static StepResult failCustom(std::string customCode)
  {
    StepResult r;
    r.success = false;
    r.code = StepCode::Custom;
    r.customCode = std::move(customCode);
    return r;
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
