/**
 * @file step_result.hpp
 * @brief Machine-readable reason codes for uav_manager orchestration steps.
 */

#pragma once

#include <cstdint>
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
  ArmServiceUnavailable,
  ArmServiceTimeout,
  ArmServiceRejected,
  SetModeServiceUnavailable,
  SetModeServiceTimeout,
  SetModeRejected,

  // Downstream trajectory_manager forwarding (used internally for takeoff trajectory)
  TrajectoryExecuteServerUnavailable,
  TrajectoryExecuteGoalTimeout,
  TrajectoryExecuteGoalRejected,
  TrajectoryExecuteResultTimeout,
  TrajectoryExecuteResultNotSucceeded,

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

}  // namespace uav_manager
