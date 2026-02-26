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
#include <uav_manager/action_orchestrator.hpp>

#include <rclcpp/rclcpp.hpp>

namespace uav_manager
{

ActionOrchestrator::ActionOrchestrator(const OrchestratorConfig config)
: config_(config)
{
  // Small POD config is copied by value into an owning member for simplicity.
}

StepResult ActionOrchestrator::callStep(
  const std::string & stepCode,
  const std::function<StepResult()> & fn,
  const std::function<bool()> & preempted,
  const std::function<bool()> & emergency) const
{
  // Pre-checks ensure emergency/cancel always wins before executing the step.
  if (emergency()) {
    return StepResult::fail(StepCode::EmergencyPreempt);
  }
  if (preempted()) {
    return StepResult::fail(StepCode::GoalPreempted);
  }

  StepResult result = fn();
  // Local variable here intentionally makes the failure-path mutation explicit
  // (we may inject a synthesized reasonCode below).
  if (result.success) {
    return result;
  }

  // If a step returns failure without reason, synthesize one from step identity.
  if (result.code == StepCode::Unknown && result.customCode.empty()) {
    result.code = StepCode::Custom;
    result.customCode = stepCode + "_FAILED";
  }
  return result;
}

// `const std::function<bool()> & condition` takes any callable (function pointer,
// lambda, functor) that returns bool and accepts no arguments. `std::function` is
// C++'s type-erased callable wrapper - similar to how Python treats all callables
// uniformly (functions, lambdas, bound methods). The `&` means it's passed by
// reference (no copy of the callable object).
StepResult ActionOrchestrator::waitForCondition(
  const std::string & timeoutCode,
  const std::chrono::milliseconds timeout,
  const std::function<bool()> & condition,
  const std::function<bool()> & preempted,
  const std::function<bool()> & emergency) const
{
  // Polling keeps waits interruptible and reason-code deterministic.
  //
  // `rclcpp::Rate` is a timing helper that sleeps for the remainder of the period
  // after work is done, similar to Python's `rospy.Rate(hz)`. The `sleep()` call
  // at the bottom of the loop blocks the thread until the next tick.
  rclcpp::Rate rate(
    std::chrono::duration<double>(config_.pollPeriod).count() > 0.0
    ? 1.0 / std::chrono::duration<double>(config_.pollPeriod).count()
    : 20.0);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    // Emergency and cancel checks are evaluated on each poll tick.
    if (emergency()) {
      return StepResult::fail(StepCode::EmergencyPreempt);
    }
    if (preempted()) {
      return StepResult::fail(StepCode::GoalPreempted);
    }
    if (condition()) {
      return StepResult::ok();
    }
    rate.sleep();
  }

  // One final condition check handles races where condition flips near deadline.
  if (condition()) {
    return StepResult::ok();
  }

  return StepResult::failCustom(timeoutCode);
}

}  // namespace uav_manager
