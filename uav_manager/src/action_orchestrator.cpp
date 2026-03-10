#include <uav_manager/action_orchestrator.hpp>

#include <rclcpp/rclcpp.hpp>

namespace uav_manager
{

ActionOrchestrator::ActionOrchestrator(const OrchestratorConfig config, rclcpp::Clock::SharedPtr clock)
: config_(config)
, clock_(std::move(clock))
{
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

StepResult ActionOrchestrator::waitForCondition(
  const std::string & timeoutCode,
  const std::chrono::milliseconds timeout,
  const std::function<bool()> & condition,
  const std::function<bool()> & preempted,
  const std::function<bool()> & emergency) const
{
  // Polling keeps waits interruptible and reason-code deterministic.
  rclcpp::Rate rate(
    std::chrono::duration<double>(config_.pollPeriod).count() > 0.0
    ? 1.0 / std::chrono::duration<double>(config_.pollPeriod).count()
    : 20.0);

  const auto deadline = clock_->now() + rclcpp::Duration(timeout);
  while (rclcpp::ok() && clock_->now() < deadline) {
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
