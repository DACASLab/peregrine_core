#include <safety_monitor/rule_engine.hpp>

#include <algorithm>

namespace safety_monitor
{

RuleEngine::RuleEngine(RuleEngineConfig config)
: config_(config)
{
}

void RuleEngine::addChecker(std::shared_ptr<SafetyChecker> checker, RuleConfig ruleConfig)
{
  rules_.push_back(Rule{std::move(checker), ruleConfig});
}

SafetyLevel RuleEngine::evaluate(
  const CheckerContext & ctx,
  std::chrono::steady_clock::time_point now)
{
  return evaluateDetailed(ctx, now).overallLevel;
}

RuleEngine::EvaluationResult RuleEngine::evaluateDetailed(
  const CheckerContext & ctx,
  std::chrono::steady_clock::time_point now)
{
  EvaluationResult result;
  result.overallLevel = SafetyLevel::Nominal;

  for (const auto & rule : rules_) {
    if (!rule.config.enabled) {
      continue;
    }

    const auto checkResult = rule.checker->check(ctx);
    const auto checkerName = rule.checker->name();

    auto & state = ruleStates_[checkerName];

    if (checkResult.level == SafetyLevel::Nominal) {
      // Check auto-clear: sustained healthy for healthy_auto_clear_s
      if (state.level != SafetyLevel::Nominal) {
        if (state.lastHealthy == std::chrono::steady_clock::time_point{}) {
          state.lastHealthy = now;
        }
        const auto healthyDuration = std::chrono::duration<double>(now - state.lastHealthy).count();
        if (healthyDuration >= config_.healthy_auto_clear_s) {
          state.level = SafetyLevel::Nominal;
          state.reason.clear();
          state.graceExpired = false;
        }
      }
    } else {
      state.lastHealthy = {};  // Reset healthy timer

      if (state.level == SafetyLevel::Nominal) {
        // First trigger
        state.firstTriggered = now;
        state.graceExpired = false;
      }

      const auto elapsed = std::chrono::duration<double>(now - state.firstTriggered).count();

      if (checkResult.level >= SafetyLevel::Critical) {
        if (elapsed >= rule.config.critical_grace_s) {
          state.graceExpired = true;
        }
      } else if (checkResult.level >= SafetyLevel::Warning) {
        if (elapsed >= rule.config.warn_grace_s) {
          state.graceExpired = true;
        }
      }

      state.level = checkResult.level;
      state.reason = checkResult.reason;
    }

    // Report the current level (grace only affects action trigger, not reporting)
    CheckResult reportedResult;
    reportedResult.level = state.level;
    reportedResult.reason = checkResult.reason;
    result.results.push_back(reportedResult);

    if (state.level > result.overallLevel) {
      result.overallLevel = state.level;
    }
  }

  return result;
}

}  // namespace safety_monitor
