#include <safety_monitor/rule_engine.hpp>

namespace safety_monitor
{

void RuleState::update(
  const CheckResult & checkResult,
  const RuleConfig & ruleConfig,
  double healthyAutoClearS,
  std::chrono::steady_clock::time_point now)
{
  if (checkResult.level == SafetyLevel::Nominal) {
    if (level != SafetyLevel::Nominal) {
      if (lastHealthy == std::chrono::steady_clock::time_point{}) {
        lastHealthy = now;
      }
      const auto healthyDuration = std::chrono::duration<double>(now - lastHealthy).count();
      if (healthyDuration >= healthyAutoClearS) {
        level = SafetyLevel::Nominal;
        reason.clear();
        graceExpired = false;
      }
    }
    return;
  }

  lastHealthy = {};

  if (level == SafetyLevel::Nominal) {
    firstTriggered = now;
    graceExpired = false;
  }

  const auto elapsed = std::chrono::duration<double>(now - firstTriggered).count();

  if (checkResult.level >= SafetyLevel::Critical) {
    if (elapsed >= ruleConfig.critical_grace_s) {
      graceExpired = true;
    }
  } else if (checkResult.level >= SafetyLevel::Warning) {
    if (elapsed >= ruleConfig.warn_grace_s) {
      graceExpired = true;
    }
  }

  level = checkResult.level;
  reason = checkResult.reason;
}

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
    auto & state = ruleStates_[rule.checker->name()];

    state.update(checkResult, rule.config, config_.healthy_auto_clear_s, now);

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
