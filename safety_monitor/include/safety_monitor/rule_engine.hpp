#pragma once

#include <safety_monitor/safety_checker.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace safety_monitor
{

struct RuleConfig
{
  bool enabled{true};
  double warn_grace_s{5.0};
  double critical_grace_s{2.0};
};

struct RuleEngineConfig
{
  double healthy_auto_clear_s{3.0};
};

struct RuleState
{
  SafetyLevel level{SafetyLevel::Nominal};
  std::string reason;
  std::chrono::steady_clock::time_point firstTriggered;
  std::chrono::steady_clock::time_point lastHealthy;
  bool graceExpired{false};
};

class RuleEngine
{
public:
  explicit RuleEngine(RuleEngineConfig config);

  void addChecker(std::shared_ptr<SafetyChecker> checker, RuleConfig ruleConfig);

  SafetyLevel evaluate(
    const CheckerContext & ctx,
    std::chrono::steady_clock::time_point now);

  struct EvaluationResult
  {
    SafetyLevel overallLevel{SafetyLevel::Nominal};
    std::vector<CheckResult> results;
  };

  EvaluationResult evaluateDetailed(
    const CheckerContext & ctx,
    std::chrono::steady_clock::time_point now);

private:
  struct Rule
  {
    std::shared_ptr<SafetyChecker> checker;
    RuleConfig config;
  };

  RuleEngineConfig config_;
  std::vector<Rule> rules_;
  std::unordered_map<std::string, RuleState> ruleStates_;
};

}  // namespace safety_monitor
