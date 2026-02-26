#pragma once

#include <safety_monitor/safety_checker.hpp>

namespace safety_monitor
{

struct BatteryCheckerConfig
{
  float warn_pct{0.25f};
  float critical_pct{0.15f};
  float emergency_pct{0.10f};
  float min_voltage{10.0f};
};

class BatteryChecker : public SafetyChecker
{
public:
  explicit BatteryChecker(BatteryCheckerConfig config);
  CheckResult check(const CheckerContext & ctx) const override;
  std::string name() const override;

private:
  BatteryCheckerConfig config_;
};

}  // namespace safety_monitor
