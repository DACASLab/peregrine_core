#pragma once

#include <safety_monitor/safety_checker.hpp>

namespace safety_monitor
{

struct GpsCheckerConfig
{
  int min_fix_type{3};
  float max_hdop{5.0f};
  float max_vdop{5.0f};
  int min_satellites{6};
};

class GpsChecker : public SafetyChecker
{
public:
  explicit GpsChecker(GpsCheckerConfig config);
  CheckResult check(const CheckerContext & ctx) const override;
  std::string name() const override;

private:
  GpsCheckerConfig config_;
};

}  // namespace safety_monitor
