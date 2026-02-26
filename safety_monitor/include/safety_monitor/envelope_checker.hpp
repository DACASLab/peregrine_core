#pragma once

#include <safety_monitor/safety_checker.hpp>

namespace safety_monitor
{

struct EnvelopeCheckerConfig
{
  double max_velocity_ms{15.0};
  double max_altitude_m{120.0};
  double max_tilt_rad{0.7};
};

class EnvelopeChecker : public SafetyChecker
{
public:
  explicit EnvelopeChecker(EnvelopeCheckerConfig config);
  CheckResult check(const CheckerContext & ctx) const override;
  std::string name() const override;

private:
  EnvelopeCheckerConfig config_;
};

}  // namespace safety_monitor
