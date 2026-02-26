#pragma once

#include <safety_monitor/safety_checker.hpp>

namespace safety_monitor
{

struct GeofenceCheckerConfig
{
  double max_radius_m{500.0};
  double max_altitude_m{120.0};
  double min_altitude_m{-5.0};
};

class GeofenceChecker : public SafetyChecker
{
public:
  explicit GeofenceChecker(GeofenceCheckerConfig config);
  CheckResult check(const CheckerContext & ctx) const override;
  std::string name() const override;

private:
  GeofenceCheckerConfig config_;
};

}  // namespace safety_monitor
