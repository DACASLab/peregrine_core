#pragma once

#include <safety_monitor/safety_checker.hpp>

#include <mutex>
#include <optional>

namespace safety_monitor
{

struct GeofenceCheckerConfig
{
  double max_radius_m{500.0};
  double max_altitude_m{120.0};
  double min_altitude_m{-5.0};
};

struct GeofenceBoundary
{
  double center_x_m{0.0};
  double center_y_m{0.0};
  double min_altitude_m{-5.0};
  double max_altitude_m{120.0};
  double max_radius_m{500.0};
};

class GeofenceChecker : public SafetyChecker
{
public:
  explicit GeofenceChecker(GeofenceCheckerConfig config);
  CheckResult check(const CheckerContext & ctx) const override;
  std::string name() const override;
  void updateBoundary(const GeofenceBoundary & boundary);

private:
  GeofenceCheckerConfig config_;
  mutable std::mutex mutex_;
  std::optional<GeofenceBoundary> boundary_;
};

}  // namespace safety_monitor
