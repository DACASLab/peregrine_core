#include <safety_monitor/geofence_checker.hpp>

#include <cmath>
#include <sstream>

namespace safety_monitor
{

GeofenceChecker::GeofenceChecker(GeofenceCheckerConfig config)
: config_(config)
{
}

CheckResult GeofenceChecker::check(const CheckerContext & ctx) const
{
  if (!ctx.position.has_value()) {
    return {SafetyLevel::Nominal, "position_data_missing_skip"};
  }

  const auto & pos = *ctx.position;
  std::ostringstream oss;

  const double horizontalDist = std::sqrt(pos.x * pos.x + pos.y * pos.y);
  if (horizontalDist > config_.max_radius_m) {
    oss << "horizontal_dist=" << horizontalDist << "m > max=" << config_.max_radius_m << "m";
    return {SafetyLevel::Critical, oss.str()};
  }

  if (pos.z > config_.max_altitude_m) {
    oss << "altitude=" << pos.z << "m > max=" << config_.max_altitude_m << "m";
    return {SafetyLevel::Critical, oss.str()};
  }

  if (pos.z < config_.min_altitude_m) {
    oss << "altitude=" << pos.z << "m < min=" << config_.min_altitude_m << "m";
    return {SafetyLevel::Warning, oss.str()};
  }

  return {SafetyLevel::Nominal, "geofence_ok"};
}

std::string GeofenceChecker::name() const
{
  return "geofence";
}

}  // namespace safety_monitor
