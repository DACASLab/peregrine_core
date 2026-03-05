#include <safety_monitor/geofence_checker.hpp>

#include <cmath>
#include <sstream>

namespace safety_monitor
{

GeofenceChecker::GeofenceChecker(GeofenceCheckerConfig config)
: config_(config)
{
  GeofenceBoundary initial;
  initial.center_x_m = 0.0;
  initial.center_y_m = 0.0;
  initial.min_altitude_m = config_.min_altitude_m;
  initial.max_altitude_m = config_.max_altitude_m;
  initial.max_radius_m = config_.max_radius_m;
  boundary_ = initial;
}

CheckResult GeofenceChecker::check(const CheckerContext & ctx) const
{
  if (!ctx.position.has_value()) {
    return {SafetyLevel::Nominal, "position_data_missing_skip"};
  }

  std::optional<GeofenceBoundary> boundary;
  {
    std::scoped_lock lock(mutex_);
    boundary = boundary_;
  }
  if (!boundary.has_value()) {
    return {SafetyLevel::Warning, "geofence_boundary_missing"};
  }

  const auto & pos = *ctx.position;
  std::ostringstream oss;

  const double dx = pos.x - boundary->center_x_m;
  const double dy = pos.y - boundary->center_y_m;
  const double horizontalDist = std::sqrt(dx * dx + dy * dy);
  if (horizontalDist > boundary->max_radius_m) {
    oss << "horizontal_dist=" << horizontalDist << "m > max=" << boundary->max_radius_m << "m";
    return {SafetyLevel::Critical, oss.str()};
  }

  if (pos.z > boundary->max_altitude_m) {
    oss << "altitude=" << pos.z << "m > max=" << boundary->max_altitude_m << "m";
    return {SafetyLevel::Critical, oss.str()};
  }

  if (pos.z < boundary->min_altitude_m) {
    oss << "altitude=" << pos.z << "m < min=" << boundary->min_altitude_m << "m";
    return {SafetyLevel::Warning, oss.str()};
  }

  return {SafetyLevel::Nominal, "geofence_ok"};
}

std::string GeofenceChecker::name() const
{
  return "geofence";
}

void GeofenceChecker::updateBoundary(const GeofenceBoundary & boundary)
{
  std::scoped_lock lock(mutex_);
  boundary_ = boundary;
}

}  // namespace safety_monitor
