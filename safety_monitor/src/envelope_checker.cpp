#include <safety_monitor/envelope_checker.hpp>

#include <cmath>
#include <sstream>

namespace safety_monitor
{

EnvelopeChecker::EnvelopeChecker(EnvelopeCheckerConfig config)
: config_(config)
{
}

CheckResult EnvelopeChecker::check(const CheckerContext & ctx) const
{
  if (!ctx.position.has_value()) {
    return {SafetyLevel::Nominal, "position_data_missing_skip"};
  }

  const auto & pos = *ctx.position;
  std::ostringstream oss;

  const double speed = std::sqrt(pos.vx * pos.vx + pos.vy * pos.vy + pos.vz * pos.vz);
  if (speed > config_.max_velocity_ms) {
    oss << "velocity=" << speed << "m/s > max=" << config_.max_velocity_ms << "m/s";
    return {SafetyLevel::Critical, oss.str()};
  }

  if (pos.z > config_.max_altitude_m) {
    oss << "altitude=" << pos.z << "m > max=" << config_.max_altitude_m << "m";
    return {SafetyLevel::Critical, oss.str()};
  }

  const double tilt = std::sqrt(pos.roll * pos.roll + pos.pitch * pos.pitch);
  if (tilt > config_.max_tilt_rad) {
    oss << "tilt=" << tilt << "rad > max=" << config_.max_tilt_rad << "rad";
    return {SafetyLevel::Warning, oss.str()};
  }

  return {SafetyLevel::Nominal, "envelope_ok"};
}

std::string EnvelopeChecker::name() const
{
  return "envelope";
}

}  // namespace safety_monitor
