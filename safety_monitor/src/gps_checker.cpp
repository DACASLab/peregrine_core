#include <safety_monitor/gps_checker.hpp>

#include <sstream>

namespace safety_monitor
{

GpsChecker::GpsChecker(GpsCheckerConfig config)
: config_(config)
{
}

CheckResult GpsChecker::check(const CheckerContext & ctx) const
{
  if (!ctx.gps.has_value()) {
    return {SafetyLevel::Warning, "gps_data_missing"};
  }

  const auto & gps = *ctx.gps;
  std::ostringstream oss;

  if (gps.fix_type < config_.min_fix_type) {
    oss << "fix_type=" << static_cast<int>(gps.fix_type)
        << " < min=" << config_.min_fix_type;
    return {SafetyLevel::Critical, oss.str()};
  }

  if (gps.satellites_used < config_.min_satellites) {
    oss << "satellites=" << static_cast<int>(gps.satellites_used)
        << " < min=" << config_.min_satellites;
    return {SafetyLevel::Critical, oss.str()};
  }

  if (gps.hdop > config_.max_hdop) {
    oss << "hdop=" << gps.hdop << " > max=" << config_.max_hdop;
    return {SafetyLevel::Warning, oss.str()};
  }

  if (gps.vdop > config_.max_vdop) {
    oss << "vdop=" << gps.vdop << " > max=" << config_.max_vdop;
    return {SafetyLevel::Warning, oss.str()};
  }

  return {SafetyLevel::Nominal, "gps_ok"};
}

std::string GpsChecker::name() const
{
  return "gps";
}

}  // namespace safety_monitor
