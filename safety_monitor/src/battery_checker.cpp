#include <safety_monitor/battery_checker.hpp>

#include <sstream>

namespace safety_monitor
{

BatteryChecker::BatteryChecker(BatteryCheckerConfig config)
: config_(config)
{
}

CheckResult BatteryChecker::check(const CheckerContext & ctx) const
{
  if (!ctx.battery.has_value()) {
    return {SafetyLevel::Warning, "battery_data_missing"};
  }

  const auto & bat = *ctx.battery;
  std::ostringstream oss;

  if (bat.voltage > 0.0f && bat.voltage < config_.min_voltage) {
    oss << "voltage=" << bat.voltage << "V < min=" << config_.min_voltage << "V";
    return {SafetyLevel::Emergency, oss.str()};
  }

  if (bat.percentage >= 0.0f && bat.percentage <= config_.emergency_pct) {
    oss << "battery_pct=" << bat.percentage << " <= emergency=" << config_.emergency_pct;
    return {SafetyLevel::Emergency, oss.str()};
  }

  if (bat.percentage >= 0.0f && bat.percentage <= config_.critical_pct) {
    oss << "battery_pct=" << bat.percentage << " <= critical=" << config_.critical_pct;
    return {SafetyLevel::Critical, oss.str()};
  }

  if (bat.percentage >= 0.0f && bat.percentage <= config_.warn_pct) {
    oss << "battery_pct=" << bat.percentage << " <= warning=" << config_.warn_pct;
    return {SafetyLevel::Warning, oss.str()};
  }

  return {SafetyLevel::Nominal, "battery_ok"};
}

std::string BatteryChecker::name() const
{
  return "battery";
}

}  // namespace safety_monitor
