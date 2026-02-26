#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace safety_monitor
{

enum class SafetyLevel : uint8_t
{
  Nominal = 0,
  Warning = 1,
  Critical = 2,
  Emergency = 3
};

struct CheckResult
{
  SafetyLevel level{SafetyLevel::Nominal};
  std::string reason;
};

struct BatteryData
{
  float percentage{0.0f};
  float voltage{0.0f};
};

struct GpsData
{
  uint8_t fix_type{0};
  float hdop{99.0f};
  float vdop{99.0f};
  float eph{99.0f};
  float epv{99.0f};
  uint8_t satellites_used{0};
};

struct PositionData
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double roll{0.0};
  double pitch{0.0};
};

struct Px4Data
{
  bool connected{false};
  bool armed{false};
  bool failsafe{false};
};

struct CheckerContext
{
  std::optional<BatteryData> battery;
  std::optional<GpsData> gps;
  std::optional<PositionData> position;
  std::optional<Px4Data> px4;
};

class SafetyChecker
{
public:
  virtual ~SafetyChecker() = default;
  virtual CheckResult check(const CheckerContext & ctx) const = 0;
  virtual std::string name() const = 0;
};

}  // namespace safety_monitor
