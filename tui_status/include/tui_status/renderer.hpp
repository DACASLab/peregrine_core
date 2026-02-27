#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <tui_status/alert_buffer.hpp>

namespace tui_status
{

struct SafetyCheckerView
{
  std::string name;
  uint8_t level{0};
  std::string reason;
};

struct StatusSnapshot
{
  std::string state{"UNKNOWN"};
  std::string mode{"UNKNOWN"};
  bool armed{false};
  bool offboard{false};
  bool connected{false};
  bool failsafe{false};

  bool dependencies_ready{false};
  bool safety_ready{false};
  std::string readiness_detail;

  bool has_pose{false};
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};

  bool has_velocity{false};
  double vx_mps{0.0};
  double vy_mps{0.0};
  double vz_mps{0.0};

  std::string estimation_module{"-"};
  bool estimation_healthy{false};
  double estimation_rate_hz{0.0};
  std::string control_module{"-"};
  bool control_healthy{false};
  double control_rate_hz{0.0};
  std::string trajectory_module{"-"};
  bool trajectory_healthy{false};
  double trajectory_rate_hz{0.0};

  float battery_percent{-1.0F};
  float battery_voltage{-1.0F};
  bool has_motor_data{false};
  float motor_output[4]{0.0F, 0.0F, 0.0F, 0.0F};
  uint8_t gps_fix_type{0};
  uint8_t gps_satellites{0};
  float gps_hdop{0.0F};
  float gps_vdop{0.0F};

  uint8_t safety_level{0};
  bool has_safety_status{false};
  std::string safety_reason{"unknown"};
  std::vector<SafetyCheckerView> checker_levels;

  // Staleness ages in seconds (-1 = never received)
  double estimated_state_age_s{-1.0};
  double safety_status_age_s{-1.0};
  double px4_status_age_s{-1.0};
  double gps_status_age_s{-1.0};
  double uav_state_age_s{-1.0};

  // Timers
  double uptime_s{0.0};
  double flight_time_s{0.0};
};

class Renderer
{
public:
  Renderer();
  ~Renderer();

  Renderer(const Renderer &) = delete;
  Renderer & operator=(const Renderer &) = delete;

  bool initialized() const {return initialized_;}
  int pollKey() const;

  void render(
    const StatusSnapshot & snapshot,
    const std::vector<AlertEntry> & alerts,
    std::size_t alert_scroll,
    const std::string & uav_namespace) const;

private:
  static std::string truncate(const std::string & text, int max_width);

  bool initialized_{false};
  bool hasColors_{false};
};

}  // namespace tui_status
