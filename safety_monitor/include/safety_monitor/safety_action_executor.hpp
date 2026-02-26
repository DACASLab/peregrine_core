#pragma once

#include <peregrine_interfaces/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace safety_monitor
{

enum class LandCommandState
{
  Idle,
  Sending,
  Sent,
  Timeout,
  Rejected
};

struct SafetyActionConfig
{
  double land_command_timeout_s{5.0};
  int land_command_retry_count{3};
};

class SafetyActionExecutor
{
public:
  SafetyActionExecutor(
    rclcpp::Client<peregrine_interfaces::srv::SetMode>::SharedPtr setModeClient,
    rclcpp::Logger logger,
    SafetyActionConfig config);

  void requestLand(const std::string & reason);

  void tick(std::chrono::steady_clock::time_point now);

  LandCommandState state() const { return state_; }

  void reset();

private:
  void sendLandCommand();

  rclcpp::Client<peregrine_interfaces::srv::SetMode>::SharedPtr setModeClient_;
  rclcpp::Logger logger_;
  SafetyActionConfig config_;

  LandCommandState state_{LandCommandState::Idle};
  std::string reason_;
  int retryCount_{0};
  std::chrono::steady_clock::time_point lastSendTime_;
  bool pendingResponse_{false};
};

}  // namespace safety_monitor
