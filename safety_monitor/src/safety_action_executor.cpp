#include <safety_monitor/safety_action_executor.hpp>

namespace safety_monitor
{

SafetyActionExecutor::SafetyActionExecutor(
  rclcpp::Client<peregrine_interfaces::srv::SetMode>::SharedPtr setModeClient,
  rclcpp::Logger logger,
  SafetyActionConfig config)
: setModeClient_(std::move(setModeClient))
, logger_(logger)
, config_(config)
{
}

void SafetyActionExecutor::requestLand(const std::string & reason)
{
  if (state_ != LandCommandState::Idle) {
    return;  // Already in progress
  }

  reason_ = reason;
  retryCount_ = 0;
  sendLandCommand();
}

void SafetyActionExecutor::tick(std::chrono::steady_clock::time_point now)
{
  if (state_ != LandCommandState::Sending) {
    return;
  }

  if (pendingResponse_) {
    const auto elapsed = std::chrono::duration<double>(now - lastSendTime_).count();
    if (elapsed >= config_.land_command_timeout_s) {
      RCLCPP_WARN(logger_, "Land command timeout (attempt %d/%d)",
        retryCount_, config_.land_command_retry_count);

      if (retryCount_ < config_.land_command_retry_count) {
        pendingResponse_ = false;
        sendLandCommand();
      } else {
        state_ = LandCommandState::Timeout;
        RCLCPP_ERROR(logger_, "Land command failed after %d retries: %s",
          config_.land_command_retry_count, reason_.c_str());
      }
    }
  }
}

void SafetyActionExecutor::reset()
{
  state_ = LandCommandState::Idle;
  reason_.clear();
  retryCount_ = 0;
  pendingResponse_ = false;
}

void SafetyActionExecutor::sendLandCommand()
{
  if (!setModeClient_ || !setModeClient_->service_is_ready()) {
    RCLCPP_WARN(logger_, "set_mode service not available for safety land command");
    state_ = LandCommandState::Rejected;
    return;
  }

  auto request = std::make_shared<peregrine_interfaces::srv::SetMode::Request>();
  request->mode = "land";

  state_ = LandCommandState::Sending;
  pendingResponse_ = true;
  lastSendTime_ = std::chrono::steady_clock::now();
  retryCount_++;

  RCLCPP_WARN(logger_, "Sending safety land command (attempt %d): %s",
    retryCount_, reason_.c_str());

  setModeClient_->async_send_request(
    request,
    [this](rclcpp::Client<peregrine_interfaces::srv::SetMode>::SharedFuture future) {
      pendingResponse_ = false;
      try {
        auto response = future.get();
        if (response->success) {
          state_ = LandCommandState::Sent;
          RCLCPP_WARN(logger_, "Safety land command accepted");
        } else {
          RCLCPP_WARN(logger_, "Safety land command rejected: %s", response->message.c_str());
          if (retryCount_ < config_.land_command_retry_count) {
            sendLandCommand();
          } else {
            state_ = LandCommandState::Rejected;
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Safety land command exception: %s", e.what());
        state_ = LandCommandState::Rejected;
      }
    });
}

}  // namespace safety_monitor
