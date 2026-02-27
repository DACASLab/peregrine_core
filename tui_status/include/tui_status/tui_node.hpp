#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <peregrine_interfaces/msg/gps_status.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/px4_status.hpp>
#include <peregrine_interfaces/msg/safety_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>

#include <tui_status/alert_buffer.hpp>
#include <tui_status/renderer.hpp>

namespace tui_status
{

class TuiNode : public rclcpp::Node
{
public:
  explicit TuiNode(const std::shared_ptr<Renderer> & renderer);

  bool shouldExit() const;

private:
  std::string topicName(const std::string & base_topic) const;

  void onTimer();
  void onUavState(const peregrine_interfaces::msg::UAVState::SharedPtr msg);
  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);
  void onSafetyStatus(const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg);
  void onEstimationStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  void onControlStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  void onTrajectoryStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  void onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg);
  void onGpsStatus(const peregrine_interfaces::msg::GpsStatus::SharedPtr msg);

  StatusSnapshot buildSnapshot() const;

  std::shared_ptr<Renderer> renderer_;
  AlertBuffer alertBuffer_;

  std::string uavNamespace_;

  mutable std::mutex mutex_;
  bool exitRequested_{false};
  int lastSafetyLevel_{-1};
  std::size_t alertScroll_{0};

  std::optional<peregrine_interfaces::msg::UAVState> latestUavState_;
  std::optional<peregrine_interfaces::msg::State> latestEstimatedState_;
  std::optional<peregrine_interfaces::msg::SafetyStatus> latestSafetyStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestEstimationStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestControlStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestTrajectoryStatus_;
  std::optional<peregrine_interfaces::msg::PX4Status> latestPx4Status_;
  std::optional<peregrine_interfaces::msg::GpsStatus> latestGpsStatus_;

  rclcpp::Subscription<peregrine_interfaces::msg::UAVState>::SharedPtr uavStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::SafetyStatus>::SharedPtr safetyStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr estimationStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr controlStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr trajectoryStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::PX4Status>::SharedPtr px4StatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::GpsStatus>::SharedPtr gpsStatusSub_;

  rclcpp::TimerBase::SharedPtr refreshTimer_;
};

}  // namespace tui_status
