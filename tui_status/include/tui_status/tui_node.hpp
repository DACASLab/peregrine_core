#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tui_status/tui_status_parameters.hpp>

#include <peregrine_interfaces/action/land.hpp>
#include <peregrine_interfaces/action/takeoff.hpp>
#include <peregrine_interfaces/msg/compute_status.hpp>
#include <peregrine_interfaces/msg/gps_status.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/px4_status.hpp>
#include <peregrine_interfaces/msg/safety_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>
#include <peregrine_interfaces/srv/arm.hpp>
#include <peregrine_interfaces/srv/clear_emergency.hpp>

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
  void handleKeyInput(int key);
  void sendArm(bool arm);
  void sendTakeoff();
  void sendLand();
  void sendClearEmergency();
  void setCommandStatus(const std::string & command, const std::string & result, bool pending = false);

  void onUavState(const peregrine_interfaces::msg::UAVState::SharedPtr msg);
  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);
  void onSafetyStatus(const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg);
  void onManagerStatus(
    const peregrine_interfaces::msg::ManagerStatus::SharedPtr & msg,
    bool & hadFirst,
    bool & lastHealthy,
    std::optional<peregrine_interfaces::msg::ManagerStatus> & latestStatus,
    const char * label);
  void onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg);
  void onGpsStatus(const peregrine_interfaces::msg::GpsStatus::SharedPtr msg);
  void onComputeStatus(const peregrine_interfaces::msg::ComputeStatus::SharedPtr msg);

  StatusSnapshot buildSnapshot() const;

  std::shared_ptr<tui_status_node::ParamListener> paramListener_;
  tui_status_node::Params params_;

  std::shared_ptr<Renderer> renderer_;
  AlertBuffer alertBuffer_;

  mutable std::mutex mutex_;
  bool exitRequested_{false};
  std::size_t alertScroll_{0};

  // Previous values for transition-based alerting
  int lastSafetyLevel_{-1};
  std::string lastUavStateStr_;
  bool lastArmed_{false};
  bool lastConnected_{false};
  bool lastFailsafe_{false};
  bool lastEstimationHealthy_{true};
  bool lastControlHealthy_{true};
  bool lastTrajectoryHealthy_{true};
  bool hadFirstUavState_{false};
  bool hadFirstPx4Status_{false};
  bool hadFirstEstimationStatus_{false};
  bool hadFirstControlStatus_{false};
  bool hadFirstTrajectoryStatus_{false};

  // Timer tracking
  bool inFlight_{false};
  rclcpp::Time flightStartTime_;
  double accumulatedFlightS_{0.0};

  bool wasArmed_{false};
  rclcpp::Time armStartTime_;
  double accumulatedArmedS_{0.0};

  rclcpp::Time firstUavStateTime_;

  // Staleness timestamps (steady_clock-based via rclcpp)
  rclcpp::Time lastUavStateTime_;
  rclcpp::Time lastEstimatedStateTime_;
  rclcpp::Time lastSafetyStatusTime_;
  rclcpp::Time lastPx4StatusTime_;
  rclcpp::Time lastGpsStatusTime_;
  bool receivedUavState_{false};
  bool receivedEstimatedState_{false};
  bool receivedSafetyStatus_{false};
  bool receivedPx4Status_{false};
  bool receivedGpsStatus_{false};

  std::optional<peregrine_interfaces::msg::UAVState> latestUavState_;
  std::optional<peregrine_interfaces::msg::State> latestEstimatedState_;
  std::optional<peregrine_interfaces::msg::SafetyStatus> latestSafetyStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestEstimationStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestControlStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestTrajectoryStatus_;
  std::optional<peregrine_interfaces::msg::PX4Status> latestPx4Status_;
  std::optional<peregrine_interfaces::msg::GpsStatus> latestGpsStatus_;
  std::optional<peregrine_interfaces::msg::ComputeStatus> latestComputeStatus_;

  rclcpp::Subscription<peregrine_interfaces::msg::UAVState>::SharedPtr uavStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::SafetyStatus>::SharedPtr safetyStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr estimationStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr controlStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr trajectoryStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::PX4Status>::SharedPtr px4StatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::GpsStatus>::SharedPtr gpsStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ComputeStatus>::SharedPtr computeStatusSub_;

  rclcpp::TimerBase::SharedPtr refreshTimer_;

  // Command clients
  rclcpp::Client<peregrine_interfaces::srv::Arm>::SharedPtr armClient_;
  rclcpp::Client<peregrine_interfaces::srv::ClearEmergency>::SharedPtr clearEmergencyClient_;
  rclcpp_action::Client<peregrine_interfaces::action::Takeoff>::SharedPtr takeoffClient_;
  rclcpp_action::Client<peregrine_interfaces::action::Land>::SharedPtr landClient_;

  // Command feedback state
  std::string lastCommand_;
  std::string lastCommandResult_;
  bool commandPending_{false};

  // Arm confirmation (double-press within timeout)
  bool armConfirmPending_{false};
  bool disarmConfirmPending_{false};
  rclcpp::Time confirmDeadline_;
};

}  // namespace tui_status
