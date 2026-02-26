#pragma once

#include <safety_monitor/rule_engine.hpp>
#include <safety_monitor/safety_action_executor.hpp>
#include <safety_monitor/safety_checker.hpp>

#include <peregrine_interfaces/msg/gps_status.hpp>
#include <peregrine_interfaces/msg/px4_status.hpp>
#include <peregrine_interfaces/msg/safety_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/srv/set_mode.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <memory>
#include <mutex>
#include <optional>

namespace safety_monitor
{

class SafetyMonitorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SafetyMonitorNode(const rclcpp::NodeOptions & options);

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  void onBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void onGpsStatus(const peregrine_interfaces::msg::GpsStatus::SharedPtr msg);
  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);
  void onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg);

  void evaluateAndPublish();

  CheckerContext buildContext() const;

  mutable std::mutex mutex_;
  std::optional<BatteryData> latestBattery_;
  std::optional<GpsData> latestGps_;
  std::optional<PositionData> latestPosition_;
  std::optional<Px4Data> latestPx4_;
  std::chrono::steady_clock::time_point lastGpsTime_;

  double evaluateRateHz_{2.0};
  bool commandLandEnabled_{false};
  double gpsFreshnessTimeoutS_{2.0};
  std::string batteryTopic_{"battery"};
  std::string gpsStatusTopic_{"gps_status"};
  std::string estimatedStateTopic_{"estimated_state"};
  std::string px4StatusTopic_{"status"};

  std::unique_ptr<RuleEngine> ruleEngine_;
  std::unique_ptr<SafetyActionExecutor> actionExecutor_;

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batterySub_;
  rclcpp::Subscription<peregrine_interfaces::msg::GpsStatus>::SharedPtr gpsStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::PX4Status>::SharedPtr px4StatusSub_;

  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::SafetyStatus>::SharedPtr safetyStatusPub_;
  rclcpp::Client<peregrine_interfaces::srv::SetMode>::SharedPtr setModeClient_;

  rclcpp::TimerBase::SharedPtr evaluateTimer_;

  bool autoStart_{true};
  rclcpp::TimerBase::SharedPtr startupTimer_;
};

}  // namespace safety_monitor
