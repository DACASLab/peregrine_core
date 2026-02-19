/**
 * @file px4_hardware_abstraction.hpp
 * @brief PX4 boundary node for state conversion and command routing.
 */

#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <peregrine_interfaces/msg/control_output.hpp>
#include <peregrine_interfaces/msg/px4_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/srv/arm.hpp>
#include <peregrine_interfaces/srv/set_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <atomic>
#include <cstdint>
#include <cmath>
#include <string>

namespace hardware_abstraction
{

/**
 * @class PX4HardwareAbstraction
 * @brief Bridges manager-facing interfaces and PX4 uXRCE-DDS topics.
 *
 * Responsibilities:
 * - Convert PX4 NED/FRD telemetry to ROS ENU/FLU interfaces.
 * - Convert manager control outputs to PX4-compatible setpoints.
 * - Publish PX4 command messages for arm/disarm and mode transitions.
 * - Periodically publish offboard control mode heartbeat.
 */
class PX4HardwareAbstraction : public rclcpp::Node
{
public:
  /**
   * @brief Constructs the bridge node and initializes pubs/subs/services.
   */
  explicit PX4HardwareAbstraction(const rclcpp::NodeOptions& options);

private:
  /**
   * @brief Handles PX4 odometry and publishes manager-facing state.
   */
  void onVehicleOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  /**
   * @brief Converts battery telemetry into ROS battery status.
   */
  void onBatteryStatus(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  /**
   * @brief Converts PX4 GPS telemetry into NavSatFix.
   */
  void onSensorGps(const px4_msgs::msg::SensorGps::SharedPtr msg);
  /**
   * @brief Updates arming/mode/failsafe state from PX4 vehicle status.
   */
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  /**
   * @brief Routes manager command envelope into the selected PX4 input topic.
   */
  void onControlOutput(const peregrine_interfaces::msg::ControlOutput::SharedPtr msg);

  /**
   * @brief Service callback to arm or disarm the vehicle.
   */
  void onArmService(const std::shared_ptr<peregrine_interfaces::srv::Arm::Request> request,
                    std::shared_ptr<peregrine_interfaces::srv::Arm::Response> response);
  /**
   * @brief Service callback to request PX4 mode changes.
   */
  void onSetModeService(const std::shared_ptr<peregrine_interfaces::srv::SetMode::Request> request,
                        std::shared_ptr<peregrine_interfaces::srv::SetMode::Response> response);

  /**
   * @brief Publishes OffboardControlMode at a fixed periodic rate.
   */
  void publishOffboardControlMode();
  /**
   * @brief Publishes consolidated PX4 status for manager/state-machine logic.
   */
  void publishStatus();

  /**
   * @brief Creates a fully-populated VehicleCommand packet.
   */
  px4_msgs::msg::VehicleCommand makeVehicleCommand(uint32_t command, float param1 = 0.0f, float param2 = 0.0f) const;
  /**
   * @brief Maps a string mode request into PX4 nav-state enum.
   */
  uint8_t navStateFromModeString(const std::string& mode, bool* valid) const;
  /**
   * @brief Returns current node time in microseconds for PX4 timestamp fields.
   */
  uint64_t nowMicros() const;
  /**
   * @brief Checks whether PX4 telemetry has been received recently.
   */
  bool isConnected() const;
  /**
   * @brief Prefixes PX4 topic suffixes with configured namespace.
   */
  std::string px4Topic(const std::string& suffix) const;

  /// PX4 ROS namespace prefix (e.g. "/uav1"), empty for global topics.
  std::string px4Namespace_;
  /// Frame used for published odometry parent.
  std::string odomFrame_;
  /// Frame used for published odometry child.
  std::string baseLinkFrame_;
  /// PX4 SensorGps topic suffix under px4_namespace.
  std::string sensorGpsTopicSuffix_;
  /// Periodic offboard mode publication rate.
  double offboardRateHz_;
  /// Periodic status publication rate.
  double statusRateHz_;
  /// Maximum allowed telemetry silence before marking disconnected.
  double connectionTimeoutS_;

  /// Target system ID for PX4 command messages.
  int targetSystemId_;
  /// Target component ID for PX4 command messages.
  int targetComponentId_;
  /// Source system ID for PX4 command messages.
  int sourceSystemId_;
  /// Source component ID for PX4 command messages.
  int sourceComponentId_;

  std::atomic<int64_t> lastPx4RxTimeNs_{0};
  std::atomic<uint8_t> navState_{0};
  std::atomic<uint8_t> armingState_{0};
  std::atomic<uint16_t> failureDetectorStatus_{0};
  std::atomic<bool> failsafe_{false};
  std::atomic<float> batteryRemaining_{NAN};
  std::atomic<float> batteryVoltage_{NAN};
  /// Packed snapshot of control mode + trajectory flags for coherent offboard heartbeat publication.
  std::atomic<uint32_t> offboardModeFlags_{0};

  rclcpp::Publisher<peregrine_interfaces::msg::State>::SharedPtr statePub_;
  rclcpp::Publisher<peregrine_interfaces::msg::PX4Status>::SharedPtr statusPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryPub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsPub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicleOdometrySub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr batteryStatusSub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr sensorGpsSub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicleStatusSub_;

  rclcpp::Subscription<peregrine_interfaces::msg::ControlOutput>::SharedPtr controlOutputSub_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboardControlModePub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectorySetpointPub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr vehicleRatesSetpointPub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr vehicleAttitudeSetpointPub_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuatorMotorsPub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicleCommandPub_;

  rclcpp::Service<peregrine_interfaces::srv::Arm>::SharedPtr armService_;
  rclcpp::Service<peregrine_interfaces::srv::SetMode>::SharedPtr setModeService_;

  rclcpp::TimerBase::SharedPtr offboardModeTimer_;
  rclcpp::TimerBase::SharedPtr statusTimer_;
};

}  // namespace hardware_abstraction
