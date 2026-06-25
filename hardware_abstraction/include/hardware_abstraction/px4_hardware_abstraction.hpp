/**
 * @file px4_hardware_abstraction.hpp
 * @brief PX4 boundary node for state conversion and command routing.
 *
 * Pipeline position: PX4 (uXRCE-DDS) <-> [hardware_abstraction] <-> peregrine managers
 *
 * This node is the ONLY component in the stack that directly interacts with PX4 topics.
 * It serves as a protocol boundary, translating between:
 *   - PX4 NED/FRD conventions  <->  ROS ENU/FLU conventions
 *   - px4_msgs message types    <->  peregrine_interfaces message types
 *   - PX4 best_effort QoS       <->  manager reliable QoS
 *
 * Data flow (PX4 -> managers):
 *   VehicleOdometry (NED/FRD) -> State + Odometry (ENU/FLU)
 *   BatteryStatus             -> BatteryState (ROS standard)
 *   SensorGps                 -> NavSatFix (ROS standard)
 *   VehicleStatus             -> PX4Status (arming/mode/failsafe)
 *
 * Data flow (managers -> PX4):
 *   ControlOutput (ENU)       -> TrajectorySetpoint / RatesSetpoint / etc. (NED)
 *   Arm service               -> VehicleCommand (arm/disarm)
 *   SetMode service           -> VehicleCommand (nav state change)
 *   OffboardControlMode       -> periodic heartbeat (required for offboard mode)
 *
 * This intentionally inherits from rclcpp::Node (not a lifecycle node) because the
 * hardware bridge must remain reachable for safety-critical operations (e.g., disarm,
 * offboard heartbeat) regardless of the lifecycle state of the manager pipeline.
 */

#pragma once

#include <hardware_abstraction/hardware_abstraction_parameters.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <peregrine_interfaces/msg/control_output.hpp>
#include <peregrine_interfaces/msg/frame_anchor.hpp>
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
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <peregrine_interfaces/msg/gps_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <atomic>
#include <cstdint>
#include <cmath>
#include <mutex>
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
 *
 * This intentionally inherits from rclcpp::Node rather than a lifecycle node.
 * Hardware abstraction must always be reachable for safety-critical operations
 * (offboard heartbeat, arm/disarm services). If this were a lifecycle node,
 * the vehicle could not be disarmed while the node was in an inactive state.
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
   * @brief Handles PX4 odometry and publishes manager-facing state (anchored to world).
   */
  void onVehicleOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  /**
   * @brief Consumes VehicleLocalPosition for ref_*, validity, reset counters and deltas.
   *
   * Maintains the map->odom reset offset (INVERSE of the PX4 estimate jump), latches the
   * world->map anchor, captures the ground datum, and publishes FrameAnchor. The raw pose
   * still comes from VehicleOdometry (onVehicleOdometry); this only drives the anchor.
   * See docs/investigations/world-frame-anchoring-plan.md.
   */
  void onVehicleLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
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
  void handleTrajectoryMode(const peregrine_interfaces::msg::ControlOutput & msg);
  void handleBodyRateMode(const peregrine_interfaces::msg::ControlOutput & msg);
  void handleAttitudeMode(const peregrine_interfaces::msg::ControlOutput & msg);
  void handleDirectActuatorMode(const peregrine_interfaces::msg::ControlOutput & msg);

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

  std::shared_ptr<hardware_abstraction::ParamListener> paramListener_;
  hardware_abstraction::Params params_;

  /// PX4 ROS namespace prefix (e.g. "/uav1"), empty for global topics.
  std::string px4Namespace_;
  /// MAV_SYS_ID of the PX4 instance this stack commands (instance N → N+1).
  uint8_t targetSystemId_;
  /// Frame used for published odometry parent (per-UAV, e.g. uav1/odom).
  std::string odomFrame_;
  /// Frame used for published odometry child (per-UAV, e.g. uav1/base_link).
  std::string baseLinkFrame_;
  /// Per-UAV map frame id (e.g. uav1/map).
  std::string mapFrame_;
  /// Shared, fleet-wide world frame id (NOT namespaced). Parent of the anchored State.
  std::string worldFrame_;
  /// Frame used for published GPS messages.
  std::string gpsFrame_;
  /// PX4 SensorGps topic suffix under px4_namespace.
  std::string sensorGpsTopicSuffix_;

  /**
   * @brief Anchor state relating the world frame to PX4's local estimator frame.
   *
   * Stored as scalars (no Eigen in the header). All ENU. Mutated only in
   * onVehicleLocalPosition, read in onVehicleOdometry (State) and handleTrajectoryMode
   * (setpoints) under anchorMutex_. See world-frame-anchoring-plan.md §5.2/§6.2.
   */
  struct FrameAnchorState
  {
    // map->odom translation (ENU) = INVERSE of accumulated PX4 estimate jumps.
    // Pure translation (frames north-aligned). Steps only on a reset-counter change.
    double mapToOdomX{0.0};
    double mapToOdomY{0.0};
    double mapToOdomZ{0.0};
    // map->odom yaw (ENU) = inverse of accumulated heading-reset deltas (orientation only).
    double mapToOdomYaw{0.0};
    // world->map horizontal placement (ENU), latched once from first valid ref_* + datum.
    double worldToMapX{0.0};
    double worldToMapY{0.0};
    // Ground datum: map-frame Z taken as ground (so world z=0 == ground). Tracked while
    // disarmed, frozen at arm.
    double groundDatumZ{0.0};
    bool groundDatumValid{false};
    // Last-seen reset counters (for change detection).
    uint8_t xyResetCounter{0};
    uint8_t zResetCounter{0};
    uint8_t headingResetCounter{0};
    bool countersInit{false};
    // Latched global reference of the EKF origin (world->map source).
    double refLat{0.0};
    double refLon{0.0};
    double refAlt{0.0};
    uint64_t refTimestamp{0};
    bool refValid{false};
  };
  FrameAnchorState anchor_;
  mutable std::mutex anchorMutex_;
  /// True while PX4 reports ARMED; gates ground-datum freezing.
  std::atomic<bool> vehicleArmed_{false};


  // These members are std::atomic because they are written from PX4 subscription callbacks
  // and read from timer callbacks (offboard heartbeat, status publication) and service
  // callbacks (arm, set_mode), all of which may execute on different threads in a
  // multi-threaded executor. Atomics avoid mutex lock contention on the high-frequency
  // telemetry path (~100-250 Hz odometry updates).
  std::atomic<int64_t> lastPx4RxTimeNs_{0};
  std::atomic<uint8_t> navState_{0};
  std::atomic<uint8_t> armingState_{0};
  std::atomic<uint16_t> failureDetectorStatus_{0};
  std::atomic<bool> failsafe_{false};
  std::atomic<float> batteryRemaining_{NAN};
  std::atomic<float> batteryVoltage_{NAN};
  std::atomic<float> motorOutput0_{0.0f};
  std::atomic<float> motorOutput1_{0.0f};
  std::atomic<float> motorOutput2_{0.0f};
  std::atomic<float> motorOutput3_{0.0f};
  /// Packed snapshot of control mode + trajectory flags for coherent offboard heartbeat publication.
  /// See packOffboardModeFlags() in the .cpp for the bit layout.
  std::atomic<uint32_t> offboardModeFlags_{0};

  // ── Local-position validity (mirrors PX4's offboard gate) ──────────────────────────────
  // Written in onVehicleLocalPosition(), read lock-free in publishStatus(). These surface the
  // EKF view that PX4's commander uses to permit OFFBOARD (offboardCheck.cpp/estimatorCheck.cpp),
  // so our pre-arm gate and TUI watch the SAME signal instead of raw GPS quality.
  std::atomic<bool> lposXyValid_{false};
  std::atomic<bool> lposZValid_{false};
  std::atomic<bool> lposVxyValid_{false};
  std::atomic<bool> lposDeadReckoning_{false};
  std::atomic<float> lposEph_{NAN};
  std::atomic<float> lposEvh_{NAN};
  std::atomic<int64_t> lastLposRxNs_{0};
  /// Hysteresis-aware horizontal-position validity from the last VehicleLocalPosition sample
  /// (eph compared against threshold; freshness is applied separately in publishStatus()).
  std::atomic<bool> localPositionValidLatched_{false};
  /// Previous validity, for the accuracy hysteresis band. Mutated only in onVehicleLocalPosition.
  bool lposValidHysteresis_{false};

  /**
   * @brief Mirror of PX4's checkPosVelValidity() accuracy gate (estimatorCheck.cpp).
   *
   * Returns whether horizontal position is usable: the EKF flag is set AND the reported error
   * (eph) is below the configured threshold, widened by 2.5x once already valid to avoid
   * chattering at the boundary (matching PX4's hysteresis). An unknown (non-finite) eph falls
   * back to the raw flag so we don't over-block when the estimator doesn't populate accuracy.
   */
  bool evaluateLocalPositionValidity(bool xyValid, float eph, bool wasValid) const;

  rclcpp::Publisher<peregrine_interfaces::msg::State>::SharedPtr statePub_;
  rclcpp::Publisher<peregrine_interfaces::msg::PX4Status>::SharedPtr statusPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPub_;
  rclcpp::Publisher<peregrine_interfaces::msg::FrameAnchor>::SharedPtr frameAnchorPub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryPub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsPub_;
  rclcpp::Publisher<peregrine_interfaces::msg::GpsStatus>::SharedPtr gpsStatusPub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicleOdometrySub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicleLocalPositionSub_;
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
