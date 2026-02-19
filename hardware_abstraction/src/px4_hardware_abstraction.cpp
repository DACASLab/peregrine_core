#include <hardware_abstraction/msg_version.hpp>
#include <hardware_abstraction/px4_hardware_abstraction.hpp>

#include <frame_transforms/conversions.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>

namespace hardware_abstraction
{
namespace
{

// Canonicalizes topic namespace to either "" or "/prefix" (no trailing slash).
std::string normalizeNamespace(const std::string& ns)
{
  if (ns.empty())
  {
    return "";
  }

  std::string out = ns;
  if (out.front() != '/')
  {
    out.insert(out.begin(), '/');
  }
  while (!out.empty() && out.back() == '/')
  {
    out.pop_back();
  }
  return out;
}

std::string normalizeTopicSuffix(std::string suffix)
{
  if (suffix.empty())
  {
    return "";
  }
  if (suffix.front() != '/')
  {
    suffix.insert(suffix.begin(), '/');
  }
  return suffix;
}

// Lowercase helper for case-insensitive mode string handling.
std::string toLower(std::string input)
{
  std::transform(input.begin(), input.end(), input.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return input;
}

constexpr uint32_t kModeMask = 0xFFu;
constexpr uint32_t kTrajectoryPositionBit = 1u << 8;
constexpr uint32_t kTrajectoryVelocityBit = 1u << 9;
constexpr uint32_t kTrajectoryAccelerationBit = 1u << 10;

uint32_t packOffboardModeFlags(uint8_t controlMode, bool usePosition, bool useVelocity, bool useAcceleration)
{
  uint32_t flags = static_cast<uint32_t>(controlMode) & kModeMask;
  if (usePosition)
  {
    flags |= kTrajectoryPositionBit;
  }
  if (useVelocity)
  {
    flags |= kTrajectoryVelocityBit;
  }
  if (useAcceleration)
  {
    flags |= kTrajectoryAccelerationBit;
  }
  return flags;
}

uint8_t unpackControlMode(const uint32_t flags)
{
  return static_cast<uint8_t>(flags & kModeMask);
}

bool hasTrajectoryPosition(const uint32_t flags)
{
  return (flags & kTrajectoryPositionBit) != 0u;
}

bool hasTrajectoryVelocity(const uint32_t flags)
{
  return (flags & kTrajectoryVelocityBit) != 0u;
}

bool hasTrajectoryAcceleration(const uint32_t flags)
{
  return (flags & kTrajectoryAccelerationBit) != 0u;
}

void writeLinearCovariance(const Eigen::Matrix3d& covariance, std::array<double, 36>& target)
{
  target[0] = covariance(0, 0);
  target[1] = covariance(0, 1);
  target[2] = covariance(0, 2);
  target[6] = covariance(1, 0);
  target[7] = covariance(1, 1);
  target[8] = covariance(1, 2);
  target[12] = covariance(2, 0);
  target[13] = covariance(2, 1);
  target[14] = covariance(2, 2);
}

}  // namespace

PX4HardwareAbstraction::PX4HardwareAbstraction(const rclcpp::NodeOptions& options)
: Node("px4_hardware_abstraction", options)
{
  // Names, rates, and command IDs are parameterized for single and multi-UAV deployments.
  px4Namespace_ = normalizeNamespace(this->declare_parameter<std::string>("px4_namespace", ""));
  odomFrame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  baseLinkFrame_ = this->declare_parameter<std::string>("base_link_frame", "base_link");
  sensorGpsTopicSuffix_ =
      normalizeTopicSuffix(this->declare_parameter<std::string>("sensor_gps_topic_suffix", "/fmu/out/sensor_gps"));

  offboardRateHz_ = this->declare_parameter<double>("offboard_rate_hz", 20.0);
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 5.0);
  connectionTimeoutS_ = this->declare_parameter<double>("connection_timeout_s", 1.0);

  targetSystemId_ = this->declare_parameter<int>("target_system_id", 1);
  targetComponentId_ = this->declare_parameter<int>("target_component_id", 1);
  sourceSystemId_ = this->declare_parameter<int>("source_system_id", 1);
  sourceComponentId_ = this->declare_parameter<int>("source_component_id", 1);

  if (offboardRateHz_ <= 0.0 || statusRateHz_ <= 0.0 || connectionTimeoutS_ <= 0.0)
  {
    throw std::runtime_error("offboard_rate_hz, status_rate_hz and connection_timeout_s must all be > 0");
  }

  // PX4 links generally prefer best-effort QoS, while manager APIs use reliable QoS.
  const auto rosOutputQos = rclcpp::QoS(10).reliable();
  const auto rosInputQos = rclcpp::QoS(20).reliable();
  const auto px4OutputQos = rclcpp::QoS(1).best_effort();
  const auto px4InputQos = rclcpp::QoS(1).best_effort();

  // Manager-facing outputs.
  statePub_ = this->create_publisher<peregrine_interfaces::msg::State>("state", rosOutputQos);
  statusPub_ = this->create_publisher<peregrine_interfaces::msg::PX4Status>("status", rosOutputQos);
  odometryPub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", rosOutputQos);
  batteryPub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery", rosOutputQos);
  gpsPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gnss", rosOutputQos);

  // PX4 telemetry subscriptions.
  vehicleOdometrySub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      px4Topic("/fmu/out/vehicle_odometry" + getMessageNameVersion<px4_msgs::msg::VehicleOdometry>()), px4InputQos,
      std::bind(&PX4HardwareAbstraction::onVehicleOdometry, this, std::placeholders::_1));
  batteryStatusSub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
      px4Topic("/fmu/out/battery_status" + getMessageNameVersion<px4_msgs::msg::BatteryStatus>()), px4InputQos,
      std::bind(&PX4HardwareAbstraction::onBatteryStatus, this, std::placeholders::_1));
  sensorGpsSub_ = this->create_subscription<px4_msgs::msg::SensorGps>(
      px4Topic(sensorGpsTopicSuffix_ + getMessageNameVersion<px4_msgs::msg::SensorGps>()), px4InputQos,
      std::bind(&PX4HardwareAbstraction::onSensorGps, this, std::placeholders::_1));
  vehicleStatusSub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      px4Topic("/fmu/out/vehicle_status" + getMessageNameVersion<px4_msgs::msg::VehicleStatus>()), px4InputQos,
      std::bind(&PX4HardwareAbstraction::onVehicleStatus, this, std::placeholders::_1));

  // Manager command input.
  controlOutputSub_ = this->create_subscription<peregrine_interfaces::msg::ControlOutput>(
      "control_output", rosInputQos, std::bind(&PX4HardwareAbstraction::onControlOutput, this, std::placeholders::_1));

  // PX4 command/setpoint publishers.
  offboardControlModePub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      px4Topic("/fmu/in/offboard_control_mode" + getMessageNameVersion<px4_msgs::msg::OffboardControlMode>()),
      px4OutputQos);
  trajectorySetpointPub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      px4Topic("/fmu/in/trajectory_setpoint" + getMessageNameVersion<px4_msgs::msg::TrajectorySetpoint>()),
      px4OutputQos);
  vehicleRatesSetpointPub_ = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      px4Topic("/fmu/in/vehicle_rates_setpoint" + getMessageNameVersion<px4_msgs::msg::VehicleRatesSetpoint>()),
      px4OutputQos);
  vehicleAttitudeSetpointPub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      px4Topic("/fmu/in/vehicle_attitude_setpoint" + getMessageNameVersion<px4_msgs::msg::VehicleAttitudeSetpoint>()),
      px4OutputQos);
  actuatorMotorsPub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(
      px4Topic("/fmu/in/actuator_motors" + getMessageNameVersion<px4_msgs::msg::ActuatorMotors>()), px4OutputQos);
  vehicleCommandPub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      px4Topic("/fmu/in/vehicle_command" + getMessageNameVersion<px4_msgs::msg::VehicleCommand>()), px4OutputQos);

  // Service interface for manager/state-machine orchestration.
  armService_ = this->create_service<peregrine_interfaces::srv::Arm>(
      "arm", std::bind(&PX4HardwareAbstraction::onArmService, this, std::placeholders::_1, std::placeholders::_2));
  setModeService_ = this->create_service<peregrine_interfaces::srv::SetMode>(
      "set_mode",
      std::bind(&PX4HardwareAbstraction::onSetModeService, this, std::placeholders::_1, std::placeholders::_2));

  // Timers for offboard heartbeat and status publication.
  offboardModeTimer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / offboardRateHz_)),
      std::bind(&PX4HardwareAbstraction::publishOffboardControlMode, this));
  statusTimer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / statusRateHz_)),
      std::bind(&PX4HardwareAbstraction::publishStatus, this));

  offboardModeFlags_.store(packOffboardModeFlags(peregrine_interfaces::msg::ControlOutput::MODE_TRAJECTORY, true, false,
                                                 false));
  lastPx4RxTimeNs_.store(this->now().nanoseconds());

  RCLCPP_INFO(this->get_logger(), "px4_hardware_abstraction started. px4_namespace='%s', sensor_gps_topic='%s'",
              px4Namespace_.c_str(), px4Topic(sensorGpsTopicSuffix_).c_str());
}

void PX4HardwareAbstraction::onVehicleOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  lastPx4RxTimeNs_.store(this->now().nanoseconds());

  if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Unsupported pose_frame=%u in VehicleOdometry. Only POSE_FRAME_NED is supported.", msg->pose_frame);
    return;
  }

  // PX4 orientation is world NED <- body FRD (wxyz). Guard against invalid quaternions.
  Eigen::Quaterniond qNedFrd(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  if (!std::isfinite(qNedFrd.w()) || !std::isfinite(qNedFrd.x()) || !std::isfinite(qNedFrd.y()) ||
      !std::isfinite(qNedFrd.z()) || qNedFrd.norm() < 1e-6)
  {
    qNedFrd = Eigen::Quaterniond::Identity();
  }
  qNedFrd.normalize();

  // Convert world and body-frame quantities from PX4 conventions to ROS conventions.
  const Eigen::Vector3d positionEnu = frame_transforms::nedToEnu(
      Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]));
  const Eigen::Quaterniond qEnuFlu = frame_transforms::orientationNedFrdToEnuFlu(qNedFrd);

  // VehicleOdometry velocity can be in multiple frames; normalize all supported cases into body FLU.
  Eigen::Vector3d velocityBodyFrd = Eigen::Vector3d::Zero();
  Eigen::Matrix3d velocityCovarianceFlu = Eigen::Matrix3d::Zero();
  const Eigen::Matrix3d rotationFluFrd = frame_transforms::frdToFluMatrix();
  switch (msg->velocity_frame)
  {
    case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD:
    {
      velocityBodyFrd = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
      const Eigen::Matrix3d velocityCovarianceFrd =
          Eigen::Vector3d(msg->velocity_variance[0], msg->velocity_variance[1], msg->velocity_variance[2]).asDiagonal();
      velocityCovarianceFlu = rotationFluFrd * velocityCovarianceFrd * rotationFluFrd.transpose();
      break;
    }

    case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED:
    {
      const Eigen::Vector3d velocityNed(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
      const Eigen::Matrix3d rotationFrdNed = qNedFrd.toRotationMatrix().transpose();
      velocityBodyFrd = rotationFrdNed * velocityNed;

      const Eigen::Matrix3d velocityCovarianceNed =
          Eigen::Vector3d(msg->velocity_variance[0], msg->velocity_variance[1], msg->velocity_variance[2]).asDiagonal();
      const Eigen::Matrix3d rotationFluNed = rotationFluFrd * rotationFrdNed;
      velocityCovarianceFlu = rotationFluNed * velocityCovarianceNed * rotationFluNed.transpose();
      break;
    }

    case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD:
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Unsupported velocity_frame=VELOCITY_FRAME_FRD. Cannot infer body-frame velocity without additional heading reference.");
      return;

    default:
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Unsupported velocity_frame=%u in VehicleOdometry.", msg->velocity_frame);
      return;
  }
  const Eigen::Vector3d velocityBodyFlu = frame_transforms::frdToFlu(velocityBodyFrd);
  const Eigen::Vector3d angularRatesFlu = frame_transforms::frdToFlu(
      Eigen::Vector3d(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]));

  // Publish a ROS-standard odometry view for TF and visualization consumers.
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = this->now();
  odometry.header.frame_id = odomFrame_;
  odometry.child_frame_id = baseLinkFrame_;

  odometry.pose.pose.position.x = positionEnu.x();
  odometry.pose.pose.position.y = positionEnu.y();
  odometry.pose.pose.position.z = positionEnu.z();
  odometry.pose.pose.orientation = frame_transforms::toRosQuaternion(qEnuFlu);

  odometry.twist.twist.linear.x = velocityBodyFlu.x();
  odometry.twist.twist.linear.y = velocityBodyFlu.y();
  odometry.twist.twist.linear.z = velocityBodyFlu.z();

  odometry.twist.twist.angular.x = angularRatesFlu.x();
  odometry.twist.twist.angular.y = angularRatesFlu.y();
  odometry.twist.twist.angular.z = angularRatesFlu.z();

  // Populate covariance from PX4 variance fields in the ROS output frames.
  odometry.pose.covariance.fill(0.0);
  odometry.twist.covariance.fill(0.0);

  odometry.pose.covariance[0] = msg->position_variance[1];
  odometry.pose.covariance[7] = msg->position_variance[0];
  odometry.pose.covariance[14] = msg->position_variance[2];
  odometry.pose.covariance[21] = msg->orientation_variance[0];
  odometry.pose.covariance[28] = msg->orientation_variance[1];
  odometry.pose.covariance[35] = msg->orientation_variance[2];

  writeLinearCovariance(velocityCovarianceFlu, odometry.twist.covariance);

  odometryPub_->publish(odometry);

  // Publish manager-facing state message used by estimator/controller managers.
  peregrine_interfaces::msg::State state;
  state.header = odometry.header;
  state.pose = odometry.pose;
  state.twist = odometry.twist;
  state.linear_acceleration.x = 0.0;
  state.linear_acceleration.y = 0.0;
  state.linear_acceleration.z = 0.0;
  state.source = "px4_vehicle_odometry";

  if (msg->quality > 0)
  {
    state.confidence = std::clamp(static_cast<float>(msg->quality) / 100.0f, 0.0f, 1.0f);
  }
  else
  {
    state.confidence = 1.0f;
  }

  statePub_->publish(state);
}

void PX4HardwareAbstraction::onBatteryStatus(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
  lastPx4RxTimeNs_.store(this->now().nanoseconds());

  batteryRemaining_.store(msg->remaining);
  batteryVoltage_.store(msg->voltage_v);

  // Keep a standard ROS battery view in parallel to manager status output.
  sensor_msgs::msg::BatteryState battery;
  battery.header.stamp = this->now();
  battery.header.frame_id = baseLinkFrame_;
  battery.voltage = msg->voltage_v;
  battery.current = msg->current_a;
  battery.percentage = msg->remaining;
  battery.temperature = msg->temperature;
  battery.cell_voltage.clear();
  const std::size_t boundedCellCount =
      msg->cell_count > 0 ? std::min<std::size_t>(msg->cell_count, msg->voltage_cell_v.size()) : msg->voltage_cell_v.size();
  for (std::size_t i = 0; i < boundedCellCount; ++i)
  {
    const float voltage = msg->voltage_cell_v[i];
    if (std::isfinite(voltage) && voltage > 0.0f)
    {
      battery.cell_voltage.push_back(voltage);
    }
  }

  batteryPub_->publish(battery);
}

void PX4HardwareAbstraction::onSensorGps(const px4_msgs::msg::SensorGps::SharedPtr msg)
{
  lastPx4RxTimeNs_.store(this->now().nanoseconds());

  sensor_msgs::msg::NavSatFix gps;
  gps.header.stamp = this->now();
  gps.header.frame_id = "gps";

  gps.latitude = msg->latitude_deg;
  gps.longitude = msg->longitude_deg;
  gps.altitude = msg->altitude_ellipsoid_m;

  gps.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  switch (msg->fix_type)
  {
    case px4_msgs::msg::SensorGps::FIX_TYPE_2D:
    case px4_msgs::msg::SensorGps::FIX_TYPE_3D:
    case px4_msgs::msg::SensorGps::FIX_TYPE_EXTRAPOLATED:
      gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;

    case px4_msgs::msg::SensorGps::FIX_TYPE_RTCM_CODE_DIFFERENTIAL:
      gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      break;

    case px4_msgs::msg::SensorGps::FIX_TYPE_RTK_FLOAT:
    case px4_msgs::msg::SensorGps::FIX_TYPE_RTK_FIXED:
      gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;

    case 0:
    case px4_msgs::msg::SensorGps::FIX_TYPE_NONE:
    default:
      gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
  }

  // Approximate covariance from eph/epv for downstream localization consumers.
  const double eph2 = std::pow(msg->eph, 2.0);
  const double epv2 = std::pow(msg->epv, 2.0);
  gps.position_covariance = {eph2, 0.0, 0.0, 0.0, eph2, 0.0, 0.0, 0.0, epv2};
  gps.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

  gpsPub_->publish(gps);
}

void PX4HardwareAbstraction::onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  lastPx4RxTimeNs_.store(this->now().nanoseconds());

  navState_.store(msg->nav_state);
  armingState_.store(msg->arming_state);
  failureDetectorStatus_.store(msg->failure_detector_status);
  failsafe_.store(msg->failsafe);

  publishStatus();
}

void PX4HardwareAbstraction::onControlOutput(const peregrine_interfaces::msg::ControlOutput::SharedPtr msg)
{
  offboardModeFlags_.store(
      packOffboardModeFlags(msg->control_mode, msg->use_position, msg->use_velocity, msg->use_acceleration),
      std::memory_order_release);

  // The control mode determines which PX4 input topic is actively driven.
  switch (msg->control_mode)
  {
    case peregrine_interfaces::msg::ControlOutput::MODE_TRAJECTORY:
    {
      // NaN means "do not control this axis/field" for PX4 setpoints.
      const float nan = std::numeric_limits<float>::quiet_NaN();
      px4_msgs::msg::TrajectorySetpoint setpoint;
      setpoint.timestamp = nowMicros();

      for (std::size_t i = 0; i < setpoint.position.size(); ++i)
      {
        setpoint.position[i] = nan;
        setpoint.velocity[i] = nan;
        setpoint.acceleration[i] = nan;
        setpoint.jerk[i] = nan;
      }
      setpoint.yaw = nan;
      setpoint.yawspeed = nan;

      if (msg->use_position)
      {
        // Trajectory fields are expected in ENU from managers; convert to NED for PX4.
        const Eigen::Vector3d ned =
            frame_transforms::enuToNed(Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z));
        setpoint.position[0] = static_cast<float>(ned.x());
        setpoint.position[1] = static_cast<float>(ned.y());
        setpoint.position[2] = static_cast<float>(ned.z());
      }

      if (msg->use_velocity)
      {
        const Eigen::Vector3d ned =
            frame_transforms::enuToNed(Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z));
        setpoint.velocity[0] = static_cast<float>(ned.x());
        setpoint.velocity[1] = static_cast<float>(ned.y());
        setpoint.velocity[2] = static_cast<float>(ned.z());
      }

      if (msg->use_acceleration)
      {
        const Eigen::Vector3d ned =
            frame_transforms::enuToNed(Eigen::Vector3d(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z));
        setpoint.acceleration[0] = static_cast<float>(ned.x());
        setpoint.acceleration[1] = static_cast<float>(ned.y());
        setpoint.acceleration[2] = static_cast<float>(ned.z());
      }

      if (msg->use_yaw)
      {
        setpoint.yaw = static_cast<float>(frame_transforms::yawEnuToNed(msg->yaw));
      }
      if (msg->use_yaw_rate)
      {
        setpoint.yawspeed = static_cast<float>(frame_transforms::yawRateEnuToNed(msg->yaw_rate));
      }

      trajectorySetpointPub_->publish(setpoint);
      break;
    }

    case peregrine_interfaces::msg::ControlOutput::MODE_BODY_RATE:
    {
      px4_msgs::msg::VehicleRatesSetpoint rates;
      rates.timestamp = nowMicros();

      const Eigen::Vector3d ratesFrd = frame_transforms::fluToFrd(
          Eigen::Vector3d(msg->body_rates.x, msg->body_rates.y, msg->body_rates.z));
      rates.roll = static_cast<float>(ratesFrd.x());
      rates.pitch = static_cast<float>(ratesFrd.y());
      rates.yaw = static_cast<float>(ratesFrd.z());

      // PX4 thrust convention for multicopters is negative Z in FRD.
      const float thrust = std::clamp(msg->thrust, 0.0f, 1.0f);
      rates.thrust_body[0] = 0.0f;
      rates.thrust_body[1] = 0.0f;
      rates.thrust_body[2] = -thrust;
      rates.reset_integral = false;

      vehicleRatesSetpointPub_->publish(rates);
      break;
    }

    case peregrine_interfaces::msg::ControlOutput::MODE_ATTITUDE:
    {
      px4_msgs::msg::VehicleAttitudeSetpoint attitude;
      attitude.timestamp = nowMicros();

      Eigen::Quaterniond qEnuFlu(msg->attitude.w, msg->attitude.x, msg->attitude.y, msg->attitude.z);
      if (qEnuFlu.norm() < 1e-6)
      {
        qEnuFlu = Eigen::Quaterniond::Identity();
      }
      qEnuFlu.normalize();

      const Eigen::Quaterniond qNedFrd = frame_transforms::orientationEnuFluToNedFrd(qEnuFlu);
      attitude.q_d[0] = static_cast<float>(qNedFrd.w());
      attitude.q_d[1] = static_cast<float>(qNedFrd.x());
      attitude.q_d[2] = static_cast<float>(qNedFrd.y());
      attitude.q_d[3] = static_cast<float>(qNedFrd.z());
      attitude.yaw_sp_move_rate = static_cast<float>(frame_transforms::yawRateEnuToNed(msg->yaw_rate));

      // PX4 uses FRD thrust with -Z as "upward thrust" for multicopters.
      const float thrust = std::clamp(msg->thrust, 0.0f, 1.0f);
      attitude.thrust_body[0] = 0.0f;
      attitude.thrust_body[1] = 0.0f;
      attitude.thrust_body[2] = -thrust;

      vehicleAttitudeSetpointPub_->publish(attitude);
      break;
    }

    case peregrine_interfaces::msg::ControlOutput::MODE_DIRECT_ACTUATOR:
    {
      px4_msgs::msg::ActuatorMotors actuators;
      actuators.timestamp = nowMicros();
      actuators.timestamp_sample = actuators.timestamp;
      actuators.reversible_flags = 0;

      // Unused channels remain NaN so PX4 can treat them as disarmed/ignored.
      const float nan = std::numeric_limits<float>::quiet_NaN();
      for (auto& control : actuators.control)
      {
        control = nan;
      }

      const std::size_t count = std::min(msg->motor_commands.size(), actuators.control.size());
      for (std::size_t i = 0; i < count; ++i)
      {
        actuators.control[i] = std::clamp(msg->motor_commands[i], -1.0f, 1.0f);
      }

      actuatorMotorsPub_->publish(actuators);
      break;
    }

    default:
      RCLCPP_WARN(this->get_logger(), "Unsupported control_mode: %u", msg->control_mode);
      break;
  }
}

void PX4HardwareAbstraction::onArmService(const std::shared_ptr<peregrine_interfaces::srv::Arm::Request> request,
                                          std::shared_ptr<peregrine_interfaces::srv::Arm::Response> response)
{
  if (!isConnected())
  {
    response->success = false;
    response->message = "PX4 is not connected.";
    return;
  }

  // Manager requests are translated into MAVLink-compatible VehicleCommand fields.
  const float armValue = request->arm ? 1.0f : 0.0f;
  vehicleCommandPub_->publish(
      makeVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, armValue, 0.0f));

  response->success = true;
  response->message = request->arm ? "Arm command sent." : "Disarm command sent.";
}

void PX4HardwareAbstraction::onSetModeService(const std::shared_ptr<peregrine_interfaces::srv::SetMode::Request> request,
                                              std::shared_ptr<peregrine_interfaces::srv::SetMode::Response> response)
{
  if (!isConnected())
  {
    response->success = false;
    response->message = "PX4 is not connected.";
    return;
  }

  // We intentionally map user-level mode names to nav_state enums in one place.
  bool valid = false;
  const uint8_t navState = navStateFromModeString(request->mode, &valid);
  if (!valid)
  {
    response->success = false;
    response->message = "Unknown mode. Valid: manual, altitude, position, offboard, loiter, mission, land";
    return;
  }

  vehicleCommandPub_->publish(
      makeVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE, static_cast<float>(navState), 0.0f));

  response->success = true;
  response->message = "Mode command sent: " + request->mode;
}

void PX4HardwareAbstraction::publishOffboardControlMode()
{
  px4_msgs::msg::OffboardControlMode mode;
  mode.timestamp = nowMicros();

  const uint32_t flags = offboardModeFlags_.load(std::memory_order_acquire);
  const auto activeMode = unpackControlMode(flags);
  const bool trajectoryMode = activeMode == peregrine_interfaces::msg::ControlOutput::MODE_TRAJECTORY;

  // Keep offboard heartbeat coherent with whichever manager output mode is active.
  mode.position = trajectoryMode && hasTrajectoryPosition(flags);
  mode.velocity = trajectoryMode && hasTrajectoryVelocity(flags);
  mode.acceleration = trajectoryMode && hasTrajectoryAcceleration(flags);
  mode.attitude = activeMode == peregrine_interfaces::msg::ControlOutput::MODE_ATTITUDE;
  mode.body_rate = activeMode == peregrine_interfaces::msg::ControlOutput::MODE_BODY_RATE;
  mode.thrust_and_torque = false;
  mode.direct_actuator = activeMode == peregrine_interfaces::msg::ControlOutput::MODE_DIRECT_ACTUATOR;

  offboardControlModePub_->publish(mode);
}

void PX4HardwareAbstraction::publishStatus()
{
  // Consolidated status topic supports manager FSM decisions and watchdogs.
  peregrine_interfaces::msg::PX4Status status;
  status.header.stamp = this->now();
  status.connected = isConnected();
  status.armed = armingState_.load() == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  status.offboard = navState_.load() == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  status.failsafe = failsafe_.load();
  status.nav_state = navState_.load();
  status.arming_state = armingState_.load();
  status.failure_detector_status = failureDetectorStatus_.load();
  status.battery_remaining = batteryRemaining_.load();
  status.battery_voltage = batteryVoltage_.load();

  statusPub_->publish(status);
}

px4_msgs::msg::VehicleCommand PX4HardwareAbstraction::makeVehicleCommand(uint32_t command, float param1, float param2) const
{
  // Centralized command construction ensures consistent system/component IDs.
  px4_msgs::msg::VehicleCommand vehicleCommand;
  vehicleCommand.timestamp = nowMicros();
  vehicleCommand.command = command;
  vehicleCommand.param1 = param1;
  vehicleCommand.param2 = param2;
  vehicleCommand.target_system = static_cast<uint8_t>(targetSystemId_);
  vehicleCommand.target_component = static_cast<uint8_t>(targetComponentId_);
  vehicleCommand.source_system = static_cast<uint8_t>(sourceSystemId_);
  vehicleCommand.source_component = static_cast<uint16_t>(sourceComponentId_);
  vehicleCommand.confirmation = 0;
  vehicleCommand.from_external = true;
  return vehicleCommand;
}

uint8_t PX4HardwareAbstraction::navStateFromModeString(const std::string& mode, bool* valid) const
{
  // Keep mode aliases compact and explicit to avoid ambiguous string handling.
  const std::string normalized = toLower(mode);

  *valid = true;
  if (normalized == "manual")
  {
    return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL;
  }
  if (normalized == "altitude" || normalized == "altctl")
  {
    return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL;
  }
  if (normalized == "position" || normalized == "posctl")
  {
    return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL;
  }
  if (normalized == "offboard")
  {
    return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }
  if (normalized == "loiter" || normalized == "hold")
  {
    return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;
  }
  if (normalized == "mission")
  {
    return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION;
  }
  if (normalized == "land")
  {
    return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
  }

  *valid = false;
  return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL;
}

uint64_t PX4HardwareAbstraction::nowMicros() const
{
  return static_cast<uint64_t>(this->now().nanoseconds() / 1000LL);
}

bool PX4HardwareAbstraction::isConnected() const
{
  const double dt = (static_cast<double>(this->now().nanoseconds() - lastPx4RxTimeNs_.load())) * 1e-9;
  return dt <= connectionTimeoutS_;
}

std::string PX4HardwareAbstraction::px4Topic(const std::string& suffix) const
{
  return px4Namespace_ + suffix;
}

}  // namespace hardware_abstraction

RCLCPP_COMPONENTS_REGISTER_NODE(hardware_abstraction::PX4HardwareAbstraction)
