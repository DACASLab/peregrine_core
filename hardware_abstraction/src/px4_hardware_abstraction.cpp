#include <hardware_abstraction/msg_version.hpp>
#include <hardware_abstraction/px4_hardware_abstraction.hpp>
namespace px4_hardware_abstraction
{

PX4APIManager::PX4APIManager(rclcpp::NodeOptions options)
  : Node("px4_hardware_abstraction", options), initialized_(false)
{
  RCLCPP_INFO(this->get_logger(), "Starting PX4 Hardware Abstraction Node...");
  timer_init = this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&PX4APIManager::timerInit, this),
                                       cbkgrps_timers_);
}

void PX4APIManager::timerInit()
{
  RCLCPP_INFO(this->get_logger(), "PX4 Hardware Abstraction Node Initializing...");
  node_ = this->shared_from_this();
  clock_ = this->get_clock();

  cbkgrps_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrps_subs_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrps_services_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::on_shutdown([this]() { this->shutdown(); });

  // Get parameters
  _uav_name_ = this->declare_parameter<std::string>("uav_name", "peregrine_1");
  _uav_namespace_ = this->declare_parameter<std::string>("uav_namespace", "/peregrine_1");
  RCLCPP_INFO(this->get_logger(), "UAV Name: %s", _uav_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "UAV Namespace: %s", _uav_namespace_.c_str());

  // --------------- Data out to ROS2 ---------------
  // Odometry
  pub_odometry_ = node_->create_publisher<nav_msgs::msg::Odometry>(odometry_tpc_, rclcpp::QoS(10).reliable());
  pub_position_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(position_tpc_, rclcpp::QoS(10).reliable());
  pub_velocity_ =
      node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(velocity_tpc_, rclcpp::QoS(10).reliable());
  pub_orientation_ =
      node_->create_publisher<geometry_msgs::msg::QuaternionStamped>(orientation_tpc_, rclcpp::QoS(10).reliable());
  pub_angulate_velocity_ =
      node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(angulate_velocity_tpc_, rclcpp::QoS(10).reliable());

  sub_opts.callback_group = cbkgrps_subs_;
  sub_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  _px4_sub_odometry_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
      _px4_odometry_tpc_, rclcpp::QoS(1).best_effort(),
      std::bind(&PX4APIManager::callback_px4_odometry, this, std::placeholders::_1), sub_opts);

  // Battery
  pub_battery_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(battery_tpc_, rclcpp::QoS(10).reliable());
  _px4_sub_battery_ = node_->create_subscription<px4_msgs::msg::BatteryStatus>(
      _px4_battery_tpc_, rclcpp::QoS(1).best_effort(),
      std::bind(&PX4APIManager::callback_px4_battery, this, std::placeholders::_1), sub_opts);

  // GPS
  pub_gps_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(gps_tpc_, rclcpp::QoS(10).reliable());
  _px4_sub_gps_ = node_->create_subscription<px4_msgs::msg::SensorGps>(
      _px4_gps_tpc_, rclcpp::QoS(1).best_effort(),
      std::bind(&PX4APIManager::callback_px4_gps, this, std::placeholders::_1), sub_opts);

  // --------------- Data in from ROS2 ---------------
  // Trajectory setpoint
  sub_trajectory_setpoint_ = node_->create_subscription<peregrine_interfaces::msg ::PX4TrajectorySetpoint>(
      trajectory_setpoint_tpc_, rclcpp::QoS(1).best_effort(),
      std::bind(&PX4APIManager::callback_trajectory_setpoint, this, std::placeholders::_1), sub_opts);
  _px4_pub_trajectory_setpoint_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      _px4_trajectory_setpoint_tpc_, rclcpp::QoS(1).best_effort());

  // Actuator motors
  sub_actuator_cmd_ = node_->create_subscription<peregrine_interfaces::msg::PX4ActuatorCmd>(
      actuator_cmd_tpc_, rclcpp::QoS(1).best_effort(),
      std::bind(&PX4APIManager::callback_actuator_cmd, this, std::placeholders::_1), sub_opts);
  _px4_pub_actuator_motors_ =
      node_->create_publisher<px4_msgs::msg::ActuatorMotors>(_px4_actuator_motors_tpc_, rclcpp::QoS(1).best_effort());

  initialized_ = true;
  timer_init->cancel();
  RCLCPP_INFO(this->get_logger(), "PX4 Hardware Abstraction Node Initialized.");
}

void PX4APIManager::shutdown()
{
  RCLCPP_INFO(this->get_logger(), "PX4 Hardware Abstraction Node shutting down...");

  initialized_ = false;
}

void PX4APIManager::callback_px4_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  // TODO: Implement odometry conversion and publishing
  (void)msg;
}

/** Convert px4_msgs::msg::BatteryStatus to sensor_msgs::msg::BatteryState and publish. */
void PX4APIManager::callback_px4_battery(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
  sensor_msgs::msg::BatteryState battery_msg;

  battery_msg.header.stamp = node_->get_clock()->now();
  battery_msg.header.frame_id = "battery";  // TODO: see if we need to change frame_id to something like base_link

  battery_msg.voltage = msg->voltage_v;
  battery_msg.current = msg->current_a;

  // TODO: Add more fields as necessary

  // Publish
  pub_battery_->publish(battery_msg);
}

void PX4APIManager::callback_trajectory_setpoint(const peregrine_interfaces::msg::PX4TrajectorySetpoint::SharedPtr msg)
{
  // TODO: Implement trajectory setpoint conversion and publishing to PX4
  (void)msg;
}

/**
 * @brief Convert PX4ActuatorCmd to px4_msgs::msg::ActuatorMotors for a quadrotor.
 *
 * Uses the first four motor values, sets timestamp = 0 and reversible_flags = 0.
 * Remaining controls are set to std::numeric_limits<float>::quiet_NaN().
 *
 * @param msg Incoming PX4ActuatorCmd shared pointer.
 */
void PX4APIManager::callback_actuator_cmd(const peregrine_interfaces::msg::PX4ActuatorCmd::SharedPtr msg)
{
  // Assume quadrotor for now. Thus, only the first four floats from PX4ActuatorCmd are used.

  px4_msgs::msg::ActuatorMotors actuator_msg;
  actuator_msg.timestamp = 0;  // Let the ROS2-uXRCE handle timestamping

  // Assume no motor is reversible
  actuator_msg.reversible_flags = 0;

  // Copy up to first 4 motor commands, set the rest to quite NaN
  for (int i = 0; i < actuator_msg.NUM_CONTROLS; i++)
  {
    if (i < 4)
    {
      actuator_msg.control[i] = msg->motors[i];
    }
    else
    {
      actuator_msg.control[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }
}

/**
 * @brief Convert PX4 SensorGps to sensor_msgs::msg::NavSatFix and publish.
 *
 * Sets header, frame_id ("gps"), status.service = SERVICE_GPS,
 * copies latitude, longitude, altitude, and uses diagonal covariance
 * {eph^2, eph^2, epv^2} (approximate).
 *
 * @param msg Incoming px4_msgs::msg::SensorGps message.
 */
void PX4APIManager::callback_px4_gps(const px4_msgs::msg::SensorGps::SharedPtr msg)
{
  sensor_msgs::msg::NavSatFix gps_msg;

  gps_msg.header.stamp = node_->get_clock()->now();
  gps_msg.header.frame_id = "gps";  // TODO: see if we need to change frame_id to something like world

  // Assume GPS service
  gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  // Position
  gps_msg.latitude = msg->latitude_deg;
  gps_msg.longitude = msg->longitude_deg;
  gps_msg.altitude = msg->altitude_ellipsoid_m;  // WGS84

  // Covariance approximated
  double eph2 = std::pow(msg->eph, 2);
  double epv2 = std::pow(msg->epv, 2);
  gps_msg.position_covariance = { eph2, 0.0, 0.0, 0.0, eph2, 0.0, 0.0, 0.0, epv2 };
  gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

  // Publish
  pub_gps_->publish(gps_msg);
}
}  // namespace px4_hardware_abstraction

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(px4_hardware_abstraction::PX4APIManager)