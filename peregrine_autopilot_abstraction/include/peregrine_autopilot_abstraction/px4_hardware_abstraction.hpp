#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <peregrine_interfaces/msg/px4_actuator_cmd.hpp>
#include <peregrine_interfaces/msg/px4_trajectory_setpoint.hpp>

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <peregrine_autopilot_abstraction/msg_version.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace px4_hardware_abstraction
{

class PX4APIManager : public rclcpp::Node
{
public:
  PX4APIManager(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  std::atomic<bool> initialized_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr cbkgrps_timers_;
  rclcpp::CallbackGroup::SharedPtr cbkgrps_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrps_services_;

  // Timers and initialization
  rclcpp::TimerBase::SharedPtr timer_init;
  void timerInit();
  void shutdown();

  std::string _uav_name_;
  std::string _uav_namespace_;

  // Options
  rclcpp::SubscriptionOptions sub_opts;
  rclcpp::PublisherOptions pub_opts;

  // --------------- Data out to ROS2 ---------------

  // Odometry
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_position_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_velocity_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_orientation_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_angulate_velocity_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _px4_sub_odometry_;
  void callback_px4_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  // GPS
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr _px4_sub_gps_;
  void callback_px4_gps(const px4_msgs::msg::SensorGps::SharedPtr msg);

  // Battery
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr _px4_sub_battery_;
  void callback_px4_battery(const px4_msgs::msg::BatteryStatus::SharedPtr msg);

  // --------------- Data in from ROS2 ---------------

  // Trajectory setpoint
  rclcpp::Subscription<peregrine_interfaces::msg::PX4TrajectorySetpoint>::SharedPtr sub_trajectory_setpoint_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _px4_pub_trajectory_setpoint_;
  void callback_trajectory_setpoint(const peregrine_interfaces::msg::PX4TrajectorySetpoint::SharedPtr msg);

  // Direct actuator control
  rclcpp::Subscription<peregrine_interfaces::msg::PX4ActuatorCmd>::SharedPtr sub_actuator_cmd_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr _px4_pub_actuator_motors_;
  void callback_actuator_cmd(const peregrine_interfaces::msg::PX4ActuatorCmd::SharedPtr msg);

  // Srvs
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_arm_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_takeoff_;

  // Topics names
  // The topics that are going to and from ROS2 will be namespaced under the UAV
  // namespace
  std::string battery_tpc_ = _uav_namespace_ + "/battery";
  std::string trajectory_setpoint_tpc_ = _uav_namespace_ + "/trajectory_setpoint";
  std::string actuator_cmd_tpc_ = _uav_namespace_ + "/actuator_cmd";
  std::string odometry_tpc_ = _uav_namespace_ + "/odometry";
  std::string gps_tpc_ = _uav_namespace_ + "/gnss";
  std::string mocap_tpc_ = _uav_namespace_ + "/mocap";
  std::string position_tpc_ = _uav_namespace_ + "/position";
  std::string velocity_tpc_ = _uav_namespace_ + "/velocity";
  std::string orientation_tpc_ = _uav_namespace_ + "/orientation";
  std::string angulate_velocity_tpc_ = _uav_namespace_ + "/angulate_velocity";
  std::string arm_srv_ = _uav_namespace_ + "/arm";
  std::string takeoff_srv_ = _uav_namespace_ + "/takeoff";
  std::string offboard_mode_srv_ = _uav_namespace_ + "/offboard_mode";

  // The topics that are going to and from PX4 will be namespaced under the
  // _uav_namespace_/fmu etc.
  std::string _px4_odometry_tpc_ =
      _uav_namespace_ + "/fmu/out/vehicle_odometry" + getMessageNameVersion<px4_msgs::msg::VehicleOdometry>();
  std::string _px4_battery_tpc_ =
      _uav_namespace_ + "/fmu/out/battery_status" + getMessageNameVersion<px4_msgs::msg::BatteryStatus>();
  std::string _px4_trajectory_setpoint_tpc_ =
      _uav_namespace_ + "/fmu/in/trajectory_setpoint" + getMessageNameVersion<px4_msgs::msg::TrajectorySetpoint>();
  std::string _px4_actuator_motors_tpc_ =
      _uav_namespace_ + "/fmu/in/actuator_motors" + getMessageNameVersion<px4_msgs::msg::ActuatorMotors>();
  std::string _px4_gps_tpc_ =
      _uav_namespace_ + "/fmu/out/vehicle_gps_position" + getMessageNameVersion<px4_msgs::msg::SensorGps>();
};
}  // namespace px4_hardware_abstraction