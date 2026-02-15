#include <frame_transforms/frame_transformer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace frame_transforms
{
FrameTransformer::FrameTransformer(const rclcpp::NodeOptions& options) : Node("frame_transformer", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing FrameTransformer node...");

  // Parameters
  uav_name_ = this->declare_parameter<std::string>("uav_name", "peregrine_1");
  uav_namespace_ = this->declare_parameter<std::string>("uav_namespace", "peregrine_1/");
  localization_mode_ = this->declare_parameter<std::string>("localization_mode", "gps");

  // Frame namespace
  base_link_frame_ = uav_namespace_ + "/base_link";
  aircraft_frame_ = uav_namespace_ + "/aircraft_frame";
  odom_frame_ = uav_namespace_ + "/odom";
  odom_ned_frame_ = uav_namespace_ + "/odom_ned";
  map_frame_ = uav_namespace_ + "/map";
  world_origin_frame_ = uav_namespace_ + "/world_origin";

  // TF broadcasters
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  // Subscriptions
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      uav_name_ + "/odometry", rclcpp::QoS(10),
      std::bind(&FrameTransformer::odometry_callback, this, std::placeholders::_1));

  if (localization_mode_ == "gps")
  {
    double lat = this->declare_parameter<double>("gps_origin_latitude", 0.0);
    double lon = this->declare_parameter<double>("gps_origin_longitude", 0.0);
    double alt = this->declare_parameter<double>("gps_origin_altitude", 0.0);
    geodetic_converter_.initialiseReference(lat, lon, alt);

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        uav_name_ + "/gnss", rclcpp::QoS(10), std::bind(&FrameTransformer::gps_callback, this, std::placeholders::_1));
  }
  else if (localization_mode_ == "mocap")
  {
    mocap_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        uav_name_ + "/mocap", rclcpp::QoS(10),
        std::bind(&FrameTransformer::mocap_callback, this, std::placeholders::_1));
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid localization_mode parameter. Use 'gps' or 'mocap'.");
    throw std::runtime_error("Invalid localization_mode parameter");
  }

  // static transform publisher
  this->publish_static_transforms();

  // dynamic transform publisher
  tf_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / tf_publish_rate_)),
                                      std::bind(&FrameTransformer::publish_dynamic_transforms, this));
}

void FrameTransformer::publish_static_transforms()
{
  RCLCPP_INFO(this->get_logger(), "Publishing static transforms for UAV: %s", uav_name_.c_str());
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

  auto now = this->get_clock()->now();

  // base_link -> aircraft_frame i.e. ENU to NED conversion
  geometry_msgs::msg::TransformStamped base_to_aircraft;
  base_to_aircraft.header.stamp = now;
  base_to_aircraft.header.frame_id = base_link_frame_;
  base_to_aircraft.child_frame_id = aircraft_frame_;
  base_to_aircraft.transform.translation.x = 0.0;
  base_to_aircraft.transform.translation.y = 0.0;
  base_to_aircraft.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(M_PI / 2, 0.0, M_PI / 2);  // ENU to NED
  base_to_aircraft.transform.rotation = tf2::toMsg(q);
  static_transforms.push_back(base_to_aircraft);

  // odom_ned -> odom i.e. NED to ENU conversion
  geometry_msgs::msg::TransformStamped odom_ned_to_odom;
  odom_ned_to_odom.header.stamp = now;
  odom_ned_to_odom.header.frame_id = odom_ned_frame_;
  odom_ned_to_odom.child_frame_id = odom_frame_;
  odom_ned_to_odom.transform.translation.x = 0.0;
  odom_ned_to_odom.transform.translation.y = 0.0;
  odom_ned_to_odom.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, M_PI / 2);
  odom_ned_to_odom.transform.rotation = tf2::toMsg(q);
  static_transforms.push_back(odom_ned_to_odom);
}
void FrameTransformer::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::scoped_lock lock(data_mutex_);
  latest_odom_ = *msg;
}
void FrameTransformer::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::scoped_lock lock(data_mutex_);
  latest_gps_ = *msg;
}
void FrameTransformer::mocap_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::scoped_lock lock(data_mutex_);
  latest_mocap_ = *msg;
}

void FrameTransformer::publish_dynamic_transforms()
{
  std::scoped_lock lock(data_mutex_);

  if (!latest_odom_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No odometry data received yet.");
    return;
  }
  if (localization_mode_ == "gps" && !latest_gps_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No GPS data received yet.");
    return;
  }
  if (localization_mode_ == "mocap" && !latest_mocap_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No motion capture data received yet.");
    return;
  }
  std::vector<geometry_msgs::msg::TransformStamped> dynamic_transforms;
  auto now = this->get_clock()->now();

  // Transform: odom -> base_link
  // The data that we get from the PX4 odometry is the position of the drone, i.e aircraft in the
  // odom frame.

  geometry_msgs::msg::TransformStamped odom_to_base;
  odom_to_base.header.stamp = now;
  odom_to_base.header.frame_id = odom_ned_frame_;
  odom_to_base.child_frame_id = aircraft_frame_;
  odom_to_base.transform.translation.x = latest_odom_->pose.pose.position.x;
  odom_to_base.transform.translation.y = latest_odom_->pose.pose.position.y;
  odom_to_base.transform.translation.z = latest_odom_->pose.pose.position.z;
  odom_to_base.transform.rotation = latest_odom_->pose.pose.orientation;

  dynamic_transforms.push_back(odom_to_base);

  // Transform: map -> odom
  // Currently Identity. This can be updated later if we implement SLAM pipeline.

  tf2::Transform map_to_odom_tf;
  map_to_odom_tf.setIdentity();
  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = now;
  map_to_odom.header.frame_id = map_frame_;
  map_to_odom.child_frame_id = odom_frame_;
  map_to_odom.transform = tf2::toMsg(map_to_odom_tf);
  dynamic_transforms.push_back(map_to_odom);

  // Transform: world_origin -> map
  if (localization_mode_ == "gps" && latest_gps_)
  {
    if (!gps_origin_set_)
    {
      geodetic_converter_.initialiseReference(latest_gps_->latitude, latest_gps_->longitude, latest_gps_->altitude);
      gps_origin_set_ = true;
      RCLCPP_INFO(this->get_logger(), "GPS origin set to lat: %.6f, lon: %.6f, alt: %.2f", latest_gps_->latitude,
                  latest_gps_->longitude, latest_gps_->altitude);
    }

    double x, y, z;
    geodetic_converter_.geodetic2Enu(latest_gps_->latitude, latest_gps_->longitude, latest_gps_->altitude, &x, &y, &z);

    tf2::Transform world_to_map_tf;
    world_to_map_tf.setOrigin(tf2::Vector3(x, y, z));
    world_to_map_tf.setRotation(tf2::Quaternion::getIdentity());

    geometry_msgs::msg::TransformStamped world_to_map;
    world_to_map.header.stamp = now;
    world_to_map.header.frame_id = world_origin_frame_;
    world_to_map.child_frame_id = map_frame_;
    world_to_map.transform = tf2::toMsg(world_to_map_tf);
    dynamic_transforms.push_back(world_to_map);
  }
  else if (localization_mode_ == "mocap" && latest_mocap_)
  {
    tf2::Transform world_to_map_tf;
    world_to_map_tf.setOrigin(
        tf2::Vector3(latest_mocap_->pose.position.x, latest_mocap_->pose.position.y, latest_mocap_->pose.position.z));
    tf2::Quaternion q;
    tf2::fromMsg(latest_mocap_->pose.orientation, q);
    world_to_map_tf.setRotation(q);

    geometry_msgs::msg::TransformStamped world_to_map;
    world_to_map.header.stamp = now;
    world_to_map.header.frame_id = world_origin_frame_;
    world_to_map.child_frame_id = map_frame_;
    world_to_map.transform = tf2::toMsg(world_to_map_tf);
    dynamic_transforms.push_back(world_to_map);
  }  // namespace frame_transforms
