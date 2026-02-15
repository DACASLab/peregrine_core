#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <frame_transforms/geodetic_conv.hpp>
#include <frame_transforms/px4_transforms.hpp>

#include <mutex>
#include <optional>

namespace frame_transforms
{

/**
 * @class FrameTransformer
 * @brief Publishes a complete, self-contained, inverted TF tree for a single UAV.
 *
 * It makes the UAV's
 * base_link frame the root of its own TF tree. All other frames, including
 * odom, map, and world_origin, are published as children of base_link by
 * calculating and publishing the inverse transforms.
 */
class FrameTransformer : public rclcpp::Node
{
public:
  explicit FrameTransformer(const rclcpp::NodeOptions& options);

private:
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void mocap_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publish_dynamic_transforms();

  // Helper to invert a transform
  tf2::Transform get_inverse_transform(const tf2::Transform& transform);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  int tf_publish_rate_ = 100;  // Hz
  void publish_static_transforms();

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_sub_;

  // Frame names
  std::string uav_name_;
  std::string uav_namespace_;
  std::string base_link_frame_;
  std::string aircraft_frame_;
  std::string odom_frame_;
  std::string odom_ned_frame_;
  std::string map_frame_;
  std::string world_origin_frame_;

  // Data storage with mutex for thread safety
  std::mutex data_mutex_;
  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  std::optional<sensor_msgs::msg::NavSatFix> latest_gps_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_mocap_;

  // Geodetic converter for GPS mode
  GeodeticConverter geodetic_converter_;
  bool gps_origin_set_ = false;
  std::string localization_mode_;
};

}  // namespace frame_transforms
