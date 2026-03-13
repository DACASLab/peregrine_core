/**
 * @file mocap_relay.hpp
 * @brief Composable relay that splits a NamedPoseArray into per-UAV PoseStamped topics.
 *
 * Motion capture systems (Vicon, OptiTrack) typically publish all tracked rigid bodies
 * in a single NamedPoseArray message. This node filters for one specific body name and
 * republishes its pose as a standard geometry_msgs/PoseStamped, which hardware_abstraction
 * consumes to feed PX4's EKF2 as visual odometry.
 *
 * For multi-UAV setups, each UAV's component container loads its own MocapRelay instance
 * with a different mocap_body_name parameter.
 */

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace peregrine_bringup
{

class MocapRelay : public rclcpp::Node
{
public:
  explicit MocapRelay(const rclcpp::NodeOptions & options);

private:
  void onNamedPoseArray(
    const motion_capture_tracking_interfaces::msg::NamedPoseArray::SharedPtr msg);

  /// Rigid body name to filter from the array.
  std::string bodyName_;

  rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

}  // namespace peregrine_bringup
