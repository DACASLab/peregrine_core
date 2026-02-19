/**
 * @file frame_transformer.hpp
 * @brief TF broadcaster component for a single UAV TF subtree.
 */

#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <mutex>
#include <optional>

namespace frame_transforms
{

/**
 * @class FrameTransformer
 * @brief Publishes a local TF tree for one UAV from odometry input.
 *
 * Static transforms:
 * - world -> map
 * - map -> odom
 * - base_link -> base_link_frd
 *
 * Dynamic transform:
 * - odom -> base_link (from latest odometry sample, with configured frame IDs)
 */
class FrameTransformer : public rclcpp::Node
{
public:
  /**
   * @brief Constructs the component and initializes subscriptions/timers.
   */
  explicit FrameTransformer(const rclcpp::NodeOptions& options);

private:
  /**
   * @brief Stores latest odometry sample for TF publication.
   */
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  /**
   * @brief Publishes static transforms once at startup.
   */
  void publishStaticTransforms();
  /**
   * @brief Publishes the dynamic odom -> base_link transform.
   */
  void publishDynamicTransforms();

  /// Optional frame prefix (typically UAV namespace without leading slash).
  std::string framePrefix_;
  /// Topic to read odometry from.
  std::string odometryTopic_;
  /// Global world frame name.
  std::string worldFrame_;
  /// Local map frame name.
  std::string mapFrame_;
  /// Odometry frame name.
  std::string odomFrame_;
  /// Vehicle body frame in FLU convention.
  std::string baseLinkFrame_;
  /// Vehicle body frame in FRD convention.
  std::string baseLinkFrdFrame_;
  /// Dynamic TF publication frequency.
  double publishRateHz_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;
  rclcpp::TimerBase::SharedPtr dynamicTfTimer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster_;

  std::mutex odometryMutex_;
  std::optional<nav_msgs::msg::Odometry> latestOdometry_;
};

}  // namespace frame_transforms
