/**
 * @file frame_transformer.hpp
 * @brief TF broadcaster component for a single UAV TF subtree.
 *
 * This component bridges the gap between hardware_abstraction's odometry output and
 * the ROS2 TF tree that visualization tools (RViz) and other consumers rely on.
 *
 * Unlike the lifecycle managers, this is a plain rclcpp::Node because TF frames should
 * be available as long as hardware_abstraction is publishing odometry, regardless of
 * the lifecycle state of the manager pipeline.
 *
 * TF tree structure published by this node:
 *
 *   world ─(static identity)─> map ─(static identity)─> odom ─(dynamic)─> base_link
 *                                                                             │
 *                                                                    (static FLU->FRD)
 *                                                                             │
 *                                                                       base_link_frd
 *
 * The world->map and map->odom transforms are identity by default. They exist as
 * placeholders so that higher-level localization packages (e.g., SLAM, GPS-based
 * global alignment) can later take ownership of these transforms without restructuring
 * the TF tree.
 *
 * The base_link->base_link_frd static transform allows nodes that need PX4's FRD body
 * frame (e.g., for sensor data expressed in FRD) to look it up via TF.
 *
 * All frame names are parameterized and support an optional prefix for multi-UAV
 * deployments (e.g., "uav1/base_link").
 */

#pragma once

#include <Eigen/Core>
#include <frame_transforms/frame_transforms_parameters.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <peregrine_interfaces/msg/frame_anchor.hpp>
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
 * Static transforms (published once at startup via latched topic):
 * - world -> map       (identity; placeholder for global localization)
 * - map -> odom        (identity; placeholder for drift correction)
 * - base_link -> base_link_frd  (180-degree rotation about X axis: FLU to FRD)
 *
 * Dynamic transform (published at configurable rate):
 * - odom -> base_link  (from latest odometry sample)
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
   * @brief Stores the latest FrameAnchor and latches world->map on first valid sample.
   */
  void onFrameAnchor(const peregrine_interfaces::msg::FrameAnchor::SharedPtr msg);
  /**
   * @brief Publishes the static base_link -> base_link_frd transform once at startup.
   */
  void publishStaticTransforms();
  /**
   * @brief Publishes the dynamic TF edges: world->map, map->odom, odom->base_link.
   */
  void publishDynamicTransforms();

  std::shared_ptr<frame_transforms::ParamListener> paramListener_;
  frame_transforms::Params params_;

  /// Optional frame prefix (typically UAV namespace without leading slash).
  std::string framePrefix_;
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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;
  rclcpp::Subscription<peregrine_interfaces::msg::FrameAnchor>::SharedPtr frameAnchorSub_;
  rclcpp::TimerBase::SharedPtr dynamicTfTimer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster_;

  std::mutex odometryMutex_;
  std::optional<nav_msgs::msg::Odometry> latestOdometry_;

  // Anchor state (from FrameAnchor). map->odom is dynamic (steps on PX4 reset); world->map is
  // latched once from the first valid ref_* + world_datum and from the ground datum (z).
  std::mutex anchorMutex_;
  std::optional<peregrine_interfaces::msg::FrameAnchor> latestAnchor_;
  bool worldToMapLatched_{false};
  Eigen::Vector3d worldToMapTranslation_{0.0, 0.0, 0.0};
};

}  // namespace frame_transforms
