/**
 * @file flight_visualizer_node.hpp
 * @brief RViz-friendly visualization node for PEREGRINE flight stack data.
 */

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <peregrine_interfaces/msg/safety_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/trajectory_setpoint.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_plugins/rviz_plugins_parameters.hpp>

#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace rviz_plugins
{

/**
 * @class FlightVisualizerNode
 * @brief Publishes RViz-native geometry for state, trajectory, and safety overlays.
 *
 * Subscriptions:
 * - `estimated_state`
 * - `trajectory_setpoint`
 * - `uav_state`
 * - `safety_status`
 *
 * Publications:
 * - `viz/actual_path` (`nav_msgs/Path`)
 * - `viz/reference_path` (`nav_msgs/Path`)
 * - `viz/markers` (`visualization_msgs/MarkerArray`)
 */
class FlightVisualizerNode : public rclcpp::Node
{
public:
  explicit FlightVisualizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);
  void onTrajectorySetpoint(const peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg);
  void onUavState(const peregrine_interfaces::msg::UAVState::SharedPtr msg);
  void onSafetyStatus(const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg);
  void onPublishTimer();

  std::string topicName(const std::string & base_topic) const;
  std::string resolveFrameId(const std::string & frame_id_hint) const;

  void appendPose(
    nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & pose, std::size_t max_points,
    double min_separation_m);
  void addOrDeleteVehicleMarkers(
    std::vector<visualization_msgs::msg::Marker> & markers, const rclcpp::Time & stamp) const;
  void addOrDeleteSetpointMarkers(
    std::vector<visualization_msgs::msg::Marker> & markers, const rclcpp::Time & stamp) const;
  void addSafetyTextMarker(
    std::vector<visualization_msgs::msg::Marker> & markers, const rclcpp::Time & stamp) const;
  void addOrDeleteGeofenceMarker(
    std::vector<visualization_msgs::msg::Marker> & markers, const rclcpp::Time & stamp) const;

  std::shared_ptr<flight_visualizer::ParamListener> paramListener_;
  flight_visualizer::Params params_;

  std::string lastFrameId_;

  mutable std::mutex mutex_;

  std::optional<peregrine_interfaces::msg::UAVState> latestUavState_;
  std::optional<peregrine_interfaces::msg::SafetyStatus> latestSafetyStatus_;
  std::optional<peregrine_interfaces::msg::TrajectorySetpoint> latestTrajectorySetpoint_;

  bool hasEstimatedPose_{false};
  geometry_msgs::msg::PoseStamped latestEstimatedPose_;

  bool hasReferencePose_{false};
  geometry_msgs::msg::PoseStamped latestReferencePose_;

  nav_msgs::msg::Path actualPath_;
  nav_msgs::msg::Path referencePath_;

  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::TrajectorySetpoint>::SharedPtr
    trajectorySetpointSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::UAVState>::SharedPtr uavStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::SafetyStatus>::SharedPtr safetyStatusSub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actualPathPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr referencePathPub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;

  rclcpp::TimerBase::SharedPtr publishTimer_;
};

}  // namespace rviz_plugins

