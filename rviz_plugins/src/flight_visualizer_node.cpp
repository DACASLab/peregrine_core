/**
 * @file flight_visualizer_node.cpp
 * @brief RViz visualization node implementation for flight geometry and safety overlays.
 */

#include <rviz_plugins/flight_visualizer_node.hpp>

#include <frame_transforms/viz_colormap.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace rviz_plugins
{
using namespace std::chrono_literals;

namespace
{

constexpr char kNodeName[] = "flight_visualizer";
constexpr char kNsVehicle[] = "vehicle";
constexpr char kNsReference[] = "reference";
constexpr char kNsSafety[] = "safety";
constexpr char kNsGeofence[] = "geofence";

constexpr int kIdVehicleBody = 0;
constexpr int kIdVehicleHeading = 1;
constexpr int kIdReferencePoint = 10;
constexpr int kIdReferenceVelocity = 11;
constexpr int kIdSafetyText = 20;
constexpr int kIdGeofenceVolume = 30;
constexpr int kIdGeofenceText = 31;

constexpr double kMinSetpointVelocityMps = 1e-3;
constexpr float kAlphaOpaque = 1.0F;

inline bool hasZeroStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

inline double distance3d(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

std_msgs::msg::ColorRGBA makeColor(const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

// Parses the trailing integer of a UAV namespace ("/uav2" -> 2). Returns 0 for root/empty.
int uavIndexFromNamespace(const std::string & ns)
{
  std::string digits;
  for (auto it = ns.rbegin(); it != ns.rend(); ++it) {
    if (std::isdigit(static_cast<unsigned char>(*it))) {
      digits.insert(digits.begin(), *it);
    } else if (!digits.empty()) {
      break;
    }
  }
  return digits.empty() ? 0 : std::stoi(digits);
}

std_msgs::msg::ColorRGBA uavColor(const int uav_index, const float alpha)
{
  const auto rgb = frame_transforms::viz::colorForUav(uav_index);
  return makeColor(rgb[0], rgb[1], rgb[2], alpha);
}

std_msgs::msg::ColorRGBA safetyLevelColor(const uint8_t level)
{
  if (level >= peregrine_interfaces::msg::SafetyStatus::LEVEL_CRITICAL) {
    return makeColor(0.95F, 0.20F, 0.20F, kAlphaOpaque);
  }
  if (level == peregrine_interfaces::msg::SafetyStatus::LEVEL_WARNING) {
    return makeColor(0.95F, 0.75F, 0.15F, kAlphaOpaque);
  }
  return makeColor(0.10F, 0.85F, 0.35F, kAlphaOpaque);
}

visualization_msgs::msg::Marker makeDeleteMarker(
  const std::string & frame_id, const rclcpp::Time & stamp,
  const std::string & ns, const int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  return marker;
}

visualization_msgs::msg::Marker makeBaseMarker(
  const std::string & frame_id, const rclcpp::Time & stamp,
  const std::string & ns, const int id, const int type)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(0, 0);
  return marker;
}

}  // namespace

FlightVisualizerNode::FlightVisualizerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node(kNodeName, options)
{
  paramListener_ = std::make_shared<flight_visualizer::ParamListener>(
    this->get_node_parameters_interface());
  params_ = paramListener_->get_params();
  lastFrameId_ = params_.fixed_frame;
  uavColorIndex_ = uavIndexFromNamespace(params_.uav_namespace);

  // Reliable QoS keeps visualization streams stable across teleop and debugging sessions.
  const auto streamQos = rclcpp::QoS(20).reliable();
  const auto vizQos = rclcpp::QoS(10).reliable();

  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    topicName("estimated_state"), streamQos,
    [this](peregrine_interfaces::msg::State::SharedPtr msg) { onEstimatedState(msg); });
  trajectorySetpointSub_ = this->create_subscription<peregrine_interfaces::msg::TrajectorySetpoint>(
    topicName("trajectory_setpoint"), streamQos,
    [this](peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg) { onTrajectorySetpoint(msg); });
  uavStateSub_ = this->create_subscription<peregrine_interfaces::msg::UAVState>(
    topicName("uav_state"), rclcpp::QoS(10).reliable(),
    [this](peregrine_interfaces::msg::UAVState::SharedPtr msg) { onUavState(msg); });
  safetyStatusSub_ = this->create_subscription<peregrine_interfaces::msg::SafetyStatus>(
    topicName("safety_status"), rclcpp::QoS(10).reliable(),
    [this](peregrine_interfaces::msg::SafetyStatus::SharedPtr msg) { onSafetyStatus(msg); });

  actualPathPub_ = this->create_publisher<nav_msgs::msg::Path>(topicName(params_.actual_path_topic), vizQos);
  referencePathPub_ = this->create_publisher<nav_msgs::msg::Path>(topicName(params_.reference_path_topic), vizQos);
  markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topicName(params_.marker_topic), vizQos);

  const auto period = std::chrono::duration<double>(1.0 / params_.publish_rate_hz);
  publishTimer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { onPublishTimer(); });

  actualPath_.header.frame_id = params_.fixed_frame;
  referencePath_.header.frame_id = params_.fixed_frame;

  RCLCPP_INFO(
    this->get_logger(),
    "flight_visualizer started (ns='%s', fixed_frame='%s', rate=%.1fHz)",
    params_.uav_namespace.c_str(), params_.fixed_frame.c_str(), params_.publish_rate_hz);
}

std::string FlightVisualizerNode::topicName(const std::string & base_topic) const
{
  if (params_.uav_namespace.empty() || params_.uav_namespace == "/") {
    return base_topic;
  }

  std::string ns = params_.uav_namespace;
  if (ns.front() != '/') {
    ns = "/" + ns;
  }
  if (ns.back() == '/') {
    ns.pop_back();
  }
  return ns + "/" + base_topic;
}

std::string FlightVisualizerNode::resolveFrameId(const std::string & frame_id_hint) const
{
  if (!frame_id_hint.empty()) {
    return frame_id_hint;
  }
  if (!lastFrameId_.empty()) {
    return lastFrameId_;
  }
  return params_.fixed_frame;
}

void FlightVisualizerNode::appendPose(
  nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & pose, const std::size_t max_points,
  const double min_separation_m)
{
  if (path.header.frame_id.empty()) {
    path.header.frame_id = pose.header.frame_id;
  }

  if (path.header.frame_id != pose.header.frame_id) {
    path.poses.clear();
    path.header.frame_id = pose.header.frame_id;
  }

  if (!path.poses.empty()) {
    const auto & last_pose = path.poses.back();
    if (distance3d(last_pose.pose.position, pose.pose.position) < min_separation_m) {
      // Skip near-duplicates, but keep the last COMMITTED point as the spacing reference.
      // (Replacing it instead would let the reference drift by sub-threshold steps every
      // sample, so cumulative motion never crosses min_separation and the trail never grows
      // -- which silently flatlines the path to one point at high state rates.)
      return;
    }
  }

  if (path.poses.size() >= max_points) {
    const std::size_t overflow = (path.poses.size() - max_points) + 1;
    path.poses.erase(path.poses.begin(), path.poses.begin() + static_cast<std::ptrdiff_t>(overflow));
  }

  path.poses.push_back(pose);
}

void FlightVisualizerNode::onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose;
  if (hasZeroStamp(msg->header.stamp)) {
    pose.header.stamp = this->now();
  } else {
    pose.header.stamp = msg->header.stamp;
  }
  pose.header.frame_id = msg->header.frame_id;
  pose.pose = msg->pose.pose;

  std::scoped_lock lock(mutex_);
  pose.header.frame_id = resolveFrameId(pose.header.frame_id);
  lastFrameId_ = pose.header.frame_id;
  latestEstimatedPose_ = pose;
  hasEstimatedPose_ = true;
  appendPose(actualPath_, pose, static_cast<std::size_t>(params_.max_actual_path_points), params_.path_min_separation_m);
}

void FlightVisualizerNode::onTrajectorySetpoint(
  const peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose;
  if (hasZeroStamp(msg->header.stamp)) {
    pose.header.stamp = this->now();
  } else {
    pose.header.stamp = msg->header.stamp;
  }
  pose.pose.position = msg->position;

  const double half_yaw = 0.5 * msg->yaw;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(half_yaw);
  pose.pose.orientation.w = std::cos(half_yaw);

  std::scoped_lock lock(mutex_);
  pose.header.frame_id = resolveFrameId(msg->header.frame_id);
  lastFrameId_ = pose.header.frame_id;
  latestTrajectorySetpoint_ = *msg;
  latestReferencePose_ = pose;
  hasReferencePose_ = true;
  appendPose(
    referencePath_, pose, static_cast<std::size_t>(params_.max_reference_path_points), params_.path_min_separation_m);
}

void FlightVisualizerNode::onUavState(const peregrine_interfaces::msg::UAVState::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  const bool wasArmed = latestUavState_.has_value() ? latestUavState_->armed : false;
  latestUavState_ = *msg;

  if (params_.clear_reference_path_on_disarm && wasArmed && !msg->armed) {
    referencePath_.poses.clear();
    hasReferencePose_ = false;
  }
}

void FlightVisualizerNode::onSafetyStatus(const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestSafetyStatus_ = *msg;
}

void FlightVisualizerNode::addOrDeleteVehicleMarkers(
  std::vector<visualization_msgs::msg::Marker> & markers, const rclcpp::Time & stamp) const
{
  if (!hasEstimatedPose_) {
    markers.push_back(makeDeleteMarker(params_.fixed_frame, stamp, kNsVehicle, kIdVehicleBody));
    markers.push_back(makeDeleteMarker(params_.fixed_frame, stamp, kNsVehicle, kIdVehicleHeading));
    return;
  }

  // Body color encodes UAV identity (per-UAV colormap), so multiple quads are
  // distinguishable at a glance. Safety overrides it whenever status is not nominal, so a
  // warning/critical vehicle still flips to amber/red regardless of its identity color.
  const bool safetyOverride = latestSafetyStatus_.has_value() &&
    latestSafetyStatus_->level != peregrine_interfaces::msg::SafetyStatus::LEVEL_NOMINAL;
  const auto color = safetyOverride
    ? safetyLevelColor(latestSafetyStatus_->level)
    : uavColor(uavColorIndex_, kAlphaOpaque);

  auto body = makeBaseMarker(
    latestEstimatedPose_.header.frame_id, stamp, kNsVehicle, kIdVehicleBody,
    visualization_msgs::msg::Marker::SPHERE);
  body.pose = latestEstimatedPose_.pose;
  body.scale.x = 0.35;
  body.scale.y = 0.35;
  body.scale.z = 0.18;
  body.color = color;
  markers.push_back(std::move(body));

  auto heading = makeBaseMarker(
    latestEstimatedPose_.header.frame_id, stamp, kNsVehicle, kIdVehicleHeading,
    visualization_msgs::msg::Marker::ARROW);
  heading.pose = latestEstimatedPose_.pose;
  heading.scale.x = 0.80;
  heading.scale.y = 0.08;
  heading.scale.z = 0.08;
  heading.color = uavColor(uavColorIndex_, kAlphaOpaque);
  markers.push_back(std::move(heading));
}

void FlightVisualizerNode::addOrDeleteSetpointMarkers(
  std::vector<visualization_msgs::msg::Marker> & markers, const rclcpp::Time & stamp) const
{
  if (!hasReferencePose_) {
    markers.push_back(makeDeleteMarker(params_.fixed_frame, stamp, kNsReference, kIdReferencePoint));
    markers.push_back(makeDeleteMarker(params_.fixed_frame, stamp, kNsReference, kIdReferenceVelocity));
    return;
  }

  auto setpoint = makeBaseMarker(
    latestReferencePose_.header.frame_id, stamp, kNsReference, kIdReferencePoint,
    visualization_msgs::msg::Marker::SPHERE);
  setpoint.pose = latestReferencePose_.pose;
  setpoint.scale.x = 0.22;
  setpoint.scale.y = 0.22;
  setpoint.scale.z = 0.22;
  setpoint.color = makeColor(0.95F, 0.35F, 0.05F, kAlphaOpaque);
  markers.push_back(std::move(setpoint));

  if (
    !params_.show_setpoint_velocity || !latestTrajectorySetpoint_.has_value() ||
    !latestTrajectorySetpoint_->use_velocity)
  {
    markers.push_back(makeDeleteMarker(
      latestReferencePose_.header.frame_id, stamp, kNsReference, kIdReferenceVelocity));
    return;
  }

  const auto & velocity = latestTrajectorySetpoint_->velocity;
  const double speed = std::sqrt(
    (velocity.x * velocity.x) + (velocity.y * velocity.y) + (velocity.z * velocity.z));
  if (speed < kMinSetpointVelocityMps || params_.setpoint_velocity_scale <= 0.0) {
    markers.push_back(makeDeleteMarker(
      latestReferencePose_.header.frame_id, stamp, kNsReference, kIdReferenceVelocity));
    return;
  }

  auto velocity_arrow = makeBaseMarker(
    latestReferencePose_.header.frame_id, stamp, kNsReference, kIdReferenceVelocity,
    visualization_msgs::msg::Marker::ARROW);
  velocity_arrow.scale.x = 0.05;
  velocity_arrow.scale.y = 0.10;
  velocity_arrow.scale.z = 0.14;
  velocity_arrow.color = makeColor(0.95F, 0.55F, 0.05F, kAlphaOpaque);

  geometry_msgs::msg::Point start = latestReferencePose_.pose.position;
  geometry_msgs::msg::Point end = start;
  end.x += velocity.x * params_.setpoint_velocity_scale;
  end.y += velocity.y * params_.setpoint_velocity_scale;
  end.z += velocity.z * params_.setpoint_velocity_scale;
  velocity_arrow.points.push_back(start);
  velocity_arrow.points.push_back(end);
  markers.push_back(std::move(velocity_arrow));
}

void FlightVisualizerNode::addOrDeleteGeofenceMarker(
  std::vector<visualization_msgs::msg::Marker> & markers, const rclcpp::Time & stamp) const
{
  if (!params_.show_geofence) {
    markers.push_back(makeDeleteMarker(params_.fixed_frame, stamp, kNsGeofence, kIdGeofenceVolume));
    markers.push_back(makeDeleteMarker(params_.fixed_frame, stamp, kNsGeofence, kIdGeofenceText));
    return;
  }

  const double min_alt = std::min(params_.geofence_min_altitude_m, params_.geofence_max_altitude_m);
  const double max_alt = std::max(params_.geofence_min_altitude_m, params_.geofence_max_altitude_m);
  const double height = std::max(0.1, max_alt - min_alt);

  auto volume = makeBaseMarker(
    params_.fixed_frame, stamp, kNsGeofence, kIdGeofenceVolume,
    visualization_msgs::msg::Marker::CYLINDER);
  volume.pose.orientation.w = 1.0;
  volume.pose.position.x = 0.0;
  volume.pose.position.y = 0.0;
  volume.pose.position.z = min_alt + (0.5 * height);
  volume.scale.x = 2.0 * params_.geofence_radius_m;
  volume.scale.y = 2.0 * params_.geofence_radius_m;
  volume.scale.z = height;
  volume.color = makeColor(0.20F, 0.55F, 0.95F, 0.12F);
  markers.push_back(std::move(volume));

  auto label = makeBaseMarker(
    params_.fixed_frame, stamp, kNsGeofence, kIdGeofenceText,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  label.pose.orientation.w = 1.0;
  label.pose.position.x = 0.0;
  label.pose.position.y = 0.0;
  label.pose.position.z = max_alt + 0.8;
  label.scale.z = 0.30;
  label.color = makeColor(0.20F, 0.65F, 1.0F, 0.9F);

  std::ostringstream ss;
  ss << "geofence R=" << params_.geofence_radius_m << "m Z=[" << min_alt << ", " << max_alt << "]m";
  label.text = ss.str();
  markers.push_back(std::move(label));
}

void FlightVisualizerNode::onPublishTimer()
{
  const auto now = this->now();

  nav_msgs::msg::Path actual_path;
  nav_msgs::msg::Path reference_path;
  visualization_msgs::msg::MarkerArray marker_array;

  {
    std::scoped_lock lock(mutex_);
    actual_path = actualPath_;
    reference_path = referencePath_;

    if (actual_path.header.frame_id.empty()) {
      actual_path.header.frame_id = params_.fixed_frame;
    }
    if (reference_path.header.frame_id.empty()) {
      reference_path.header.frame_id = params_.fixed_frame;
    }

    addOrDeleteVehicleMarkers(marker_array.markers, now);
    addOrDeleteSetpointMarkers(marker_array.markers, now);
    // The floating per-vehicle status text (safety level / flight mode / armed) was removed;
    // keep deleting any previously-published instance so it clears in late-joining RViz too.
    marker_array.markers.push_back(makeDeleteMarker(params_.fixed_frame, now, kNsSafety, kIdSafetyText));
    addOrDeleteGeofenceMarker(marker_array.markers, now);
  }

  actual_path.header.stamp = now;
  reference_path.header.stamp = now;

  actualPathPub_->publish(actual_path);
  referencePathPub_->publish(reference_path);
  markerPub_->publish(marker_array);
}

}  // namespace rviz_plugins

RCLCPP_COMPONENTS_REGISTER_NODE(rviz_plugins::FlightVisualizerNode)
