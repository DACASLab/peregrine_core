/**
 * @file flight_visualizer_node.cpp
 * @brief RViz visualization node implementation for flight geometry and safety overlays.
 */

#include <rviz_plugins/flight_visualizer_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace rviz_plugins
{
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

std::string uavStateToString(const uint8_t state)
{
  switch (state) {
    case peregrine_interfaces::msg::UAVState::STATE_IDLE:
      return "IDLE";
    case peregrine_interfaces::msg::UAVState::STATE_ARMED:
      return "ARMED";
    case peregrine_interfaces::msg::UAVState::STATE_TAKING_OFF:
      return "TAKING_OFF";
    case peregrine_interfaces::msg::UAVState::STATE_HOVERING:
      return "HOVERING";
    case peregrine_interfaces::msg::UAVState::STATE_FLYING:
      return "FLYING";
    case peregrine_interfaces::msg::UAVState::STATE_LANDING:
      return "LANDING";
    case peregrine_interfaces::msg::UAVState::STATE_LANDED:
      return "LANDED";
    case peregrine_interfaces::msg::UAVState::STATE_EMERGENCY:
      return "EMERGENCY";
    default:
      return "UNKNOWN";
  }
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
  uavNamespace_ = this->declare_parameter<std::string>("uav_namespace", "");
  fixedFrame_ = this->declare_parameter<std::string>("fixed_frame", "map");
  lastFrameId_ = fixedFrame_;

  publishRateHz_ = this->declare_parameter<double>("publish_rate_hz", 15.0);
  maxActualPathPoints_ = this->declare_parameter<int>("max_actual_path_points", 2000);
  maxReferencePathPoints_ = this->declare_parameter<int>("max_reference_path_points", 2000);
  pathMinSeparationM_ = this->declare_parameter<double>("path_min_separation_m", 0.05);

  showGeofence_ = this->declare_parameter<bool>("show_geofence", true);
  geofenceRadiusM_ = this->declare_parameter<double>("geofence_radius_m", 500.0);
  geofenceMinAltitudeM_ = this->declare_parameter<double>("geofence_min_altitude_m", -5.0);
  geofenceMaxAltitudeM_ = this->declare_parameter<double>("geofence_max_altitude_m", 120.0);

  showSetpointVelocity_ = this->declare_parameter<bool>("show_setpoint_velocity", true);
  setpointVelocityScale_ = this->declare_parameter<double>("setpoint_velocity_scale", 0.5);
  clearReferencePathOnDisarm_ = this->declare_parameter<bool>("clear_reference_path_on_disarm", true);

  const std::string actualPathTopic = this->declare_parameter<std::string>(
    "actual_path_topic", "viz/actual_path");
  const std::string referencePathTopic = this->declare_parameter<std::string>(
    "reference_path_topic", "viz/reference_path");
  const std::string markerTopic = this->declare_parameter<std::string>(
    "marker_topic", "viz/markers");

  if (publishRateHz_ <= 0.0) {
    throw std::runtime_error("publish_rate_hz must be > 0");
  }
  maxActualPathPoints_ = std::max(10, maxActualPathPoints_);
  maxReferencePathPoints_ = std::max(10, maxReferencePathPoints_);
  pathMinSeparationM_ = std::max(0.0, pathMinSeparationM_);
  geofenceRadiusM_ = std::max(0.1, geofenceRadiusM_);
  setpointVelocityScale_ = std::max(0.0, setpointVelocityScale_);

  // Reliable QoS keeps visualization streams stable across teleop and debugging sessions.
  const auto streamQos = rclcpp::QoS(20).reliable();
  const auto vizQos = rclcpp::QoS(10).reliable();

  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    topicName("estimated_state"), streamQos,
    std::bind(&FlightVisualizerNode::onEstimatedState, this, std::placeholders::_1));
  trajectorySetpointSub_ = this->create_subscription<peregrine_interfaces::msg::TrajectorySetpoint>(
    topicName("trajectory_setpoint"), streamQos,
    std::bind(&FlightVisualizerNode::onTrajectorySetpoint, this, std::placeholders::_1));
  uavStateSub_ = this->create_subscription<peregrine_interfaces::msg::UAVState>(
    topicName("uav_state"), rclcpp::QoS(10).reliable(),
    std::bind(&FlightVisualizerNode::onUavState, this, std::placeholders::_1));
  safetyStatusSub_ = this->create_subscription<peregrine_interfaces::msg::SafetyStatus>(
    topicName("safety_status"), rclcpp::QoS(10).reliable(),
    std::bind(&FlightVisualizerNode::onSafetyStatus, this, std::placeholders::_1));

  actualPathPub_ = this->create_publisher<nav_msgs::msg::Path>(topicName(actualPathTopic), vizQos);
  referencePathPub_ = this->create_publisher<nav_msgs::msg::Path>(topicName(referencePathTopic), vizQos);
  markerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topicName(markerTopic), vizQos);

  const auto period = std::chrono::duration<double>(1.0 / publishRateHz_);
  publishTimer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&FlightVisualizerNode::onPublishTimer, this));

  actualPath_.header.frame_id = fixedFrame_;
  referencePath_.header.frame_id = fixedFrame_;

  RCLCPP_INFO(
    this->get_logger(),
    "flight_visualizer started (ns='%s', fixed_frame='%s', rate=%.1fHz)",
    uavNamespace_.c_str(), fixedFrame_.c_str(), publishRateHz_);
}

std::string FlightVisualizerNode::topicName(const std::string & base_topic) const
{
  if (uavNamespace_.empty() || uavNamespace_ == "/") {
    return base_topic;
  }

  std::string ns = uavNamespace_;
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
  return fixedFrame_;
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
      path.poses.back() = pose;
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
  appendPose(actualPath_, pose, static_cast<std::size_t>(maxActualPathPoints_), pathMinSeparationM_);
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
    referencePath_, pose, static_cast<std::size_t>(maxReferencePathPoints_), pathMinSeparationM_);
}

void FlightVisualizerNode::onUavState(const peregrine_interfaces::msg::UAVState::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  const bool wasArmed = latestUavState_.has_value() ? latestUavState_->armed : false;
  latestUavState_ = *msg;

  if (clearReferencePathOnDisarm_ && wasArmed && !msg->armed) {
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
    markers.push_back(makeDeleteMarker(fixedFrame_, stamp, kNsVehicle, kIdVehicleBody));
    markers.push_back(makeDeleteMarker(fixedFrame_, stamp, kNsVehicle, kIdVehicleHeading));
    return;
  }

  const auto color = latestSafetyStatus_.has_value()
    ? safetyLevelColor(latestSafetyStatus_->level)
    : makeColor(0.10F, 0.75F, 0.95F, kAlphaOpaque);

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
  heading.color = makeColor(0.10F, 0.80F, 0.95F, kAlphaOpaque);
  markers.push_back(std::move(heading));
}

void FlightVisualizerNode::addOrDeleteSetpointMarkers(
  std::vector<visualization_msgs::msg::Marker> & markers, const rclcpp::Time & stamp) const
{
  if (!hasReferencePose_) {
    markers.push_back(makeDeleteMarker(fixedFrame_, stamp, kNsReference, kIdReferencePoint));
    markers.push_back(makeDeleteMarker(fixedFrame_, stamp, kNsReference, kIdReferenceVelocity));
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
    !showSetpointVelocity_ || !latestTrajectorySetpoint_.has_value() ||
    !latestTrajectorySetpoint_->use_velocity)
  {
    markers.push_back(makeDeleteMarker(
      latestReferencePose_.header.frame_id, stamp, kNsReference, kIdReferenceVelocity));
    return;
  }

  const auto & velocity = latestTrajectorySetpoint_->velocity;
  const double speed = std::sqrt(
    (velocity.x * velocity.x) + (velocity.y * velocity.y) + (velocity.z * velocity.z));
  if (speed < kMinSetpointVelocityMps || setpointVelocityScale_ <= 0.0) {
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
  end.x += velocity.x * setpointVelocityScale_;
  end.y += velocity.y * setpointVelocityScale_;
  end.z += velocity.z * setpointVelocityScale_;
  velocity_arrow.points.push_back(start);
  velocity_arrow.points.push_back(end);
  markers.push_back(std::move(velocity_arrow));
}

void FlightVisualizerNode::addSafetyTextMarker(
  std::vector<visualization_msgs::msg::Marker> & markers, const rclcpp::Time & stamp) const
{
  const std::string frame_id = hasEstimatedPose_ ? latestEstimatedPose_.header.frame_id : fixedFrame_;
  auto text = makeBaseMarker(
    frame_id, stamp, kNsSafety, kIdSafetyText,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);

  if (hasEstimatedPose_) {
    text.pose.position = latestEstimatedPose_.pose.position;
    text.pose.position.z += 1.0;
  } else {
    text.pose.position.x = 0.0;
    text.pose.position.y = 0.0;
    text.pose.position.z = 1.0;
  }
  text.pose.orientation.w = 1.0;
  text.scale.z = 0.35;

  uint8_t level = peregrine_interfaces::msg::SafetyStatus::LEVEL_NOMINAL;
  std::string reason = "waiting";
  if (latestSafetyStatus_.has_value()) {
    level = latestSafetyStatus_->level;
    if (!latestSafetyStatus_->reason.empty()) {
      reason = latestSafetyStatus_->reason;
    }
  }
  text.color = safetyLevelColor(level);

  std::ostringstream ss;
  ss << "Safety L" << static_cast<int>(level);
  if (latestUavState_.has_value()) {
    ss << " | " << uavStateToString(latestUavState_->state);
    ss << (latestUavState_->armed ? " | armed" : " | disarmed");
    ss << (latestUavState_->offboard ? " | offboard" : " | manual");
  }
  ss << "\n" << reason;
  text.text = ss.str();

  markers.push_back(std::move(text));
}

void FlightVisualizerNode::addOrDeleteGeofenceMarker(
  std::vector<visualization_msgs::msg::Marker> & markers, const rclcpp::Time & stamp) const
{
  if (!showGeofence_) {
    markers.push_back(makeDeleteMarker(fixedFrame_, stamp, kNsGeofence, kIdGeofenceVolume));
    markers.push_back(makeDeleteMarker(fixedFrame_, stamp, kNsGeofence, kIdGeofenceText));
    return;
  }

  const double min_alt = std::min(geofenceMinAltitudeM_, geofenceMaxAltitudeM_);
  const double max_alt = std::max(geofenceMinAltitudeM_, geofenceMaxAltitudeM_);
  const double height = std::max(0.1, max_alt - min_alt);

  auto volume = makeBaseMarker(
    fixedFrame_, stamp, kNsGeofence, kIdGeofenceVolume,
    visualization_msgs::msg::Marker::CYLINDER);
  volume.pose.orientation.w = 1.0;
  volume.pose.position.x = 0.0;
  volume.pose.position.y = 0.0;
  volume.pose.position.z = min_alt + (0.5 * height);
  volume.scale.x = 2.0 * geofenceRadiusM_;
  volume.scale.y = 2.0 * geofenceRadiusM_;
  volume.scale.z = height;
  volume.color = makeColor(0.20F, 0.55F, 0.95F, 0.12F);
  markers.push_back(std::move(volume));

  auto label = makeBaseMarker(
    fixedFrame_, stamp, kNsGeofence, kIdGeofenceText,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  label.pose.orientation.w = 1.0;
  label.pose.position.x = 0.0;
  label.pose.position.y = 0.0;
  label.pose.position.z = max_alt + 0.8;
  label.scale.z = 0.30;
  label.color = makeColor(0.20F, 0.65F, 1.0F, 0.9F);

  std::ostringstream ss;
  ss << "geofence R=" << geofenceRadiusM_ << "m Z=[" << min_alt << ", " << max_alt << "]m";
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
      actual_path.header.frame_id = fixedFrame_;
    }
    if (reference_path.header.frame_id.empty()) {
      reference_path.header.frame_id = fixedFrame_;
    }

    addOrDeleteVehicleMarkers(marker_array.markers, now);
    addOrDeleteSetpointMarkers(marker_array.markers, now);
    addSafetyTextMarker(marker_array.markers, now);
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
