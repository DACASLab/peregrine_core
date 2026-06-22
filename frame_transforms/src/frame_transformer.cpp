#include <frame_transforms/conversions.hpp>
#include <frame_transforms/frame_transformer.hpp>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace frame_transforms
{
using namespace std::chrono_literals;

namespace
{

/// Removes leading/trailing slashes so frame IDs remain stable when concatenated.
std::string normalizePrefix(const std::string& prefix)
{
  if (prefix.empty())
  {
    return "";
  }

  std::string out = prefix;
  if (out.front() == '/')
  {
    out.erase(out.begin());
  }
  while (!out.empty() && out.back() == '/')
  {
    out.pop_back();
  }
  return out;
}

// Composes "<prefix>/<frame>" when a prefix is provided.
std::string composeFrame(const std::string& prefix, const std::string& frame)
{
  if (prefix.empty())
  {
    return frame;
  }
  return prefix + "/" + frame;
}

// Canonicalizes frame IDs for tolerant comparison (e.g. optional leading slash).
std::string canonicalFrameId(std::string frameId)
{
  while (!frameId.empty() && frameId.front() == '/')
  {
    frameId.erase(frameId.begin());
  }
  while (!frameId.empty() && frameId.back() == '/')
  {
    frameId.pop_back();
  }
  return frameId;
}

bool sameFrameId(const std::string& lhs, const std::string& rhs)
{
  return canonicalFrameId(lhs) == canonicalFrameId(rhs);
}

// Shared utility for identity static transforms.
geometry_msgs::msg::Transform identityTransform()
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 0.0;
  transform.translation.y = 0.0;
  transform.translation.z = 0.0;
  transform.rotation.w = 1.0;
  transform.rotation.x = 0.0;
  transform.rotation.y = 0.0;
  transform.rotation.z = 0.0;
  return transform;
}

}  // namespace

FrameTransformer::FrameTransformer(const rclcpp::NodeOptions& options)
: Node("frame_transformer", options)
{
  paramListener_ = std::make_shared<frame_transforms::ParamListener>(
      get_node_parameters_interface());
  params_ = paramListener_->get_params();

  framePrefix_ = normalizePrefix(params_.frame_prefix);
  // world is shared across the fleet (NOT prefixed); map/odom/base_link are per-UAV.
  worldFrame_ = params_.world_frame;
  mapFrame_ = composeFrame(framePrefix_, params_.map_frame);
  odomFrame_ = composeFrame(framePrefix_, params_.odom_frame);
  baseLinkFrame_ = composeFrame(framePrefix_, params_.base_link_frame);
  baseLinkFrdFrame_ = composeFrame(framePrefix_, params_.base_link_frd_frame);

  tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  staticTfBroadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  odometrySub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      params_.odometry_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { odometryCallback(msg); });

  publishStaticTransforms();

  // FrameAnchor drives world->map (latched once) and map->odom (dynamic, steps on PX4 reset).
  // transient_local matches hardware_abstraction so we get the latest anchor even if we start
  // after it.
  frameAnchorSub_ = this->create_subscription<peregrine_interfaces::msg::FrameAnchor>(
      params_.frame_anchor_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](peregrine_interfaces::msg::FrameAnchor::SharedPtr msg) { onFrameAnchor(msg); });

  // Timer periodically emits the dynamic TF edges (world->map, map->odom, odom->base_link).
  const auto period = std::chrono::duration<double>(1.0 / params_.publish_rate_hz);
  dynamicTfTimer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishDynamicTransforms(); });

  RCLCPP_INFO(this->get_logger(), "frame_transformer started: odometry_topic=%s", params_.odometry_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "frames: world=%s map=%s odom=%s base_link=%s base_link_frd=%s",
              worldFrame_.c_str(), mapFrame_.c_str(), odomFrame_.c_str(), baseLinkFrame_.c_str(),
              baseLinkFrdFrame_.c_str());
}

void FrameTransformer::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::scoped_lock lock(odometryMutex_);
  // Keep only the latest sample; the timer publishes at the configured TF rate.
  latestOdometry_ = *msg;
}

void FrameTransformer::publishStaticTransforms()
{
  // Only base_link->base_link_frd is static now. world->map and map->odom are dynamic and
  // driven by FrameAnchor (see publishDynamicTransforms).
  const auto now = this->get_clock()->now();

  geometry_msgs::msg::TransformStamped baseLinkToFrd;
  baseLinkToFrd.header.stamp = now;
  baseLinkToFrd.header.frame_id = baseLinkFrame_;
  baseLinkToFrd.child_frame_id = baseLinkFrdFrame_;
  baseLinkToFrd.transform.translation.x = 0.0;
  baseLinkToFrd.transform.translation.y = 0.0;
  baseLinkToFrd.transform.translation.z = 0.0;
  baseLinkToFrd.transform.rotation = toRosQuaternion(Eigen::Quaterniond(fluToFrdMatrix()));

  staticTfBroadcaster_->sendTransform(baseLinkToFrd);
}

void FrameTransformer::publishDynamicTransforms()
{
  // Copy-under-lock pattern:
  // 1) acquire lock and copy shared data to a local variable
  // 2) release lock quickly
  // 3) do heavier work without holding the mutex
  //
  // This minimizes contention between callback and timer threads.
  std::optional<nav_msgs::msg::Odometry> odometry;
  {
    std::scoped_lock lock(odometryMutex_);
    odometry = latestOdometry_;
  }

  if (!odometry.has_value())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No odometry received yet.");
    return;
  }

  geometry_msgs::msg::TransformStamped odomToBase;
  odomToBase.header.stamp = odometry->header.stamp;
  // If the incoming odometry timestamp is unset, use local clock to keep TF valid.
  if (odomToBase.header.stamp.nanosec == 0 && odomToBase.header.stamp.sec == 0)
  {
    odomToBase.header.stamp = this->get_clock()->now();
  }

  // Static transforms use configured names, so dynamic TF enforces the same names.
  if (!odometry->header.frame_id.empty() && !sameFrameId(odometry->header.frame_id, odomFrame_))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Odometry parent frame_id '%s' does not match configured odom frame '%s'; using configured "
                         "frame for TF.",
                         odometry->header.frame_id.c_str(), odomFrame_.c_str());
  }
  if (!odometry->child_frame_id.empty() && !sameFrameId(odometry->child_frame_id, baseLinkFrame_))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Odometry child frame_id '%s' does not match configured base_link frame '%s'; using "
                         "configured frame for TF.",
                         odometry->child_frame_id.c_str(), baseLinkFrame_.c_str());
  }
  odomToBase.header.frame_id = odomFrame_;
  odomToBase.child_frame_id = baseLinkFrame_;
  odomToBase.transform.translation.x = odometry->pose.pose.position.x;
  odomToBase.transform.translation.y = odometry->pose.pose.position.y;
  odomToBase.transform.translation.z = odometry->pose.pose.position.z;

  // Normalize incoming orientation before TF publish; invalid quaternions fall back to identity.
  Eigen::Quaterniond orientation = toEigenQuaternion(odometry->pose.pose.orientation);
  if (!std::isfinite(orientation.w()) || !std::isfinite(orientation.x()) || !std::isfinite(orientation.y()) ||
      !std::isfinite(orientation.z()) || orientation.norm() < 1e-6)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Invalid odometry orientation received; publishing identity quaternion.");
    orientation = Eigen::Quaterniond::Identity();
  }
  else
  {
    orientation.normalize();
  }
  odomToBase.transform.rotation = toRosQuaternion(orientation);

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(3);

  // world->map (latched) and map->odom (dynamic) from the latest FrameAnchor. Until an anchor
  // arrives, fall back to identity so the tree stays connected.
  std::optional<peregrine_interfaces::msg::FrameAnchor> anchor;
  Eigen::Vector3d worldToMap{0.0, 0.0, 0.0};
  {
    std::scoped_lock lock(anchorMutex_);
    anchor = latestAnchor_;
    worldToMap = worldToMapTranslation_;
  }

  const auto stamp = odomToBase.header.stamp;

  geometry_msgs::msg::TransformStamped worldToMapTf;
  worldToMapTf.header.stamp = stamp;
  worldToMapTf.header.frame_id = worldFrame_;
  worldToMapTf.child_frame_id = mapFrame_;
  worldToMapTf.transform = identityTransform();
  worldToMapTf.transform.translation.x = worldToMap.x();
  worldToMapTf.transform.translation.y = worldToMap.y();
  worldToMapTf.transform.translation.z = worldToMap.z();
  transforms.push_back(worldToMapTf);

  // map->odom = INVERSE of the accumulated PX4 estimate jump (translation). yaw-reset
  // compensation is applied to orientation in hardware_abstraction's State, not rotated into
  // this TF edge (avoids rotating position about the origin); v1 keeps this a pure translation.
  geometry_msgs::msg::TransformStamped mapToOdomTf;
  mapToOdomTf.header.stamp = stamp;
  mapToOdomTf.header.frame_id = mapFrame_;
  mapToOdomTf.child_frame_id = odomFrame_;
  mapToOdomTf.transform = identityTransform();
  if (anchor.has_value())
  {
    mapToOdomTf.transform.translation.x = anchor->map_to_odom_enu.x;
    mapToOdomTf.transform.translation.y = anchor->map_to_odom_enu.y;
    mapToOdomTf.transform.translation.z = anchor->map_to_odom_enu.z;
  }
  transforms.push_back(mapToOdomTf);

  transforms.push_back(odomToBase);

  tfBroadcaster_->sendTransform(transforms);
}

void FrameTransformer::onFrameAnchor(const peregrine_interfaces::msg::FrameAnchor::SharedPtr msg)
{
  std::scoped_lock lock(anchorMutex_);
  latestAnchor_ = *msg;

  // Latch world->map ONCE: horizontal from the first valid ref_* + world_datum; vertical from
  // the ground datum so world z=0 is the takeoff surface. Subsequent origin moves arrive as
  // map->odom deltas (already in the anchor), so we do not re-latch (avoids double-counting).
  if (!worldToMapLatched_ && msg->ref_valid && msg->ground_datum_valid)
  {
    Eigen::Vector3d horiz{0.0, 0.0, 0.0};
    if (params_.world_datum_set)
    {
      horiz = geodeticToEnu(
          params_.world_datum_lat, params_.world_datum_lon, params_.world_datum_alt,
          msg->ref_lat, msg->ref_lon, msg->ref_alt);
    }
    worldToMapTranslation_ = Eigen::Vector3d(horiz.x(), horiz.y(), -msg->ground_datum_z);
    worldToMapLatched_ = true;
    RCLCPP_INFO(this->get_logger(),
        "world->map latched: ref=(%.6f, %.6f, %.2f) datum_set=%d translation=(%.2f, %.2f, %.2f)m",
        msg->ref_lat, msg->ref_lon, msg->ref_alt, params_.world_datum_set ? 1 : 0,
        worldToMapTranslation_.x(), worldToMapTranslation_.y(), worldToMapTranslation_.z());
  }
}

}  // namespace frame_transforms

RCLCPP_COMPONENTS_REGISTER_NODE(frame_transforms::FrameTransformer)
