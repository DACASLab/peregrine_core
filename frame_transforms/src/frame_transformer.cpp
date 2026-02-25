/**
 * @note C++ Primer for Python ROS2 readers
 *
 * This file follows a few recurring C++ patterns:
 * - Ownership is explicit: `std::unique_ptr` means single owner, `std::shared_ptr` means shared ownership.
 * - References (`T&`) and `const` are used to avoid unnecessary copies and make mutation intent explicit.
 * - RAII is used for resource safety: objects such as locks clean themselves up automatically at scope exit.
 * - ROS2 callbacks may run concurrently depending on executor/callback-group setup, so shared state is guarded.
 * - Templates (for example `create_subscription<MsgT>`) are compile-time type binding, not runtime reflection.
 */
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
namespace
{

// C++ linkage note:
// Symbols inside an unnamed namespace have "internal linkage", meaning they are visible
// only in this translation unit (.cpp file). This is similar to module-private helper
// functions in Python (by convention, `_helper`), but enforced by the linker.

// Removes leading/trailing slashes so frame IDs remain stable when concatenated.
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
  // `declare_parameter<T>()` is a function template. The `<std::string>` and `<double>`
  // arguments are compile-time type parameters, similar to Python type hints but enforced
  // by the compiler and used to select concrete function instantiations.
  // Parameters define topic and the local frame naming convention for one UAV.
  framePrefix_ = normalizePrefix(this->declare_parameter<std::string>("frame_prefix", ""));
  odometryTopic_ = this->declare_parameter<std::string>("odometry_topic", "odometry");
  worldFrame_ = composeFrame(framePrefix_, this->declare_parameter<std::string>("world_frame", "world"));
  mapFrame_ = composeFrame(framePrefix_, this->declare_parameter<std::string>("map_frame", "map"));
  odomFrame_ = composeFrame(framePrefix_, this->declare_parameter<std::string>("odom_frame", "odom"));
  baseLinkFrame_ = composeFrame(framePrefix_, this->declare_parameter<std::string>("base_link_frame", "base_link"));
  baseLinkFrdFrame_ =
      composeFrame(framePrefix_, this->declare_parameter<std::string>("base_link_frd_frame", "base_link_frd"));
  publishRateHz_ = this->declare_parameter<double>("publish_rate_hz", 100.0);

  if (publishRateHz_ <= 0.0)
  {
    throw std::runtime_error("publish_rate_hz must be > 0");
  }

  // One dynamic and one static broadcaster keeps TF ownership clear in this node.
  // `std::make_unique<T>(...)` returns a `std::unique_ptr<T>`, which gives single-owner
  // semantics for heap objects. Ownership cannot be copied (only moved), making lifetime
  // deterministic and preventing accidental shared ownership.
  tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  staticTfBroadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  odometrySub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometryTopic_, rclcpp::SensorDataQoS(),
      std::bind(&FrameTransformer::odometryCallback, this, std::placeholders::_1));

  publishStaticTransforms();

  // Timer periodically emits the latest odom->base_link transform.
  const auto period = std::chrono::duration<double>(1.0 / publishRateHz_);
  dynamicTfTimer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FrameTransformer::publishDynamicTransforms, this));

  RCLCPP_INFO(this->get_logger(), "frame_transformer started: odometry_topic=%s", odometryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "frames: world=%s map=%s odom=%s base_link=%s base_link_frd=%s",
              worldFrame_.c_str(), mapFrame_.c_str(), odomFrame_.c_str(), baseLinkFrame_.c_str(),
              baseLinkFrdFrame_.c_str());
}

void FrameTransformer::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // `std::scoped_lock` is RAII-based locking: lock acquired at construction, released
  // automatically at scope exit. Equivalent Python mental model is `with lock: ...`.
  std::scoped_lock lock(odometryMutex_);
  // Keep only latest sample; timer publishes at configured TF rate.
  // `latestOdometry_` is std::optional<Odometry>, so assignment engages the optional.
  latestOdometry_ = *msg;
}

void FrameTransformer::publishStaticTransforms()
{
  // `std::vector` is C++'s dynamic contiguous array (closest to Python list). `reserve(3)`
  // pre-allocates capacity for exactly three pushes to avoid reallocations.
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(3);

  const auto now = this->get_clock()->now();

  // Keep world->map identity in this package; global fleet/world alignment belongs elsewhere.
  geometry_msgs::msg::TransformStamped worldToMap;
  worldToMap.header.stamp = now;
  worldToMap.header.frame_id = worldFrame_;
  worldToMap.child_frame_id = mapFrame_;
  worldToMap.transform = identityTransform();
  transforms.push_back(worldToMap);

  // map->odom starts as identity; higher-level localization can own this later if needed.
  geometry_msgs::msg::TransformStamped mapToOdom;
  mapToOdom.header.stamp = now;
  mapToOdom.header.frame_id = mapFrame_;
  mapToOdom.child_frame_id = odomFrame_;
  mapToOdom.transform = identityTransform();
  transforms.push_back(mapToOdom);

  // Explicitly publish FLU->FRD relationship for packages that consume PX4-body conventions.
  geometry_msgs::msg::TransformStamped baseLinkToFrd;
  baseLinkToFrd.header.stamp = now;
  baseLinkToFrd.header.frame_id = baseLinkFrame_;
  baseLinkToFrd.child_frame_id = baseLinkFrdFrame_;
  baseLinkToFrd.transform.translation.x = 0.0;
  baseLinkToFrd.transform.translation.y = 0.0;
  baseLinkToFrd.transform.translation.z = 0.0;
  baseLinkToFrd.transform.rotation = toRosQuaternion(Eigen::Quaterniond(fluToFrdMatrix()));
  transforms.push_back(baseLinkToFrd);

  staticTfBroadcaster_->sendTransform(transforms);
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
  // `odometry->...` uses pointer-like access on std::optional (`operator->`), equivalent
  // to writing `(*odometry).header.stamp`.
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

  tfBroadcaster_->sendTransform(odomToBase);
}

}  // namespace frame_transforms

RCLCPP_COMPONENTS_REGISTER_NODE(frame_transforms::FrameTransformer)
