#include <peregrine_bringup/mocap_relay.hpp>

#include <rclcpp_components/register_node_macro.hpp>

namespace peregrine_bringup
{

MocapRelay::MocapRelay(const rclcpp::NodeOptions & options)
: Node("mocap_relay", options)
{
  bodyName_ = this->declare_parameter<std::string>("mocap_body_name", "");
  if (bodyName_.empty()) {
    throw std::runtime_error("mocap_body_name parameter is required and must not be empty.");
  }

  const auto mocapTopic = this->declare_parameter<std::string>("mocap_topic", "/poses");
  const auto outputTopic = this->declare_parameter<std::string>("mocap_output_topic", "mocap_pose");

  // Mocap drivers typically publish best_effort; match that QoS to avoid silent discovery failure.
  sub_ = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(
    mocapTopic, rclcpp::SensorDataQoS(),
    [this](motion_capture_tracking_interfaces::msg::NamedPoseArray::SharedPtr msg) {
      onNamedPoseArray(msg);
    });

  // Output uses reliable QoS for the manager pipeline.
  pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    outputTopic, rclcpp::QoS(10).reliable());

  RCLCPP_INFO(this->get_logger(),
    "MocapRelay started: body='%s', input='%s', output='%s'",
    bodyName_.c_str(), mocapTopic.c_str(), outputTopic.c_str());
}

void MocapRelay::onNamedPoseArray(
  const motion_capture_tracking_interfaces::msg::NamedPoseArray::SharedPtr msg)
{
  for (const auto & named_pose : msg->poses) {
    if (named_pose.name == bodyName_) {
      geometry_msgs::msg::PoseStamped out;
      out.header = msg->header;
      out.pose = named_pose.pose;
      pub_->publish(out);
      return;
    }
  }

  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    "Body '%s' not found in NamedPoseArray (%zu poses).",
    bodyName_.c_str(), msg->poses.size());
}

}  // namespace peregrine_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(peregrine_bringup::MocapRelay)
