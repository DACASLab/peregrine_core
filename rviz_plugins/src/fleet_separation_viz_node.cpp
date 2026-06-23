/**
 * @file fleet_separation_viz_node.cpp
 * @brief GCS-side RViz overlay of inter-UAV separation.
 *
 * Consumes the shared /fleet/agent_state bus (already bridged into the GCS by Zenoh; every
 * UAV broadcasts its own world-frame pose there for BVC avoidance) and renders, in RViz:
 *   - one line per UAV pair, colored green -> red as the pair approaches the warning radius,
 *   - a distance label on the closest pair,
 *   - a fixed min-separation readout (text) so the worst-case gap is visible at a glance.
 *
 * This is fleet-level (not per-UAV) so it runs once in the GCS. No new bridge entry is
 * needed: all inputs are already on /fleet/agent_state. Output is local to the GCS.
 *
 * Generalizes to N UAVs: pairs and markers are built from whatever agents are currently
 * publishing; stale agents are pruned by timeout. No fleet-size assumptions.
 */

#include <geometry_msgs/msg/point.hpp>
#include <peregrine_interfaces/msg/fleet_agent_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rviz_plugins
{
using namespace std::chrono_literals;

class FleetSeparationVizNode : public rclcpp::Node
{
public:
  explicit FleetSeparationVizNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("fleet_separation_viz", options)
  {
    fixedFrame_ = this->declare_parameter<std::string>("fixed_frame", "world");
    warningRadiusM_ = this->declare_parameter<double>("warning_radius_m", 4.0);
    agentTimeoutS_ = this->declare_parameter<double>("agent_timeout_s", 3.0);
    const double rateHz = this->declare_parameter<double>("publish_rate_hz", 5.0);

    sub_ = this->create_subscription<peregrine_interfaces::msg::FleetAgentState>(
      "/fleet/agent_state", rclcpp::QoS(20).reliable(),
      [this](peregrine_interfaces::msg::FleetAgentState::SharedPtr msg) {
        agents_[msg->uav_id] = Entry{*msg, this->now()};
      });

    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fleet/viz/separation", rclcpp::QoS(10).reliable());

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, rateHz));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() { onTimer(); });

    RCLCPP_INFO(
      this->get_logger(), "fleet_separation_viz up (frame=%s, warn=%.1fm)", fixedFrame_.c_str(),
      warningRadiusM_);
  }

private:
  struct Entry
  {
    peregrine_interfaces::msg::FleetAgentState state;
    rclcpp::Time stamp;
  };

  static double distance3d(
    const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
  }

  void onTimer()
  {
    const auto now = this->now();

    // Drop agents that have gone silent so removed/landed UAVs do not leave ghost lines.
    for (auto it = agents_.begin(); it != agents_.end();) {
      if ((now - it->second.stamp).seconds() > agentTimeoutS_) {
        it = agents_.erase(it);
      } else {
        ++it;
      }
    }

    std::vector<const Entry *> live;
    live.reserve(agents_.size());
    for (const auto & [id, entry] : agents_) {
      live.push_back(&entry);
    }

    visualization_msgs::msg::MarkerArray out;

    // A single DELETEALL keeps the previous frame's lines from accumulating as pairs change.
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = fixedFrame_;
    clear.header.stamp = now;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    out.markers.push_back(clear);

    double min_sep = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::Point min_a;
    geometry_msgs::msg::Point min_b;

    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = fixedFrame_;
    lines.header.stamp = now;
    lines.ns = "separation_lines";
    lines.id = 0;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.scale.x = 0.05;
    lines.pose.orientation.w = 1.0;

    for (std::size_t i = 0; i < live.size(); ++i) {
      for (std::size_t j = i + 1; j < live.size(); ++j) {
        const auto & pa = live[i]->state.position;
        const auto & pb = live[j]->state.position;
        const double d = distance3d(pa, pb);

        // Per-segment vertex colors: green when clear, ramping to red near the warning radius.
        const double t = std::clamp(1.0 - (d / std::max(1e-3, warningRadiusM_)), 0.0, 1.0);
        std_msgs::msg::ColorRGBA c;
        c.r = static_cast<float>(t);
        c.g = static_cast<float>(1.0 - t);
        c.b = 0.15F;
        c.a = 0.9F;
        lines.points.push_back(pa);
        lines.points.push_back(pb);
        lines.colors.push_back(c);
        lines.colors.push_back(c);

        if (d < min_sep) {
          min_sep = d;
          min_a = pa;
          min_b = pb;
        }
      }
    }
    out.markers.push_back(std::move(lines));

    if (std::isfinite(min_sep)) {
      // Distance label on the closest pair, at the segment midpoint.
      visualization_msgs::msg::Marker mid;
      mid.header.frame_id = fixedFrame_;
      mid.header.stamp = now;
      mid.ns = "separation_min_label";
      mid.id = 0;
      mid.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      mid.action = visualization_msgs::msg::Marker::ADD;
      mid.pose.position.x = 0.5 * (min_a.x + min_b.x);
      mid.pose.position.y = 0.5 * (min_a.y + min_b.y);
      mid.pose.position.z = 0.5 * (min_a.z + min_b.z) + 0.4;
      mid.pose.orientation.w = 1.0;
      mid.scale.z = 0.5;
      mid.color.r = min_sep < warningRadiusM_ ? 0.95F : 0.85F;
      mid.color.g = min_sep < warningRadiusM_ ? 0.30F : 0.90F;
      mid.color.b = 0.20F;
      mid.color.a = 1.0F;
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.2f m", min_sep);
      mid.text = buf;
      out.markers.push_back(std::move(mid));
    }

    pub_->publish(out);
  }

  std::string fixedFrame_;
  double warningRadiusM_{4.0};
  double agentTimeoutS_{3.0};

  std::map<std::string, Entry> agents_;
  rclcpp::Subscription<peregrine_interfaces::msg::FleetAgentState>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rviz_plugins

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rviz_plugins::FleetSeparationVizNode>());
  rclcpp::shutdown();
  return 0;
}
