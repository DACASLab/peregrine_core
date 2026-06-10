/**
 * @file neighbor_tracker.hpp
 * @brief Stores the latest FleetAgentState from each peer and classifies its
 *        freshness so the node can grow safety buffers / freeze lost agents.
 */

#pragma once

#include <peregrine_interfaces/msg/fleet_agent_state.hpp>
#include <rclcpp/time.hpp>

#include <Eigen/Core>

#include <map>
#include <string>
#include <vector>

namespace multi_agent_coordinator
{

/// Freshness tier of a neighbor, used to scale conservatism.
enum class NeighborHealth
{
  Fresh,  ///< recent update: nominal buffer, predict forward
  Stale,  ///< missed a few updates: inflate buffer, keep predicting
  Lost    ///< silent: freeze last position as a static obstacle, raise diagnostic
};

/// 2D fleet-frame view of a neighbor for BVC building (z kept for altitude gating).
struct NeighborView
{
  std::string uav_id;
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
  double z{0.0};
  uint8_t mode{0};
  NeighborHealth health{NeighborHealth::Fresh};
  double age_s{0.0};
};

class NeighborTracker
{
public:
  struct Config
  {
    double stale_timeout_s{0.4};
    double lost_timeout_s{1.5};
    double hard_expiry_s{30.0};  ///< drop entries older than this (agent gone)
  };

  void setConfig(const Config & cfg) { cfg_ = cfg; }

  /// Insert/replace the latest state for the sender. Ignores empty uav_id.
  void update(const peregrine_interfaces::msg::FleetAgentState & msg, const rclcpp::Time & recv);

  /// Classified views for every tracked peer except self_id, dropping
  /// hard-expired entries. Health/age computed against `now`.
  std::vector<NeighborView> neighbors(const rclcpp::Time & now, const std::string & self_id);

private:
  struct Entry
  {
    peregrine_interfaces::msg::FleetAgentState msg;
    rclcpp::Time recv;
  };
  std::map<std::string, Entry> entries_;
  Config cfg_{};
};

}  // namespace multi_agent_coordinator
