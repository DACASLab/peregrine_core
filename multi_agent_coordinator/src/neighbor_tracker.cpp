#include <multi_agent_coordinator/neighbor_tracker.hpp>

namespace multi_agent_coordinator
{

void NeighborTracker::update(
  const peregrine_interfaces::msg::FleetAgentState & msg, const rclcpp::Time & recv)
{
  if (msg.uav_id.empty()) {
    return;
  }
  entries_[msg.uav_id] = Entry{msg, recv};
}

std::vector<NeighborView> NeighborTracker::neighbors(
  const rclcpp::Time & now, const std::string & self_id)
{
  std::vector<NeighborView> out;
  out.reserve(entries_.size());

  for (auto it = entries_.begin(); it != entries_.end();) {
    const double age = (now - it->second.recv).seconds();

    // Drop agents that have been silent long enough to be considered gone, so a
    // landed/departed UAV does not block the airspace forever.
    if (age > cfg_.hard_expiry_s) {
      it = entries_.erase(it);
      continue;
    }

    if (it->first == self_id) {
      ++it;
      continue;
    }

    const auto & m = it->second.msg;
    NeighborView v;
    v.uav_id = m.uav_id;
    v.position = Eigen::Vector2d(m.position.x, m.position.y);
    v.velocity = Eigen::Vector2d(m.velocity.x, m.velocity.y);
    v.z = m.position.z;
    v.mode = m.mode;
    v.age_s = age;

    if (age <= cfg_.stale_timeout_s) {
      v.health = NeighborHealth::Fresh;
    } else if (age <= cfg_.lost_timeout_s) {
      v.health = NeighborHealth::Stale;
    } else {
      v.health = NeighborHealth::Lost;
    }
    out.push_back(std::move(v));
    ++it;
  }
  return out;
}

}  // namespace multi_agent_coordinator
