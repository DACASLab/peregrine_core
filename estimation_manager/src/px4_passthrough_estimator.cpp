#include <estimation_manager/px4_passthrough_estimator.hpp>

namespace estimation_manager
{

void Px4PassthroughEstimator::processState(const peregrine_interfaces::msg::State & state)
{
  auto snap = std::make_shared<StateSnapshot>();
  snap->state = state;
  snap->state.source = "estimation_manager/px4_passthrough";
  snap->updateTime = rclcpp::Time(state.header.stamp);
  std::atomic_store(&snapshot_, std::shared_ptr<const StateSnapshot>(std::move(snap)));
}

bool Px4PassthroughEstimator::hasEstimate() const
{
  return std::atomic_load(&snapshot_) != nullptr;
}

peregrine_interfaces::msg::State Px4PassthroughEstimator::getEstimate() const
{
  auto snap = std::atomic_load(&snapshot_);
  return snap ? snap->state : peregrine_interfaces::msg::State{};
}

rclcpp::Time Px4PassthroughEstimator::lastUpdateTime() const
{
  auto snap = std::atomic_load(&snapshot_);
  return snap ? snap->updateTime : rclcpp::Time(0, 0, RCL_ROS_TIME);
}

}  // namespace estimation_manager
