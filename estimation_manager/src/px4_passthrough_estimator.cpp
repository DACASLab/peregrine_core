#include <estimation_manager/px4_passthrough_estimator.hpp>

#include <utility>

namespace estimation_manager
{

void Px4PassthroughEstimator::processState(const peregrine_interfaces::msg::State& state)
{
  std::scoped_lock lock(mutex_);
  latestState_ = state;
  latestState_.source = "estimation_manager/px4_passthrough";
  hasEstimate_ = true;
  lastUpdateTime_ = rclcpp::Time(state.header.stamp);
}

bool Px4PassthroughEstimator::hasEstimate() const
{
  std::scoped_lock lock(mutex_);
  return hasEstimate_;
}

peregrine_interfaces::msg::State Px4PassthroughEstimator::getEstimate() const
{
  std::scoped_lock lock(mutex_);
  return latestState_;
}

rclcpp::Time Px4PassthroughEstimator::lastUpdateTime() const
{
  std::scoped_lock lock(mutex_);
  return lastUpdateTime_;
}

}  // namespace estimation_manager
