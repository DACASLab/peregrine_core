/**
 * @file px4_passthrough_estimator.hpp
 * @brief Passthrough estimator implementation for estimation_manager.
 *
 * This estimator trusts PX4's onboard EKF2 filter output and does not apply any
 * additional filtering, smoothing, or sensor fusion. It simply stores the most
 * recent state sample and returns it on demand.
 *
 * The `source` field is overwritten to "estimation_manager/px4_passthrough" to
 * indicate which estimator produced the output.
 *
 * Thread safety uses atomic shared_ptr operations (lock-free on most platforms)
 * instead of a mutex, since the cached data is a single immutable snapshot.
 */

#pragma once

#include <estimation_manager/estimator_base.hpp>

#include <atomic>
#include <memory>

namespace estimation_manager
{

struct StateSnapshot
{
  peregrine_interfaces::msg::State state;
  rclcpp::Time updateTime{0, 0, RCL_ROS_TIME};
};

/**
 * @class Px4PassthroughEstimator
 * @brief Stores and returns the latest state sample without modification.
 */
class Px4PassthroughEstimator : public EstimatorBase
{
public:
  void processState(const peregrine_interfaces::msg::State & state) override;
  bool hasEstimate() const override;
  peregrine_interfaces::msg::State getEstimate() const override;
  rclcpp::Time lastUpdateTime() const override;

private:
  std::shared_ptr<const StateSnapshot> snapshot_;
};

}  // namespace estimation_manager
