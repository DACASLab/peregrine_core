/**
 * @file px4_passthrough_estimator.hpp
 * @brief Passthrough estimator implementation for estimation_manager.
 */

#pragma once

#include <estimation_manager/estimator_base.hpp>

#include <mutex>

namespace estimation_manager
{

/**
 * @class Px4PassthroughEstimator
 * @brief Stores and returns the latest state sample without modification.
 */
class Px4PassthroughEstimator : public EstimatorBase
{
public:
  /**
   * @brief Stores the latest state received from hardware_abstraction.
   */
  void processState(const peregrine_interfaces::msg::State& state) override;

  /**
   * @brief Returns whether at least one state sample has been received.
   */
  bool hasEstimate() const override;

  /**
   * @brief Returns the latest stored state snapshot.
   */
  peregrine_interfaces::msg::State getEstimate() const override;

  /**
   * @brief Returns the timestamp from the latest stored sample.
   */
  rclcpp::Time lastUpdateTime() const override;

private:
  mutable std::mutex mutex_;
  bool hasEstimate_{false};
  peregrine_interfaces::msg::State latestState_;
  rclcpp::Time lastUpdateTime_{0, 0, RCL_ROS_TIME};
};

}  // namespace estimation_manager
