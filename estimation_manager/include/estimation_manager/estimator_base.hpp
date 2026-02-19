/**
 * @file estimator_base.hpp
 * @brief Abstract estimator interface for estimation_manager.
 */

#pragma once

#include <peregrine_interfaces/msg/state.hpp>
#include <rclcpp/time.hpp>

namespace estimation_manager
{

/**
 * @class EstimatorBase
 * @brief Interface implemented by all estimator backends in estimation_manager.
 */
class EstimatorBase
{
public:
  virtual ~EstimatorBase() = default;

  /**
   * @brief Processes a new state sample from hardware_abstraction.
   */
  virtual void processState(const peregrine_interfaces::msg::State& state) = 0;

  /**
   * @brief Returns true when at least one estimate is available.
   */
  virtual bool hasEstimate() const = 0;

  /**
   * @brief Returns the latest estimate snapshot.
   */
  virtual peregrine_interfaces::msg::State getEstimate() const = 0;

  /**
   * @brief Returns the timestamp of the last processed sample.
   */
  virtual rclcpp::Time lastUpdateTime() const = 0;
};

}  // namespace estimation_manager
