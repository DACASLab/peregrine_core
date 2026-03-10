/**
 * @file estimator_base.hpp
 * @brief Abstract estimator interface for estimation_manager.
 *
 * Estimator backends implement this interface to process raw state samples from
 * hardware_abstraction. The manager calls processState() on each incoming sample,
 * then periodically calls getEstimate() at the configured publication rate.
 *
 * Thread safety: processState() is called from a subscription callback while
 * hasEstimate()/getEstimate()/lastUpdateTime() are called from a timer callback.
 * Implementations must synchronize access internally (e.g., via std::mutex).
 */

#pragma once

#include <peregrine_interfaces/msg/state.hpp>
#include <rclcpp/time.hpp>

namespace estimation_manager
{

/**
 * @class EstimatorBase
 * @brief Interface implemented by all estimator backends in estimation_manager.
 *
 * Contract:
 *  - processState() is called once per incoming sample (may be high-frequency)
 *  - hasEstimate() must return false until at least one valid sample has been processed
 *  - getEstimate() returns a complete snapshot (deep copy, not a reference)
 *  - lastUpdateTime() is used by the manager for freshness-based health evaluation
 */
class EstimatorBase
{
public:
  virtual ~EstimatorBase() = default;

  /**
   * @brief Processes a new state sample from hardware_abstraction.
   */
  virtual void processState(const peregrine_interfaces::msg::State & state) = 0;

  /**
   * @brief Returns true when at least one estimate is available.
   */
  virtual bool hasEstimate() const = 0;

  /**
   * @brief Returns the latest estimate as an independent snapshot.
   */
  virtual peregrine_interfaces::msg::State getEstimate() const = 0;

  /**
   * @brief Returns the timestamp of the last processed sample.
   */
  virtual rclcpp::Time lastUpdateTime() const = 0;
};

}  // namespace estimation_manager
