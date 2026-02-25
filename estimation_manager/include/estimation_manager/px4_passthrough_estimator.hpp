/**
 * @note C++ Primer for Python ROS2 readers
 *
 * This file follows a few recurring C++ patterns:
 * - Ownership is explicit: `std::unique_ptr` means single owner, `std::shared_ptr` means shared ownership.
 * - References (`T&`) and `const` are used to avoid unnecessary copies and make mutation intent explicit.
 * - RAII is used for resource safety: objects such as locks clean themselves up automatically at scope exit.
 * - ROS2 callbacks may run concurrently depending on executor/callback-group setup, so shared state is guarded.
 * - Templates (for example `create_subscription<MsgT>`) are compile-time type binding, not runtime reflection.
 */
/**
 * @file px4_passthrough_estimator.hpp
 * @brief Passthrough estimator implementation for estimation_manager.
 *
 * This estimator trusts PX4's onboard EKF2 filter output and does not apply any
 * additional filtering, smoothing, or sensor fusion. It simply stores the most
 * recent state sample and returns it on demand.
 *
 * The `source` field is overwritten to "estimation_manager/px4_passthrough" to
 * indicate which estimator produced the output. Downstream consumers can use this
 * field for diagnostics or to select behavior based on estimator type.
 *
 * All public methods are thread-safe (guarded by an internal mutex) because they
 * are accessed from both subscription callbacks (processState) and timer callbacks
 * (hasEstimate, getEstimate, lastUpdateTime) which may execute concurrently in a
 * MultiThreadedExecutor.
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
// `: public EstimatorBase` is C++ inheritance syntax. Equivalent to Python's
// `class Px4PassthroughEstimator(EstimatorBase):`. The `public` keyword means
// the base class's public interface remains public in the derived class (this is
// almost always what you want; private/protected inheritance exists but is rare).
class Px4PassthroughEstimator : public EstimatorBase
{
public:
  /**
   * @brief Stores the latest state received from hardware_abstraction.
   */
  // `override` tells the compiler this method replaces a virtual method from the
  // base class. If the base signature changes and this no longer matches, the
  // compiler produces an error. Without `override`, a signature mismatch would
  // silently create a NEW method instead of overriding the base one.
  void processState(const peregrine_interfaces::msg::State & state) override;

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

// `private:` members are only accessible from within this class. Unlike Python's
// convention of prefixing with underscore (_private), C++ enforces this at compile
// time — external code literally cannot access these fields.
private:
  // `mutable` allows this mutex to be locked even inside `const` methods (like
  // hasEstimate() const). Normally, `const` methods cannot modify member
  // variables. `mutable` is an exception specifically designed for synchronization
  // primitives — locking a mutex doesn't change the object's logical state, only
  // its internal synchronization state.
  //
  // std::mutex is a mutual exclusion lock, similar to Python's threading.Lock().
  // C++ does not have a GIL (Global Interpreter Lock), so concurrent access to
  // shared data MUST be explicitly synchronized. Without this mutex, two threads
  // reading/writing latestState_ simultaneously would cause undefined behavior.
  mutable std::mutex mutex_;

  // `{false}` and `{0, 0, RCL_ROS_TIME}` are in-class member initializers
  // (C++11). They provide default values similar to Python's `self.has_estimate = False`
  // in __init__. If the constructor doesn't explicitly initialize these, these
  // defaults are used.
  bool hasEstimate_{false};
  peregrine_interfaces::msg::State latestState_;
  rclcpp::Time lastUpdateTime_{0, 0, RCL_ROS_TIME};
};

}  // namespace estimation_manager
