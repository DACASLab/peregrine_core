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
 * @file estimator_base.hpp
 * @brief Abstract estimator interface for estimation_manager.
 *
 * Estimator backends implement this interface to process raw state samples from
 * hardware_abstraction. The manager calls processState() on each incoming sample,
 * then periodically calls getEstimate() at the configured publication rate.
 *
 * Thread safety: processState() is called from a subscription callback while
 * hasEstimate()/getEstimate()/lastUpdateTime() are called from a timer callback.
 * Implementations must synchronize access (e.g., via std::mutex) since these
 * callbacks may run concurrently on different executor threads.
 *
 * C++ note for Python developers:
 *   This is an "abstract base class" (ABC), analogous to a Python ABC with
 *   @abstractmethod decorators. In C++, methods ending in `= 0` are "pure virtual"
 *   -- they MUST be overridden by any subclass, just like @abstractmethod. You
 *   cannot instantiate this class directly; only concrete subclasses (like
 *   Px4PassthroughEstimator) can be created.
 */

// `#pragma once` is a header guard that prevents this file from being included
// more than once in a single compilation unit. In Python, imports are naturally
// deduplicated; in C++, `#include` is literally a text copy-paste, so without this
// guard you'd get "duplicate definition" compiler errors.
#pragma once

// These `#include` directives are like Python `import` statements, but they
// literally copy the contents of the referenced header file into this file at
// compile time. Unlike Python, C++ has no module system (until C++20 modules),
// so every file must explicitly include everything it uses.
#include <peregrine_interfaces/msg/state.hpp>
#include <rclcpp/time.hpp>

// C++ namespaces are similar to Python packages/modules. They prevent name
// collisions (e.g., estimation_manager::EstimatorBase won't clash with
// control_manager::ControllerBase). The `::` operator is like Python's `.` for
// accessing nested names.
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
// `public:` means everything below is accessible from outside the class.
// C++ has three access levels: public (like Python default), protected (subclass
// only), and private (class only). Python uses naming conventions (_underscore);
// C++ enforces access at compile time.
public:
  // `virtual ~EstimatorBase() = default;`
  //   - `virtual` makes the destructor polymorphic. In Python, __del__ always
  //     works polymorphically. In C++, if you `delete` a base pointer without a
  //     virtual destructor, the derived class destructor won't run (memory leak).
  //   - `~EstimatorBase()` is the destructor (like Python's __del__).
  //   - `= default` tells the compiler to generate the default implementation
  //     (equivalent to not writing __del__ in Python at all).
  virtual ~EstimatorBase() = default;

  /**
   * @brief Processes a new state sample from hardware_abstraction.
   */
  // `virtual ... = 0` makes this a "pure virtual" method -- the C++ equivalent
  // of Python's @abstractmethod. Any class that inherits EstimatorBase MUST
  // implement this method, or it too becomes abstract and can't be instantiated.
  //
  // `const peregrine_interfaces::msg::State & state`:
  //   - `const` means the function promises not to modify `state`
  //   - `&` makes this a reference (like Python's default pass-by-reference for
  //     objects), avoiding an expensive copy of the entire State message.
  //     Without `&`, C++ would copy the whole struct on every call.
  virtual void processState(const peregrine_interfaces::msg::State & state) = 0;

  /**
   * @brief Returns true when at least one estimate is available.
   */
  // The trailing `const` means this method does not modify the object's state.
  // Python has no equivalent -- it's a compile-time promise to callers that
  // calling hasEstimate() has no side effects on the object.
  virtual bool hasEstimate() const = 0;

  /**
   * @brief Returns the latest estimate snapshot.
   */
  // Returns by value (a full copy), not by reference. This is intentional:
  // returning a reference to internal data would let callers see partially-
  // updated state if another thread is writing concurrently. Returning a copy
  // gives the caller an independent snapshot.
  virtual peregrine_interfaces::msg::State getEstimate() const = 0;

  /**
   * @brief Returns the timestamp of the last processed sample.
   */
  virtual rclcpp::Time lastUpdateTime() const = 0;
};

}  // namespace estimation_manager
