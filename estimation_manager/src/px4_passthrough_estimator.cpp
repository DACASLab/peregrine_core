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
#include <estimation_manager/px4_passthrough_estimator.hpp>

#include <utility>

namespace estimation_manager
{

// `Px4PassthroughEstimator::processState` — the `::` is the scope resolution
// operator, indicating this function belongs to the Px4PassthroughEstimator class.
// In Python, methods are defined inside the class body. In C++, the class
// declaration (in the .hpp header) only declares method signatures; the actual
// implementation lives in the .cpp source file, connected via `ClassName::method`.
void Px4PassthroughEstimator::processState(const peregrine_interfaces::msg::State & state)
{
  // `std::scoped_lock lock(mutex_)` acquires the mutex and automatically releases
  // it when `lock` goes out of scope (at the closing `}`). This is RAII (Resource
  // Acquisition Is Initialization) — the C++ equivalent of Python's `with` statement:
  //   with threading.Lock():
  //       ...
  // The lock is released even if an exception is thrown, preventing deadlocks.
  std::scoped_lock lock(mutex_);
  // `latestState_ = state` performs a deep copy of the entire State message.
  // In Python, `self.latest_state = state` would create a reference (alias) to
  // the same object. In C++, the `=` operator on structs/classes copies all fields.
  // This is intentional: the copy-under-lock pattern ensures that getEstimate()
  // always returns a complete, self-consistent snapshot (no torn reads across fields).
  latestState_ = state;
  // Tag the output so downstream consumers can identify which estimator produced it.
  latestState_.source = "estimation_manager/px4_passthrough";
  hasEstimate_ = true;
  // Freshness checks in manager status use this timestamp directly.
  lastUpdateTime_ = rclcpp::Time(state.header.stamp);
}

bool Px4PassthroughEstimator::hasEstimate() const
{
  std::scoped_lock lock(mutex_);
  return hasEstimate_;
}

// Returns a deep copy of the latest state. The lock ensures the returned message is
// coherent even if processState() is writing concurrently on another thread.
//
// `const` at the end means this method does not modify the object's logical state.
// The mutex is declared `mutable` in the header, which allows locking it even
// inside a `const` method — an accepted C++ pattern for thread-safe accessors.
peregrine_interfaces::msg::State Px4PassthroughEstimator::getEstimate() const
{
  std::scoped_lock lock(mutex_);
  // `return latestState_` returns a COPY. The caller gets their own independent
  // State object. Once this function returns and the lock releases, the caller's
  // copy is unaffected by subsequent processState() calls on other threads.
  return latestState_;
}

rclcpp::Time Px4PassthroughEstimator::lastUpdateTime() const
{
  std::scoped_lock lock(mutex_);
  return lastUpdateTime_;
}

}  // namespace estimation_manager
